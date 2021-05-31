import pybullet
import pybullet_data
from character_motion.src.character import *
import time
import numpy as np
import transformations as transform
from pybullet_utils import bullet_client

class Environment():
    def __init__(self, args):
        
        # Setup arguments
        self.useGUI = "useGUI" in args and args["useGUI"]
        self.usePlane = "usePlane" in args and args["usePlane"]
        self.timeStep = (1. / 60.) if "timeStep" not in args else args["timeStep"]
        self.solverIterations = 32 if "solverIterations" not in args else args["solverIterations"]
        self.gravity = -10 if "upGravity" not in args else args["upGravity"]
        self.useDatetime = True if "useDatetime" not in args else args["useDatetime"]
        self.visualizeOffset = "visualizeOffset" in args and args["visualizeOffset"]
        self.globalTime = 0.0
        self.startTime = time.time() if self.useDatetime else 0

        # Connect to PyBullet
        if (self.useGUI): 
            self.p = bullet_client.BulletClient(connection_mode=pybullet.SHARED_MEMORY)
            self.p.configureDebugVisualizer(self.p.COV_ENABLE_GUI, 0)
        else: self.p = bullet_client.BulletClient()
        
        # Set parameters
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.p.setPhysicsEngineParameter(numSolverIterations=self.solverIterations)
        self.p.setPhysicsEngineParameter(fixedTimeStep=self.timeStep)
        self.p.setGravity(0, 0, self.gravity)
        if (self.usePlane): 
            self.planeID = self.p.loadURDF("plane.urdf")
            self.p.resetBasePositionAndOrientation(self.planeID, [0,0,-0.05], [0,0,0,1])
            self.p.setCollisionFilterGroupMask(self.planeID, -1, 1, 0)

        self.characters = []

    def stepSimulation(self, updateCharacters=True):
        self.globalTime = time.time() - self.startTime if self.useDatetime else self.globalTime + self.timeStep
        if (updateCharacters):
            for character in self.characters:
                Character.stepCharacter(character, self.globalTime)
        
    def resetTime(self):
        self.startTime = time.time() if self.useDatetime else 0
        self.globalTime = 0


    prevState = [0] * 110
    # Construct the state features used for RL
    def constructRLState(self, prevAction, desiredCMVel):
        (simModel, kinModel) = self.fetchModels()
        dt = self.timeStep
        negAngleKin = -math.atan2(kinModel.forward.y, kinModel.forward.x)
        vSim = (simModel.poseInfo[0] - simModel.prevPoseInfo[0]) / dt
        vKin = (kinModel.poseInfo[0] - kinModel.prevPoseInfo[0]) / dt
        vSim2D = helpers.rotateBy(vSim.Vector2(), negAngleKin)
        vKin2D = helpers.rotateBy(vKin.Vector2(), negAngleKin)
        vSim = Vector3(vSim2D.x, vSim2D.y, vSim.z)
        vKin = Vector3(vKin2D.x, vKin2D.y, vKin.z)
        desiredCMVel = helpers.rotateBy(desiredCMVel, negAngleKin)
        vDiff = (desiredCMVel - vSim.Vector2())
        sSim = simModel.bodyState.copy()
        sKin = kinModel.bodyState.copy()
        for i in range(len(sSim)):
            sSimCurr = sSim[i]
            sKinCurr = sKin[i]
            sSimPrev = simModel.prevBodyState[i]
            sKinPrev = kinModel.prevBodyState[i]
            sSimCurr -= simModel.poseInfo[0]
            sKinCurr -= kinModel.poseInfo[0]
            sSimPrev -= simModel.prevPoseInfo[0]
            sKinPrev -= kinModel.prevPoseInfo[0]
            sSimCurr2D = helpers.rotateBy(sSimCurr.Vector2(), negAngleKin)
            sKinCurr2D = helpers.rotateBy(sKinCurr.Vector2(), negAngleKin)
            sSimVel = (sSimCurr - sSimPrev) / dt
            sKinVel = (sKinCurr - sKinPrev) / dt
            sSimVel2D = helpers.rotateBy(sSimVel.Vector2(), negAngleKin)
            sKinVel2D = helpers.rotateBy(sKinVel.Vector2(), negAngleKin)
            sSim[i] = Vector3(sSimCurr2D.x, sSimCurr2D.y, sSimCurr.z)
            sKin[i] = Vector3(sKinCurr2D.x, sKinCurr2D.y, sKinCurr.z)
            sSim.append(Vector3(sSimVel2D.x, sSimVel2D.y, sSimVel.z))
            sKin.append(Vector3(sKinVel2D.x, sKinVel2D.y, sKinVel.z))
        state = []
        state.extend(vKin.tolist())
        state.extend(vSim.tolist())
        state.extend((vSim - vKin).tolist())
        state.extend(desiredCMVel.tolist())
        state.extend(vDiff.tolist())
        for posVel in sSim: state.extend(posVel.tolist())
        for i in range(len(sSim)): state.extend((sSim[i] - sKin[i]).tolist())
        state.extend(prevAction)

        if (any(abs(x) > 5 for x in state)): return self.prevState
        self.prevState = np.array(state)
        return self.prevState

    # Reset simulated character to the same pose and position as the kinematic character
    def resetSimModelState(self):
        (simModel, kinModel) = self.fetchModels()
        (pos, orn) = self.p.getBasePositionAndOrientation(kinModel.id)
        self.p.resetBasePositionAndOrientation(simModel.id, pos, orn)
        for jointID in simModel.jointIDs.values():
            kinState = self.p.getJointStateMultiDof(kinModel.id, jointID)
            self.p.resetJointStateMultiDof(simModel.id, jointID, kinState[0], kinState[1])

    # Set up the PD controller for the simulated character
    def setupPDController(self):
        (simModel, kinModel) = self.fetchModels()
        self.oneDOFsZ = [simModel.jointIDs["LeftLeg"], simModel.jointIDs["RightLeg"], simModel.jointIDs["LeftToe"], simModel.jointIDs["RightToe"]]
        self.oneDOFsY = [simModel.jointIDs["LeftForeArm"], simModel.jointIDs["RightForeArm"]]
        self.maxForces = [[5000, 5000, 5000]] * len(simModel.jointIDs)
        self.maxForce = 200
        self.kps = [2250] + ([2000] + [3000] + [4000] * 2) * 2 + [2000] * 4 + [3000] + ([2000] + [3000] * 3) * 2
        self.kds = []
        for i in range(len(self.kps)):
            self.kps[i] = self.kps[i] / 1000 * 500
            self.kds.append(self.kps[i] * self.timeStep * 5.5)
        self.p.changeDynamics(simModel.id, -1, lateralFriction=1, spinningFriction=1, rollingFriction=0)
        for jointID in simModel.jointIDs.values():
            self.p.changeDynamics(simModel.id, jointID, lateralFriction=1, spinningFriction=1, rollingFriction=0)
            self.p.resetJointStateMultiDof(simModel.id, jointID, [0,0,0,1], [0,0,0])
            self.p.setJointMotorControlMultiDof(simModel.id, jointID, self.p.POSITION_CONTROL, [0, 0, 0, 1], targetVelocity=[0, 0, 0],
                                                positionGain=0, velocityGain=1, force=[0, 0, 0])

    # Update PD control targets of the simulated model to that of the kinematic model, apply offsets if action is specified
    def updatePDTargetPoints(self, action=None):
        (simModel, kinModel) = self.fetchModels()
        kinPose = kinModel.getAnimator().currPose.copy()
        
        if (type(action) != type(None)): 
            # Fetch kinematic joint positions, update by offsets from action and set them as targets for the PD controller
            self.__applyActionToJoints(kinPose, action)
            
        for jointID in simModel.jointIDs.values():
            self.p.setJointMotorControlMultiDof(simModel.id, jointID, self.p.POSITION_CONTROL, positionGain=self.kps[jointID], targetPosition=kinPose[jointID + 1], force=[self.maxForce])
        
        if (self.visualizeOffset):
            offsetModel = next(c for c in self.characters if (c.tag == CharacterType.NONE))
            (pos, orn) = self.p.getBasePositionAndOrientation(kinModel.id)
            self.p.resetBasePositionAndOrientation(offsetModel.id, pos, orn)
            for jointID in kinModel.jointIDs.values():
                self.p.resetJointStateMultiDof(offsetModel.id, jointID, kinPose[jointID + 1])

    # Update the PD controller's parameters using a common kp and maxforce
    def setPDParams2(self, kps, maxForce):
        self.kps = [kps[0], kps[1], kps[2], kps[3], kps[4], kps[1], kps[2], kps[3], kps[4], kps[5], kps[6], kps[7], kps[8], kps[9], kps[10], kps[11], kps[12], kps[13], kps[10], kps[11], kps[12], kps[13]]
        self.maxForce = maxForce

    # Update the PD controller's parameters, not currently in use
    def setPDParams(self, kpMultiplier, kdMultiplier, maxForce, jointBaseForce):
        (simModel, kinModel) = self.fetchModels()
        self.maxForces = [[maxForce, maxForce, maxForce]] * len(simModel.jointIDs)
        self.kps = [2.25] + ([2] + [3] + [4] * 2) * 2 + [2] * 4 + [3] + ([2] + [3] * 3) * 2
        self.kds = []
        for i in range(len(self.kps)):
            self.kps[i] = self.kps[i] * kpMultiplier
            self.kds.append(self.kps[i] * self.timeStep * kdMultiplier)
        self.p.changeDynamics(simModel.id, -1, lateralFriction=1, spinningFriction=1, rollingFriction=0)
        for jointID in simModel.jointIDs.values():
            self.p.changeDynamics(simModel.id, jointID, lateralFriction=1, spinningFriction=1, rollingFriction=0)
            self.p.resetJointStateMultiDof(simModel.id, jointID, [0,0,0,1], [0,0,0])

    obbDebugLines = []
    
    # Calculate the reward signal used for RL
    def calculateReward(self):
        (simModel, kinModel) = self.fetchModels()
        dt = self.timeStep
        rbCount = len(simModel.rigidBodiesOfInterest)

        # Position+orientation reward, linear+angular velocity reward
        posTotalDist = 0
        velTotalDist = 0
        kinCM = kinModel.poseInfo[0]
        kinPrevCM = kinModel.prevPoseInfo[0]
        simCM = simModel.poseInfo[0]
        simPrevCM = simModel.prevPoseInfo[0]
        for i in range(len(simModel.obbPositions)): # for each OBB...
            rbKin = kinModel.obbPositions[i]
            rbSim = simModel.obbPositions[i]
            rbKinPrev = kinModel.obbPrevPositions[i]
            rbSimPrev = simModel.obbPrevPositions[i]
            for j in range(len(rbSim)): # for each face center...
                p1 = Vector3.fromlist(rbKin[j]) - kinCM
                p2 = Vector3.fromlist(rbSim[j]) - simCM
                p1Prev = Vector3.fromlist(rbKinPrev[j]) - kinPrevCM
                p2Prev = Vector3.fromlist(rbSimPrev[j]) - simPrevCM
                posTotalDist += (p1 - p2).magnitude() # add to total OBB position distance
                velTotalDist += ((p1 - p1Prev) / dt - (p2 - p2Prev) / dt).magnitude() # add to total OBB velocity distance
        rp = math.exp(-10.0 / rbCount * posTotalDist)
        rv = math.exp(-1.0 / rbCount * velTotalDist)

        # local orientation reward
        localTotalDist = 0
        for jointID in simModel.rigidBodiesOfInterest:
            kinJointOrn = self.p.getJointStateMultiDof(kinModel.id, jointID)[0]
            simJointOrn = self.p.getJointStateMultiDof(simModel.id, jointID)[0]
            diff = transform.quaternion_multiply([kinJointOrn[3], kinJointOrn[0], kinJointOrn[1], kinJointOrn[2]], 
                    transform.quaternion_inverse([simJointOrn[3], simJointOrn[0], simJointOrn[1], simJointOrn[2]]))
            localTotalDist += 2.0 * math.atan2(math.sqrt(diff[1]**2 + diff[2]**2 + diff[3]**2), diff[0])
        rlocal = math.exp(-10.0 / rbCount * localTotalDist)

        # center of mass velocity reward
        rvcm = math.exp(-((kinCM - kinPrevCM) / dt - (simCM - simPrevCM) / dt).magnitude())

        # head distance reward multiplier
        headDist = (kinModel.bodyState[3] - simModel.bodyState[3]).magnitude()
        efall = helpers.clamp(1.3 - 1.4 * headDist, 0, 1)

        return (efall * (rp + rv + rlocal + rvcm), headDist > 1 or self.globalTime > 5)
        
    def fetchModels(self):
        simModel = None
        kinModel = None
        for character in self.characters:
            if (character.tag == CharacterType.SIMULATED): simModel = character
            elif (character.tag == CharacterType.KINEMATIC): kinModel = character
        return (simModel, kinModel)

    openLoopJointMapping = [(2, [0,1,2]), (3, [3]), (4, [4,5,6]), (5, [7]), # LeftUpleg, LeftLeg, LeftFoot, LeftToe
                            (6, [8,9,10]), (7, [11]), (8, [12,13,14]), (9, [15]), # right side
                            (10, [16,17,18]), (16, [19,20,21]), (20, [22,23,24])] # Spine, LeftArm, RightArm

    # Given action of orientation-offsets, update the corresponding joints of the given pose
    def __applyActionToJoints(self, pose, action):
        for mapping in self.openLoopJointMapping:
            poseID = mapping[0]
            actionIDs = mapping[1]
            rot = Vector3(action[actionIDs[0]], action[actionIDs[1]], action[actionIDs[2]]) if (len(actionIDs) == 3) else Vector3(action[actionIDs[0]], 0, 0)
            angle = rot.magnitude()
            if (angle != 0):
                rot /= angle # axis
            orn = pose[poseID]
            orn = transform.quaternion_multiply([orn[3], orn[0], orn[1], orn[2]], transform.quaternion_about_axis(angle, rot.tolist()))
            pose[poseID] = [orn[1], orn[2], orn[3], orn[0]]