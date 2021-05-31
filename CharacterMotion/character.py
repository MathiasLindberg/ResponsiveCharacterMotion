from enum import Enum
import pybullet as p
import os
import helpers
from vectors import *
import transformations as transform
import numpy as np

characterPath = os.path.dirname(os.path.realpath(__file__)) + "/externals/character_updated.urdf"

class CharacterType(Enum):
    NONE = 0
    KINEMATIC = 1
    SIMULATED = 2
    
class Character():
    jointIDs = {"Hips":0,"LeftUpLeg":1,"LeftLeg":2,"LeftFoot":3,"LeftToe":4,"RightUpLeg":5,"RightLeg":6,"RightFoot":7,"RightToe":8,"Spine":9,"Spine1":10,"Spine2":11,"Neck":12,"Head":13,"LeftShoulder":14,"LeftArm":15,"LeftForeArm":16,"LeftHand":17,"RightShoulder":18,"RightArm":19,"RightForeArm":20,"RightHand":21}

    def __init__(self, controlForce, color):
        self.position = Vector3()
        self.orientation = [0,0,0,1]
        self.positionOffset = Vector3()
        self.forward = Vector2()
        self.zRotation = 0
        self.poseInfo = [Vector3()] * 6
        self.prevPoseInfo = [Vector3()] * 6
        self.bodyState = [Vector3()] * 6
        self.prevBodyState = [Vector3()] * 6
        self.conversion = []
        self.prevConversion = []
        self.prevTime = -0.1
        self.currTime = 0
        self.tag = CharacterType.KINEMATIC
        self.animator = None
        self.obbs = []
        self.obbLines = []
        self.obbPoints = []
        self.obbPositions = []
        self.obbPrevPositions = []
        self.id = p.loadURDF(characterPath, globalScaling=1, useFixedBase=False, flags=p.URDF_MAINTAIN_LINK_ORDER)
        self.controlForce = controlForce
        self.debug = False
        self.color = color[0:3]
        for jointID in self.jointIDs.values():
            p.setJointMotorControlMultiDof(self.id, jointID, p.POSITION_CONTROL, [0,0,0,1], [0,0,0], [32, 32, 32], 0, 0)
        for i in range(p.getNumJoints(self.id) + 1):
            p.changeVisualShape(self.id, i - 1, rgbaColor=color)
        self.setupOBBs()
        self.jointWeights = []
        ids = list(self.jointIDs.values())
        totalMass = 0
        masses = []
        for i in range(len(ids)):
            mass = p.getDynamicsInfo(self.id, ids[i])[0]
            masses.append(mass)
            totalMass += mass
        baseMass = p.getDynamicsInfo(self.id, -1)[0]
        totalMass += baseMass
        for m in masses:
            self.jointWeights.append(m / totalMass)
        self.baseWeight = baseMass / totalMass

        self.maxForce = 1000
        self.lines = []

        self.conversion = helpers.constructInverseMatrix(Vector3(), 0)
        self.prevConversion = self.conversion

        if (self.debug):
            for k in self.jointIDs:
                p.addUserDebugText(k, [0,0,0], [1,0,0], 0.8, parentObjectUniqueId=self.id, parentLinkIndex=self.jointIDs[k])
            for i in range(1, p.getNumJoints(self.id)):
                info = p.getJointInfo(self.id, i)
                pos = p.getLinkState(self.id, i)[0]
                parentPos = p.getLinkState(self.id, info[16])[0]
                self.lines.append(p.addUserDebugLine(pos, parentPos, [0,1,0], 2.0))
                
    def stepCharacter(self, globalTime):
        if (self.debug):
            for i in range(1, p.getNumJoints(self.id)):
                info = p.getJointInfo(self.id, i)
                pos = p.getLinkState(self.id, i)[0]
                parentPos = p.getLinkState(self.id, info[16])[0]
                p.addUserDebugLine(pos, parentPos, [0,1,0], 2.0, replaceItemUniqueId=self.lines[i - 1])
        self.prevPoseInfo = self.poseInfo
        self.prevBodyState = self.bodyState
        self.prevConversion = self.conversion
        self.prevTime = self.currTime
        self.currTime = globalTime
        self.poseInfo = self.__calculatePoseInfo()
        self.bodyState = self.__calculateBodyState()
        self.forward = helpers.calculateForward(self.poseInfo)
        self.conversion = helpers.constructInverseMatrix(self.poseInfo[0], math.atan2(self.forward.y, self.forward.x))
        if (self.animator != None): self.animator.stepAnimation(globalTime)
        if (self.tag == CharacterType.KINEMATIC):
            p.resetJointStateMultiDof(self.id, self.jointIDs["Hips"], self.orientation)
            p.resetBasePositionAndOrientation(self.id, (self.position + self.positionOffset).tolist(), p.getQuaternionFromEuler([0,0,self.zRotation]))
        elif (self.tag == CharacterType.SIMULATED):
            0
        self.updateOBBs(False)

    def setAnimator(self, animator):
        self.animator = animator

    def getAnimator(self):
        return self.animator
    
    def __calculatePoseInfo(self):
        hipsPos = p.getLinkState(self.id, self.jointIDs["Hips"])[0]
        leftFootPos = p.getLinkState(self.id, self.jointIDs["LeftFoot"])[0]
        rightFootPos = p.getLinkState(self.id, self.jointIDs["RightFoot"])[0]
        rightShoulderPos = p.getLinkState(self.id, self.jointIDs["RightShoulder"])[0]
        leftShoulderPos = p.getLinkState(self.id, self.jointIDs["LeftShoulder"])[0]
        centerOfMass = self.getCenterOfMass()
        return [centerOfMass, 
                Vector3(hipsPos[0], hipsPos[1], hipsPos[2]),
                Vector3(leftFootPos[0], leftFootPos[1], leftFootPos[2]), 
                Vector3(rightFootPos[0], rightFootPos[1], rightFootPos[2]),
                Vector3(leftShoulderPos[0], leftShoulderPos[1], leftShoulderPos[2]),
                Vector3(rightShoulderPos[0], rightShoulderPos[1], rightShoulderPos[2])]

    def __calculateBodyState(self):
        leftToePos = p.getLinkState(self.id, self.jointIDs["LeftToe"])[0]
        rightToePos = p.getLinkState(self.id, self.jointIDs["RightToe"])[0]
        spinePos = p.getLinkState(self.id, self.jointIDs["Spine"])[0]
        headPos = p.getLinkState(self.id, self.jointIDs["Head"])[0]
        leftForeArmPos = p.getLinkState(self.id, self.jointIDs["LeftForeArm"])[0]
        rightForeArmPos = p.getLinkState(self.id, self.jointIDs["RightForeArm"])[0]
        return [Vector3(leftToePos[0], leftToePos[1], leftToePos[2]), 
                Vector3(rightToePos[0], rightToePos[1], rightToePos[2]),
                Vector3(spinePos[0], spinePos[1], spinePos[2]), 
                Vector3(headPos[0], headPos[1], headPos[2]),
                Vector3(leftForeArmPos[0], leftForeArmPos[1], leftForeArmPos[2]), 
                Vector3(rightForeArmPos[0], rightForeArmPos[1], rightForeArmPos[2])]

    # Returns the center of mass of the kinematic model
    def getCenterOfMass(self):
        ids = list(self.jointIDs.values())
        states = p.getLinkStates(self.id, ids)
        centerOfMass = Vector3()
        for i in range(len(states)): # TODO: Consider combining calculatePoseInfo with getCenterOfMass
            linkCM = states[i][0]
            linkCM = Vector3(linkCM[0], linkCM[1], linkCM[2])
            w = self.jointWeights[i]
            centerOfMass += linkCM * w
        (baseCM, _) = p.getBasePositionAndOrientation(self.id)
        baseCM = Vector3(baseCM[0], baseCM[1], baseCM[2])
        centerOfMass += baseCM * self.baseWeight
        return centerOfMass

    def setPosition(self, position : Vector3):
        self.position = position
    def setOrientation(self, quaternion):
        self.orientation = quaternion
    def setPositionOffset(self, position : Vector3):
        self.positionOffset = position

    #"Hips":0,"LeftUpLeg":1,"LeftLeg":2,"LeftFoot":3,"LeftToe":4,"RightUpLeg":5,"RightLeg":6,"RightFoot":7,"RightToe":8,"Spine":9,"Spine1":10,"Spine2":11,"Neck":12,"Head":13,"LeftShoulder":14,"LeftArm":15,"LeftForeArm":16,"LeftHand":17,"RightShoulder":18,"RightArm":19,"RightForeArm":20,"RightHand":21
    rigidBodiesOfInterest = [jointIDs["LeftLeg"], jointIDs["RightLeg"], jointIDs["LeftUpLeg"], jointIDs["RightUpLeg"], jointIDs["Spine"], jointIDs["Head"], jointIDs["LeftForeArm"], jointIDs["RightForeArm"]]
    def setupOBBs(self):
        self.obbs.clear()
        shapes = p.getVisualShapeData(self.id)
        for shape in shapes:
            linkID = shape[1]
            if (linkID in self.rigidBodiesOfInterest):
                geomType = shape[2]
                dimensions = shape[3]
                if (geomType == 2): # GEOM_SPHERE
                    self.obbs.append({"bounds":helpers.sphereBounds(dimensions[0]), "id":linkID})
                elif (geomType == 3): # GEOM_BOX
                    self.obbs.append({"bounds":helpers.boxBounds(dimensions[0], dimensions[1], dimensions[2]), "id":linkID})
                elif (geomType == 7): # GEOM_CAPSULE
                    self.obbs.append({"bounds":helpers.capsuleBounds(dimensions[0], dimensions[1]), "id":linkID})
            self.obbPositions.append(np.array([[0, 0, 0]] * 6))
        self.updateOBBs(False, True)
    def updateOBBs(self, draw=True, setup=False):
        self.obbPrevPositions = self.obbPositions.copy()
        i = 0
        for obb in self.obbs:
            linkState = p.getLinkState(self.id, obb["id"])  
            low = obb["bounds"][0]
            high = obb["bounds"][1]
            rot = linkState[1]
            r = transform.quaternion_matrix([rot[3], rot[0], rot[1], rot[2]])
            t = transform.translation_matrix(linkState[0])
            conversion = transform.concatenate_matrices(t, r)
            p1 = conversion.dot([low.x, low.y, low.z, 1])
            p2 = conversion.dot([high.x, low.y, low.z, 1])
            p3 = conversion.dot([high.x, high.y, low.z, 1])
            p4 = conversion.dot([low.x, high.y, low.z, 1])
            p5 = conversion.dot([low.x, low.y, high.z, 1])
            p6 = conversion.dot([high.x, low.y, high.z, 1])
            p7 = conversion.dot([high.x, high.y, high.z, 1])
            p8 = conversion.dot([low.x, high.y, high.z, 1])
            self.obbPositions[i] = np.array([[(p1[0] + p2[0] + p3[0] + p4[0]) * 0.25, (p1[1] + p2[1] + p3[1] + p4[1]) * 0.25, (p1[2] + p2[2] + p3[2] + p4[2]) * 0.25],
                                             [(p5[0] + p6[0] + p7[0] + p8[0]) * 0.25, (p5[1] + p6[1] + p7[1] + p8[1]) * 0.25, (p5[2] + p6[2] + p7[2] + p8[2]) * 0.25],
                                             [(p1[0] + p2[0] + p5[0] + p6[0]) * 0.25, (p1[1] + p2[1] + p5[1] + p6[1]) * 0.25, (p1[2] + p2[2] + p5[2] + p6[2]) * 0.25],
                                             [(p2[0] + p3[0] + p6[0] + p7[0]) * 0.25, (p2[1] + p3[1] + p6[1] + p4[1]) * 0.25, (p2[2] + p3[2] + p6[2] + p7[2]) * 0.25],
                                             [(p3[0] + p4[0] + p7[0] + p8[0]) * 0.25, (p3[1] + p4[1] + p7[1] + p8[1]) * 0.25, (p3[2] + p4[2] + p7[2] + p8[2]) * 0.25],
                                             [(p1[0] + p4[0] + p5[0] + p8[0]) * 0.25, (p1[1] + p4[1] + p5[1] + p8[1]) * 0.25, (p1[2] + p4[2] + p5[2] + p8[2]) * 0.25]])
            if (draw): self.drawOBBOutline(p1[:3], p2[:3], p3[:3], p4[:3], p5[:3], p6[:3], p7[:3], p8[:3], -1 if setup else i)
            i += 1
    def drawOBBOutline(self, p1, p2, p3, p4, p5, p6, p7, p8, boxID = -1):
        if (boxID == -1):
            lines = []
            lines.append(p.addUserDebugLine(p1, p2, self.color, 2))
            lines.append(p.addUserDebugLine(p2, p3, self.color, 2))
            lines.append(p.addUserDebugLine(p3, p4, self.color, 2))
            lines.append(p.addUserDebugLine(p4, p1, self.color, 2))
            lines.append(p.addUserDebugLine(p5, p6, self.color, 2))
            lines.append(p.addUserDebugLine(p6, p7, self.color, 2))
            lines.append(p.addUserDebugLine(p7, p8, self.color, 2))
            lines.append(p.addUserDebugLine(p8, p5, self.color, 2))
            lines.append(p.addUserDebugLine(p1, p5, self.color, 2))
            lines.append(p.addUserDebugLine(p2, p6, self.color, 2))
            lines.append(p.addUserDebugLine(p3, p7, self.color, 2))
            lines.append(p.addUserDebugLine(p4, p8, self.color, 2))
            self.obbLines.append(lines)
            points = []
            pointShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.01, rgbaColor=[1,0,0,1])
            for _ in range(6): points.append(p.createMultiBody(0, baseVisualShapeIndex=pointShape))
            self.obbPoints.append(points)
        else:
            p.addUserDebugLine(p1, p2, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][0])
            p.addUserDebugLine(p2, p3, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][1])
            p.addUserDebugLine(p3, p4, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][2])
            p.addUserDebugLine(p4, p1, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][3])
            p.addUserDebugLine(p5, p6, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][4])
            p.addUserDebugLine(p6, p7, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][5])
            p.addUserDebugLine(p7, p8, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][6])
            p.addUserDebugLine(p8, p5, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][7])
            p.addUserDebugLine(p1, p5, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][8])
            p.addUserDebugLine(p2, p6, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][9])
            p.addUserDebugLine(p3, p7, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][10])
            p.addUserDebugLine(p4, p8, self.color, 2, replaceItemUniqueId=self.obbLines[boxID][11])
            p.resetBasePositionAndOrientation(self.obbPoints[boxID][0], self.obbPositions[boxID][0], [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(self.obbPoints[boxID][1], self.obbPositions[boxID][1], [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(self.obbPoints[boxID][2], self.obbPositions[boxID][2], [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(self.obbPoints[boxID][3], self.obbPositions[boxID][3], [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(self.obbPoints[boxID][4], self.obbPositions[boxID][4], [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(self.obbPoints[boxID][5], self.obbPositions[boxID][5], [0, 0, 0, 1])
    """first = True
    headPoint = -1
    headLine = -1
    def test(self):
        if (self.first):
            shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.1, rgbaColor=[1,1,0,1])
            self.headPoint = p.createMultiBody(0, baseVisualShapeIndex=shape)
            self.headLine = p.addUserDebugLine([0,0,0], [0,0,0])
            p.setJointMotorControl2(self.id, self.jointIDs["Head"], p.VELOCITY_CONTROL, targetVelocity=1, force=10)
            self.first = False
        vals = p.getLinkState(self.id, self.jointIDs["Head"], computeLinkVelocity=True)
        pos = vals[4]
        vel = vals[6]
        print(vel)
        p.resetBasePositionAndOrientation(self.headPoint, pos, [0,0,0,1])
        p.addUserDebugLine(pos, [pos[0] + vel[0], pos[1] + vel[1], pos[2] + vel[2]], [1,1,0], 5, replaceItemUniqueId=self.headLine)"""