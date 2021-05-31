import gym
from gym import error, spaces, utils
from numpy.core.numeric import Inf
import pybullet
import numpy as np
from pybullet_utils import bullet_client
import pybullet_data
import transformations as transform
import math

class BasicEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    timestep = 1.0 / 60.0
    startPos = [0,0,0.25]
    vel = [0,0,0]
    stepNum = 0
    maxSteps = 100
    actionLine = -1
    forceLine = -1

    def __init__(self, trainMode):
        self.action_space = spaces.Box(-Inf, Inf, (3,))
        self.observation_space = spaces.Box(-Inf, Inf, (4,))
        if (trainMode): self.p = bullet_client.BulletClient()
        else: self.p = bullet_client.BulletClient(connection_mode=pybullet.SHARED_MEMORY)
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.p.setPhysicsEngineParameter(numSolverIterations=32, fixedTimeStep=self.timestep)
        self.p.setGravity(0, 0, -10)
        self.p.loadURDF("plane.urdf")
        sphereShape = self.p.createVisualShape(self.p.GEOM_SPHERE, radius=0.5, visualFramePosition=[0,0,1.5], rgbaColor=[0.8, 0.2, 0.2, 1])
        sphereCollider = self.p.createCollisionShape(self.p.GEOM_SPHERE, radius=0.5, collisionFramePosition=[0,0,1.5])
        boxShape = self.p.createVisualShape(self.p.GEOM_BOX, halfExtents=[1, 0.5, 0.5], rgbaColor=[0.12, 0.62, 0.62, 1])
        boxCollider = self.p.createCollisionShape(self.p.GEOM_BOX, halfExtents=[1, 0.5, 0.5])
        poleShape = self.p.createVisualShape(self.p.GEOM_BOX, halfExtents=[0.1, 0.2, 1.5], visualFramePosition=[0,0,1.5], rgbaColor=[0.45, 0.45, 0.78, 1])
        poleCollider = self.p.createCollisionShape(self.p.GEOM_BOX, halfExtents=[0.1, 0.2, 1.5], collisionFramePosition=[0,0,1.5])
        self.robot = self.p.createMultiBody(100, boxCollider, boxShape, self.startPos, [0,0,0,1], [0,0,0], [0,0,0,1],
                                            linkMasses=[20, 20], linkCollisionShapeIndices=[poleCollider, sphereCollider], linkVisualShapeIndices=[poleShape, sphereShape],
                                            linkPositions=[[0,0,0], [0,0,1.5]], linkOrientations=[[0,0,0,1], [0,0,0,1]], linkInertialFramePositions=[[0,0,0], [0,0,0]],
                                            linkInertialFrameOrientations=[[0,0,0,1], [0,0,0,1]], linkParentIndices=[0,1], 
                                            linkJointTypes=[self.p.JOINT_SPHERICAL, self.p.JOINT_FIXED], linkJointAxis=[[0,1,0], [0,0,1]])
        self.p.resetJointStateMultiDof(self.robot, 0, [0,0,0,1], [0,0,0])
        self.p.setJointMotorControlMultiDof(self.robot, 0, self.p.POSITION_CONTROL, [0,0,0,1], targetVelocity=[0,0,0], positionGain=0, velocityGain=1, force=[0,0,0])
        self.actionLine = self.p.addUserDebugLine([0,0,0],[0,0,0])
        self.forceLine = self.p.addUserDebugLine([0,0,0],[0,0,0])

        self.kp = self.p.addUserDebugParameter("kp", 0, 5000, 1000)
        self.kd = self.p.addUserDebugParameter("kd", 0, 20, 1)

    def step(self, action):
        self.stepNum += 1
        base = self.p.getBasePositionAndOrientation(self.robot)
        basePos = base[0]
        baseOrn = base[1]
        self.p.resetBasePositionAndOrientation(self.robot, [basePos[0] + self.vel[0] * self.timestep, 
                                                            basePos[1] + self.vel[1] * self.timestep, 
                                                            basePos[2] + self.vel[2] * self.timestep], baseOrn)   
        mag = math.sqrt(action[0] * action[0] + action[1] * action[1] + action[2] * action[2])
        ornOffset = [1,0,0,0] if (mag == 0) else transform.quaternion_about_axis(mag, [action[0] / mag, action[1] / mag, action[2] / mag])
        ornOffset = [ornOffset[1], ornOffset[2], ornOffset[3], ornOffset[0]]

        desPos = []
        desPos.extend(basePos)
        desPos.extend(baseOrn)
        desPos.extend(ornOffset)
        desPos.append(1)
        kp = self.p.readUserDebugParameter(self.kp)
        kd = self.timestep * kp * self.p.readUserDebugParameter(self.kd)
        self.p.setJointMotorControlMultiDofArray(self.robot, [0], self.p.STABLE_PD_CONTROL, targetPositions=[[0,0,0,1]],
                                                 positionGains=[kp], velocityGains=[kd], forces=[[2000,2000,2000]])
        state = [self.vel[0]]
        state.extend(self.p.getEulerFromQuaternion(self.p.getJointStateMultiDof(self.robot, 0)[0]))
        state = np.array(state)
    
        spherePos = self.p.getLinkState(self.robot, 1)[0]
        reward = np.exp(spherePos[2])

        self.p.addUserDebugLine(spherePos,[spherePos[0] + action[0], spherePos[1] + action[1], spherePos[2] + action[2]], [1,1,0], 10, replaceItemUniqueId=self.actionLine)
            
        done = spherePos[2] < 0.7 or self.stepNum > self.maxSteps

        info = {}

        self.p.stepSimulation()

        return state, reward, done, info

    def reset(self):
        self.p.resetBasePositionAndOrientation(self.robot, self.startPos, [0,0,0,1])
        self.p.resetJointStateMultiDof(self.robot, 0, self.p.getQuaternionFromEuler([np.random.rand() - 0.5, np.random.rand() - 0.5, np.random.rand() - 0.5]))
        state = [self.vel[0]]
        state.extend(self.p.getEulerFromQuaternion(self.p.getJointStateMultiDof(self.robot, 0)[0]))
        state = np.array(state)
        velX = np.random.rand() * 50 - 25
        if (velX < 5 and velX > -5): velX = 5 if velX > 0 else -5
        self.vel[0] = velX
        self.stepNum = 0
        return state
  
    def render(self, mode='human'):
        pass

    def close(self):
        pass
