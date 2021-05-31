import pybullet as p
import pybullet_data
from character_motion.externals.pdControllerExplicit import PDControllerExplicitMultiDof

timestep = 1.0 / 600.0

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=32)
p.setTimeStep(timestep)
p.setGravity(0,0,-10)
kpsIn = p.addUserDebugParameter("kps", 0, 1000, 200)
kdsIn = p.addUserDebugParameter("kds", 0, 100, 50)
maxForcesIn = p.addUserDebugParameter("max_forces", 0, 2000, 200)

pd = PDControllerExplicitMultiDof(p)

box1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5,0.5,0.5], rgbaColor=[0,1,0,1], visualFramePosition=[0,0,0.9])
box2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5,0.5,0.5], rgbaColor=[0,0,1,1], visualFramePosition=[0,0,0.9])
coll1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5,0.5,0.5], collisionFramePosition=[0,0,1])
pole1 = p.createVisualShape(p.GEOM_CYLINDER, length=2, radius=0.2, rgbaColor=[0,1,0,1], visualFramePosition=[0,0,0.9])
pole2 = p.createVisualShape(p.GEOM_CYLINDER, length=2, radius=0.2, rgbaColor=[0,0,1,1], visualFramePosition=[0,0,0.9])
coll2 = p.createCollisionShape(p.GEOM_CYLINDER, height=2, radius=0.2, collisionFramePosition=[0,0,1])

mb1 = p.createMultiBody(5, coll1, box1, [2,0,0], [0,0,0,1],
    linkMasses=[1], linkCollisionShapeIndices=[coll2], linkVisualShapeIndices=[pole1], linkPositions=[[0,0,0.9]], 
    linkOrientations=[[0,0,0,1]], linkParentIndices=[0], linkJointTypes=[p.JOINT_SPHERICAL], linkJointAxis=[[0,0,1]],
    linkInertialFramePositions=[[0,0,0]], linkInertialFrameOrientations=[[0,0,0,1]])
mb2 = p.createMultiBody(5, coll1, box2, [-2,0,0], [0,0,0,1], 
    linkMasses=[1], linkCollisionShapeIndices=[coll2], linkVisualShapeIndices=[pole2], linkPositions=[[0,0,0.9]], 
    linkOrientations=[[0,0,0,1]], linkParentIndices=[0], linkJointTypes=[p.JOINT_SPHERICAL], linkJointAxis=[[0,1,0]],
    linkInertialFramePositions=[[0,0,0]], linkInertialFrameOrientations=[[0,0,0,1]])
p.setCollisionFilterGroupMask(mb1, 0, 0, 0)
p.setCollisionFilterGroupMask(mb2, 0, 0, 0)
p.setCollisionFilterGroupMask(mb1, -1, 1, 1)
p.setCollisionFilterGroupMask(mb2, -1, 1, 1)
p.changeDynamics(mb1, -1, lateralFriction=1, spinningFriction=1, rollingFriction=0)
p.changeDynamics(mb2, -1, lateralFriction=1, spinningFriction=1, rollingFriction=0)
p.setJointMotorControlMultiDof(mb1, 0, p.POSITION_CONTROL, targetPosition=[0,0,0], targetVelocity=[0,0,0], positionGain=0, velocityGain=1, force=[32,32,32])
p.setJointMotorControlMultiDof(mb2, 0, p.POSITION_CONTROL, targetPosition=[0,0,0], targetVelocity=[0,0,0], positionGain=0, velocityGain=1, force=[32,32,32])

p.createConstraint(mb1, -1, -1, -1, p.JOINT_FIXED, [0,0,0], [0,0,0], [2,0,0], [0,0,0,1])
p.createConstraint(mb2, -1, -1, -1, p.JOINT_FIXED, [0,0,0], [0,0,0], [-2,0,0], [0,0,0,1])
p.loadURDF("plane.urdf",[0,0,-1.03])

p.addUserDebugText("Target", [0,0,3], [0,1,0], 1, parentObjectUniqueId=mb1, parentLinkIndex=-1)
p.addUserDebugText("PD Control", [0,0,3], [0,0,1], 1, parentObjectUniqueId=mb2, parentLinkIndex=-1)

desVel = [0] * 11
kps = [0,0,0,0,0,0,0,100,100,100,100]
kds = [0,0,0,0,0,0,0,100,100,100,100]
maxForces = [0,0,0,0,0,0,0,2000,2000,2000,2000]

while (p.isConnected()):
    pos, orn = p.getBasePositionAndOrientation(mb1)
    pose = []
    pose.extend(pos)
    pose.extend(orn)
    pose.extend(p.getJointStateMultiDof(mb1, 0)[0])
    taus3 = pd.computePD(mb2, [0], pose, desVel, [p.readUserDebugParameter(kpsIn)] * 11, 
                         [p.readUserDebugParameter(kdsIn)] * 11, [p.readUserDebugParameter(maxForcesIn)] * 11, timestep)
    p.setJointMotorControlMultiDof(mb2, 0, controlMode=p.TORQUE_CONTROL, force=[taus3[7], taus3[8], taus3[9]])
    p.stepSimulation()