import pybullet as p
from vectors import *
import numpy as np
import math
import helpers

upKey = p.B3G_UP_ARROW
downKey = p.B3G_DOWN_ARROW
leftKey = p.B3G_LEFT_ARROW
rightKey = p.B3G_RIGHT_ARROW
shiftKey = p.B3G_SHIFT
bKey = ord('b')
nKey = ord('n')
mKey = ord('m')
deg2Rad = 0.0174533

def calculateCriticallyDampedSpring(value, velocity, goal, dampingCoefficient, deltaTime):
    acceleration = dampingCoefficient * dampingCoefficient * (value - goal)
    deltaDamping = 1.0 + dampingCoefficient * deltaTime
    newVelocity = (velocity - acceleration * deltaTime) / (deltaDamping * deltaDamping)
    return (value + deltaTime * newVelocity, newVelocity)

# Camera and character movement controller
class Controller():
    def __init__(self, character, automatic = False):
        self.automatic = automatic
        self.debug = False
        self.character = None
        self.lastUpdate = 0 # <-- temporary value for testing purposes
        self.position = Vector2()
        self.desiredForward = Vector2.up()
        self.desiredRight = Vector2.right()
        self.charForward = Vector2.up()
        self.charRight = Vector2.right()
        self.moveForward = False
        self.moveBackwards = False
        self.moveRight = False
        self.moveLeft = False
        self.moving = False
        self.walkSpeed = 0.03
        self.runSpeed = 0.05
        self.movementSpeed = self.walkSpeed
        self.speedDamping = 10.0
        self.rotDamping = 1.0
        
        # Movement control
        self.currRot = 0
        self.currRotVel = 0
        self.currMovementSpeed = 0
        self.currMovementAcc = 0
        self.desiredRot = 0

        # Trajectory
        self.pointCount = 3
        self.framesPerPoint = 20
        self.trajectory = [0, 0, 0, 0] * self.pointCount
        self.conversion = []

        # Debugging variables
        self.debugCamForwardLine = -1
        self.debugCharForwardLine = -1
        self.debugTrajectoryLines = []
        self.debugTrajectoryPoints = []
        self.debugTrajectoryForwards = [] # lines displaying rotation of each trajectory point
        self.pendingCharacterChange = False
        self.pendingBlendChange = False
        self.pendingAnimationChange = False

        # Automatic movement
        self.rng = None

        # RL state info
        self.desiredCMVel = Vector2()

        self.automatic = automatic
        self.character = character
        
        # Setup debug lines and points for trajectory
        if (self.debug):
            self.debugCamForwardLine = p.addUserDebugLine([0,0,0], [0,0,0])
            self.debugCharForwardLine = p.addUserDebugLine([0,0,0], [0,0,0])
            pointShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1,0,0,1])
            for _ in range(self.pointCount):
                self.debugTrajectoryPoints.append(p.createMultiBody(0, baseVisualShapeIndex=pointShape))
                self.debugTrajectoryForwards.append(p.addUserDebugLine([0,0,0], [0,0,0]))
                for _ in range(self.framesPerPoint):
                    self.debugTrajectoryLines.append(p.addUserDebugLine([0,0,0], [0,0,0]))
        self.rng = np.random.default_rng()
    

    def __manageUserInputs(self, env):
        keys = p.getKeyboardEvents()
        if (upKey in keys):
            if (keys[upKey] & p.KEY_WAS_TRIGGERED): self.moveForward = True
            elif (keys[upKey] & p.KEY_WAS_RELEASED): self.moveForward = False
        if (downKey in keys):
            if (keys[downKey] & p.KEY_WAS_TRIGGERED): self.moveBackwards = True
            elif (keys[downKey] & p.KEY_WAS_RELEASED): self.moveBackwards = False
        if (leftKey in keys):
            if (keys[leftKey] & p.KEY_WAS_TRIGGERED): self.moveLeft = True
            elif (keys[leftKey] & p.KEY_WAS_RELEASED): self.moveLeft = False
        if (rightKey in keys):
            if (keys[rightKey] & p.KEY_WAS_TRIGGERED): self.moveRight = True
            elif (keys[rightKey] & p.KEY_WAS_RELEASED): self.moveRight = False
        if (bKey in keys and keys[bKey] & p.KEY_WAS_TRIGGERED): self.pendingBlendChange = True
        if (mKey in keys and keys[mKey] & p.KEY_WAS_TRIGGERED): self.pendingAnimationChange = True
        if (nKey in keys and keys[nKey] & p.KEY_WAS_TRIGGERED): self.pendingCharacterChange = True
        if (shiftKey in keys): 
            if (keys[shiftKey] & p.KEY_WAS_TRIGGERED): self.movementSpeed = self.runSpeed
            elif (keys[shiftKey] & p.KEY_WAS_RELEASED): self.movementSpeed = self.walkSpeed
        self.moving = self.moveForward or self.moveBackwards or self.moveLeft or self.moveRight

    def __updateCamera(self):
        camInfo = p.getDebugVisualizerCamera() # width, height, view, projection, up, forward, horizontal, vertical, yaw, pitch, dist, target
        #p.resetDebugVisualizerCamera(3, camInfo[8], camInfo[9], (self.position.Vector3() + Vector3.up()).tolist()) # update camera to target player's current location
        p.resetDebugVisualizerCamera(3, camInfo[8], camInfo[9], self.character.position.tolist()) # update camera to target player's current location
        forward = Vector3.fromlist(camInfo[5])
        right = forward.cross(Vector3.fromlist(camInfo[4]))
        self.desiredForward = forward.Vector2().normalize()
        self.desiredRight = right.Vector2().normalize()
        self.desiredRot = camInfo[8]
        #if (self.debug):
            #p.addUserDebugLine(self.position, [self.position[0] + self.camForward[0], self.position[1] + self.camForward[1], self.position[2]], [0, 1.0, 0], 2.0, replaceItemUniqueId=self.debugCamForwardLine)
            #p.addUserDebugLine(self.position, [self.position[0] + self.charForward[0], self.position[1] + self.charForward[1], self.position[2]], [1.0, 0, 0], 2.0, replaceItemUniqueId=self.debugCharForwardLine)

    def __move(self, env):
        deltaTime = env.globalTime - self.lastUpdate
        if (self.moving): 
            (newPos, newSpeed, newAcc, newRot, newAngVel, newForward, newRight) = self.__stepOnce(self.position, self.currMovementSpeed, self.currMovementAcc, self.currRot, self.currRotVel, deltaTime)
            self.position = newPos
            self.currMovementSpeed = newSpeed
            self.currMovementAcc = newAcc
            self.currRot = newRot
            self.currRotVel = newAngVel
            self.charForward = newForward
            self.charRight = newRight
            self.conversion = helpers.constructInverseMatrix(newPos.Vector3(), math.atan2(newForward.y, newForward.x))

            prevPos = newPos
            self.trajectory = []
            for i in range(self.pointCount): # calculate trajectory
                idx = i * self.framesPerPoint
                for j in range(self.framesPerPoint):
                    (newPos, newSpeed, newAcc, newRot, newAngVel, newForward, _) = self.__stepOnce(newPos, newSpeed, newAcc, newRot, newAngVel, deltaTime)
                    if (self.debug): p.addUserDebugLine(prevPos.Vector3().tolist(), newPos.Vector3().tolist(), [1.0, 0, 0], 0.2, replaceItemUniqueId=self.debugTrajectoryLines[idx + j])
                    prevPos = newPos
                if (self.debug): 
                    p.resetBasePositionAndOrientation(self.debugTrajectoryPoints[i], newPos.Vector3().tolist(), [0,0,0,1])
                    p.addUserDebugLine(newPos.Vector3().tolist(), (newPos + newForward).Vector3().tolist(), [0, 1.0, 0], 0.2, replaceItemUniqueId=self.debugTrajectoryForwards[i])
                #angle = math.atan2(newForward.y, newForward.x) - math.atan2(self.charForward.y, self.charForward.x)
                #relPos = helpers.rotateBy(newPos - self.position, -self.currRot * deg2Rad - 1.570797)
                self.trajectory.append((newPos.x, newPos.y, newForward.x, newForward.y))
        
        #self.character.setPosition(self.position)
        #self.character.setOrientation(p.getQuaternionFromEuler([0, 0, self.currRot * deg2Rad]))
        #p.resetBasePositionAndOrientation(self.character.id, self.position, p.getQuaternionFromEuler([0.0, -1.57, self.currRot * deg2Rad]))
        self.lastUpdate = env.globalTime

    def update(self, env):
        if (self.automatic): # automatic change to inputs given some probability
            if (self.rng.random() <= 0.05):
                self.takeRandomAction()
        else:
            self.__manageUserInputs(env)
            self.__updateCamera()
        self.__move(env)
        
    def takeRandomAction(self):
        choice = self.rng.integers(0, 3, 1)[0]
        #print("Random: ")
        if (choice == 0): # desired movement direction
            direction = self.rng.integers(0, 8, 1)[0]
            self.moveForward = False
            self.moveBackwards = False
            self.moveLeft = False
            self.moveRight = False
            if (direction == 0): # forward
                #print("Move forward")
                self.moveForward = True
            elif (direction == 1): # backwards
                #print("Move backwards")
                self.moveBackwards = True
            elif (direction == 2): # left
                #print("Move left")
                self.moveLeft = True
            elif (direction == 3): # right
                #print("Move right")
                self.moveRight = True
            elif (direction == 4): # forward left
                #print("Move forward left")
                self.moveForward = True
                self.moveLeft = True
            elif (direction == 5): # forward right
                #print("Move forward right")
                self.moveForward = True
                self.moveRight = True
            elif (direction == 6): # backwards left
                #print("Move backwards left")
                self.moveBackwards = True
                self.moveLeft = True
            elif (direction == 7): # backwards right
                #print("Move backwards right")
                self.moveBackwards = True
                self.moveRight = True
        elif (choice == 1): # desired facing direction
            self.desiredRot = self.rng.random() * 2.0 * math.pi
            #print("face angle " + str(self.desiredRot))
        elif (choice == 2): # movement style: walk, run
            self.movementSpeed = self.walkSpeed if (self.rng.random() > 0.5) else self.runSpeed
            #print("Movement speed " + str(self.movementSpeed))
        self.moving = self.moveForward or self.moveBackwards or self.moveLeft or self.moveRight
        self.desiredForward = Vector2(math.cos(self.desiredRot + 1.570797), math.sin(self.desiredRot + 1.570797))
        self.desiredRight = Vector2(math.cos(self.desiredRot), math.sin(self.desiredRot))
        direction = Vector2()
        if (self.moveForward): direction += self.desiredForward
        elif (self.moveBackwards): direction -= self.desiredForward
        if (self.moveRight): direction += self.desiredRight
        elif (self.moveLeft): direction -= self.desiredRight
        direction = direction.normalize()
        direction *= self.movementSpeed
        self.desiredCMVel = direction

    def __stepOnce(self, position, speed, acceleration, rotation, angularVelocity, deltaTime):
        (currMovementSpeed, currMovementAcc) = calculateCriticallyDampedSpring(speed, acceleration, self.movementSpeed, self.speedDamping, deltaTime)
        (currRot, currAngularVel) = calculateCriticallyDampedSpring(rotation, angularVelocity, self.desiredRot, self.rotDamping, deltaTime)
        rotInRad = currRot * deg2Rad
        forward = Vector2(math.cos(rotInRad + 1.570797), math.sin(rotInRad + 1.570797))
        right = Vector2(math.cos(rotInRad), math.sin(rotInRad))
        direction = Vector2()
        if (self.moveForward): direction += forward
        elif (self.moveBackwards): direction -= forward
        if (self.moveRight): direction += right
        elif (self.moveLeft): direction -= right
        direction = direction.normalize()
        newPosition = position + direction * currMovementSpeed
        return (newPosition, currMovementSpeed, currMovementAcc, currRot, currAngularVel, forward, right)