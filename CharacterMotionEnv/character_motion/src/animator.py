import pybullet
from enum import Enum
from character_motion.src.animation import *
from character_motion.src.vectors import *
import character_motion.src.helpers as helpers
from character_motion.src.character import Character
import transformations as transform
import math
import numpy as np
from character_motion.src.helpers import clamp, lerpVectors

class AnimationType(Enum):
    KINEMATIC = 0
    PD_EXPLICIT = 1
class MotionType(Enum):
    NONE = 0
    ROOT = 1
    DISPLACEMENT = 2

class Animator():
    def __init__(self, animationType : AnimationType, character : Character, p=pybullet):
        self.p = p
        self.playbackRate = 1
        self.looping = True
        self.blendTime = 0.3
        self.animations = []
        self.startTime = 0
        self.targetAnimationID = -1
        self.playing = False
        self.motion = MotionType.NONE
        self.animDeltaTime = 0
        self.__prevTime = 0
        # Blending control variables
        self.__blending = False
        self.currPose = []
        self.prevPose = []
        self.__blendPolynomials = []
        # Motion matching variables
        self.offset = 0
        self.desiredAngleGain = 0
        self.animationElapsedTime = 0.0
        self.animationType = animationType        
        self.character = character

    def playAnimation(self, startTime, name, localOffset = 0):
        self.startTime = startTime
        self.targetAnimationID = -1
        self.playing = False
        self.__blending = False
        self.offset = localOffset
        for i in range(len(self.animations)):
            if (self.animations[i].name == name):
                self.targetAnimationID = i
                self.playing = True
                self.__prevTime = 0
                break

    def getAnimation(self):
        if (self.targetAnimationID == -1): return None
        return self.animations[self.targetAnimationID]

    def stop(self, character):
        self.playing = False
        self.targetAnimationID = -1
        character.resetJointDOFs()
        self.__blending = False
        
    def playNextAnimation(self, startTime):
        self.startTime = startTime
        self.targetAnimationID += 1
        self.playing = True
        if (self.targetAnimationID >= len(self.animations)):
            if (len(self.animations) == 0):
                self.targetAnimationID = -1
                self.playing = False
            else: self.targetAnimationID = 0

    def blendNextAnimation(self, startTime):
        self.startTime = startTime
        self.targetAnimationID += 1
        self.playing = True
        if (self.targetAnimationID >= len(self.animations)):
            if (len(self.animations) == 0):
                self.targetAnimationID = -1
                self.playing = False
            else: self.targetAnimationID = 0
        # Consider edge-case when the animation has run for less than 2 frames
        self.startTime = 0
        self.__blending = False
        if (len(self.prevPose) == 0): # step twice to generate reference poses for inertialization blending
            self.stepAnimation(0)
            self.stepAnimation(0.01)
        self.startTime = startTime
        self.__blending = True
        self.__calculateInertia() # calculate the 5th degree polynomial function variables for each value of each joint

    def blendAnimation(self, startTime, name, localOffset = 0):
        self.targetAnimationID = -1
        self.playing = False
        self.offset = localOffset
        for i in range(len(self.animations)):
            if (self.animations[i].name == name):
                self.targetAnimationID = i
                self.playing = True
                self.__prevTime = 0
                break
        if (not self.playing): return # could not find animation
        # Consider edge-case when the animation has run for less than 2 frames
        if (len(self.prevPose) == 0): # step twice to generate reference poses for inertialization blending
            self.startTime = 0
            self.__blending = False
            self.stepAnimation(0)
            self.stepAnimation(0.01)
        self.prevTime = startTime
        self.startTime = startTime
        self.__blending = True
        self.__calculateInertia()

    def stepAnimation(self, globalTime):
        if (not self.playing): return
        self.animationElapsedTime = self.toLocalTime(globalTime)
        t = self.animationElapsedTime + self.offset # offset local time to support playing from fixed time
        if (self.getAnimation().duration < t): # wrap time and blend towards beginning of current animation
            self.blendAnimation(globalTime, self.getAnimation().name)
            return
        pose = self.getPose(t)
        if (self.__blending): # blend from previous pose towards current pose
            for i in range(len(pose)):
                (polynomials, axes) = self.__blendPolynomials[i]
                val = self.inertialize(polynomials, self.animationElapsedTime)
                if (type(pose[i]) == tuple): # quaternion
                    s = math.sin(val / 2.0)
                    _, pose[i] = self.p.multiplyTransforms([0,0,0], [axes[0] * s, axes[1] * s, axes[2] * s, math.cos(val / 2.0)], [0,0,0], pose[i])
                else: # position vector
                    pose[i] = axes * val + pose[i]
        
        self.prevPose = self.currPose
        self.currPose = pose
        self.animDeltaTime = self.animationElapsedTime - self.__prevTime
        self.__prevTime = self.animationElapsedTime

        if (self.motion == MotionType.ROOT): # Animation driven, root motion
            self.character.setPosition(pose[0])
            self.character.setOrientation(pose[1])
        elif (self.motion == MotionType.DISPLACEMENT): # Animation driven used for motion matching
            dt = (self.character.currTime - self.character.prevTime)
            charAngle = math.atan2(self.character.forward.y, self.character.forward.x)
            posOffset = helpers.rotateBy(pose[0], charAngle).Vector3() # hip offset
            posOffset.z = pose[0].z
            self.character.setPositionOffset(posOffset)
            animation = self.getAnimation()
            currFrame = int(t / animation.frameTime)
            nextFrame = min(currFrame + 5, animation.frameCount - 1)
            conversion = self.getWorldToCharMatrix(currFrame)
            deltaPos = animation.data["Frames"][nextFrame][0:3]
            deltaPos = Vector3.fromlist(conversion.dot([deltaPos[0], deltaPos[1], deltaPos[2], 1]))
            deltaPos = helpers.rotateBy(deltaPos.Vector2(), charAngle).Vector3()
            self.character.setPosition(self.character.position + deltaPos / animation.frameTime * dt * 0.2)
            currZRot = transform.quaternion_inverse(transform.quaternion_from_matrix(self.getWorldToCharMatrix(currFrame)))
            nextZRot = transform.quaternion_from_matrix(self.getWorldToCharMatrix(nextFrame))
            rotDiff = transform.euler_from_quaternion(transform.quaternion_multiply(nextZRot, currZRot))[2] * 0.2
            self.character.zRotation -= (rotDiff / animation.frameTime + self.desiredAngleGain) * dt
            self.character.setOrientation(pose[1])
        else:
            self.character.setOrientation(pose[1])
            
            
        if (self.animationType == AnimationType.KINEMATIC):
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["LeftUpLeg"], pose[2])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["LeftLeg"], pose[3])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["LeftFoot"], pose[4])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["LeftToe"], pose[5])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["RightUpLeg"], pose[6])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["RightLeg"], pose[7])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["RightFoot"], pose[8])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["RightToe"], pose[9])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["Spine"], pose[10])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["Spine1"], pose[11])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["Spine2"], pose[12])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["Neck"], pose[13])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["Head"], pose[14])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["LeftShoulder"], pose[15])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["LeftArm"], pose[16])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["LeftForeArm"], pose[17])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["LeftHand"], pose[18])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["RightShoulder"], pose[19])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["RightArm"], pose[20])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["RightForeArm"], pose[21])
            self.p.resetJointStateMultiDof(self.character.id, self.character.jointIDs["RightHand"], pose[22])

    def getPose(self, localTime):
        animation = self.animations[self.targetAnimationID]
        frame = localTime / animation.frameTime
        currFrame = int(frame)
        nextFrame = currFrame + 1
        if (nextFrame >= animation.frameCount):
            nextFrame = currFrame
        frameFrac = frame - currFrame
        frameData = animation.data["Frames"][currFrame]
        nextFrameData = animation.data["Frames"][nextFrame]
        framePos = [frameData[0], frameData[1], frameData[2]]
        nextFramePos = [nextFrameData[0], nextFrameData[1], nextFrameData[2]]
        frameRot = [frameData[3], frameData[4], frameData[5], frameData[6]]
        nextFrameRot = [nextFrameData[3], nextFrameData[4], nextFrameData[5], nextFrameData[6]]
        if (self.motion == MotionType.DISPLACEMENT): # only keep hip displacement
            currConversion = self.getWorldToCharMatrix(currFrame)
            nextConversion = self.getWorldToCharMatrix(nextFrame)
            framePos = currConversion.dot([framePos[0], framePos[1], framePos[2], 1])
            nextFramePos = nextConversion.dot([nextFramePos[0], nextFramePos[1], nextFramePos[2], 1])
            frameRot = transform.quaternion_multiply(transform.quaternion_from_matrix(currConversion), 
                                                     [frameRot[3], frameRot[0], frameRot[1], frameRot[2], frameRot[3]])
            nextFrameRot = transform.quaternion_multiply(transform.quaternion_from_matrix(nextConversion), 
                                                         [nextFrameRot[3], nextFrameRot[0], nextFrameRot[1], nextFrameRot[2], nextFrameRot[3]])
            frameRot = [frameRot[1], frameRot[2], frameRot[3], frameRot[0]]
            nextFrameRot = [nextFrameRot[1], nextFrameRot[2], nextFrameRot[3], nextFrameRot[0]]
            
        pose = [lerpVectors(Vector3(framePos[0], framePos[1], framePos[2]), Vector3(nextFramePos[0], nextFramePos[1], nextFramePos[2]), frameFrac),
                self.p.getQuaternionSlerp(frameRot, nextFrameRot, frameFrac)]
        for i in range(7, len(frameData), 4):
            pose.append(self.p.getQuaternionSlerp(frameData[i:(i + 4)], nextFrameData[i:(i + 4)], frameFrac))

        return pose

    # Call before first blend step, but after setting target animation and time
    def __calculateInertia(self):
        self.__blendPolynomials.clear()
        targetPose = self.getPose(self.offset)
        for i in range(len(self.currPose)):
            # Calculate differences between current pose and target pose, as well as previous pose and target pose
            currJoint = self.currPose[i]
            prevJoint = self.prevPose[i]
            targetJoint = targetPose[i]
            polynomials = []
            jointAxis = None # stores axis of the current joint angle or position, needed for later quaternion and vector reconstruction

            if (type(currJoint) == tuple): # check if the joint rotation is a quaternion
                jointDiff = self.p.getDifferenceQuaternion(targetJoint, currJoint)
                prevJointDiff = self.p.getDifferenceQuaternion(targetJoint, prevJoint)
                # Get axis and angle of jointDiff and angle of prevJointDiff along the same axis
                (jointAxis, magnitude) = self.p.getAxisAngleFromQuaternion(jointDiff)
                prevMagnitude = 2.0 * math.atan((prevJointDiff[0] * jointAxis[0] + prevJointDiff[1] * jointAxis[1] + prevJointDiff[2] * jointAxis[2]) / prevJointDiff[3])
            else: # otherwise it must be a position vector
                jointDiff = currJoint - targetJoint
                prevJointDiff = prevJoint - targetJoint
                magnitude = jointDiff.magnitude()
                jointAxis = jointDiff / magnitude
                prevMagnitude = prevJoint.dot(jointAxis)

            # Calculate inertia of magnitude
            velocity = 0 if (self.animDeltaTime == 0) else (magnitude - prevMagnitude) / self.animDeltaTime
            negate = magnitude < 0
            if (negate): magnitude = -magnitude # this method only works with positive values, negate input and output if value is negative
            if (velocity > 0): velocity = 0 # we want velocity to always decrease towards the target value
            blendTime = self.blendTime if (velocity == 0) else min(self.blendTime, -5.0 * magnitude / velocity) # avoid overshoot
            if (abs(blendTime) < 0.0001): blendTime = 0.0001 # quick-fix to avoid division-by-zero on very small changes
            acc = (-8.0 * velocity * blendTime - 20.0 * magnitude) / (blendTime * blendTime)
            if (acc < 0): acc = 0 # we want acceleration to counter-act the velocity
            polynomials.append(magnitude) # constant
            polynomials.append(velocity) # 1st polynomial
            polynomials.append(acc / 2.0) # 2nd polynomial
            blendTimeN = blendTime * blendTime
            jerk = acc * blendTimeN
            blendTimeN *= blendTime
            inc = velocity * blendTime
            polynomials.append(-((3.0 * jerk + 12.0 * inc + 20.0 * magnitude) / (2.0 * blendTimeN))) # 3rd polynomial
            blendTimeN *= blendTime
            polynomials.append(((3.0 * jerk + 16.0 * inc + 30.0 * magnitude) / (2.0 * blendTimeN))) # 4th polynomial
            blendTimeN *= blendTime
            polynomials.append(-((jerk + 6.0 * inc + 12.0 * magnitude) / (2.0 * blendTimeN))) # 5th polynomial
            if (negate): polynomials = [-polynomials[0], -polynomials[1], -polynomials[2], -polynomials[3], -polynomials[4], -polynomials[5]]
            self.__blendPolynomials.append((polynomials, jointAxis))

    def toLocalTime(self, globalTime):
        t = (globalTime - self.startTime) * self.playbackRate
        duration = self.animations[self.targetAnimationID].duration
        t = (t % duration) if self.looping else clamp(t, 0.0, duration)
        return t
    def toGlobalTime(self, localTime): # note: this conversion does not loop nor clamp the time
        return (self.startTime + 1.0 / self.playbackRate * localTime)

    def inertialize(self, polynomials, t):
        if (self.blendTime < t or self.blendTime == 0): return 0 # no need to calculate the blend when time t is larger than the blend time
        tt = t * t
        ttt = tt * t
        tttt = ttt * t
        ttttt = tttt * t
        val = polynomials[0] + polynomials[1] * t + polynomials[2] * tt + polynomials[3] * ttt + polynomials[4] * tttt + polynomials[5] * ttttt
        return val

    # Get the inverse character matrix at frame index
    def getWorldToCharMatrix(self, frameID):
        animation = self.animations[self.targetAnimationID]
        return animation.conversionMatrix[frameID]

    def getFlattenedPose(self):
        res = []
        res.extend(self.currPose[0].tolist())
        res.extend([0, 0, 0, 1]) # base orientation, not stored in pose
        for i in range(1, len(self.currPose)):
            res.extend(self.currPose[i])
        return res

    # Get center of mass velocity from animation data
    def getDesiredCMVel(self):
        t = self.animationElapsedTime + self.offset
        animation = self.animations[self.targetAnimationID]
        frame = min(t / animation.frameTime, animation.frameCount - 2)
        desiredFrame = min((t + 1) / animation.frameTime, animation.frameCount - 2)
        currFrame = int(frame)
        desiredCurrFrame = int(desiredFrame)
        frameFrac = frame - currFrame
        pos = animation.data["Frames"][currFrame][0:3]
        nextPos = animation.data["Frames"][currFrame + 1][0:3]
        desiredPos = animation.data["Frames"][desiredCurrFrame][0:3]
        desiredNextPos = animation.data["Frames"][desiredCurrFrame + 1][0:3]
        return (lerpVectors(Vector3.fromlist(desiredPos), Vector3.fromlist(desiredNextPos), frameFrac) - 
                lerpVectors(Vector3.fromlist(pos), Vector3.fromlist(nextPos), frameFrac)) / ((desiredFrame - frame) * animation.frameTime)