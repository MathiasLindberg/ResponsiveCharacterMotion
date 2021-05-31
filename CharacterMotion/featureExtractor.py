from environment import Environment
import animator as animator_
from character import Character
import pybullet as p
from vectors import *
from animationLoader import loadAnimations, storeSampleTrajectory
from motionMatcher import *
import math
import helpers
import numpy as np
import time
import transformations as transform

normalize = False

def extract():
    global normalize
    posFwds = []
    trajectories = []
    poses = []
    frameTimes = []
    animationNames = []
    animationIDs = []
    conversions = []
    args = {"useGUI":False}
    env = Environment(args)
    kinModel = Character(31, [0, 0, 1, 1])
    animator = animator_.Animator(animator_.AnimationType.KINEMATIC, kinModel)
    animator.animations = loadAnimations(True)
    kinModel.setAnimator(animator)
    env.characters.append(kinModel)
    p.getCameraImage(1024, 768)

    # Extract data from animations
    frameNum = 0
    totalFrameNum = 0
    print("\nExtracting pose features from animations...")
    animator.playNextAnimation(env.globalTime)
    animationNames.append(animator.getAnimation().name)
    animationIDs.append(0)
    animCount = 1
    animator.motion = animator_.MotionType.ROOT
    prevPose = [Vector3()] * 6
    first = True
    while (p.isConnected()):
        frameNum += 1
        env.stepSimulation(False)
        animation = animator.getAnimation()
        frameTime = (frameNum - 1) * animation.frameTime
        kinModel.stepCharacter(frameTime)
        frameTimes.append(frameTime)
        pose = kinModel.poseInfo
        #poseFeatures = helpers.extractPoseFeatures(pose, prevPose, animation.frameTime)
        conversion = np.array(animator.getWorldToCharMatrix(frameNum - 1))
        prevConversion = np.array(animator.getWorldToCharMatrix(max(frameNum - 2, 0)))
        leftFootPos = conversion.dot(vec3To4List(pose[2]))
        rightFootPos = conversion.dot(vec3To4List(pose[3]))
        hipPos = conversion.dot(vec3To4List(pose[1]))
        prevLeftFootPos = prevConversion.dot(vec3To4List(prevPose[2]))
        prevRightFootPos = prevConversion.dot(vec3To4List(prevPose[3]))
        prevHipPos = prevConversion.dot(vec3To4List(prevPose[1]))
        leftFootVel = (leftFootPos - prevLeftFootPos) / animation.frameTime
        rightFootVel = (rightFootPos - prevRightFootPos) / animation.frameTime
        hipVel = (hipPos - prevHipPos) / animation.frameTime
        forward = helpers.calculateForward(pose)
        poses.append([leftFootPos[0], leftFootPos[1], leftFootPos[2], rightFootPos[0], rightFootPos[1], rightFootPos[2], 
                      leftFootVel[0], leftFootVel[1], leftFootVel[2], rightFootVel[0], rightFootVel[1], rightFootVel[2], 
                      hipVel[0], hipVel[1], hipVel[2]])
        prevPose = pose
        posFwds.append((pose[0], forward))
        conversions.append(conversion)
        if (frameNum >= animation.frameCount): # next animation
            if (first): first = False
            else:
                animationNames.append(animation.name)
                animationIDs.append(totalFrameNum)
            print(animation.name + ": " + str(frameTime) + "/" + str(animation.duration) + ", animation ID: " + str(totalFrameNum))
            totalFrameNum += frameNum
            if (animCount >= len(animator.animations)): break
            env.resetTime()
            frameNum = 0
            animator.playNextAnimation(env.globalTime)
            animCount += 1
            prevPose = [Vector3()] * 6
    posFwds[0] = posFwds[1]
    storeSampleTrajectory(posFwds)

    # Calculate trajectory directions
    print("Extracting trajectory positions and directions...")
    for i in range(len(animationIDs)): # for each animation...
        startID = animationIDs[i]
        endID = animationIDs[i + 1] if ((i + 1) < len(animationIDs)) else (len(posFwds)) # clamp if exceeding length of data
        for f in range(startID, endID): # for each frame in animation...
            trajectory = []
            (_, currFwd) = posFwds[f]
            conversion = conversions[f]
            for n in range(20, 80, 20): # 20, 40 and 60 frames in the future
                frameID = min(f + n, endID - 1)
                (nextPos, nextFwd) = posFwds[frameID]
                nextPos = conversion.dot(helpers.vec3To4List(nextPos))
                relAngle = math.atan2(nextFwd.y, nextFwd.x) - math.atan2(currFwd.y, currFwd.x)
                trajectory.append(nextPos[0]) # posX
                trajectory.append(nextPos[1]) # posY
                trajectory.append(math.cos(relAngle)) # rotX
                trajectory.append(math.sin(relAngle)) # rotY
            trajectories.append(trajectory)

    # Concatenate features
    featureMatrix = []
    print("Concatenating features...")
    for i in range(len(posFwds)):
        features = []
        features.extend(poses[i])
        features.extend(trajectories[i])
        featureMatrix.append(features)

    if (normalize):
        # Normalize features
        print("Normalizing features...")
        stdDevs = np.std(featureMatrix, 0, dtype='float32')
        means = np.mean(featureMatrix, 0, dtype='float32')
        for i in range(len(featureMatrix)):
            for j in range(len(featureMatrix[i])):
                featureMatrix[i][j] = (featureMatrix[i][j] - means[j]) / stdDevs[j]
        # Export data
        print("Saving data...")
        saveMMDB(MotionMatchingDB(np.array(featureMatrix, dtype='float32'), stdDevs, means, np.array(frameTimes, dtype='float32'), 
                                  np.array(animationNames, dtype='S'), np.array(animationIDs, 'int32')))
    else:
        saveMMDB(MotionMatchingDB(np.array(featureMatrix, dtype='float32'), np.array([1] * 27, dtype='float32'), np.array([0] * 27, dtype='float32'), 
                                  np.array(frameTimes, dtype='float32'), np.array(animationNames, dtype='S'), np.array(animationIDs, dtype='int32')))

    print("Done.")

def visualize():
    trajectory = []
    feet = []
    hip = -1

    args = {"useGUI":True, "usePlane":False}
    env = Environment(args)
    kinModel = Character(31, [0, 0, 1, 1])
    animator = animator_.Animator(animator_.AnimationType.KINEMATIC, kinModel)
    animator.animations = loadAnimations(True)
    kinModel.setAnimator(animator)
    env.characters.append(kinModel)
    animator.playNextAnimation(env.globalTime)
    animator.motion = animator_.MotionType.ROOT
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.getCameraImage(1024, 768)
    mmdb = loadMMDB()
    pointShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.1, rgbaColor=[0,1,1,1])
    for _ in range(4): trajectory.append(p.addUserDebugLine([0,0,0], [0,0,0]))
    for _ in range(2): feet.append(p.createMultiBody(0, baseVisualShapeIndex=pointShape))
    for _ in range(2): feet.append(p.addUserDebugLine([0,0,0], [0,0,0]))
    hip = p.addUserDebugLine([0,0,0], [0,0,0])
    while (p.isConnected()):
        env.stepSimulation(False)
        kinModel.stepCharacter(env.globalTime)
        frameID = int(animator.animationElapsedTime / animator.getAnimation().frameTime)
        feature = mmdb.featureMatrix[frameID]
        centerOfMass = kinModel.poseInfo[0]
        startAngle = math.atan2(kinModel.forward.y, kinModel.forward.x)
        p.addUserDebugLine([centerOfMass.x, centerOfMass.y, 1], [centerOfMass.x + math.cos(startAngle), centerOfMass.y + math.sin(startAngle), 1], [0, 0, 1], 2.0, replaceItemUniqueId=trajectory[0])
        for i in range(3):
            ind = 15 + i * 4
            nextFwd = Vector2(feature[ind + 2], feature[ind + 3])
            fromP = Vector2(feature[ind], feature[ind + 1])
            toP = fromP + nextFwd
            p.addUserDebugLine([fromP.x, fromP.y, centerOfMass.z], [toP.x, toP.y, centerOfMass.z], [0, 1, 0], 2.0, replaceItemUniqueId=trajectory[i + 1])
        #conversion = transform.inverse_matrix(np.array(animator.getWorldToCharMatrix(animator.animationElapsedTime)))
        leftFootPos = Vector3(feature[0], feature[1], feature[2])
        rightFootPos = Vector3(feature[3], feature[4], feature[5])
        leftFootVelEnd = leftFootPos + Vector3(feature[6], feature[7], feature[8])
        rightFootVelEnd = rightFootPos + Vector3(feature[9], feature[10], feature[11])
        p.resetBasePositionAndOrientation(feet[0], leftFootPos.tolist(), [0,0,0,1])
        p.resetBasePositionAndOrientation(feet[1], rightFootPos.tolist(), [0,0,0,1])

        p.addUserDebugLine(leftFootPos.tolist(), leftFootVelEnd.tolist(), [0, 1, 1], 2.0, replaceItemUniqueId=feet[2])
        p.addUserDebugLine(rightFootPos.tolist(), rightFootVelEnd.tolist(), [0, 1, 1], 2.0, replaceItemUniqueId=feet[3])
        #p.addUserDebugLine(poseFeatures[1].tolist(), (poseFeatures[1] + Vector3(feature[12], feature[13], feature[14])).tolist(), [1, 0, 0], 2.0, replaceItemUniqueId=hip)
        
        time.sleep(env.timeStep)

#extract()
visualize()

# Feature vector: left-foot pos, right-foot pos, left-foot vel, right-foot vel, hip vel, pos1XY, rot1XY, pos2XY, rot2XY, pos3XY, rot3XY