from environment import Environment
import animator as animator_
from character import Character, CharacterType
from vectors import *
from animationLoader import loadAnimations
import transformations as transform
import motionMatcher
import pybullet as p
import math
import helpers
import numpy as np
from animationDrawer import AnimationDrawer

env = None
player = None
controller = None
mmdb = None
playerEnvID = 0
targetTrajectory = []
targetPose = []
positions = []
feet = []
useAccelerationStructure = False
rlAgent = None
animDrawer = None
playMM = False
pathStartTime = 0
segmentFrameTimeParam = -1
pathTraj = None
pathConversion = None
endOfPath = None

def main():
    global env, player, controller, mmdb, playerEnvID, targetTrajectory, positions, feet, rlAgent, animDrawer, playMM
    global pathStartTime, segmentFrameTimeParam, pathTraj, pathConversion, endOfPath
    args = {"useGUI":True, "usePlane":True}
    env = Environment(args)
    mkCharacters()
    player = env.characters[playerEnvID]
    animator = player.getAnimator()
    drawPlane = p.loadURDF("plane.urdf") # need 2nd plane because pybullet's raytest does not support non-default collision masks...
    segmentFrameTimeParam = p.addUserDebugParameter("segment frame time", 1, 240, 22)
    realignmentRateParam = p.addUserDebugParameter("angle realignment", 0, 1, 0.01)
    repositionRateParam = p.addUserDebugParameter("repositioning", 0, 1, 0.1)
    animDrawer = AnimationDrawer(drawPlane)
    #p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.getCameraImage(1024, 768)
    lastMM = 0
    mmdb = motionMatcher.loadMMDB()
    for _ in range(9):
        targetTrajectory.append(p.addUserDebugLine([0,0,0],[0,0,0]))
    for _ in range(12):    
        targetPose.append(p.addUserDebugLine([0,0,0],[0,0,0]))
    pointShape1 = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1,0,0,1])
    pointShape2 = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0,0,1,1])
    for _ in range(3):
        positions.append((p.createMultiBody(0, baseVisualShapeIndex=pointShape1)))
        positions.append((p.createMultiBody(0, baseVisualShapeIndex=pointShape2)))
    pointShape3 = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0,1,1,1])
    for _ in range(2):
        feet.append((p.createMultiBody(0, baseVisualShapeIndex=pointShape3)))

    motionMatcher.searchDBDirect(mmdb.featureMatrix, mmdb.frameTimes, mmdb.weights, mmdb.animationNames, mmdb.animationIDs, np.array([0] * 27))
    env.resetTime()
    env.stepSimulation()
    while (p.isConnected()):
        env.stepSimulation()
        if (playMM):
            elapsedPathFrameTime = (env.globalTime - pathStartTime) * 30
            pathTraj, pathConversion, endOfPath = animDrawer.getTrajectory(elapsedPathFrameTime, p.readUserDebugParameter(segmentFrameTimeParam))
            desPos = Vector2(pathTraj[0][0], pathTraj[0][1])
            posDiff = desPos - player.position
            if (posDiff.magnitude() > 0.15): # clamp character
                dt = player.currTime - player.prevTime
                clampedPos = posDiff * dt * p.readUserDebugParameter(repositionRateParam) + player.position
                player.setPosition(Vector3(clampedPos.x, clampedPos.y, player.position.z))
            animator.desiredAngleGain = (helpers.shortestSignedAngleBetween(player.forward, Vector2(pathTraj[0][2], pathTraj[0][3]))) * p.readUserDebugParameter(realignmentRateParam)
        mouseEvents = p.getMouseEvents()
        keyEvents = p.getKeyboardEvents()
        animDrawer.update(keyEvents, mouseEvents)
        if (playMM and env.globalTime - lastMM > 0.33): # motion match on ever ~10th animation frame
            motionMatch()
            lastMM = env.globalTime

        if (p.B3G_SPACE in keyEvents):
            if (keyEvents[p.B3G_SPACE] & p.KEY_WAS_RELEASED):
                if (not playMM):
                    pathTraj, _, endOfPath = animDrawer.getTrajectory(0, p.readUserDebugParameter(segmentFrameTimeParam))
                    if (not endOfPath):
                        firstPoint = pathTraj[0]
                        player.setPosition(Vector3(firstPoint[0], firstPoint[1], player.position.z))
                        player.zRotation = math.atan2(firstPoint[3], firstPoint[2])
                        pathStartTime = env.globalTime
                        playMM = True
                else:
                    player.getAnimator().stop(player)
                    playMM = False
        elif (ord('n') in keyEvents):
            if (keyEvents[ord('n')] & p.KEY_WAS_RELEASED):
                animator.blendNextAnimation(env.globalTime)
        

def mkCharacters():
    kinModel = Character(31, [1, 1, 1, 1])
    kinModel.setPosition(Vector3(0, 0, 1))
    animator = animator_.Animator(animator_.AnimationType.KINEMATIC, kinModel)
    animator.animations = loadAnimations(True)
    animator.motion = animator_.MotionType.DISPLACEMENT
    animator.playNextAnimation(env.globalTime)
    kinModel.setAnimator(animator)
    env.characters.append(kinModel)
    kinModel.zRotation = 0.7853985
    kinModel.tag = CharacterType.KINEMATIC
    

def motionMatch():
    global targetTrajectory, useAccelerationStructure, positions, feet, pathStartTime, playMM, animDrawer, segmentFrameTimeParam, env, pathTraj, pathConversion, endOfPath
    animator = player.getAnimator()
    animation = animator.getAnimation()

    leftFootPos = player.conversion.dot(vec3To4List(player.poseInfo[2]))
    rightFootPos = player.conversion.dot(vec3To4List(player.poseInfo[3]))
    hipPos = player.conversion.dot(vec3To4List(player.poseInfo[1]))
    prevLeftFootPos = player.prevConversion.dot(vec3To4List(player.prevPoseInfo[2]))
    prevRightFootPos = player.prevConversion.dot(vec3To4List(player.prevPoseInfo[3]))
    prevHipPos = player.prevConversion.dot(vec3To4List(player.prevPoseInfo[1]))
    dt = player.currTime - player.prevTime
    leftFootVel = (leftFootPos - prevLeftFootPos) / dt
    rightFootVel = (rightFootPos - prevRightFootPos) / dt
    hipVel = (hipPos - prevHipPos) / dt
    featureVector = [leftFootPos[0], leftFootPos[1], leftFootPos[2], rightFootPos[0], rightFootPos[1], rightFootPos[2], 
                     leftFootVel[0], leftFootVel[1], leftFootVel[2], rightFootVel[0], rightFootVel[1], rightFootVel[2], 
                     hipVel[0], hipVel[1], hipVel[2]]
    if (endOfPath):
        playMM = False
        animator.stop(player)
        return
    for point in pathTraj:
        dir = pathConversion.dot([point[0] + point[2], point[1] + point[3], player.position.z, 1])
        pos = pathConversion.dot([point[0], point[1], player.position.z, 1])
        dir = [dir[0] - pos[0], dir[1] - pos[1]]
        featureVector.append(pos[0]) # posX
        featureVector.append(pos[1]) # posY
        featureVector.append(dir[0]) # dirX
        featureVector.append(dir[1]) # dirY

    mmdb.normalize(featureVector)
    
    (animName, animTime, candidateVector) = motionMatcher.searchDBDirect(mmdb.featureMatrix, mmdb.frameTimes, mmdb.weights, mmdb.animationNames, mmdb.animationIDs, np.array(featureVector, dtype='float32'))
    animName = animName.decode('UTF-8')
    
    for i in range(3):
        ind = 15 + i * 4
        point = pathTraj[i]
        desiredFwd = Vector3(point[2], point[3], 0)
        desiredFromP = Vector3(point[0], point[1], 0.5)
        desiredToP = desiredFromP + desiredFwd
        p.resetBasePositionAndOrientation(positions[i * 2], desiredFromP.tolist(), [0,0,0,1])
        p.addUserDebugLine(desiredFromP.tolist(), desiredToP.tolist(), [1, 0, 0], 2.0, replaceItemUniqueId=targetTrajectory[i * 3])
        foundFwd = Vector3(candidateVector[ind + 2], candidateVector[ind + 3], 0)
        foundFromP = Vector3(candidateVector[ind], candidateVector[ind + 1], 0)
        foundToP = foundFromP + foundFwd
        foundFromP = transform.inverse_matrix(player.conversion).dot(vec3To4List(foundFromP))
        foundToP = transform.inverse_matrix(player.conversion).dot(vec3To4List(foundToP))
        p.resetBasePositionAndOrientation(positions[i * 2 + 1], [foundFromP[0], foundFromP[1], foundFromP[2]], [0,0,0,1])
        p.addUserDebugLine([foundFromP[0], foundFromP[1], foundFromP[2]], [foundToP[0], foundToP[1], foundToP[2]], [0, 0, 1], 2.0, replaceItemUniqueId=targetTrajectory[i * 3 + 1])
        p.addUserDebugLine(desiredToP.tolist(), [foundToP[0], foundToP[1], foundToP[2]], [1, 0.8, 0.1], 1.0, replaceItemUniqueId=targetTrajectory[i * 3 + 2])
    leftFootFromP = [featureVector[0], featureVector[1], featureVector[2]]
    leftFootToP = [leftFootFromP[0] + featureVector[6], leftFootFromP[1] + featureVector[7], leftFootFromP[2] + featureVector[8]]
    rightFootFromP = [featureVector[3], featureVector[4], featureVector[5]]
    rightFootToP = [rightFootFromP[0] + featureVector[9], rightFootFromP[1] + featureVector[10], rightFootFromP[2] + featureVector[11]]
    hipsFromP = [0, 0, 0]
    hipsToP = [hipsFromP[0] + featureVector[12], hipsFromP[1] + featureVector[13], hipsFromP[2] + featureVector[14]]
    p.resetBasePositionAndOrientation(feet[0], leftFootFromP, [0,0,0,1])
    p.resetBasePositionAndOrientation(feet[1], rightFootFromP, [0,0,0,1])
    p.addUserDebugLine(leftFootFromP, leftFootToP, [1, 0, 1], 2.0, replaceItemUniqueId=targetPose[0])
    p.addUserDebugLine(rightFootFromP, rightFootToP, [1, 0, 1], 2.0, replaceItemUniqueId=targetTrajectory[1])
    p.addUserDebugLine(hipsFromP, hipsToP, [1, 0, 1], 2.0, replaceItemUniqueId=targetTrajectory[2])
    targetLeftFootFromP = [candidateVector[0], candidateVector[1], candidateVector[2]]
    targetLeftFootToP = [leftFootFromP[0] + candidateVector[6], leftFootFromP[1] + candidateVector[7], leftFootFromP[2] + candidateVector[8]]
    targetRightFootFromP = [candidateVector[3], candidateVector[4], candidateVector[5]]
    targetRightFootToP = [rightFootFromP[0] + candidateVector[9], rightFootFromP[1] + candidateVector[10], rightFootFromP[2] + candidateVector[11]]
    targetHipsFromP = [0, 0, 0]
    targetHipsToP = [hipsFromP[0] + candidateVector[12], hipsFromP[1] + candidateVector[13], hipsFromP[2] + candidateVector[14]]
    p.addUserDebugLine(targetLeftFootFromP, targetLeftFootToP, [0, 1, 1], 2.0, replaceItemUniqueId=targetPose[3])
    p.addUserDebugLine(targetRightFootFromP, targetRightFootToP, [0, 1, 1], 2.0, replaceItemUniqueId=targetTrajectory[4])
    p.addUserDebugLine(targetHipsFromP, targetHipsToP, [0, 1, 1], 2.0, replaceItemUniqueId=targetTrajectory[5])

    if (animation == None or animName != animation.name or abs(animTime - (animator.offset + animator.animationElapsedTime)) > 0.2): # do not transit to the frame already being played
        #animator.playAnimation(env.globalTime, animName, animTime) # no blending
        animator.blendAnimation(env.globalTime, animName, animTime) # Inertialization blending
    

main()


