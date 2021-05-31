import math
from bvh import Bvh
import os
from animation import Animation
from vectors import *
import pybullet as p
import transformations as transform
import helpers
import json
import environment
import character
import animator as animator_

deg2Rad = 0.0174532925

translationScale = 0.01

def __loadBVH(filepath):
    with open(filepath) as f:
        data = Bvh(f.read())
        for i in range(len(data.frames)): # Convert angles to quaternions and change to left-handed system
            frame = data.frames[i]
            newFrame = [float(frame[2]) * translationScale, float(frame[0]) * translationScale, float(frame[1]) * translationScale]
            for j in range(3, len(frame), 3): # ignore first 3 root position values
                if (j < 6): 
                    rot = transform.quaternion_from_euler(float(frame[j]) * deg2Rad, float(frame[j + 1]) * deg2Rad, float(frame[j + 2]) * deg2Rad, "rzyx")
                    rot = transform.quaternion_multiply(transform.quaternion_from_euler(1.570797, 0, 1.570797), rot)
                else: rot = transform.quaternion_from_euler(float(frame[j]) * deg2Rad, float(frame[j + 1]) * deg2Rad, float(frame[j + 2]) * deg2Rad, "rzyx")
                newFrame.append(rot[1])
                newFrame.append(rot[2])
                newFrame.append(rot[3])
                newFrame.append(rot[0])
            data.frames[i] = newFrame
    return Animation(os.path.basename(filepath).split(".")[0], data.frame_time, {"Frames":data.frames})
def __loadJSON(filepath):
    with open(filepath) as f:
        return Animation.fromJSON(json.load(f))

def loadAnimation(filepath):
    if (filepath.__contains__(".bvh")):
        return __loadBVH(filepath)
    elif (filepath.__contains__(".txt")):
        return __loadJSON(filepath)
    print("Failed to load animation: Count not recognize file type (" + filepath + ")")
    return None

#animationNames = ["sample_arch"]
#animationCuts = [[(500, 800)]]
animationNames = ["walk1_subject1", "walk2_subject1", "walk3_subject1", "run2_subject1"]
animationCuts = [
    [(100, 1185), (2480, 3610), (6010, 6280)], # walk 1 subject 1
    [(100, 700), (2250, 2785)], # walk 2 subject 1
    [(70, 1400), (6660, 7350)], # walk 3 subject 1
    [(80, 7310)] # run 2 subject 1
]
"""animationNames = ["walk1_subject1", "walk2_subject1", "walk3_subject1"]
animationCuts = [
    [(600, 1000), (2500, 3600)],
    [(6400,7100)],
    [(5400,6700)]
]"""

def loadAnimations(fromJSON):
    global animationNames
    path = os.path.dirname(os.path.realpath(__file__)) + "/externals/lafan1"
    animations = []
    (_, _, filenames) = next(os.walk(path))
    for filename in filenames:
        if (filename.__contains__(".txt" if fromJSON else ".bvh") and any(filename.__contains__(anim) for anim in animationNames)):
            animations.append(loadAnimation(path + "/" + filename))
    return animations

def __convertAnimations():
    global animationNames
    path = os.path.dirname(os.path.realpath(__file__)) + "/externals/lafan1"
    (_, _, filenames) = next(os.walk(path))
    for filename in filenames:
        if (not (filename.__contains__(".bvh") and filename.__contains__("filtered"))): continue
        if any(filename.__contains__(anim) for anim in animationNames): # only convert specified animations
            filepath = path + "/" + filename
            animation = loadAnimation(filepath)
            filepath = filepath.replace(".bvh", ".txt")
            with open(filepath, 'w') as f:
                json.dump(animation.toJSON(), f)

def __filterAnimation(filepath, startFrame, endFrame):
    res = ""
    frameNum = 0
    with open(filepath) as f:
        for line in f.readlines():
            if (frameNum == 0):
                if (line.__contains__("Frames")): # update frame count
                    res += "Frames: " + str(endFrame - startFrame) + "\n"
                else: # append unchanged information
                    res += line
                    if (line.__contains__("Frame Time")): # start counting frames
                        frameNum = 1
            else: # parsing frames
                if (frameNum <= endFrame and frameNum >= startFrame): # append data if within desired frame interval
                    res += line
                elif (frameNum > endFrame): # no need to parse rest of file
                    break
                frameNum += 1
        return res


def __filterAnimations():
    global animationNames, animationCuts
    path = os.path.dirname(os.path.realpath(__file__)) + "/externals/lafan1"
    (_, _, filenames) = next(os.walk(path))
    for filename in filenames:
        if (filename.__contains__("filtered") or not filename.__contains__(".bvh")): continue
        for i in range(len(animationNames)):
            if (filename.__contains__(animationNames[i])):
                for j in range(len(animationCuts[i])):
                    filepath = path + "/" + filename
                    (startFrame, endFrame) = animationCuts[i][j]
                    animation = __filterAnimation(filepath, startFrame, endFrame)
                    filepath = filepath.replace(".bvh", "_filtered" + str(j + 1) + ".bvh")
                    with open(filepath, 'w') as f:
                        f.writelines(animation)
                break

def __constructWorld2CharacterSpaces():
    args = {"useGUI":False}
    env = environment.Environment(args)
    kinModel = character.Character(31, [0, 0, 1, 1])
    animator = animator_.Animator(animator_.AnimationType.KINEMATIC, kinModel)
    animator.motion = animator_.MotionType.ROOT
    animator.animations = loadAnimations(True)
    kinModel.setAnimator(animator)
    env.characters.append(kinModel)
    frameNum = 0
    totalFrameNum = 0
    animator.playNextAnimation(env.globalTime)
    animCount = 1
    animation = animator.getAnimation()
    animation.conversionMatrix.clear()
    while (p.isConnected()):
        frameNum += 1
        env.stepSimulation(False)
        frameTime = (frameNum - 1) * animation.frameTime
        kinModel.stepCharacter(frameTime)
        centerOfMass = kinModel.getCenterOfMass()
        charForward = kinModel.forward
        charAngle = math.atan2(charForward.y, charForward.x)
        animation.conversionMatrix.append(helpers.constructInverseMatrix(centerOfMass, charAngle).tolist())
        if (frameNum >= animation.frameCount): # next animation
            totalFrameNum += frameNum
            if (animCount >= len(animator.animations)): break
            env.resetTime()
            frameNum = 0
            animator.playNextAnimation(env.globalTime)
            animCount += 1
            animation = animator.getAnimation()
            animation.conversionMatrix.clear()
    path = os.path.dirname(os.path.realpath(__file__)) + "/externals/lafan1"
    for animation in animator.animations:
        filepath = path + "/" + animation.name + ".txt"
        with open(filepath, 'w') as f:
                json.dump(animation.toJSON(), f)

# Filter animations if specified in the animations list
#__filterAnimations()
# Load filtered animations and save them as JSON
#__convertAnimations()
# Generate local motion velocities and store them in the given animations
#__constructWorld2CharacterSpaces()


def storeSampleTrajectory(trajectory):
    path = os.path.dirname(os.path.realpath(__file__)) + "/sample_trajectory.txt"
    convertedTraj = []
    for (pos,fwd) in trajectory:
        convertedTraj.append((pos.tolist(), fwd.tolist()))
    with open(path, 'w') as f:
        json.dump(convertedTraj, f)
def loadSampleTrajectory():
    path = os.path.dirname(os.path.realpath(__file__)) + "/sample_trajectory.txt"
    with open(path) as f:
        return json.load(f)
