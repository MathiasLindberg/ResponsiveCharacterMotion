import gym
from gym import error, spaces, utils
from numpy import random
from character_motion.src.environment import Environment
from character_motion.src.vectors import Vector3
from character_motion.src.character import Character, CharacterType
import character_motion.src.animator as animator_
import character_motion.src.animationLoader as animationLoader
import os
import numpy as np
import matplotlib.pyplot as plt
import json

class BasicEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    plotpath = os.path.dirname(os.path.realpath(__file__)) + "/../../plots"
    smoothingRate = 0.2

    def __init__(self, args):
        self.plotRewards = "plotRewards" in args and args["plotRewards"]
        self.storeRewards = "storeRewards" in args and args["storeRewards"]
        self.randomStart = "randomStart" in args and args["randomStart"]
        self.env = Environment(args)
        self.kinModel = Character((1, 1, 1), self.env.p)
        self.kinModel.setPosition(Vector3(0, 0, 1))
        if (self.env.visualizeOffset):
            self.offsetModel = Character((0, 1, 0), self.env.p)
            self.offsetModel.setPosition(Vector3(0, 0, 1))
            self.offsetModel.tag = CharacterType.NONE
            self.env.characters.append(self.offsetModel)
        self.animator = animator_.Animator(animator_.AnimationType.KINEMATIC, self.kinModel, self.env.p)
        self.animator.animations = animationLoader.loadAnimations(True)
        #self.animator.motion = animator_.MotionType.DISPLACEMENT
        #self.kinModel.zRotation = 0.7853985
        self.animator.motion = animator_.MotionType.ROOT
        self.animator.playAnimation(0, "sample_arch_filtered1")
        self.kinModel.setAnimator(self.animator)
        self.env.characters.append(self.kinModel)
        self.kinModel.tag = CharacterType.KINEMATIC
        self.simModel = Character((0, 0, 1), self.env.p)
        self.simModel.setPosition(Vector3(0, 0, 1))
        self.simModel.tag = CharacterType.SIMULATED
        self.env.characters.append(self.simModel)

        self.env.p.setCollisionFilterGroupMask(self.kinModel.id, -1, 0, 1)
        self.env.p.setCollisionFilterGroupMask(self.simModel.id, -1, 0, 1)
        if (self.env.visualizeOffset): self.env.p.setCollisionFilterGroupMask(self.offsetModel.id, -1, 0, 1)
        for jointID in self.kinModel.jointIDs.values():
            self.env.p.setCollisionFilterGroupMask(self.kinModel.id, jointID, 0, 1)
            self.env.p.setCollisionFilterGroupMask(self.simModel.id, jointID, 0, 1)
            if (self.env.visualizeOffset): self.env.p.setCollisionFilterGroupMask(self.offsetModel.id, jointID, 0, 1)
        self.env.setupPDController()
        self.env.resetTime()
        self.env.stepSimulation()
        self.env.resetSimModelState()

        self.action = np.zeros(25)
        self.prevAction = np.zeros(25)
        self.timestep = 1
        self.totalReward = 0.0
        self.episode = 1
        self.scorehistory = []

        self.action_space = spaces.Box(-0.785, 0.785, (25,))
        #self.action_space = spaces.Box(-1.57, 1.57, (25,))
        #self.action_space = spaces.Box(-0.3925, 0.3925, (25,))
        self.observation_space = spaces.Box(-5, 5, (110,))

        if (self.storeRewards):
            path = os.path.dirname(os.path.realpath(__file__)) + "/metrics.txt"
            with open(path, "w") as f: f.write("") # clear metrics file
            self.stepRewards = []
            self.stepTimeLengths = []
            self.totalTimesteps = 0

    def step(self, action):
        self.action = np.add(np.multiply(action, self.smoothingRate), np.multiply(self.prevAction, 1.0 - self.smoothingRate))
        self.env.stepSimulation()
        self.env.updatePDTargetPoints(self.action)
        dt = self.simModel.currTime - self.simModel.prevTime
        if (dt == 0): dt = 0.001
        state = self.env.constructRLState(self.action, self.animator.getDesiredCMVel().Vector2())
        reward, done = self.env.calculateReward()
        self.prevAction = self.action

        if (self.plotRewards):
            self.totalReward += reward
            if (done):
                print(self.timestep, "timesteps,", self.totalReward, "total reward")
                self.scorehistory.append(self.totalReward / self.timestep)
                self.totalReward = 0
                self.episode += 1
        elif (self.storeRewards):
            self.timestep += 1
            if (done): 
                self.episode += 1
                self.totalTimesteps += self.timestep
                self.stepTimeLengths.append(self.env.globalTime)
            self.stepRewards.append(reward)
            if (self.totalTimesteps >= 2048): self.storeMetrics()
        
        self.env.p.stepSimulation()
        
        

        info = {}

        return state, reward, done, info

    def reset(self):
        self.env.resetTime()
        if (self.randomStart):
            animProb = random.rand()
            if (animProb < 0.33): self.animator.playAnimation(0, "walk1_subject1_filtered1")
            elif (animProb < 0.66): self.animator.playAnimation(0, "walk1_subject1_filtered2")
            else: self.animator.playAnimation(0, "walk2_subject1_filtered1")
            self.animator.offset = random.rand() * (self.animator.getAnimation().duration - 5)
        else:
            self.animator.playNextAnimation(0)
        self.env.stepSimulation()
        self.env.resetSimModelState()
        state = self.env.constructRLState(self.action, self.animator.getDesiredCMVel().Vector2())
        self.totalReward = 0
        self.timestep = 0
        
        return state
  
    def render(self, mode='human'):
        pass

    def close(self):
        if (self.plotRewards):
            plt.clf()
            plt.plot(list(range(1, len(self.scorehistory) + 1)), self.scorehistory, linewidth=1)
            plt.xlabel("episode")
            plt.ylabel("avg. reward")
            plt.title("PPO score")
            plt.savefig(self.plotpath + "/ppo_score.png")

    def storeMetrics(self):
        path = os.path.dirname(os.path.realpath(__file__)) + "/metrics.txt"
        with open(path, "a") as f:
            f.write("\n")
            f.write(json.dumps({"rewards":self.stepRewards, "timelengths":self.stepTimeLengths}))
        self.totalTimesteps = 0
        self.stepRewards.clear()
        self.stepTimeLengths.clear()