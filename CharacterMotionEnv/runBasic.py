from stable_baselines.common.env_checker import check_env
from stable_baselines import PPO2
import gym
import os
import matplotlib.pyplot as plt
import numpy as np
import time

modelfilespecification = "_96M_DeepMimic_params_45deg" # <---------- change this to specify which model to use

loadpath = os.path.dirname(os.path.realpath(__file__)) + "/models/ppo2_model" + modelfilespecification
args = {"useGUI":True, "usePlane":True, "plotRewards":False, "useDatetime":True, "visualizeOffset":False, "timeStep":1./60., "solverIterations":32, "randomStart":False}
env = gym.make("character_motion:basic_env-v0", args=args)
model = PPO2.load(loadpath)
env.env.setPDParams2([0.8] * 14, 226)
obs = env.reset()
storeData = -1 # 0: store state, 1: store reward, -1: do not store
totalSteps = 800
plotData = []
pause = False
pauseStart = 0
if (storeData == 0):
    plt.clf()
    plt.title("State")
    plt.xlabel("time")
    plt.ylabel("feature values")
elif (storeData == 1):
    plt.clf()
    plt.title("Reward Signal")
    plt.xlabel("time")
    plt.ylabel("step reward")
    plt.ylim(top=3.2)

while (env.env.p.isConnected()):
    keys = env.env.p.getKeyboardEvents()
    if (ord('x') in keys and keys[ord('x')] & env.env.p.KEY_WAS_RELEASED): 
        pause = not pause
        if (pause):
            pauseStart = time.time()
        else:
            env.env.startTime += time.time() - pauseStart
    if (pause): continue
    action, states = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    if (storeData == 0): 
        plotData.append(obs)
        if (len(plotData) >= totalSteps):
            plt.plot(np.array(plotData))
            plotFileName = "plots/state_plot" + modelfilespecification + ".png"
            plt.savefig(plotFileName, transparent=True)
            storeData = -1
            print("state plot saved to:", plotFileName)
    elif (storeData == 1): 
        plotData.append(reward)
        if (len(plotData) >= totalSteps):
            xs = [0] + list(range(len(plotData))) + [totalSteps]
            ys = [0] + plotData + [0]
            plt.fill(xs, ys)
            plotFileName = "plots/reward_signal_plot" + modelfilespecification + ".png"
            plt.savefig(plotFileName, transparent=True)
            storeData = -1
            print("reward plot saved to:", plotFileName)

    if (done): env.reset()