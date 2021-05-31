from stable_baselines import PPO2
import gym
import os
import time

loadpath = os.path.dirname(os.path.realpath(__file__)) + "/models/PPO_model"
env = gym.make("gym_basic:basic-v0", trainMode=False)
model = PPO2.load(loadpath)

allowUserOffsets = False
force = [0,0,0]
obs = env.reset()
while (env.p.isConnected()):
    if (allowUserOffsets):
        keys = env.p.getKeyboardEvents()
        if (ord('1') in keys): force[0] += 1
        elif (ord('2') in keys): force[0] -= 1
        if (ord('3') in keys): force[1] += 1
        elif (ord('4') in keys): force[1] -= 1
        if (ord('5') in keys): force[2] += 1
        elif (ord('6') in keys): force[2] -= 1
    action, states = model.predict(obs)
    if (allowUserOffsets): action = force
    obs, reward, done, info = env.step(action)
    if (done): 
        env.reset()
    time.sleep(env.timestep)