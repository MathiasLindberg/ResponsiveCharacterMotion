import matplotlib.pyplot as plt
import os
import json
import numpy as np

metricsPath = os.path.dirname(os.path.realpath(__file__)) + "/character_motion/envs/metrics_deepmimic_9M.txt"
episodicRewards = []
timelengths = []
with open(metricsPath) as f:
    line = f.readline()
    while (line):
        metric = json.loads(line)
        episodicRewards.append(np.mean(metric["rewards"]))
        timelengths.append(np.mean(metric["timelengths"]))
        line = f.readline()
plt.title("Reward signal during training (9M timesteps)")
plt.xlabel("metric batch (2048 steps/batch)")
plt.ylabel("mean reward")
steps = list(range(len(episodicRewards)))
rewardCoeff = np.polyfit(steps, episodicRewards, 1)
rewardPoly1d = np.poly1d(rewardCoeff)
print("reward coefficient:", rewardCoeff)
plt.plot(steps, episodicRewards, 'bo', label="rewards")
plt.plot(steps, rewardPoly1d(steps), "--k", label="Regression analysis (rewards)")
plt.legend()
plt.show()
plt.clf()
plt.title("Episode lengths during training (9M timesteps)")
plt.xlabel("metric batch (2048 steps/batch)")
plt.ylabel("mean timelength")
timeCoeff = np.polyfit(steps, timelengths, 1)
timePoly1d = np.poly1d(timeCoeff)
print("time coefficient:", timeCoeff)
plt.plot(steps, timelengths, "yx", label="timelengths")
plt.plot(steps, timePoly1d(steps), "--k", label="Regression analysis (timelengths)")
plt.legend()
plt.show()