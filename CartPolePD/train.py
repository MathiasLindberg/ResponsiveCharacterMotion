from stable_baselines.common.env_checker import check_env
from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO2
import tensorflow as tf
import gym
import os

timesteps = 5000000
savepath = os.path.dirname(os.path.realpath(__file__)) + "/models/PPO_model"
env = gym.make("gym_basic:basic-v0", trainMode=True)
#print("basic environment setup was successful" if (type(check_env(env)) == type(None)) else "basic environment setup failed")
policyArguments = {"act_fun":tf.nn.tanh}
model = PPO2(MlpPolicy, env, policy_kwargs=policyArguments, verbose=1, tensorboard_log="./ppo2_tensorboard/")
model.learn(total_timesteps=timesteps, tb_log_name="run 1", reset_num_timesteps=False)
model.save(savepath)
env.close()
print("Model trained for ", timesteps, "timesteps. Saved to ", savepath)