from stable_baselines.common.env_checker import check_env
from stable_baselines.common.policies import FeedForwardPolicy, MlpPolicy
from stable_baselines.common import make_vec_env, set_global_seeds
from stable_baselines.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines import PPO2
import tensorflow as tf
import gym
import os

trainExistingModel = "" # <-------- set this to specify a model that should be trained

def makeEnv(rank, envArgs, seed=0):
    def initEnv():
        env = gym.make("character_motion:basic_env-v0", args=envArgs)
        env.seed(seed + rank)
        env.env.setPDParams2([0.8] * 14, 226)
        return env
    set_global_seeds(seed)
    return initEnv

if __name__ == "__main__":
    timesteps = 32000000
    processesCount = 6
    savepath = os.path.dirname(os.path.realpath(__file__)) + "/models/ppo2_model"
    args = {"useGUI":False, "usePlane":True, "plotRewards":False, "storeRewards":False, "useDatetime":False, "randomStart":True}
    env = SubprocVecEnv([makeEnv(i + processesCount, args) for i in range(processesCount)], start_method="spawn")
    policyArguments = {"act_fun":tf.nn.tanh, "net_arch":[dict(vf=[64, 64], pi=[64, 64])]}
    model = None
    if (trainExistingModel != ""): model = PPO2.load(trainExistingModel, env)
    else:
        # DReCon
        #model = PPO2(MlpPolicy, env, policy_kwargs=policyArguments, n_steps=40000, learning_rate=0.01, nminibatches=1250, gamma=0.9, verbose=1, tensorboard_log="./ppo2_tensorboard/")
        # RoboSchool, assuming horizon and minibatch count got swapped
        #model = PPO2(MlpPolicy, env, policy_kwargs=policyArguments, n_steps=4096, learning_rate=0.0003, noptepochs=15, nminibatches=512, gamma=0.99, verbose=1, tensorboard_log="./ppo2_tensorboard/")
        # Mujoco
        #model = PPO2(MlpPolicy, env, policy_kwargs=policyArguments, n_steps=2048, learning_rate=0.0003, noptepochs=10, nminibatches=64, gamma=0.99, verbose=1, tensorboard_log="./ppo2_tensorboard/")
        # DeepMimic
        model = PPO2(MlpPolicy, env, policy_kwargs=policyArguments, n_steps=4096, gamma=0.95, nminibatches=256, verbose=1, tensorboard_log="./ppo2_tensorboard/")
    model.learn(total_timesteps=timesteps, tb_log_name="final", reset_num_timesteps=True)
    model.save(savepath)
    env.close()
    print("Model trained for ", timesteps, "timesteps. Saved to ", savepath)