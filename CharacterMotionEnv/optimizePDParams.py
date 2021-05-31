import gym
import optuna
import os
from stable_baselines import PPO2

loadpath = os.path.dirname(os.path.realpath(__file__)) + "/ppo2_model"
args = {"useGUI":False, "usePlane":True, "plotRewards":False, "useDatetime":False, "timeStep":1./60., "solverIterations":32}
env = gym.make("character_motion:basic_env-v0", args=args)
model = PPO2.load(loadpath)
def objective(trial):
    kps = []
    # Hips, UpLeg, Leg, Foot, Toe, Spine, Spine1, Spine2, Neck, Head, Shoulder, Arm, ForeArm, Hand
    kps.append(trial.suggest_float("Hips kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("UpLeg kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Leg kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Foot kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Toe kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Spine kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Spine1 kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Spine2 kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Neck kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Head kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Shoulder kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Arm kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("ForeArm kp", 0.001, 0.999, log=True))
    kps.append(trial.suggest_float("Hand kp", 0.001, 0.999, log=True))
    #kd = trial.suggest_float("kd", 1, 100, log=True)
    #baseForce = trial.suggest_float("baseForce", 10, 250, log=True)
    env.env.setPDParams2(kps, 226)
    obs = env.reset()
    accReward = 0
    while (env.env.p.isConnected()):
        action, states = model.predict(obs)
        obs, reward, done, info = env.step(action)
        if (done): break
        accReward += reward
    return accReward

study = optuna.create_study(study_name="optimize_position_control", storage="sqlite:///pdc_optuna.db", load_if_exists=True, direction="maximize")
study.optimize(objective, n_trials=30000)
print("best params:", study.best_params)