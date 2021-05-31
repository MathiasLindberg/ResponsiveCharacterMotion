from gym.envs.registration import register

register(
    id='basic_env-v0',
    entry_point='character_motion.envs:BasicEnv',
)