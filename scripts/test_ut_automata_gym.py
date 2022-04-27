import cffi
from spinup import ppo_pytorch as ppo
import gym
from ut_automata_gym_env import UTAUTOmata

gym.register(
    id='UTAUTOmata-v0',
    entry_point='ut_automata_gym_env:UTAUTOmata',
    max_episode_steps=200,
    reward_threshold=25.0,
)

env_fn = lambda : gym.make('UTAUTOmata-v0')
logger_kwargs = dict(output_dir='ppo_logging_dir', exp_name='ut_automata_ppo')

ppo(env_fn=env_fn, logger_kwargs=logger_kwargs)