import cffi
import time
import numpy as np
import gym
from gym import spaces
import numpy as np
import random



class UTAUTOmata(gym.Env):
    """UT AUTOmata F1/10 racing car environment."""

    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(UTAUTOmata, self).__init__()
        self.episode_timeout = 1000
        self.action_size = 2
        self.observation_size = 41
        self.timestep_count = 0
        self.action_space = spaces.Box(
            -1 * np.ones(self.observation_size), # Lower bounds for each value.
            1 * np.ones(self.observation_size))   # Upper bounds for each value.
        self.observation_space = spaces.Box(
            -1 * np.ones(self.observation_size), # Lower bounds for each value.
            1 * np.ones(self.observation_size))   # Upper bounds for each value.

        print("Loading UT AUTOmata Simulation + Navigation C++ library...")
        self.ffi = cffi.FFI()
        with open("scripts/graph_nav_rl_export.h") as h_file:
            self.ffi.cdef(h_file.read())
        self.C = self.ffi.dlopen("lib/libgraph_nav_rl.so")
        print("Done loading library!")
        self.C.Init()

    def step(self, action):
        # time.sleep(0.025)
        reward = 0
        self.timestep_count += 1
        observation = np.zeros(self.observation_size, dtype = np.float64)
        observation_ptr = self.ffi.cast("double *", observation.ctypes.data)
        action_ptr = self.ffi.cast("double *", action.ctypes.data)
        success = self.C.Step(self.action_size,
                              action_ptr,
                              self.observation_size,
                              observation_ptr)
        done = (self.timestep_count == self.episode_timeout) or (not success)

        return observation, reward, done, {}

    def reset(self):
        self.timestep_count = 0
        self.C.Reset()
        observation = np.zeros(self.observation_size, dtype = np.float64)
        observation_ptr = self.ffi.cast("double *", observation.ctypes.data)
        action = np.zeros(self.action_size, dtype = np.float64)
        action_ptr = self.ffi.cast("double *", action.ctypes.data)
        success = self.C.Step(self.action_size,
                              action_ptr,
                              self.observation_size,
                              observation_ptr)
        # assert success
        return observation

    def render(self):
        pass # For now we won't be rendering anything
