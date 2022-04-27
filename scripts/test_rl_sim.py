import cffi
import time
import numpy as np

ffi = cffi.FFI()
with open("scripts/graph_nav_rl_export.h") as h_file:
    ffi.cdef(h_file.read())

C = ffi.dlopen("./lib/libgraph_nav_rl.so")


if __name__ == "__main__":
  print("Initializing simulator...")
  C.Init()
  print("Simulator initialized!")
  for i in range(1000):
    # time.sleep(0.002)
    action_size = 2
    action = 42 * np.ones(action_size, dtype = np.float64)
    action_ptr = ffi.cast("double *", action.ctypes.data)
    observation_size = 41
    observation = np.zeros(observation_size, dtype = np.float64)
    observation_ptr = ffi.cast("double *", observation.ctypes.data)
    success = C.Step(action_size, action_ptr, observation_size, observation_ptr)
    print('Simulator step: {} success: {}'.format(i, success))
