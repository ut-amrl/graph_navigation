import cffi
import time

ffi = cffi.FFI()
with open("scripts/graph_nav_rl_export.h") as h_file:
    ffi.cdef(h_file.read())

C = ffi.dlopen("./lib/libgraph_nav_rl.so")

def hello(name):
    C.Init()
    p = ffi.new("char[]", b"hello world!")
    x = C.Test(p)
    print(x)


if __name__ == "__main__":
    hello("hello cffi")
    for i in range(100):
        # Sleep for 0.1 seconds
        time.sleep(0.1)
        print(i)
        C.Step()
