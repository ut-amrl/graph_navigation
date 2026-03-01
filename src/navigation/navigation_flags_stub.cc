// Stub that defines gflags variables used by navigation.cc but normally
// defined in navigation_main.cc. This file is compiled into navigation_tests
// so that linking navigation_lib doesn't fail with undefined gflags refs.
#include <gflags/gflags.h>
DEFINE_double(min_ang_toc_sample_length, 1.0,
              "Minimum sample length required to enable angular time-optimal control");
