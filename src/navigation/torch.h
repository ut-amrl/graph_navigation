#pragma once

// Undefine glog macros before including PyTorch to prevent conflicts
#ifdef LOG
  #undef LOG
#endif
#ifdef VLOG
  #undef VLOG
#endif
#ifdef LOG_IF
  #undef LOG_IF
#endif
#ifdef VLOG_IF
  #undef VLOG_IF
#endif
#ifdef CHECK
  #undef CHECK
#endif
#ifdef DCHECK
  #undef DCHECK
#endif
#ifdef VLOG_IS_ON
  #undef VLOG_IS_ON
#endif

#include <torch/script.h>
#include <torch/torch.h>

// Restore glog macros after PyTorch includes
#include <glog/logging.h>