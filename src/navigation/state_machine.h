#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdexcept>   // for std::runtime_error
#include <string>      // for std::string

#include "navigation_flags.h"

namespace navigation {

enum class NavigationState {
  kInitialize = 0,
  kHalt = 1,
  kRun = 2,
  kTurnInPlace = 3,
  kRecovery = 4,
  // deprecated
  kStopped = 5,
  kPaused = 6,
  kGoto = 7,
  // kTurnInPlace = 3,
  kOverride = 8
};

enum class StateConditions {
  kIsInitialized = 0,
  kIsGlobalPathValid = 1,
  kIsGoalAvailable = 2,
  kIsGoalReached = 3,
  kIsGoalInFOV = 4,
  kIsRecoveryNeeded = 5,
  kIsFailureDetectionUncertain = 6, // this is an augmentation state for kIsRecoveryNeeded
};

class StateMachine {
  public:
    StateMachine() {}
    ~StateMachine() {}

    void Initialize() {
      state_ = NavigationState::kInitialize;
    }

    NavigationState GetState() { // Getter
      return state_;
    }

    std::string GetStateString() {
      switch (state_) {
        case NavigationState::kInitialize:
          return "kInitialize";
        case NavigationState::kHalt:
          return "kHalt";
        case NavigationState::kRun:
          return "kRun";
        case NavigationState::kTurnInPlace:
          return "kTurnInPlace";
        case NavigationState::kRecovery:
          return "kRecovery";
        default:
          throw std::runtime_error("Invalid state");
      }

      return "Unknown state";
    }

    void SetState(StateConditions keys, bool value) { // Setter
      switch (keys) {
        case StateConditions::kIsInitialized:
          isInitialized_ = value;
          break;
        case StateConditions::kIsGlobalPathValid:
          isGlobalPathValid_ = value;
          break;
        case StateConditions::kIsGoalAvailable:
          isGoalAvailable_ = value;
          break;
        case StateConditions::kIsGoalReached:
          isGoalReached_ = value;
          break;
        case StateConditions::kIsGoalInFOV:
          isGoalInFOV_ = value;
          break;
        case StateConditions::kIsRecoveryNeeded:
          isRecoveryNeeded_ = value;
          break;
        case StateConditions::kIsFailureDetectionUncertain:
          isFailureDetectionUncertain_ = value;
          break;
        default:
          break;
      }
    }

    void TransitionState() { // Updates
      static bool kDebug = FLAGS_v > 2;

      if (kDebug) {
        printf("TransitionState() isInitialized: %d isGlobalPathValid: %d isGoalAvailable: %d isGoalReached: %d isGoalInFOV: %d isRecoveryNeeded: %d\n",
               isInitialized_, isGlobalPathValid_, isGoalAvailable_, isGoalReached_, isGoalInFOV_, isRecoveryNeeded_);
      }

      NavigationState prev_state;
      do {
        prev_state = state_;
        bool shouldHalt = !isInitialized_ || !isGlobalPathValid_ || !isGoalAvailable_ || isGoalReached_;
        switch (state_) {
          case NavigationState::kInitialize:
            if (isInitialized_) {
              state_ = NavigationState::kHalt;
            }
            break;
          case NavigationState::kHalt:
            if (!isInitialized_) {
              state_ = NavigationState::kInitialize;
            } else if (!isGlobalPathValid_ || !isGoalAvailable_ || isGoalReached_ || isFailureDetectionUncertain_) {
              state_ = NavigationState::kHalt;
            } else if (!isGoalInFOV_ && !isRecoveryNeeded_) {
              state_ = NavigationState::kTurnInPlace;
            } else if (isGoalInFOV_ && !isRecoveryNeeded_) {
              state_ = NavigationState::kRun;
            } else if (isRecoveryNeeded_) {
              state_ = NavigationState::kRecovery;
            }
            break;
          case NavigationState::kRun:
            if (shouldHalt || isRecoveryNeeded_) {
              state_ = NavigationState::kHalt;
            } else if (!isGoalInFOV_) {
              state_ = NavigationState::kTurnInPlace;
            }
            break;
          case NavigationState::kTurnInPlace:
            if (shouldHalt) {
              state_ = NavigationState::kHalt;
            } else if (isGoalInFOV_) {
              state_ = NavigationState::kRun;
            }
            break;
          case NavigationState::kRecovery:
            if (shouldHalt || !isRecoveryNeeded_) {
              state_ = NavigationState::kHalt;
            }
            break;
          default:
            break;
        }
      } while (prev_state != state_);

      if (kDebug) {
        printf("TransitionState() state: %d\n", static_cast<int>(state_));
      }
    }

  private:
    NavigationState state_;
    // StateVariables state_vars_;
    bool isInitialized_     = false;
    bool isGlobalPathValid_ = false;
    bool isGoalAvailable_   = false;
    bool isGoalReached_     = false;
    bool isGoalInFOV_       = false;
    bool isRecoveryNeeded_  = false;
    bool isFailureDetectionUncertain_ = false;
};

}

#endif  // STATE_MACHINE_H