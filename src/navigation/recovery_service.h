#pragma once

#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "navigation_parameters.h"
#include "motion_primitives.h"  // for the base classes, etc.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <amrl_msgs/action/recovery_execute.hpp>  // from ament-based amrl_msgs
#include <amrl_msgs/srv/recovery_detect_srv.hpp>  // from ament-based amrl_msgs
#include <Eigen/Core>
#include <boost/circular_buffer.hpp>

enum class FailureStatus {
  False,
  True,
  Uncertain
};

class RecoveryServiceBase {
public:
  RecoveryServiceBase() = default;
  ~RecoveryServiceBase() = default;

  virtual void Update(std::shared_ptr<motion_primitives::PathRolloutBase> best_path) {
    std::unique_lock<std::mutex> lock(mutex_);
    best_path_ = best_path; 
  }

  /// @brief Detect if robot is stuck
  /// @return The last known stuck status (blocks if none yet known).
  virtual FailureStatus DetectFailure(bool do_recovery) = 0;

  virtual void ResetRecovery() = 0;

  /// @brief Attempt to recover if stuck
  virtual void Recover() = 0;

  virtual bool IsRecoveryInProgress() = 0; 

  virtual bool IsRecoveryTerminated() = 0;

protected:
  boost::circular_buffer<bool> detection_buffer_;
  FailureStatus failure_status_{FailureStatus::False};
  std::shared_ptr<motion_primitives::PathRolloutBase> best_path_;

  //=============================
  // Synchronization
  //=============================
  std::mutex mutex_;

};

/// @brief Concrete class implementing asynchronous calls to
///        (1) FailureDetectionService,
///        (2) RecoveryService,
///        (3) AbortRecoveryService.
///
/// On each call:
///  - If an outgoing request is in progress, do not start a new one.
///  - Return or wait for the last known result (if it doesn't exist yet, wait).
class RecoveryService : public RecoveryServiceBase {
public:
  explicit RecoveryService(navigation::NavigationParameters& params);
  ~RecoveryService();

  /// @brief Check if the robot is stuck. If no request is ongoing, starts one.
  ///        If there is already a request in progress, returns the last known result.
  ///        If no last-known result is available yet (first time), waits for it.
  /// @return True if stuck, false otherwise.
  FailureStatus DetectFailure(bool do_recovery) override;

  /// @brief If is_recovery_needed == false, calls AbortRecoveryService(), else CallRecoveryService().
  void Recover() override;

  void ResetRecovery() override {
    std::lock_guard<std::mutex> lock(mutex_);
    recovery_terminated_ = false;
    recovery_in_progress_ = false;
    failure_detection_in_progress_ = false;
    detection_buffer_.clear();
  }

  /// @brief Asynchronously abort the recovery if none is in progress. If a call is
  ///        in progress, does nothing.  The last call's success/failure is stored.
  void AbortRecoveryService();

  bool IsRecoveryInProgress() override {
    std::lock_guard<std::mutex> lock(mutex_);
    return recovery_in_progress_;
  }

  bool IsRecoveryTerminated() {
    std::lock_guard<std::mutex> lock(mutex_);
    return recovery_terminated_;
  }

private:
  navigation::NavigationParameters params_;

  //=============================
  // Failure detection variables
  //=============================
  bool failure_detection_in_progress_{false};

  //=============================
  // Recovery call variables
  //=============================
  bool recovery_in_progress_{false};
  bool recovery_terminated_{false};
  bool last_recovery_result_{false};            // e.g. success/fail

  //=============================
  // Abort call variables
  //=============================
  bool abort_in_progress_{false};

  rclcpp::Executor::SharedPtr exec_;
  std::thread spin_thread_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<amrl_msgs::srv::RecoveryDetectSrv>::SharedPtr detect_client_;
  rclcpp_action::Client<amrl_msgs::action::RecoveryExecute>::SharedPtr recovery_client_;
  rclcpp_action::ClientGoalHandle<amrl_msgs::action::RecoveryExecute>::SharedPtr goal_handle_;
  
  //=============================
  // Internal Worker Threads
  //=============================
  /// @brief Actually calls the "FailureDetectionService" (ROS).
  ///        On success/failure, updates last_failure_detection_result_ and
  ///        last_failure_detection_result_set_.
  void FailureDetectionRequestThread();

  /// @brief Actually calls the "AbortRecoveryService" (ROS).
  void AbortRecoveryRequestThread();

  /// @brief Actually calls the "RecoveryService" (ROS).
  void RecoveryRequestThread();
};