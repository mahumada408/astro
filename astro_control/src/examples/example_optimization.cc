#include <ros/ros.h>

#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include <astro_control/floating_base/floating_base.h>

// clang-format off
#include <ct/core/systems/continuous_time/System.h>
#include <ct/core/systems/continuous_time/ControlledSystem.h>
#include <ct/core/systems/continuous_time/SecondOrderSystem.h>
#include <ct/core/systems/continuous_time/ControlledSystem.h>
#include <ct/core/systems/continuous_time/SwitchedControlledSystem.h>
#include <ct/core/systems/continuous_time/linear/LinearSystem.h>
#include <ct/core/systems/continuous_time/linear/SwitchedLinearSystem.h>
#include <ct/core/systems/continuous_time/linear/LTISystem.h>
// clang-format on

#include <ct/core/core.h>

#include "tf/transform_listener.h"

int main(int argc, char** argv) {
  std::cout << "Testing!" << std::endl;
  FloatingBase test_robot(20, 0.07538, 0.1611, 0.202);
  tf::Vector3 position(0, 0, 0);

  // Orientation corresponding to 45 degree rotation about the
  // z-axis.
  tf::Quaternion orientation(0.0, 0.0, 0.3826834, 0.9238795);
  tf::StampedTransform test_pose;

  // Set zero velocities.
  geometry_msgs::Twist test_twist;
  test_twist.linear.x = 0;
  test_twist.linear.y = 0;
  test_twist.linear.z = 0;
  test_twist.angular.x = 0;
  test_twist.angular.y = 0;
  test_twist.angular.z = 0;
  test_pose.setOrigin(position);
  test_pose.setRotation(orientation);

  test_robot.SetRobotPose(test_pose, test_twist);

  Eigen::Matrix<double, 13, 1> expected_orientation;
  expected_orientation.setZero();
  expected_orientation.z() = 0.78539816339;  // [rad]

  Eigen::Vector3d foot_fl, foot_fr, foot_rl, foot_rr;
  foot_fl << 0.133795, 0.131609, -0.169671;
  foot_fr << 0.133795, -0.131609, -0.169671;
  foot_rl << -0.133795, 0.131609, -0.169671;
  foot_rr << -0.133795, -0.131609, -0.169671;
  test_robot.SetFootPositions(foot_fl, foot_fr, foot_rl, foot_rr);

  // std::shared_ptr<LQOCProblem<state_dim, control_dim>>

  const size_t state_dim = 13;
  const size_t control_dim = 12;

  Eigen::Matrix<double, state_dim, 1> x0;
  Eigen::Matrix<double, control_dim, 1> u0;
  x0.setZero();
  u0.setZero();
  x0(12) = -9.81;
  u0(2) = 20 / 4;
  u0(5) = 20 / 4;
  u0(8) = 20 / 4;
  u0(11) = 20 / 4;

  std::shared_ptr<ct::core::LTISystem<state_dim, control_dim>> quadruped_system(
      new ct::core::LTISystem<state_dim, control_dim>(test_robot.A(), test_robot.B()));
  ct::core::SensitivityApproximation<state_dim, control_dim> discrete_quad(
        0.1, quadruped_system, ct::core::SensitivityApproximationSettings::APPROXIMATION::MATRIX_EXPONENTIAL);

  std::cout << "All done" << std::endl;
}