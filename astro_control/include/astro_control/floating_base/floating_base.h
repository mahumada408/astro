#pragma once

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include "tf/transform_listener.h"

// Floating base model for a quadruped system. The model is defined as a floating base with leg enumeration as shown
// below. The forces applied to this model are only those imposed by gravity, and the reaction forces at the point
// of contact for each foot.
//
//        front
//  +-----------------+
//  |                 |
//  | 1             2 |
//  |                 |
//  |       ^ Bx      |
//  |       |         |
//  |       |   By    |
//  |       +--->     |
//  |                 |
//  |                 |
//  |                 |
//  |                 |
//  | 3             4 |
//  |                 |
//  +-----------------+

class FloatingBase {
 public:

  // State indices for floating base model.
  enum State {
    x = 0,
    y = 1, 
    z = 2, 
    x_dot = 3, 
    y_dot = 4, 
    z_dot = 5,
    roll = 6,
    pitch = 7,
    yaw = 8,
    roll_dot = 9,
    pitch_dot = 10,
    yaw_dot = 11,
    g = 12,
    state_count
  };

  // Control indices for floating base model.
  enum Control {
    // Front left leg.
    f1x = 0,
    f1y = 1,
    f1z = 2,
    f2x = 3,
    f2y = 4,
    f2z = 5,
    f3x = 6,
    f3y = 7,
    f3z = 8,
    f4x = 9,
    f4y = 10,
    f4z = 11,
    control_count
  };

  // Indices for the individual feet.
  enum Foot {
    fl = 0,
    fr = 1,
    rl = 2, 
    rr = 3,
    foot_count
  };

  FloatingBase() {}
  FloatingBase(double mass, double Ixx, double Iyy, double Izz);
  ~FloatingBase() {}

  void UpdateState();

  void SetFootPositions(Eigen::Vector3d foot_fl, Eigen::Vector3d foot_fr, Eigen::Vector3d foot_rl,
                       Eigen::Vector3d foot_rr);

  void SetRobotPose(tf::StampedTransform& robo_pose, geometry_msgs::Twist& robo_twist);

  // Update the continuous linear dynamics.
  // x_dot = Ax + Bu
  void UpdateDynamics();

  void GetDiscretizeDynamics(Eigen::Matrix<double, 13, 13>& A_discrete, Eigen::Matrix<double, 13, 12>& B_discrete);

  Eigen::Matrix<double, 13, 1> RobotState() { return robo_state_; }

  Eigen::Matrix<double, 13, 13> A() { return A_continuous_; }

  Eigen::Matrix<double, 13, 12> B() { return B_continuous_; }

  const std::vector<Eigen::Vector3d> foot_positions() { return foot_positions_; }

 private:
  // Form skew symmetric matrix for foot position.
  Eigen::Matrix3d SkewSymmetricFoot(Eigen::Vector3d foot_pos);

  // Matrix multiplication between inertia tensor and the position vector
  // from the CG to the foot location.
  Eigen::Matrix3d InertiaPos(Eigen::Matrix3d inertia, Eigen::Vector3d foot_pos);

  // Sets the position of the origin of the body frame in the world frame, expressed in the world frame.
  void SetRobotPosition(tf::Vector3& robo_pos);

  // Sets the orientation of the body frame in the world frame, expressed in the world frame.
  void SetOrientation(tf::Quaternion robo_quat);

  void SetRobotVelocities(geometry_msgs::Twist& robo_twist);

  // Mass of the robot.
  double mass_;

  // Robot inertia tensor in the quadruped body frame.
  double i_xx_;
  double i_yy_;
  double i_zz_;
  Eigen::Matrix3d inertia_;

  // Rotation matrix about the z axis (yaw).
  Eigen::Matrix3d r_yaw_;

  // Position of foot from the body cm expressed in the world frame.
  Eigen::Vector3d foot_fl_;
  Eigen::Vector3d foot_fr_;
  Eigen::Vector3d foot_rl_;
  Eigen::Vector3d foot_rr_;
  std::vector<Eigen::Vector3d> foot_positions_;
  int num_paws_ = 4;

  Eigen::Matrix<double, 13, 13> A_continuous_;
  Eigen::Matrix<double, 13, 12> B_continuous_;
  Eigen::Matrix<double, 13, 13> A_discrete_;
  Eigen::Matrix<double, 13, 12> B_discrete_;

  // State of the robot. Note that all state variables must be expressed in the world frame.
  Eigen::Matrix<double, 13, 1> robo_state_;
};
