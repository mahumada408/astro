#include <astro_control/floating_base/floating_base.h>

#include <math.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

FloatingBase::FloatingBase(double mass, double i_xx, double i_yy, double i_zz)
    : mass_(mass), i_xx_(i_xx), i_yy_(i_yy), i_zz_(i_zz) {
  // Set robot's body inertia.
  Eigen::Matrix<double, 3, 1> Id;
  Id << i_xx_, i_yy_, i_zz_;
  inertia_.diagonal() = Id;

  r_yaw_.setZero();

  A_continuous_.setZero();
  B_continuous_.setZero();
  robo_state_.setZero();
}

void FloatingBase::SetFootPositions(Eigen::Vector3d foot_fl, Eigen::Vector3d foot_fr, Eigen::Vector3d foot_rl,
                                   Eigen::Vector3d foot_rr) {
  foot_fl_ = foot_fl;
  foot_fr_ = foot_fr;
  foot_rl_ = foot_rl;
  foot_rr_ = foot_rr;

  foot_positions_.clear();
  foot_positions_.reserve(num_paws_);
  foot_positions_.push_back(foot_fl_);
  foot_positions_.push_back(foot_fr_);
  foot_positions_.push_back(foot_rl_);
  foot_positions_.push_back(foot_rr_);
}

void FloatingBase::SetRobotPosition(tf::Vector3& robo_pos) {
  robo_state_[3] = robo_pos.x();
  robo_state_[4] = robo_pos.y();
  robo_state_[5] = robo_pos.z();
}

void FloatingBase::SetOrientation(tf::Quaternion robo_quat) {
  tf::Matrix3x3 robot_pose(robo_quat);
  double roll, pitch, yaw;
  robot_pose.getRPY(roll, pitch, yaw);
  robo_state_[0] = roll;
  robo_state_[1] = pitch;
  robo_state_[2] = yaw;
}

void FloatingBase::SetRobotVelocities(geometry_msgs::Twist& robo_twist) {
  robo_state_[6] = robo_twist.linear.x;
  robo_state_[7] = robo_twist.linear.y;
  robo_state_[8] = robo_twist.linear.z;
  robo_state_[9] = robo_twist.angular.x;
  robo_state_[10] = robo_twist.angular.y;
  robo_state_[11] = robo_twist.angular.z;
}

void FloatingBase::SetRobotPose(tf::StampedTransform& robo_pose, geometry_msgs::Twist& robo_twist) {
  SetRobotPosition(robo_pose.getOrigin());
  SetOrientation(robo_pose.getRotation());
  SetRobotVelocities(robo_twist);

  // Update rotation matrix with yaw angle.
  r_yaw_ << cos(robo_state_[2]), -sin(robo_state_[2]), 0, -sin(robo_state_[2]), cos(robo_state_[2]), 0, 0, 0, 0;
}

Eigen::Matrix3d FloatingBase::SkewSymmetricFoot(Eigen::Vector3d foot_pos) {
  // Create a skew symmetric matrix from the foot position.
  // The skew matrix will have the following format:
  // 0, -z, y
  // z, 0, -x
  // -y, x, 0

  Eigen::Matrix3d skew_pos;
  skew_pos << 0, -foot_pos.z(), foot_pos.y(),
              foot_pos.z(), 0, -foot_pos.x(),
              -foot_pos.y(), foot_pos.x(), 0;
  return skew_pos;
}

Eigen::Matrix3d FloatingBase::InertiaPos(Eigen::Matrix3d inertia, Eigen::Vector3d foot_pos) {
  return inertia.inverse() * SkewSymmetricFoot(foot_pos);
}

void FloatingBase::UpdateDynamics() {
  // Get inertia in the global frame.
  Eigen::Matrix3d inertia_global{r_yaw_ * inertia_ * r_yaw_.transpose()};

  // Update the state space A matrix.
  // Zero it out just in case.
  A_continuous_.setZero();
  A_continuous_.block(0, 6, 3, 3) = Eigen::Matrix3d::Identity();
  A_continuous_.block(3, 9, 3, 3) = Eigen::Matrix3d::Identity();
  A_continuous_(8, 12) = 1.0;

  // Update the state space B matrix.
  // Zero it out just in case.
  B_continuous_.setZero();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < num_paws_; ++j) {
      // Rows 6 - 8 will have the mass goodness.
    B_continuous_.block(6 + i, j + (i * 3), 3, 3) = Eigen::Matrix3d::Identity() / mass_;
    }
    B_continuous_.block(9, i * 3, 3, 3) = InertiaPos(inertia_global, foot_positions_[i]);
  }
}

void FloatingBase::GetDiscretizeDynamics(Eigen::Matrix<double, 13, 13>& A_discrete,
                                         Eigen::Matrix<double, 13, 12>& B_discrete) {
  // Combine A and B matrix into one super 25x25 matrix.
  Eigen::Matrix<double, 25, 25> AB_continuous;
  Eigen::Matrix<double, 25, 25> exponent_matrix;
  AB_continuous.block(0, 0, 13, 13) = A_continuous_;
  AB_continuous.block(0, 13, 13, 12) = B_continuous_;
  AB_continuous = 0.030 * AB_continuous;  // 30 ms discretization.
  exponent_matrix = AB_continuous.exp();
  A_discrete = exponent_matrix.block(0, 0, 13, 13);
  B_discrete = exponent_matrix.block(0, 13, 13, 12);
}