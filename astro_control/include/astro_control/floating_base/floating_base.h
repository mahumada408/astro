#pragma once

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#include "tf/transform_listener.h"


class FloatingBase {
    public:
        FloatingBase();
        ~FloatingBase() {}

        void UpdateState();

        void SetFootPosition(Eigen::Vector3d foot_fl, Eigen::Vector3d foot_fr, Eigen::Vector3d foot_rl, Eigen::Vector3d foot_rr);

        void SetRobotPose(tf::StampedTransform& robo_pose, geometry_msgs::Twist& robo_twist);

        // Update the continuous linear dynamics.
        // x_dot = Ax + Bu
        void UpdateDynamics();

        void GetDiscretizeDynamics(Eigen::Matrix<double, 13, 13>& A_discrete, Eigen::Matrix<double, 13, 12>& B_discrete);

        Eigen::Matrix<double, 13, 1> RobotState() { return robo_state_; }
    private:
        // Initialize robot data members.
        void Initialize();

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

        // Position of foot from the body cm expressed in the body frame.
        Eigen::Vector3d foot_fl_;
        Eigen::Vector3d foot_fr_;
        Eigen::Vector3d foot_rl_;
        Eigen::Vector3d foot_rr_;

        Eigen::Matrix<double, 13, 13> A_continuous_;
        Eigen::Matrix<double, 13, 12> B_continuous_;
        Eigen::Matrix<double, 13, 13> A_discrete_;
        Eigen::Matrix<double, 13, 12> B_discrete_;

        // State of the robot.
        Eigen::Matrix<double, 13, 1> robo_state_;
};
