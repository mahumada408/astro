#pragma once

#include <eigen3/Eigen/Dense>


class FloatingBase {
    public:
        FloatingBase();
        ~FloatingBase() {}

        void SetQuadDynamics(double yaw_rad);

        void UpdateState();

        void SetFootPosition(Eigen::Vector3d foot_fl, Eigen::Vector3d foot_fr, Eigen::Vector3d foot_rl, Eigen::Vector3d foot_rr);

        // Update the continuous linear dynamics.
        // x_dot = Ax + Bu
        // @param inertia Quadruped's inertia in the global coordinate frame. 
        // @param mass Mass of the Quadruped.
        // @param r_yaw Rotation matrix about the z axis.
        void UpdateDynamics();
    private:
        // Initialize robot data members.
        void Initialize();

        // Matrix multiplication between inertia tensor and the position vector
        // from the CG to the foot location.
        Eigen::Matrix3d InertiaPos(Eigen::Matrix3d inertia, Eigen::Vector3d foot_pos);

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

        Eigen::Vector3d state_;
        Eigen::Matrix<double, 13, 13> A_continuous_;
        Eigen::Matrix<double, 13, 12> B_continuous_;
};
