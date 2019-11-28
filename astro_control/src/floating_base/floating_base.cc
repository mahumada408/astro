#include "floating_base.h"

#include <math.h>

FloatingBase::FloatingBase() {
    Initialize();
}

void FloatingBase::Initialize() {
    mass_ = 9; // 9 kg robot.
    i_xx_ = 0.07; // [kg * m^2]
    i_yy_ = 0.26; // [kg * m^2]
    i_zz_ = 0.242; // [kg * m^2]
    // Set robot's body inertia (acquired from CAD).
    Eigen::Matrix<double,3,1> Id;
    Id << i_xx_, i_yy_, i_zz_;
    inertia_.diagonal() = Id;

    r_yaw_.setZero();

    state_.setZero();
    A_continuous_.setZero();
    B_continuous_.setZero();
}

void FloatingBase::SetFootPosition(Eigen::Vector3d foot_fl, Eigen::Vector3d foot_fr, Eigen::Vector3d foot_rl, Eigen::Vector3d foot_rr) {
    foot_fl_ = foot_fl;
    foot_fr_ = foot_fr;
    foot_rl_ = foot_rl;
    foot_rr_ = foot_rr;
}

Eigen::Matrix3d FloatingBase::InertiaPos(Eigen::Matrix3d inertia, Eigen::Vector3d foot_pos) {
    // Create a skew symmetric matrix from the foot position.
    // The skew matrix will have the following format:
    // 0, -z, y
    // z, 0, -x
    // -y, x, 0

    Eigen::Matrix3d skew_pos;
    skew_pos << 0, -foot_pos.z(), foot_pos.y(), 
                foot_pos.z(), 0, -foot_pos.x(),
                -foot_pos.y(), foot_pos.x(), 0;
    return inertia.inverse() * skew_pos;
}

void FloatingBase::SetQuadDynamics(double yaw_rad) {
    r_yaw_ << cos(yaw_rad), -sin(yaw_rad), 0,
            -sin(yaw_rad), cos(yaw_rad), 0,
            0, 0, 0;
}

void FloatingBase::UpdateDynamics() {
    // Get inertia in the global frame.
    Eigen::Matrix3d inertia_global{r_yaw_ * inertia_ * r_yaw_.transpose()};

    // Update the state space A matrix.
    // Zero it out just in case.
    A_continuous_.setZero();
    A_continuous_.block(3, 9, 3, 3) = Eigen::Matrix3d::Identity();
    A_continuous_(11, 12) = 1.0;
    A_continuous_.block(0, 6, 3, 3) = r_yaw_;

    // Update the state space B matrix. 
    // Zero it out just in case.
    B_continuous_.setZero();

    Eigen::Vector3d foot_pos;
    foot_pos.setZero();

    for (int i = 0; i <=3; i++) {
        // Rows 6 - 8 will have the inertia goodness.
        B_continuous_.block(6, i*3, 3, 3) = InertiaPos(inertia_global, foot_pos);

        // Rows 9 - 11 will have the mass goodness.
        B_continuous_.block(9, i*3, 3, 3) = Eigen::Matrix3d::Identity() / mass_;
    }
}