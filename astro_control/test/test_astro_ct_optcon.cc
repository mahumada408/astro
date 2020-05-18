#include <ros/ros.h>

#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include <astro_control/floating_base/floating_base.h>
// #include <ct/models/InvertedPendulum/InvertedPendulum.h>

#include <ct/core/systems/continuous_time/System.h>
#include <ct/core/systems/continuous_time/ControlledSystem.h>
#include <ct/core/systems/continuous_time/SecondOrderSystem.h>
#include <ct/core/systems/continuous_time/ControlledSystem.h>
#include <ct/core/systems/continuous_time/SwitchedControlledSystem.h>
#include <ct/core/systems/continuous_time/linear/LinearSystem.h>
#include <ct/core/systems/continuous_time/linear/SwitchedLinearSystem.h>
#include <ct/core/systems/continuous_time/linear/LTISystem.h>

#include "tf/transform_listener.h"

// Declare a test
TEST(TestOpt, testOptCon)
{
    std::cout << "Testing!" << std::endl;
    FloatingBase test_robot(20, 0.07538, 0.1611, 0.202);
    tf::Vector3 position(0,0,0);
    
    // Orientation corresponding to 45 degree rotation about the
    // z-axis.
    tf::Quaternion orientation(0.0, 0.0, 0.3826834,0.9238795);
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
    Eigen::Matrix<double, 13, 1> robot_state = test_robot.RobotState();

    Eigen::Matrix<double, 13, 1> expected_orientation;
    expected_orientation.setZero();
    expected_orientation.z() = 0.78539816339; // [rad]

    ASSERT_TRUE(robot_state.isApprox(expected_orientation, 1e-5)) << "failed the test of champions";

    Eigen::Vector3d foot_fl, foot_fr, foot_rl, foot_rr;
    foot_fl << 0.133795, 0.131609, -0.169671;
    foot_fr << 0.133795, -0.131609, -0.169671;
    foot_rl << -0.133795, 0.131609, -0.169671;
    foot_rr << -0.133795, -0.131609, -0.169671;
    test_robot.SetFootPositions(foot_fl, foot_fr, foot_rl, foot_rr);

    std::cout << "All done" << std::endl;
    
}


// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");
    
    testing::InitGoogleTest(&argc, argv);
    
    std::thread t([]{while(ros::ok()) ros::spin();});
    
    auto res = RUN_ALL_TESTS();
    
    ros::shutdown();
    
    return res;
}