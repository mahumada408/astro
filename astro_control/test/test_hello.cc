#include <ros/ros.h>

#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include <astro_control/floating_base/floating_base.h>

#include "tf/transform_listener.h"

// Declare a test
TEST(TestSuite, testCase1)
{
    FloatingBase test_robot;
    tf::Vector3 position(0,0,0);
    tf::Quaternion orientation(0.0,0.1305262,0.0,0.9914449);
    tf::StampedTransform test_pose;

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

    std::cout << robot_state << std::endl;

    EXPECT_EQ(1, 1) << "failed the test of champions";
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