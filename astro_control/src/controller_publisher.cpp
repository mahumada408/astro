#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/TransformStamped.h>

#include "qpOASES/include/qpOASES.hpp"
#include "floating_base/floating_base.h"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_publisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/joint_position_controller/command", 1000);
  ros::Rate loop_rate(100);

  FloatingBase robot_model;
  Eigen::Vector3d foot_fl;
  Eigen::Vector3d foot_fr;
  Eigen::Vector3d foot_rl;
  Eigen::Vector3d foot_rr;

  tf::TransformListener listener;

  double count = 0;
  double joint_angle = 0;
  while (ros::ok())
  {
    std_msgs::Float64MultiArray msg;

    for (int i = 0; i <= 11; i++) {
      joint_angle = std::sin(count/100);
      msg.data.push_back(joint_angle);
    }

    tf::StampedTransform transformStamped;
    try{
      // Get body frame's state expressed in the world frame.
      listener.lookupTransform("/base_link", "/world",
                               ros::Time(0), transformStamped);
      transformStamped.getRotation();
      tf::Matrix3x3 robot_pose(transformStamped.getRotation());
      double roll, pitch, yaw;
      robot_pose.getRPY(roll, pitch, yaw);
      ROS_INFO("roll: %f", roll);
      ROS_INFO("pitch: %f", pitch);
      ROS_INFO("yaw: %f", yaw);

      // Get the position vector from the body's cm to toe_0 expressed in the body frame.
      listener.lookupTransform("/toe_0", "/base_link",
                               ros::Time(0), transformStamped);
      // Transform the tfVector3 to an Eigen Vector3d.
      tf::vectorTFToEigen(transformStamped.getOrigin(), foot_fl);
      // Repeat for the remaining feet.
      listener.lookupTransform("/toe_1", "/base_link",
                               ros::Time(0), transformStamped);
      tf::vectorTFToEigen(transformStamped.getOrigin(), foot_fr);
      listener.lookupTransform("/toe_2", "/base_link",
                               ros::Time(0), transformStamped);
      tf::vectorTFToEigen(transformStamped.getOrigin(), foot_rr);
      listener.lookupTransform("/toe_3", "/base_link",
                               ros::Time(0), transformStamped);
      tf::vectorTFToEigen(transformStamped.getOrigin(), foot_rl);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    robot_model.SetFootPosition(foot_fl, foot_fr, foot_rl, foot_rr);

    ROS_INFO("x: %f", foot_fl.x());
    ROS_INFO("y: %f", foot_fl.y());
    ROS_INFO("z: %f", foot_fl.z());

    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
