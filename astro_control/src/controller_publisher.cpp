#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "qpOASES/include/qpOASES.hpp"
#include "floating_base/floating_base.h"

#include <cmath>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_publisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/astro/joint_position_controller/command", 1000);
  ros::Rate loop_rate(100);

  FloatingBase model;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  double count = 0;
  double joint_angle = 0;
  while (ros::ok())
  {
    std_msgs::Float64MultiArray msg;

    for (int i = 0; i <= 11; i++) {
      joint_angle = std::sin(count/100);
      msg.data.push_back(joint_angle);
    }

    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("toe_2", "base_link",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    std::cout << transformStamped.transform.translation.x << std::endl;;

    // msg.data = std::sin(count/100);
    // ROS_INFO("%f", msg.data);
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
