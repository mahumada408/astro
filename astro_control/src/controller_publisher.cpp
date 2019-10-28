#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#include <cmath>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/astro/joint_position_controller/command", 1000);
  ros::Rate loop_rate(100);

  double count = 0;
  double joint_angle = 0;
  while (ros::ok())
  {
    std_msgs::Float64MultiArray msg;

    for (int i = 0; i <= 11; i++) {
      if (i == 0 || i == 3 || i == 6 || i == 9) {
        joint_angle = 0;
      } else {
        if (i == 4 || i == 7) {
          // knees.
          // Negative for opposite side of astro.
          joint_angle = -std::sin(count/100);
        } else {
          joint_angle = std::sin(count/100);
        }
      }
      msg.data.push_back(joint_angle);
    }

    // msg.data = std::sin(count/100);
    // ROS_INFO("%f", msg.data);
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
