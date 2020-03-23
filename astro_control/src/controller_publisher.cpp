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
#include <astro_control/floating_base/floating_base.h>

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
  Eigen::Matrix<double, 13, 13> A_discrete;
  Eigen::Matrix<double, 13, 12> B_discrete;

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
    geometry_msgs::Twist robot_twist;
    try{
      // Get body frame's state expressed in the world frame.
      listener.lookupTransform("/base_link", "/world",
                               ros::Time(0), transformStamped);
      listener.lookupTwist("/base_link", "/world",
                               ros::Time(0), ros::Duration(0.001), robot_twist);
      robot_model.SetRobotPose(transformStamped, robot_twist);

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

    int horizon = 10; // Horizon size of 10 with a discretization of 30ms equates to a horizon of 300ms.

    // Let's start with the trajectory.
    // First pass will be trotting in place.
    Eigen::Matrix<double, Eigen::Dynamic, 1> x_d;
    x_d.resize(13*horizon, Eigen::NoChange);
    x_d.setZero();
    x_d[2] = 0.2; // Z body height of 200mm from the ground.
    x_d.replicate(13*horizon, 1);

    // Update robot model.
    robot_model.SetFootPosition(foot_fl, foot_fr, foot_rl, foot_rr);
    robot_model.UpdateDynamics();
    robot_model.GetDiscretizeDynamics(A_discrete, B_discrete);

    Eigen::Matrix<double,13,13> powerMats[20];

    // Set up the sparse matrix formulation.
    Eigen::Matrix<double, Eigen::Dynamic, 13> A_qp;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> B_qp;
    A_qp.resize(13*horizon, Eigen::NoChange);
    B_qp.resize(13*horizon, 12*horizon);
    powerMats[0].setIdentity();
    for(int i = 1; i < horizon+1; i++) {
      powerMats[i] = A_discrete * powerMats[i-1];
    }

    for(int r = 0; r < horizon; r++) {
      A_qp.block(13*r,0,13,13) = powerMats[r+1];//Adt.pow(r+1);
      for(int c = 0; c < horizon; c++) {
        if(r >= c) {
          int a_num = r-c;
          B_qp.block(13*r,12*c,13,12) = powerMats[a_num] /*Adt.pow(a_num)*/ * B_discrete;
        }
      }
    }

    // Weights for the optimization cost function.
    Eigen::Matrix<double, 13, 1> Q;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> S;
    Q.setZero();
    Q << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2, 0;
    S.resize(13*horizon, 13*horizon);
    S.diagonal() = Q.replicate(horizon,1);

    Eigen::Matrix<double, 13, 1> x_0 = robot_model.RobotState();


    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> qH;
    Eigen::Matrix<double,Eigen::Dynamic,1> qg;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> eye_12h;
    qH.resize(12*horizon, 12*horizon);
    qg.resize(12*horizon, Eigen::NoChange);
    eye_12h.resize(12*horizon, 12*horizon);
    eye_12h.setIdentity();
    qH = 2*(B_qp.transpose()*S*B_qp + 1e-6*eye_12h);
    qg = 2*B_qp.transpose()*S*(A_qp*x_0 - x_d);

    Eigen::Matrix<double,Eigen::Dynamic,1> U_b;
    U_b.resize(20*horizon, Eigen::NoChange);
    Eigen::Matrix<double,Eigen::Dynamic,1> l_b;
    l_b.resize(20*horizon, Eigen::NoChange);
    l_b.setZero();

    int k = 0;
    for(int i = 0; i < horizon; i++) {
      for(int j = 0; j < 4; j++) {
        U_b(5*k + 0) = 4000;
        U_b(5*k + 1) = 4000;
        U_b(5*k + 2) = 4000;
        U_b(5*k + 3) = 4000;
        U_b(5*k + 4) = 120;
        k++;
      }
    }

    int num_constraints = 20*horizon;
    int num_variables = 12*horizon;

    qpOASES::int_t nWSR = 100;
    qpOASES::QProblem problem_red (num_variables, num_constraints);
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_HIGH;
    problem_red.setOptions(op);
    struct timeval t1, t2;
    int elapsed_time;
    gettimeofday(&t1,NULL);
    int rval = problem_red.init(qH.data(), qg.data(), A_qp.data(), NULL, NULL, l_b.data(), U_b.data(), nWSR);

    

    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
