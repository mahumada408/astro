#pragma once

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <vector>

#include <astro_control/floating_base/floating_base.h>

class GaitGenerator {
  public:
    enum GaitType {
      Trot
    };

    struct FootState {
      bool in_contact = false; // Is the foot in contact or swing (true if in contact).
      double time_along_state = 0.0; // Time foot has been in the current state (in contact or swing).
    };

    struct HorizonStep {
      std::vector<FootState> foot_states = std::vector<FootState>(4);
      double time = 0.0;
    };

    GaitGenerator(FloatingBase& robot_model);

    // Updated the contact state of each of the feet.
    //
    // current_time The current ROS time.
    // robot_model  Floating base model of the robot.
    void UpdateFeetState(ros::Time current_time, FloatingBase& robot_model);

    // Generates the robot gait.
    //
    std::vector<HorizonStep> GenerateGait(ros::Time current_time);

    std::vector<FootState> GetFeetState() { return feet_state_; }
  private:
    // Current configuration of each foot.
    std::vector<FootState> feet_state_;

    // Previous configuration of each foot.
    std::vector<FootState> previous_feet_state_;

    // Previous update time.
    ros::Time previous_update_time_;

    // Gait horizon.
    std::vector<HorizonStep> gait_horizon_;

    bool first_step_ = true;
};