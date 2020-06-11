#pragma once

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>

#include <astro_control/floating_base/floating_base.h>

class GaitGenerator {
  public:
    enum Foot { fl, fr, bl, br, foot_count };
    struct FootState {
      bool in_contact = false; // Is the foot in contact or swing (true if in contact).
      double time_along_state = 0.0; // Time foot has been in the current state (in contact or swing).
    };

    GaitGenerator(FloatingBase& robot_model);

    void UpdateFeetState(ros::Time current_time, FloatingBase& robot_model);

    std::vector<FootState> GetFeetState() { return feet_state_; }
  private:
    std::vector<FootState> feet_state_;
    std::vector<FootState> previous_feet_state_;
    ros::Time previous_update_time_;
};