#include <astro_control/gait_generator/gait_generator.h>

GaitGenerator::GaitGenerator(FloatingBase& robot_model) {
  // Zero out previous foot state.
  previous_feet_state_.reserve(Foot::foot_count);

  for (int i = 0; i < Foot::foot_count; ++i) {
    // Update contact state.
    FootState foot_state;
    if (robot_model.foot_positions()[i].z() <= 0) {
      foot_state.in_contact = true;
    } else {
      foot_state.in_contact = false;
    }
    previous_feet_state_.push_back(foot_state);
  }
}

void GaitGenerator::UpdateFeetState(ros::Time current_time, FloatingBase& robot_model) {
  // Check if any foot is in contact.
  // If foot is below the 0 plane, assume it's in contact.
  feet_state_.clear();
  feet_state_.reserve(Foot::foot_count);

  for (int i = 0; i < Foot::foot_count; ++i) {
    // Update contact state.
    FootState foot_state;
    if (robot_model.foot_positions()[i].z() <= 0) {
      foot_state.in_contact = true;
    } else {
      foot_state.in_contact = false;
    }

    // Update time along by looking at the previous state.
    if (foot_state.in_contact == previous_feet_state_[i].in_contact) {
      foot_state.time_along_state = (current_time - previous_update_time_).toSec();
    } else {
      foot_state.time_along_state = 0.0;
    }

    feet_state_.push_back(foot_state);
  }

  previous_update_time_ = current_time;
}