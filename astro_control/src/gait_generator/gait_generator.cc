#include <astro_control/gait_generator/gait_generator.h>

GaitGenerator::GaitGenerator(FloatingBase& robot_model) {
  // Zero out previous foot state.
  previous_feet_state_.reserve(FloatingBase::Foot::foot_count);

  for (int i = 0; i < FloatingBase::Foot::foot_count; ++i) {
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
  feet_state_.reserve(FloatingBase::Foot::foot_count);

  for (int i = 0; i < FloatingBase::Foot::foot_count; ++i) {
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

std::vector<GaitGenerator::HorizonStep> GaitGenerator::GenerateGait(ros::Time current_time) {
  std::vector<HorizonStep> gait_horizon;
  double horizon_time = 1.0;
  double dt = 0.05;
  int horizon_size = horizon_time / dt;
  gait_horizon.reserve(horizon_size);

  // Seed the initial configuration for that first step.
  if (first_step_ == true) {
    feet_state_[FloatingBase::Foot::fl].in_contact = true;
    feet_state_[FloatingBase::Foot::fr].in_contact = false;
    feet_state_[FloatingBase::Foot::rl].in_contact = false;
    feet_state_[FloatingBase::Foot::rr].in_contact = true;
    first_step_ = false;
  }

  HorizonStep foot_gait;
  for (int i = 0; i < horizon_size; ++i) {
    if (i == 0) {
      for (size_t j = 0; j < feet_state_.size(); ++j) {
        if (feet_state_[FloatingBase::Foot::fl].time_along_state >= 0.25) {
          foot_gait.foot_states[j].in_contact = !feet_state_[j].in_contact;
          foot_gait.foot_states[j].time_along_state = 0.0;
        } else {
          foot_gait.foot_states[j].in_contact = feet_state_[j].in_contact;
          foot_gait.foot_states[j].time_along_state = feet_state_[j].time_along_state + dt;
        }
      }
      foot_gait.time = 0.0;
    } else {
        // look at the previous time in contact. 
        // update the contact state on that.
        for (size_t j = 0; j < foot_gait.foot_states.size(); ++j) {
          if (gait_horizon[i - 1].foot_states[FloatingBase::Foot::fl].time_along_state >= 0.25) {
            foot_gait.foot_states[j].in_contact = !gait_horizon[i - 1].foot_states[j].in_contact;
            foot_gait.foot_states[j].time_along_state = 0.0;
          } else {
            foot_gait.foot_states[j].time_along_state = gait_horizon[i - 1].foot_states[j].time_along_state + dt;
          }
        }
        foot_gait.time = gait_horizon[i - 1].time + dt;
    }

    gait_horizon.push_back(foot_gait);
  }

  return gait_horizon;
}