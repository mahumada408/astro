#include <ros/ros.h>

#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include <astro_control/floating_base/floating_base.h>
#include <astro_control/gait_generator/gait_generator.h>

// Declare a test
TEST(TestSuite, testCase1)
{
    std::cout << "Testing!" << std::endl;
    ros::Time beginning_of_time(0.0);

    double mass = 20;     // [kg]
    double ixx = 0.07538; // [kg * m^2]
    double iyy = 0.1611;  // [kg * m^2]
    double izz = 0.202;   // [kg * m^2]
    FloatingBase test_robot(mass, ixx, iyy, izz);

    Eigen::Vector3d test_foot_position;
    test_foot_position << 0.133795, 0.131609, 0.169671;

    Eigen::Vector3d foot_fl, foot_fr, foot_rl, foot_rr;
    foot_fl << test_foot_position.x(), test_foot_position.y(), -test_foot_position.z();
    foot_fr << test_foot_position.x(), -test_foot_position.y(), -test_foot_position.z();
    foot_rl << -test_foot_position.x(), test_foot_position.y(), -test_foot_position.z();
    foot_rr << -test_foot_position.x(), -test_foot_position.y(), -test_foot_position.z();
    test_robot.SetFootPositions(foot_fl, foot_fr, foot_rl, foot_rr);

    GaitGenerator test_generator(test_robot);
    test_generator.UpdateFeetState(beginning_of_time, test_robot);

    double expected_time_in_contact = 0.0;

    for (const auto& state : test_generator.GetFeetState()) {
      EXPECT_TRUE(state.in_contact == true);
      EXPECT_TRUE(state.time_along_state == expected_time_in_contact);
    }

    expected_time_in_contact = 0.1;
    beginning_of_time += ros::Duration(expected_time_in_contact);
    foot_fl.z() += -foot_fl.z() + 0.1;
    test_robot.SetFootPositions(foot_fl, foot_fr, foot_rl, foot_rr);
    test_generator.UpdateFeetState(beginning_of_time, test_robot);

    for (size_t i = 0; i < test_generator.GetFeetState().size(); ++i) {
      if (i == FloatingBase::Foot::fl) {
        EXPECT_TRUE(test_generator.GetFeetState()[i].in_contact == false);
        EXPECT_TRUE(test_generator.GetFeetState()[i].time_along_state == 0.0);
      } else {
        EXPECT_TRUE(test_generator.GetFeetState()[i].in_contact == true);
        EXPECT_TRUE(test_generator.GetFeetState()[i].time_along_state == expected_time_in_contact);
      }
    }

    std::cout << "All done" << std::endl;
    
}

// Declare a test
TEST(TestSuite, testGaitGeneration)
{
  std::cout << "Testing 2!" << std::endl;
  ros::Time beginning_of_time(0.0);

  double mass = 20;     // [kg]
  double ixx = 0.07538; // [kg * m^2]
  double iyy = 0.1611;  // [kg * m^2]
  double izz = 0.202;   // [kg * m^2]
  FloatingBase test_robot(mass, ixx, iyy, izz);

  Eigen::Vector3d test_foot_position;
  test_foot_position << 0.133795, 0.131609, 0.169671;

  Eigen::Vector3d foot_fl, foot_fr, foot_rl, foot_rr;
  foot_fl << test_foot_position.x(), test_foot_position.y(), -test_foot_position.z();
  foot_fr << test_foot_position.x(), -test_foot_position.y(), -test_foot_position.z();
  foot_rl << -test_foot_position.x(), test_foot_position.y(), -test_foot_position.z();
  foot_rr << -test_foot_position.x(), -test_foot_position.y(), -test_foot_position.z();
  test_robot.SetFootPositions(foot_fl, foot_fr, foot_rl, foot_rr);

  GaitGenerator test_generator(test_robot);
  test_generator.UpdateFeetState(beginning_of_time, test_robot);

  std::cout << "before gen" << std::endl;
  std::vector<GaitGenerator::HorizonStep> gait_horizon = test_generator.GenerateGait(beginning_of_time);
  std::cout << "after gen" << std::endl;

  for (const auto& horizon_step : gait_horizon) {
    std::cout << "fl: " << horizon_step.foot_states[FloatingBase::Foot::fl].in_contact << std::endl;
    std::cout << "time here: " << horizon_step.foot_states[FloatingBase::Foot::fl].time_along_state << std::endl;
    std::cout << "fr: " << horizon_step.foot_states[FloatingBase::Foot::fr].in_contact << std::endl;
    std::cout << "time here: " << horizon_step.foot_states[FloatingBase::Foot::fr].time_along_state << std::endl;
    std::cout << "rl: " << horizon_step.foot_states[FloatingBase::Foot::rl].in_contact << std::endl;
    std::cout << "time here: " << horizon_step.foot_states[FloatingBase::Foot::rl].time_along_state << std::endl;
    std::cout << "rr: " << horizon_step.foot_states[FloatingBase::Foot::rr].in_contact << std::endl;
    std::cout << "time here: " << horizon_step.foot_states[FloatingBase::Foot::rr].time_along_state << std::endl;
    std::cout << "time: " << horizon_step.time << std::endl;
    std::cout << std::endl;
  }

  foot_fl << test_foot_position.x(), test_foot_position.y(), -test_foot_position.z();
  foot_fr << test_foot_position.x(), -test_foot_position.y(), test_foot_position.z();
  foot_rl << -test_foot_position.x(), test_foot_position.y(), test_foot_position.z();
  foot_rr << -test_foot_position.x(), -test_foot_position.y(), -test_foot_position.z();
  test_robot.SetFootPositions(foot_fl, foot_fr, foot_rl, foot_rr);

  beginning_of_time += ros::Duration(0.1);
  test_generator.UpdateFeetState(beginning_of_time, test_robot);
  gait_horizon = test_generator.GenerateGait(beginning_of_time);
  std::cout << "second" << std::endl;
  for (const auto& horizon_step : gait_horizon) {
    std::cout << "fl: " << horizon_step.foot_states[FloatingBase::Foot::fl].in_contact << std::endl;
    std::cout << "time here: " << horizon_step.foot_states[FloatingBase::Foot::fl].time_along_state << std::endl;
    std::cout << "fr: " << horizon_step.foot_states[FloatingBase::Foot::fr].in_contact << std::endl;
    std::cout << "time here: " << horizon_step.foot_states[FloatingBase::Foot::fr].time_along_state << std::endl;
    std::cout << "rl: " << horizon_step.foot_states[FloatingBase::Foot::rl].in_contact << std::endl;
    std::cout << "time here: " << horizon_step.foot_states[FloatingBase::Foot::rl].time_along_state << std::endl;
    std::cout << "rr: " << horizon_step.foot_states[FloatingBase::Foot::rr].in_contact << std::endl;
    std::cout << "time here: " << horizon_step.foot_states[FloatingBase::Foot::rr].time_along_state << std::endl;
    std::cout << "time: " << horizon_step.time << std::endl;
    std::cout << std::endl;
  }

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