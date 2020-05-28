#include <ros/ros.h>

#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include <astro_control/floating_base/floating_base.h>

// clang-format off
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
// clang-format on

#include <ct/core/core.h>

#include "tf/transform_listener.h"

int main(int argc, char** argv) {
  std::cout << "Testing!" << std::endl;
  FloatingBase test_robot(20, 0.07538, 0.1611, 0.202);
  tf::Vector3 position(0, 0, 0);

  // Orientation corresponding to 45 degree rotation about the
  // z-axis.
  tf::Quaternion orientation(0.0, 0.0, 0.0, 1.0);
  tf::StampedTransform test_pose;

  // Set zero velocities.
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

  Eigen::Vector3d foot_fl, foot_fr, foot_rl, foot_rr;
  foot_fl << 0.133795, 0.131609, -0.169671;
  foot_fr << 0.133795, -0.131609, -0.169671;
  foot_rl << -0.133795, 0.131609, -0.169671;
  foot_rr << -0.133795, -0.131609, -0.169671;
  test_robot.SetFootPositions(foot_fl, foot_fr, foot_rl, foot_rr);
  test_robot.UpdateDynamics();

  const size_t state_dim = 13;
  const size_t control_dim = 12;
  int N = 10;
  double dt = 0.01;

  typedef ct::optcon::LQOCProblem<state_dim, control_dim> LQOCProblem_t;
  std::shared_ptr<LQOCProblem_t> lqocProblem_hpipm(new LQOCProblem_t(N));

  std::shared_ptr<ct::core::LTISystem<state_dim, control_dim>> quadruped_system(
      new ct::core::LTISystem<state_dim, control_dim>(test_robot.A(), test_robot.B()));
  ct::core::SensitivityApproximation<state_dim, control_dim> discrete_quad(
      dt, quadruped_system, ct::core::SensitivityApproximationSettings::APPROXIMATION::BIG_MATRIX_EXPONENTIAL);

  ct::core::StateVector<state_dim> x0;
  ct::core::StateVector<state_dim> x_next;
  ct::core::StateVector<state_dim> xf;
  ct::core::ControlVector<control_dim> u0;

  x0.setZero();
  u0.setZero();
  x0(12) = -9.81;
  u0(2) = (20 * 9.81) / 4;
  u0(5) = u0(2);
  u0(8) = u0(2);
  u0(11) = u0(2);

  xf = x0;
  // x_next.setZero();
  // std::cout << "states" << std::endl;
  // for (int i = 0; i < 100; ++i) {
  //   std::cout << "time: " << dt * i + dt << std::endl;
  //   discrete_quad.propagateControlledDynamics(x0, i, u0, x_next);
  //   std::cout << x_next << std::endl;
  //   std::cout << std::endl;
  // }

  // define cost function matrices
  ct::core::StateMatrix<state_dim> Q;
  Q.setIdentity();
  ct::core::ControlMatrix<control_dim> R;
  R.setIdentity();

  // create a cost function
  ct::optcon::CostFunctionQuadraticSimple<state_dim, control_dim> costFunction(Q, R, xf, u0, xf, Q);
  
  // initialize the linear quadratic optimal control problems
  lqocProblem_hpipm->setFromTimeInvariantLinearQuadraticProblem(discrete_quad, costFunction, x0, dt);

  // Eigen::VectorXi x_box_sparsity(state_dim);
  // x_box_sparsity << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;

  // // Initial state constraint.
  // lqocProblem_hpipm->setIntermediateStateBoxConstraint(0, 1, x0, x0, x_box_sparsity, x0);

  // create hpipm solver instance, set and solve problem
  ct::optcon::HPIPMInterface<state_dim, control_dim> hpipm;
  hpipm.setProblem(lqocProblem_hpipm);
  hpipm.solve();
  hpipm.computeStatesAndControls();
  hpipm.computeFeedbackMatrices();
  hpipm.compute_lv();

   // compute and retrieve solutions
  ct::core::StateVectorArray<state_dim> x_sol_hpipm = hpipm.getSolutionState();
  ct::core::ControlVectorArray<control_dim> u_sol_hpipm = hpipm.getSolutionControl();
  ct::core::FeedbackArray<state_dim, control_dim> K_sol_hpipm = hpipm.getSolutionFeedback();
  ct::core::ControlVectorArray<control_dim> lv_sol_hpipm = hpipm.get_lv();


  for (int i = 0; i < x_sol_hpipm.size(); ++i) {
    std::cout << "i: " << i << std::endl;
    std::cout << "states" << std::endl;
    std::cout << x_sol_hpipm[i] << std::endl;
    std::cout << "control" << std::endl;
    std::cout << u_sol_hpipm[i] << std::endl;
    std::cout << std::endl;
  }


}