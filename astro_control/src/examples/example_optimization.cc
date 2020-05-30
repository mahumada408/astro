#include <ros/ros.h>

#include <gtest/gtest.h>
#include <iostream>
#include <thread>

#include <astro_control/floating_base/floating_base.h>
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>

#include "tf/transform_listener.h"

int main(int argc, char** argv) {
  // Robot setup.
  FloatingBase test_robot(20, 0.07538, 0.1611, 0.202);
  tf::Vector3 position(0, 0, 0);

  // Orientation corresponding to 45 degree rotation about the z-axis.
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

  // Optimization setup.

  const size_t state_dim = FloatingBase::State::count;
  const size_t control_dim = FloatingBase::Control::count;
  double dt = 0.01;
  ct::core::Time timeHorizon = 10.0;  // and a final time horizon in [sec]
  int N = (int)timeHorizon / dt;
  std::cout << "Horizon size: " << N << std::endl;

  typedef ct::optcon::LQOCProblem<state_dim, control_dim> LQOCProblem_t;
  std::shared_ptr<LQOCProblem_t> lqocProblem_hpipm(new LQOCProblem_t(N));

  std::shared_ptr<ct::core::LTISystem<state_dim, control_dim>> quadruped_system(
      new ct::core::LTISystem<state_dim, control_dim>(test_robot.A(), test_robot.B()));
  ct::core::SensitivityApproximation<state_dim, control_dim> discrete_quad(
      dt, quadruped_system, ct::core::SensitivityApproximationSettings::APPROXIMATION::BIG_MATRIX_EXPONENTIAL);

  // STEP 1-B: System Linearizer.
  std::shared_ptr<ct::core::SystemLinearizer<state_dim, control_dim>> adLinearizer(
      new ct::core::SystemLinearizer<state_dim, control_dim>(quadruped_system));

  // Set initial conditions for state and control.
  ct::core::StateVector<state_dim> x0;
  ct::core::StateVector<state_dim> x_next;
  ct::core::StateVector<state_dim> xf;
  ct::core::ControlVector<control_dim> u0;

  x0.setZero();
  u0.setZero();
  x0(FloatingBase::State::g) = -9.81;
  u0(FloatingBase::Control::f1z) = (20 * 9.81) / 4;
  u0(FloatingBase::Control::f2z) = u0(FloatingBase::Control::f1z);
  u0(FloatingBase::Control::f3z) = u0(FloatingBase::Control::f1z);
  u0(FloatingBase::Control::f4z) = u0(FloatingBase::Control::f1z);

  // Desired state of the robot.
  xf.setZero();
  xf(FloatingBase::State::z) = 1.0;

  // Define cost function matrices.
  ct::core::StateMatrix<state_dim> Q;
  Q.setIdentity();
  ct::core::ControlMatrix<control_dim> R;
  R.setIdentity();

  // Create a cost function'
  std::shared_ptr<ct::optcon::CostFunctionQuadraticSimple<state_dim, control_dim>> costFunction(
      new ct::optcon::CostFunctionQuadraticSimple<state_dim, control_dim>(Q, R, xf, u0, xf, Q));

  // STEP 1-E: create and initialize an "optimal control problem."
  ct::optcon::ContinuousOptConProblem<state_dim, control_dim> optConProblem(timeHorizon, x0, quadruped_system,
                                                                            costFunction, adLinearizer);

  // STEP 2: set up a nonlinear optimal control solver.

  // STEP 2-A: Create the settings.
  ct::optcon::NLOptConSettings nloc_settings;
  nloc_settings.load("/home/manuel/Downloads/NLOC_ObstacleConstraint/nlocSolver_ObstacleConstraint.info", true, "ilqr");
  nloc_settings.lqocp_solver =
      ct::optcon::NLOptConSettings::LQOCP_SOLVER::HPIPM_SOLVER;  // solve LQ-problems using HPIPM

  // STEP 2-B: provide an initial guess.
  // calculate the number of time steps K.
  size_t K = nloc_settings.computeK(timeHorizon);

  // Design trivial initial controller for NLOC.
  ct::core::FeedbackArray<state_dim, control_dim> u0_fb(K, ct::core::FeedbackMatrix<state_dim, control_dim>::Zero());
  ct::core::ControlVectorArray<control_dim> u0_ff(K, ct::core::ControlVector<control_dim>::Zero());
  ct::core::StateVectorArray<state_dim> x_ref_init(K + 1, x0);
  ct::optcon::NLOptConSolver<state_dim, control_dim>::Policy_t initController(x_ref_init, u0_ff, u0_fb,
                                                                              nloc_settings.dt);

  // STEP 2-C: create an NLOptConSolver instance.
  ct::optcon::NLOptConSolver<state_dim, control_dim> nloc(optConProblem, nloc_settings);

  // Set the initial guess.
  nloc.setInitialGuess(initController);

  // STEP 3: solve the optimal control problem.
  nloc.solve();

  // STEP 4: retrieve the solution.
  ct::core::StateFeedbackController<state_dim, control_dim> solution = nloc.getSolution();

  ct::core::StateTrajectory<state_dim, double> state_traj = nloc.getStateTrajectory();
  ct::core::ControlTrajectory<control_dim, double> control_traj = nloc.getControlTrajectory();
  std::cout << "state traj" << std::endl;
  state_traj.print();
  std::cout << std::endl;
  std::cout << "control traj" << std::endl;
  control_traj.print();

  std::cout << "the z" << std::endl;
  for (size_t i = 0; i < state_traj.getDataArray().size(); i++) {
    std::cout << state_traj.getDataArray()[i](2) << std::endl;
  }
}