#include <iostream>
#include <chrono>

#include "controller_1d.h"
#include "controller_bicycle_model.h"


constexpr int control_horizon = 500;
constexpr double control_freq = 100.0;

double time_now() {
  return std::chrono::time_point_cast<std::chrono::duration<double>>(
      std::chrono::system_clock::now()).time_since_epoch().count();
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> plan_1d() {
  Controller1D controller(1.0 / control_freq);

  Controller1D::VectorX init_state, desired_state;
  init_state << 1, 0;
  desired_state << 0, 0;

  Eigen::MatrixXd init_control(1, control_horizon);
  init_control.setZero();

  Eigen::MatrixXd control_limit(1, 2);
  control_limit << -1, 1;

  controller.init(init_state, init_control, control_horizon);
  controller.setDesiredState(desired_state);
  controller.setInputConstraint(control_limit);
  controller.setVerboseLevel(controller.vbl_info);

  controller.plan();

  return std::make_tuple(controller.getPlannedStateMatrix(), controller.getPlannedControlInputMatrix());
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> plan_2d() {
  ControllerBicycleModel controller(1.0 / control_freq);

  ControllerBicycleModel::VectorX init_state, desired_state;
  // init_state.setZero();
  // init_state(3) = 0.0;
  init_state << 0, 0, 0, 1.0, -0.2, 1.0;
  desired_state.setZero();
  desired_state(3) = 3.0;  // speed_target

  std::cout << "desired state " << desired_state.transpose() << std::endl;

  Eigen::MatrixXd init_control(ControllerBicycleModel::VectorU{}.rows(), control_horizon);
  init_control.setZero();

  Eigen::MatrixXd control_limit(2, 2);
  control_limit << -1, 1,
      -0.3, 0.3;

  controller.init(init_state, init_control, control_horizon);
  controller.setDesiredState(desired_state);
  controller.setInputConstraint(control_limit);
  controller.setVerboseLevel(controller.vbl_verbose);

  controller.plan();

  return std::make_tuple(controller.getPlannedStateMatrix(), controller.getPlannedControlInputMatrix());
}

int main() {

  double t0 = time_now();
  auto[plan, control_inputs] = plan_2d();
  double plan_time = time_now() - t0;

  std::cout << "planned in " << plan_time << " sec" << std::endl;

  for (int i = 0; i < control_horizon; i++) {
    std::cout << std::fixed << "time " << i / control_freq << std::endl;
    std::cout << "state" << std::endl << plan.col(i).transpose() << std::endl;
    std::cout << "control" << std::endl << control_inputs.col(i).transpose() << std::endl;
  }

  return 0;
}
