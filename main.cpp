#include <iostream>
#include <chrono>
#include <random>

#include "controller_1d.h"


double time_now() {
  return std::chrono::time_point_cast<std::chrono::duration<double>>(
      std::chrono::system_clock::now()).time_since_epoch().count();
}

double noise(double mean, double stddev) {
  std::normal_distribution dist(mean, stddev);
  std::random_device rd;
  std::mt19937 gen(rd());
  return dist(gen);
}

int main() {
  constexpr int control_horizon = 30;
  constexpr double control_freq = 3.0;

  Controller1D controller(1.0 / control_freq);

  Matrix<double, 2, 1> state, init_state, desired_state;
  init_state << 1, 0;
  desired_state << 0, 0;

  MatrixXd init_control(1, control_horizon);
  init_control.setZero();

  MatrixXd control_limit(1, 2);
  control_limit << -1, 1;

  controller.init(init_state, init_control, control_horizon);
  controller.setDesiredState(desired_state);
  controller.setInputConstraint(control_limit);
  controller.setVerboseLevel(controller.vbl_info);

  double t0 = time_now();
  controller.plan();
  double plan_time = time_now() - t0;

  std::cout << "planned in " << plan_time << " sec" << std::endl;

  const MatrixXd& plan = controller.getPlannedStateMatrix();
  const MatrixXd& control_inputs = controller.getPlannedControlInputMatrix();

  for (int i = 0; i < control_horizon; i++) {
    std::cout << std::fixed << "time " << i / control_freq << " pos " << plan(0,i) << " vel " << plan(1,i) << " control " << control_inputs(i) << std::endl;
  }

  return 0;
}
