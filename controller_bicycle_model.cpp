#include "controller_bicycle_model.h"

const ControllerBicycleModel::VectorX ControllerBicycleModel::dynamicsCalc(const VectorX &x, const VectorU &u) {
  double control_accel = u(0);
  double steering_angle = u(1);

  double global_x = x(0);
  double global_y = x(1);
  double heading = x(2);
  double forward_vel = x(3);
  double lateral_vel = x(4);
  double rotation_rate = x(5);

  double front_wheel_slip_angle = 0;
  double rear_wheel_slip_angle = 0;
  if (forward_vel + lateral_vel > 0.001) {
    double rear_wheel_lateral_vel = lateral_vel + -1 * wheelbase_rear_ * rotation_rate;
    rear_wheel_slip_angle = std::atan2(rear_wheel_lateral_vel, forward_vel);

    double front_wheel_lateral_vel = lateral_vel + wheelbase_front_ * rotation_rate;
    front_wheel_slip_angle = std::atan2(front_wheel_lateral_vel, forward_vel) - steering_angle;
  }

  double cornering_force_rear = -1 * cornering_stiffness_ * rear_wheel_slip_angle;
  double cornering_force_front = -1 * cornering_stiffness_ * front_wheel_slip_angle;

  double forward_accel = rotation_rate * lateral_vel + control_accel;
  double lateral_accel = -1 * rotation_rate * forward_vel
                         + 2 * (cornering_force_front * std::cos(steering_angle) + cornering_force_rear) / mass_;

  double rotation_accel = 2 * (wheelbase_front_ * cornering_force_front - wheelbase_rear_ * cornering_force_rear)
                          / rotational_inertia_;

  double new_forward_vel = forward_vel + forward_accel * dt_;
  double new_lateral_vel = lateral_vel + lateral_accel * dt_;
  double forward_dist = forward_vel * dt_ + 0.5 * forward_accel * dt_ * dt_;
  double lateral_dist = lateral_vel * dt_ + 0.5 * lateral_accel * dt_ * dt_;
  double cos_heading = std::cos(heading);
  double sin_heading = std::sin(heading);
  double new_global_x = global_x + cos_heading * forward_dist - sin_heading * lateral_dist;
  double new_global_y = global_y + sin_heading * forward_dist + cos_heading * lateral_dist;

  double new_rotation_rate = rotation_rate + rotation_accel * dt_;
  double new_heading = heading + rotation_rate * dt_ + 0.5 * rotation_accel * dt_ * dt_;

  VectorX out;
  out(0) = new_global_x;
  out(1) = new_global_y;
  out(2) = new_heading;
  out(3) = new_forward_vel;
  out(4) = new_lateral_vel;
  out(5) = new_rotation_rate;
  return out;
}

double ControllerBicycleModel::costFunction(const VectorX &x, const VectorU &u, size_t index) {
  return std::pow(x(3), 2) + 0 * std::pow(u(0), 2);
}
