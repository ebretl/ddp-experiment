#include "controller_1d.h"

const Controller1D::VectorX Controller1D::dynamicsCalc(const VectorX &x, const VectorU &u) {
  VectorX out;

  double accel = u(0);
  double v_avg = x(1) + 0.5 * accel * dt_;

  out(0) = x(0) + v_avg * dt_;
  out(1) = x(1) + accel * dt_;

  return out;
}

double Controller1D::costFunction(const VectorX &x, const VectorU &u, size_t index) {
  double pos_loss = 1.0 * x(0) * x(0);
  // double pos_loss = 0;
  double control_loss = 0.1 * u(0) * u(0);
  // double control_loss = 0;
  return pos_loss + control_loss;
}
