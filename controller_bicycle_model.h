#pragma once

#include <ilqg/ilqg.h>

/*
 * Bicycle model state:
 *    0 - global x
 *    1 - global y
 *    2 - global heading (phi)
 *    3 - forward velocity
 *    4 - lateral velocity
 *    5 - rotation rate
 *
 * Bicycle model control:
 *    0 - forward acceleration
 *    1 - steering angle
 */

class ControllerBicycleModel : public husky::iLQG<double, 6, 2> {
private:
  const double dt_;
  const double mass_;
  const double rotational_inertia_;
  const double wheelbase_front_;
  const double wheelbase_rear_;
  const double cornering_stiffness_;

public:
  ControllerBicycleModel(double dt)
      : dt_(dt)
      , mass_(5.0)
      , rotational_inertia_(0.4)
      , wheelbase_front_(0.3)
      , wheelbase_rear_(0.2)
      , cornering_stiffness_(100.0)
  {}

  const VectorX dynamicsCalc(const VectorX &x, const VectorU &u) override;

  double costFunction(const VectorX &x, const VectorU &u, size_t index) override;
};
