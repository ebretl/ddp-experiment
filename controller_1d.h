#pragma once

#include <ilqg/ilqg.h>

class Controller1D : public husky::iLQG<double, 2, 1> {
private:
  double dt_;

public:
  explicit Controller1D(double t_step) : dt_(t_step) {}

  const VectorX dynamicsCalc(const VectorX &x, const VectorU &u) override;

  double costFunction(const VectorX &x, const VectorU &u, size_t index) override;
};
