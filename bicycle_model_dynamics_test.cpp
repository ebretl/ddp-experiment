#include <iostream>

#include "controller_bicycle_model.h"


int main() {
  ControllerBicycleModel controller(0.01);

  ControllerBicycleModel::VectorX x0;
  x0 << 0, 0, 0, 0, 0, 0;

  ControllerBicycleModel::VectorU u;
  u << 1.0, 0.1;

  for (int t = 0; t < 1000; t++) {
    auto x1 = controller.dynamicsCalc(x0, u);
    std::cout << x1.transpose() << std::endl;
    x0 = x1;
  }
}
