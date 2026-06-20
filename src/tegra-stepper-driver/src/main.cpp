#include "JetsonGPIO.h"
#include "Stepper.hpp"

int main() {
  tegra_stepper::Stepper stepper{2, 3, 5};

  stepper.set(500, tegra_stepper::DIRECTION::FORWARD);
  
  return 0;
}