#include "JetsonGPIO.h"
#include "Stepper.hpp"
#include <spdlog/spdlog.h>

int main() {
  tegra_stepper::Stepper* stepper = new tegra_stepper::Stepper(2, 3, 5);

  spdlog::info("Configuring stepper motor");

  stepper->set(500, tegra_stepper::DIRECTION::FORWARD);

  spdlog::info("Moving forward 500 steps");

  delete stepper;

  return 0;
}