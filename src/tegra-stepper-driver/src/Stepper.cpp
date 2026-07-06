#include "Stepper.hpp"
#include <chrono>
#include <functional>
#include <spdlog/spdlog.h>
#include <thread>
#include <time.h>

void tegra_stepper::Stepper::configure_pins() {

}

void tegra_stepper::Stepper::setup(int frequency) {
  this->frequency = frequency;
}

void tegra_stepper::Stepper::setup(int steps_per_rev, int frequency) {
  this->steps_per_rev = steps_per_rev;
  this->frequency = frequency;
}

void tegra_stepper::Stepper::move(std::reference_wrapper<int> steps, std::reference_wrapper<ControllerInfo> info, std::reference_wrapper<DIRECTION> dir) {
  ControllerInfo _info = info.get();
}

void tegra_stepper::Stepper::set(int steps, DIRECTION dir) {
  std::jthread _exec_thread = std::jthread(&move, std::ref(steps), std::ref(info), std::ref(dir));
  _exec_thread.join();
}