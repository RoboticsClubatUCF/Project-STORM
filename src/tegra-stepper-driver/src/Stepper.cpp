#include "Stepper.hpp"
#include <JetsonGPIO.h>
#include <JetsonGPIO/PublicEnums.h>
#include <chrono>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <time.h>
#include <spdlog/spdlog.h>

void tegra_stepper::Stepper::Stepper::configure_pin() {
  GPIO::setmode(GPIO::BOARD);

  GPIO::setup(ena_pin, GPIO::OUT);
  GPIO::setup(dir_pin, GPIO::OUT);
  GPIO::setup(pul_pin, GPIO::OUT);

  GPIO::output(ena_pin, GPIO::LOW);

  // Double check if its okay to start on HIGH for Direction.
  GPIO::output(dir_pin, GPIO::LOW);
  GPIO::output(pul_pin, GPIO::LOW);
}

void tegra_stepper::Stepper::setup(int frequency) {
  this->frequency = frequency;
}

void tegra_stepper::Stepper::setup(int steps_per_rev, int frequency) {
  this->steps_per_rev = steps_per_rev;
  this->frequency = frequency;
}

void tegra_stepper::Stepper::move(std::reference_wrapper<int> steps, std::reference_wrapper<DIRECTION> dir, std::reference_wrapper<int> frequency) {
  GPIO::output(dir_pin, dir.get() == DIRECTION::FORWARD ? GPIO::LOW : GPIO::HIGH);
  GPIO::output(ena_pin, GPIO::HIGH);

  for (int i = 0; i < steps; i++) {
    GPIO::output(pul_pin, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(frequency.get()));
    GPIO::output(pul_pin, GPIO::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(frequency.get()));
  }
}

void tegra_stepper::Stepper::set(int steps, DIRECTION dir) {
  if (exec_thread.request_stop() == true) {
    exec_thread = std::jthread(&move, std::ref(steps), std::ref(dir), std::ref(frequency));
    exec_thread.detach();
  } else {
    spdlog::error("Failed to request previous stepper motor thread to HALT!");
  }
}