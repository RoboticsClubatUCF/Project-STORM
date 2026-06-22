#include "Stepper.hpp"
#include <JetsonGPIO.h>
#include <JetsonGPIO/PWM.h>
#include <chrono>
#include <functional>
#include <spdlog/spdlog.h>
#include <thread>
#include <time.h>

void tegra_stepper::Stepper::configure_pin() {
  GPIO::setmode(GPIO::BOARD);

  GPIO::setup(info.ena_pin, GPIO::OUT);
  GPIO::setup(info.dir_pin, GPIO::OUT);
  GPIO::setup(info.pul_pin, GPIO::OUT);

  GPIO::output(info.ena_pin, GPIO::LOW);

  // Double check if its okay to start on HIGH for Direction.
  GPIO::output(info.dir_pin, GPIO::LOW);
  GPIO::output(info.pul_pin, GPIO::LOW);
}

void tegra_stepper::Stepper::setup(int frequency) {
  this->frequency = frequency;
}

void tegra_stepper::Stepper::setup(int steps_per_rev, int frequency) {
  this->steps_per_rev = steps_per_rev;
  this->frequency = frequency;
}

void tegra_stepper::Stepper::move(std::reference_wrapper<int> steps, std::reference_wrapper<ControllerInfo> info, std::reference_wrapper<DIRECTION> dir, std::reference_wrapper<GPIO::PWM> channel, std::reference_wrapper<int> frequency) {
  ControllerInfo _info = info.get();

  GPIO::output(_info.dir_pin, dir.get() == DIRECTION::FORWARD ? GPIO::LOW : GPIO::HIGH);
  GPIO::output(_info.ena_pin, GPIO::HIGH);

  spdlog::info("Driver stepper in %s", ((dir.get() == DIRECTION::FORWARD)
                                           ? "forward"
                                           : "backwards"));

  GPIO::PWM &pwm = channel.get();

  for (int i = 0; i < steps.get(); ++i) {
    GPIO::output(_info.pul_pin, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds(frequency.get()));
    GPIO::output(_info.pul_pin, GPIO::LOW);
    std::this_thread::sleep_for(std::chrono::microseconds(frequency.get()));
  }

  GPIO::output(_info.ena_pin, GPIO::LOW);
}

void tegra_stepper::Stepper::set(int steps, DIRECTION dir) {
  if (exec_thread.request_stop() == true) {
    spdlog::debug("Previous thread stopped.");
  } else {
    spdlog::error("Failed to request previous stepper motor thread to HALT!");
  }

  exec_thread = std::jthread(&move, std::ref(steps), std::ref(dir),
                             std::ref(pwm_channel), std::ref(frequency));
  exec_thread.join();
}