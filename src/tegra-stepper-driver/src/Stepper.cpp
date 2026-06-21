#include "Stepper.hpp"
#include <JetsonGPIO.h>
#include <JetsonGPIO/PWM.h>
#include <chrono>
#include <functional>
#include <spdlog/spdlog.h>
#include <thread>
#include <time.h>

int tegra_stepper::Stepper::ena_pin = -1;
int tegra_stepper::Stepper::dir_pin = -1;
int tegra_stepper::Stepper::pul_pin = -1;

void tegra_stepper::Stepper::configure_pin() {
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

void tegra_stepper::Stepper::move(
    std::reference_wrapper<int> steps, std::reference_wrapper<DIRECTION> dir,
    std::reference_wrapper<GPIO::PWM> channel, std::reference_wrapper<int>
        frequency) {
  GPIO::output(dir_pin, dir.get() == DIRECTION::FORWARD ? GPIO::LOW : GPIO::HIGH);
  GPIO::output(ena_pin, GPIO::HIGH);

  spdlog::info("Driver stepper in %s", ((dir.get() == DIRECTION::FORWARD)
                                           ? "forward"
                                           : "backwards"));

  GPIO::PWM &pwm = channel.get();

  pwm.ChangeFrequency(frequency);
  pwm.start(50);

  // for (int i = 0; i < steps.get(); ++i) {
  //   GPIO::output(pul_pin, GPIO::HIGH);
  //   std::this_thread::sleep_for(std::chrono::microseconds(frequency.get()));
  //   GPIO::output(pul_pin, GPIO::LOW);
  //   std::this_thread::sleep_for(std::chrono::microseconds(frequency.get()));
  // }

  GPIO::output(ena_pin, GPIO::LOW);
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