#pragma once

#include <cstdint>
#include <functional>
#include <gpiod.h>
#include <memory>
#include <thread>
#include <threads.h>

namespace tegra_stepper {
  enum DIRECTION {
    FORWARD = 1,
    REVERSE = -1,
  };

  struct ControllerInfo {
    int ena_pin, dir_pin, pul_pin;

    ControllerInfo(int ena_pin, int dir_pin, int pul_pin) {
      this->ena_pin = ena_pin;
      this->dir_pin = dir_pin;
      this->pul_pin = pul_pin;
    }
  };

  class Stepper {
    private:
      constexpr static const char *const chip_path = "/dev/gpiochip0";
      gpiod_chip *chip_ptr;
      ControllerInfo info{
          0,
          0,
          0,
      };
      int steps_per_rev = 3200, frequency = 10000;

      void configure_pins();
      static void move(std::reference_wrapper<int> steps,
                       std::reference_wrapper<ControllerInfo> info,
                       std::reference_wrapper<DIRECTION> dir);

    public:
      Stepper(int ena_pin, int dir_pin, int pul_pin) {
        this->info = ControllerInfo(ena_pin, dir_pin, pul_pin);
        this->chip_ptr = gpiod_chip_open(chip_path);
        this->configure_pins();
    };

    ~Stepper() {
    }

    void setup(int steps_per_rev, int frequency);
    void setup(int frequency);
    void set(int steps, DIRECTION dir);
  };
};