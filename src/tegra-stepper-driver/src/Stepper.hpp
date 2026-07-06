#include <gpiod.h>
#include <cstdint>
#include <functional>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>
#include <threads.h>

#pragma once

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
    ControllerInfo info{0,0,0,};

    constexpr static const char *const chip_path = "/dev/gpiochip0";
    static const unsigned int line_offset = 3;
    gpiod_chip *chip_ptr;
    gpiod_line *line;

    int steps_per_rev = 3200, frequency = 10000;

    void configure_pins();
    static void move(std::reference_wrapper<int> steps,
                     std::reference_wrapper<ControllerInfo> info,
                     std::reference_wrapper<DIRECTION> dir);

  public: 
    Stepper(int ena_pin, int dir_pin, int pul_pin) {
      this->info = ControllerInfo(ena_pin, dir_pin, pul_pin);
      this->configure_pins();

      chip_ptr = gpiod_chip_open(chip_path);
      if (!chip_ptr) {
        spdlog::error("failed to open chip: %s\n", strerror(errno));
      }

      line = gpiod_chip_get_line(chip_ptr, line_offset);
      if (!line) {
        spdlog::error("failed to open line: %s\n", strerror(errno));
      }
    };

    ~Stepper() {
      // release the gpio line
      gpiod_line_release(line);

      // Close the gpio chip
      gpiod_chip_close(chip_ptr);
    }

    void setup(int steps_per_rev, int frequency);
    void setup(int frequency);
    void set(int steps, DIRECTION dir);
  };
};