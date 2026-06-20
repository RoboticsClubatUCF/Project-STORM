#include <cstdint>
#include <functional>
#include <thread>
#include <threads.h>

#pragma once

namespace tegra_stepper {
  enum DIRECTION {
    FORWARD = 1,
    REVERSE = -1,
  };

  class Stepper {
    private:
    static int ena_pin, dir_pin, pul_pin;
    
    int steps_per_rev = 3200, frequency = 50;
    std::jthread exec_thread;

    static void configure_pin();
    static void move(std::reference_wrapper<int> steps,
                     std::reference_wrapper<DIRECTION> dir,
                     std::reference_wrapper<int> frequency);

  public: 
    Stepper(int ena_pin, int dir_pin, int pul_pin) {
      this->ena_pin = ena_pin;
      this->dir_pin = dir_pin;
      this->pul_pin = pul_pin;
    };

    void setup(int steps_per_rev, int frequency);
    void setup(int frequency);
    void set(int steps, DIRECTION dir);
  };
};