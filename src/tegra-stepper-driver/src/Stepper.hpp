#include <JetsonGPIO.h>
#include <JetsonGPIO/PWM.h>
#include <cstdint>
#include <functional>
#include <memory>
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
    
    int steps_per_rev = 3200, frequency = 10000;
    std::jthread exec_thread;
    std::unique_ptr<GPIO::PWM> pwm_channel;

    void configure_pin();
    static void move(std::reference_wrapper<int> steps,
                     std::reference_wrapper<ControllerInfo> info,
                     std::reference_wrapper<DIRECTION> dir,
                     std::reference_wrapper<GPIO::PWM> channel,
                     std::reference_wrapper<int> frequency);

  public: 
    Stepper(int ena_pin, int dir_pin, int pul_pin) {
      this->info = ControllerInfo(ena_pin, dir_pin, pul_pin);

      pwm_channel = std::make_unique<GPIO::PWM>(GPIO::PWM(pul_pin, frequency));
    };

    ~Stepper() {
      exec_thread.request_stop();
    }

    void setup(int steps_per_rev, int frequency);
    void setup(int frequency);
    void set(int steps, DIRECTION dir);
  };
};