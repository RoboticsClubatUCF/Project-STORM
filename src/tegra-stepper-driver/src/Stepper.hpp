#include <JetsonGPIO.h>
#include <JetsonGPIO/PWM.h>
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
    
    int steps_per_rev = 3200, frequency = 10000;
    std::jthread exec_thread;
    GPIO::PWM* pwm_channel;

    static void configure_pin();
    static void move(std::reference_wrapper<int> steps,
                     std::reference_wrapper<DIRECTION> dir,
                     std::reference_wrapper<GPIO::PWM> channel,
                     std::reference_wrapper<int> frequency);

  public: 
    Stepper(int ena_pin, int dir_pin, int pul_pin) {
      Stepper::ena_pin = ena_pin;
      Stepper::dir_pin = dir_pin;
      Stepper::pul_pin = pul_pin;

      pwm_channel = new GPIO::PWM(pul_pin, frequency);
    };

    ~Stepper() {
      exec_thread.request_stop();

      if (pwm_channel) {
        pwm_channel->stop();
        delete pwm_channel;
      }
    }

    void setup(int steps_per_rev, int frequency);
    void setup(int frequency);
    void set(int steps, DIRECTION dir);
  };
};