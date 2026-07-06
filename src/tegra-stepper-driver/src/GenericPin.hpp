#pragma once

#include <gpiod.h>
#include <spdlog/spdlog.h>

enum PinType {
  OUTPUT,
  INPUT
};

class GenericPin {
  private:
  gpiod_chip *chip_ptr;
  gpiod_line *line;
  PinType type;

  public:
  GenericPin(gpiod_chip *chip_ptr, int pin, PinType type) {
    this->type = type;
    this->chip_ptr = chip_ptr;

    if (!chip_ptr) {
      spdlog::error("failed to open chip: %s\n", strerror(errno));
    }

    this->line = gpiod_chip_get_line(chip_ptr, pin);
    if (!line) {
      spdlog::error("failed to open line: %s\n", strerror(errno));
    }
  }

  void write(char byte);

  ~GenericPin() {
    // release the gpio line
    gpiod_line_release(line);

    // Close the gpio chip
    gpiod_chip_close(chip_ptr);
  }
};