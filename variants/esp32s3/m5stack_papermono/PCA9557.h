#pragma once

#include <Arduino.h>
#include <M5IOE1.h>
#include <M5PM1.h>

#include "variant.h"

// The M5PM1 frontlight is a PWM rail inside the PMIC (I2C driven), not a GPIO.
// Backlight.cpp drives the rail through io.digitalWrite(PCA_PIN_EINK_EN, level),
// so this shim routes that one virtual pin to the M5PM1 PWM and passes every
// other pin through to the M5IOE1 IO expander.
class PCA9557
{
  public:
    void digitalWrite(uint8_t pin, uint8_t value);
};