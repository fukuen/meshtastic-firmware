#pragma once

#include "pins_arduino.h"

// I2C
#define I2C_SDA 47
#define I2C_SCL 48

// Buttons
#define BUTTON_NEED_PULLUP
#define BUTTON_PIN 2 // Button A (KEY1)
#define PIN_BUTTON2 3

// Buzzer
#define PIN_BUZZER 42

// LoRa runs on SPI1 (HSPI); the E-Paper uses SPI2 (FSPI).
#define HW_SPI1_DEVICE

#undef LORA_SCK
#undef LORA_MISO
#undef LORA_MOSI
#undef LORA_CS

#define LORA_SCK 39
#define LORA_MISO 40
#define LORA_MOSI 38
#define LORA_CS 41 // NSS

#define USE_SX1262
#define LORA_DIO0 -1
#define LORA_DIO1 5   // IRQ
#define LORA_RESET -1 // Reset is driven through M5IOE1 (variant.cpp)
#define LORA_RST -1
#define LORA_IRQ 5 // DIO0
#define LORA_BUSY 21
#define LORA_DIO2 RADIOLIB_NC
#define LORA_DIO3 RADIOLIB_NC

#define SX126X_CS LORA_CS
#define SX126X_DIO1 LORA_DIO1
#define SX126X_BUSY LORA_BUSY
#define SX126X_RESET LORA_RESET

#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8
#define TCXO_OPTIONAL

// Display: 3.97" 4-level grayscale SSD1677 E-Paper, 480x800.
// Driven through LovyanGFX (Panel_SSD1677_4Gray), see src/graphics/LGFXEInkDisplay.
// USE_EINK / USE_EINK_LGFX are set as build flags in platformio.ini.
#define EINK_WIDTH 480
#define EINK_HEIGHT 800

#define PIN_EINK_CS 16   // EPD_CS
#define PIN_EINK_BUSY 18 // EPD_BUSY
#define PIN_EINK_DC 17   // EPD_D/C
#define PIN_EINK_RES -1  // Reset is driven through M5IOE1 (variant.cpp)
#define PIN_EINK_SCLK 15 // EPD_SCLK
#define PIN_EINK_MOSI 14 // EPD_MOSI

// Frontlight is a PWM rail inside the M5PM1 PMIC, routed through the PCA9557 shim.
#define HAS_PCA9557
#define PCA_PIN_EINK_EN 99
#define GPIO_BACKLIGHT_DEFAULT_ON

// RTC
#define RX8130CE_RTC 0x32