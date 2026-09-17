#include "configuration.h"

#ifdef M5STACK_PAPERMONO

#if defined(HAS_SDCARD)
#include <SD_MMC.h>
#endif
#include <SPI.h>
#include <Wire.h>
//#include <M5IOE1.h>
//#include <M5PM1.h>
#include <PCA9557.h>

#define M5IOE1_ADDR 0x4F
#define M5PM1_ADDR 0x6E
#define IP2315_ADDR 0x75

M5IOE1 ioe1;
M5PM1 pm;
PCA9557 io;

// M5stack PaperMono specific init

void earlyInitVariant()
{
//  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  pm.begin(&Wire, M5PM1_ADDR, I2C_SDA, I2C_SCL, 100000);
  // M5PM1 BOOT OUT
  pm.gpioSetFunc(M5PM1_GPIO_NUM_0, M5PM1_GPIO_FUNC_GPIO);
  pm.gpioSetMode(M5PM1_GPIO_NUM_0, M5PM1_GPIO_MODE_INPUT);

  // M5PM1 IRQ
  pm.gpioSetMode(M5PM1_GPIO_NUM_1, M5PM1_GPIO_MODE_OUTPUT);
  pm.gpioSetDrive(M5PM1_GPIO_NUM_1, M5PM1_GPIO_DRIVE_PUSHPULL);
  pm.gpioSetPull(M5PM1_GPIO_NUM_1, M5PM1_GPIO_PULL_UP);
  pm.gpioSetOutput(M5PM1_GPIO_NUM_1, true);
  pm.gpioSetFunc(M5PM1_GPIO_NUM_1, M5PM1_GPIO_FUNC_IRQ);

  // LoRa EN
  pm.gpioSetFunc(M5PM1_GPIO_NUM_2, M5PM1_GPIO_FUNC_GPIO);
  pm.gpioSet(M5PM1_GPIO_NUM_2, M5PM1_GPIO_MODE_OUTPUT, HIGH, M5PM1_GPIO_PULL_NONE, M5PM1_GPIO_DRIVE_PUSHPULL);

  // BL
  pm.gpioSetFunc(M5PM1_GPIO_NUM_3, M5PM1_GPIO_FUNC_OTHER);
  pm.gpioSetDrive(M5PM1_GPIO_NUM_3, M5PM1_GPIO_DRIVE_PUSHPULL);
  pm.setPwmFrequency(5000);
//  pm.analogWrite(M5PM1_PWM_CH_0, 10);
  pm.analogWrite(M5PM1_PWM_CH_0, 0);

  // E-Paper CS
  pinMode(PIN_EINK_CS, OUTPUT);
  digitalWrite(PIN_EINK_CS, HIGH);

  ioe1.begin(&Wire, M5IOE1_ADDR, I2C_SDA, I2C_SCL, 100000, 7);

  // E-Paper EN
  ioe1.pinMode(M5IOE1_PIN_3, OUTPUT);
  ioe1.digitalWrite(M5IOE1_PIN_3, HIGH);
  // E-Paper RST
  ioe1.pinMode(M5IOE1_PIN_5, OUTPUT);
  ioe1.pinMode(M5IOE1_PIN_6, OUTPUT);
  ioe1.setDriveMode(M5IOE1_PIN_3, M5IOE1_DRIVE_PUSHPULL);
  ioe1.setDriveMode(M5IOE1_PIN_5, M5IOE1_DRIVE_PUSHPULL);
  ioe1.setDriveMode(M5IOE1_PIN_6, M5IOE1_DRIVE_PUSHPULL);
  ioe1.setDriveMode(M5IOE1_PIN_13, M5IOE1_DRIVE_PUSHPULL);
  ioe1.digitalWrite(M5IOE1_PIN_5, LOW);
  ioe1.digitalWrite(M5IOE1_PIN_6, LOW);
  delay(10);
  ioe1.digitalWrite(M5IOE1_PIN_5, HIGH);
  ioe1.digitalWrite(M5IOE1_PIN_6, HIGH);

  // LoRa ANT SW
  ioe1.pinMode(M5IOE1_PIN_2, OUTPUT);
  ioe1.setDriveMode(M5IOE1_PIN_2, M5IOE1_DRIVE_PUSHPULL);
  ioe1.digitalWrite(M5IOE1_PIN_2, HIGH);

  // LoRa RST
  ioe1.pinMode(M5IOE1_PIN_10, OUTPUT);
  ioe1.setDriveMode(M5IOE1_PIN_10, M5IOE1_DRIVE_PUSHPULL);
  delay(200);
  ioe1.digitalWrite(M5IOE1_PIN_10, LOW);
  delay(100);
  ioe1.digitalWrite(M5IOE1_PIN_10, HIGH);
  delay(200);

  // LoRa BUSY
  pinMode(LORA_BUSY, INPUT);

  // E-Paper
  SPI.begin(PIN_EINK_SCLK, -1, PIN_EINK_MOSI, -1);

  // init ip2315
  ioe1.pinMode(M5IOE1_PIN_11, OUTPUT_OPEN_DRAIN);
  ioe1.digitalWrite(M5IOE1_PIN_11, HIGH);
  // enable charge
  Wire.beginTransmission(IP2315_ADDR);
  Wire.write(0x01);
  Wire.write(0x01);
  Wire.endTransmission();
  // detach ip2315
  ioe1.digitalWrite(M5IOE1_PIN_11, LOW);

  // SD
#if defined(HAS_SDCARD)
  ioe1.pinMode(M5IOE1_PIN_14, OUTPUT_OPEN_DRAIN);
  ioe1.digitalWrite(M5IOE1_PIN_14, HIGH);
  delay(10);
  SD_MMC.setPins(SDCARD_CLK, SDCARD_CMD, SDCARD_DAT0, SDCARD_DAT1, SDCARD_DAT2, SDCARD_DAT3);
  SD_MMC.begin();
#endif
}

#endif
