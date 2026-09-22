#define I2C_SDA 47
#define I2C_SCL 48

#define BUTTON_NEED_PULLUP
#define BUTTON_PIN 2 // Button A
#define PIN_BUTTON2 3

// BUZZER
#define PIN_BUZZER 42

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
#define LORA_DIO1 5           // IRQ
#define LORA_RESET -1         // RESET
#define LORA_RST -1           // RESET
#define LORA_IRQ 5            // DIO0
#define LORA_BUSY 21
#define LORA_DIO2 RADIOLIB_NC // Not really used
#define LORA_DIO3 RADIOLIB_NC

#define SX126X_CS LORA_CS
#define SX126X_DIO1 LORA_DIO1
#define SX126X_BUSY LORA_BUSY
#define SX126X_RESET LORA_RESET

#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8
#define TCXO_OPTIONAL

// BMI270 6-axis IMU on internal I2C bus
// #define HAS_BMI270

#define USE_EINK
// https://docs.m5stack.com/en/core/coreink
// https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/schematic/Core/coreink/coreink_sch.pdf
// #define PIN_EINK_EN -1   // N/C
#define PIN_EINK_CS 16   // EPD_CS
#define PIN_EINK_BUSY 18 // EPD_BUSY
#define PIN_EINK_DC 17   // EPD_D/C
#define PIN_EINK_RES -1  // Connected but not needed
#define PIN_EINK_SCLK 15 // EPD_SCLK
#define PIN_EINK_MOSI 14 // EPD_MOSI

//#define HAS_PCA9557
//#define PCA_PIN_EINK_EN M5IOE1_PIN_3
//#define GPIO_BACKLIGHT_DEFAULT_ON

//#define HAS_TOUCHSCREEN 1
//#define SCREEN_TOUCH_INT 4

//#define USE_POWERSAVE
//#define SLEEP_TIME 120

//#define RX8130CE_RTC 0x32

//#define HAS_SDCARD
//#define HAS_SD_MMC
#define SDCARD_CMD 12
#define SDCARD_CLK 13
#define SDCARD_DAT0 11
#define SDCARD_DAT1 10
#define SDCARD_DAT2 9
#define SDCARD_DAT3 8
