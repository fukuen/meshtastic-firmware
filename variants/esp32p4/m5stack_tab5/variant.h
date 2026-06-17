// #define BUTTON_NEED_PULLUP // if set we need to turn on the internal CPU pullup during sleep

#define I2C_SDA 31
#define I2C_SCL 32

// #define BUTTON_PIN 0
// #define BUTTON_NEED_PULLUP

// #define PIN_BUZZER -1

#define HW_SPI1_DEVICE
#undef LORA_SCK
#undef LORA_MISO
#undef LORA_MOSI
#undef LORA_CS

#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 18
#define LORA_CS 4

#define USE_RF95
#define LORA_DIO0 16 // a No connect on the SX1262 module
#define LORA_RESET 45
#define LORA_DIO1 RADIOLIB_NC // Not really used
#define LORA_DIO2 RADIOLIB_NC // Not really used

// This board has different GPS pins than all other boards
// #undef GPS_RX_PIN
// #undef GPS_TX_PIN
// #define GPS_RX_PIN 53
// #define GPS_TX_PIN 54

#define USE_TFTDISPLAY 1
#define GRAPHICS_TFT_COLORING_ENABLED 1
#define ST7789_CS // for Large Display settings
#define TFT_HEIGHT 720
#define TFT_WIDTH 1280
#define TFT_OFFSET_X 0
#define TFT_OFFSET_Y 0
#define TFT_BUSY -1

// #define USE_VIRTUAL_KEYBOARD 1

#define HAS_TOUCHSCREEN 1
#define TOUCH_I2C_PORT 0
#define TOUCH_SLAVE_ADDRESS 0x14
#define SCREEN_TOUCH_INT 23

// audio codec ES8388/ES7210
#define HAS_I2S
#define DAC_I2S_BCK 27
#define DAC_I2S_WS 29
#define DAC_I2S_DOUT 26 // ES8388
#define DAC_I2S_DIN 28  // ES7210
#define DAC_I2S_MCLK 30

#ifdef MESHTASTIC_EXCLUDE_WIFI
#undef MESHTASTIC_EXCLUDE_WIFI
#endif
#define MESHTASTIC_EXCLUDE_WEBSERVER 1
#define HAS_WIFI 1

#define SDIO2_CLK GPIO_NUM_12
#define SDIO2_CMD GPIO_NUM_13
#define SDIO2_D0 GPIO_NUM_11
#define SDIO2_D1 GPIO_NUM_10
#define SDIO2_D2 GPIO_NUM_9
#define SDIO2_D3 GPIO_NUM_8
#define SDIO2_RST GPIO_NUM_15
