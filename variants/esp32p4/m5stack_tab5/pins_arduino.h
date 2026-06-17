#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>

#define USB_VID 0x303A
#define USB_PID 0x1001

static const uint8_t TX = 54; // 16;
static const uint8_t RX = 53; // 17;

static const uint8_t SDA = 31; // 10;
static const uint8_t SCL = 32; // 8;

// Default SPI will be mapped to Radio
static const uint8_t MISO = 19;
static const uint8_t SCK = 5;
static const uint8_t MOSI = 18;
static const uint8_t SS = -1;

// Have SPI interface SD card slot
// #define HAS_SDCARD // --> needs to be in platform.ini for device-ui
#define SPI_MOSI (44)
#define SPI_SCK (43)
#define SPI_MISO (39)
#define SPI_CS (42)
#define SDCARD_CS SPI_CS
#define SD_SPI_FREQUENCY 25000000U

#endif /* Pins_Arduino_h */
