#include <Arduino.h>
#include <M5IOE1.h>
#include <M5PM1.h>
#include "variant.h"

class PCA9557 {
public:

    /**
     * @brief Write digital level (no return value)
     * @param pin GPIO pin number
     * @param value Level: LOW or HIGH
     * @note Use digitalWriteWithRes for error codes
     */
    void digitalWrite(uint8_t pin, uint8_t value);

};
