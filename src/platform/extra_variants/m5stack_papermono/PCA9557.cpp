#include "PCA9557.h"

#ifdef PCA_PIN_EINK_EN

extern M5IOE1 ioe1;
extern M5PM1 pm;

void PCA9557::digitalWrite(uint8_t pin, uint8_t value)
{
    // E-Paper EN と Backlight のオン/オフを連動する
    if (pin == PCA_PIN_EINK_EN)
    {
        ioe1.digitalWrite(M5IOE1_PIN_3, value);
        if (value == LOW)
        {
            pm.analogWrite(M5PM1_PWM_CH_0, 0);
        }
        else
        {
            pm.analogWrite(M5PM1_PWM_CH_0, 10);
        }
    }
    else
    {
        ioe1.digitalWrite(pin, value);
    }
}

#endif
