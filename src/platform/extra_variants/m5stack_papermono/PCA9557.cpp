#include "PCA9557.h"

#ifdef PCA_PIN_EINK_EN

extern M5IOE1 ioe1;
extern M5PM1 pm;

void PCA9557::digitalWrite(uint8_t pin, uint8_t value)
{
    if (pin == PCA_PIN_EINK_EN)
    {
        // フロントライトのみ制御。E-Ink 電源 (M5IOE1_PIN_3) には触れない
        pm.analogWrite(M5PM1_PWM_CH_0, value == LOW ? 0 : 30);
    }
    else
    {
        ioe1.digitalWrite(pin, value);
    }
}

#endif
