#pragma once

#include "SX126xInterface.h"

#ifdef USE_WIOE5

/**
 * Our adapter for Grove Wio E5 radios
 */
class WioE5Interface : public SX126xInterface<SX1262>
{
  public:
    WioE5Interface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                         RADIOLIB_PIN_TYPE busy);

    bool init() override;

    bool reconfigure() override;

    bool sleep() override;

    bool isIRQPending() override { return false; }

    /**
     * Glue functions called from ISR land
     */
    void disableInterrupt() override;

    /**
     * Enable a particular ISR callback glue function
     */
    void enableInterrupt(void (*callback)()) { callback(); }

    /** can we detect a LoRa preamble on the current channel? */
    bool isChannelActive() override;

    /** are we actively receiving a packet (only called during receiving state) */
    bool isActivelyReceiving() override;

    /**
     * Start waiting to receive a message
     */
    void startReceive() override;

    /**
     *  We override to turn on transmitter power as needed.
     */
    void configHardwareForSend() override;

    /**
     * Add SNR data to received messages
     */
    void addReceiveMetadata(meshtastic_MeshPacket *mp) override;

    void setStandby() override;

    void loop();


    bool startSend(meshtastic_MeshPacket *txp) override;

    void handleReceiveInterrupt();
};

// https://github.com/Seeed-Studio/LoRaWan-E5-Node/blob/main/Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/radio_driver.c
//static const float tcxoVoltage = 1.7;

/* https://wiki.seeedstudio.com/LoRa-E5_STM32WLE5JC_Module/
 * Wio-E5 module ONLY transmits through RFO_HP
 * Receive: PA4=1, PA5=0
 * Transmit(high output power, SMPS mode): PA4=0, PA5=1 */
//static const RADIOLIB_PIN_TYPE rfswitch_pins[5] = {PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};

//static const Module::RfSwitchMode_t rfswitch_table[4] = {
//    {STM32WLx::MODE_IDLE, {LOW, LOW}}, {STM32WLx::MODE_RX, {HIGH, LOW}}, {STM32WLx::MODE_TX_HP, {LOW, HIGH}}, END_OF_MODE_TABLE};

#endif // USE_WIOE5
