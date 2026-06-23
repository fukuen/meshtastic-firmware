// pass 2 only
#ifdef ESP32_ARDUINO_LIB_BUILDER
#ifdef M5STACK_TAB5

#include "configuration.h"

#include "AudioBoard.h"
#include <Wire.h>

DriverPins PinsAudioBoardES8388;
AudioBoard board(AudioDriverES8388, PinsAudioBoardES8388);

// PI4IOE5V6408
#define PI4IO_ADDR 0x43

static TwoWire *findBus()
{
    TwoWire *candidates[] = {&Wire, &Wire1};
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++)
    {
        candidates[i]->beginTransmission(PI4IO_ADDR);
        if (candidates[i]->endTransmission() == 0)
        {
            return candidates[i];
        }
    }
    return nullptr;
}

void lateInitVariant()
{
    // TAB5 SDIO WiFi
    WiFi.setPins(SDIO2_CLK, SDIO2_CMD, SDIO2_D0, SDIO2_D1, SDIO2_D2, SDIO2_D3, SDIO2_RST);

    TwoWire *bus = findBus();
    if (!bus)
    {
        LOG_ERROR("i2c not found");
        return;
    }
    // AudioDriverLogger.begin(Serial, AudioDriverLogLevel::Debug);
    //  I2C: function, scl, sda
    PinsAudioBoardES8388.addI2C(PinFunction::CODEC, *bus);
    // I2S: function, mclk, bck, ws, data_out, data_in
    PinsAudioBoardES8388.addI2S(PinFunction::CODEC, DAC_I2S_MCLK, DAC_I2S_BCK, DAC_I2S_WS, DAC_I2S_DOUT, DAC_I2S_DIN);

    // configure codec
    CodecConfig cfg;
    cfg.input_device = ADC_INPUT_LINE1;
    cfg.output_device = DAC_OUTPUT_ALL;
    cfg.i2s.bits = BIT_LENGTH_16BITS;
    cfg.i2s.rate = RATE_44K;
    board.begin(cfg);

    // extra ES8388 init
    auto es8388_write_reg = [](TwoWire *bus, uint8_t reg, uint8_t val)
    {
        bus->beginTransmission(0x10); // ES8388 i2c address
        bus->write(reg);
        bus->write(val);
        bus->endTransmission();
    };
    es8388_write_reg(bus, 0, 0x80); // reset, power on
    es8388_write_reg(bus, 0, 0x00);
    es8388_write_reg(bus, 0, 0x00);
    es8388_write_reg(bus, 0, 0x0E);
    es8388_write_reg(bus, 1, 0x00);
    es8388_write_reg(bus, 2, 0x0A);  // CHIP POWER: power up all
    es8388_write_reg(bus, 3, 0xFF);  // ADC POWER: power down all
    es8388_write_reg(bus, 4, 0x3C);  // DAC POWER: power up and LOUT1/ROUT1/LOUT2/ROUT2 enable
    es8388_write_reg(bus, 5, 0x00);  // ChipLowPower1
    es8388_write_reg(bus, 6, 0x00);  // ChipLowPower2
    es8388_write_reg(bus, 7, 0x7C);  // VSEL
    es8388_write_reg(bus, 8, 0x00);  // set I2S slave mode
    es8388_write_reg(bus, 23, 0x18); // I2S format (16bit)
    es8388_write_reg(bus, 24, 0x00); // I2S MCLK ratio (128)
    es8388_write_reg(bus, 25, 0x20); // DAC unmute
    es8388_write_reg(bus, 26, 0x00); // LDACVOL 0x00~0xC0
    es8388_write_reg(bus, 27, 0x00); // RDACVOL 0x00~0xC0
    es8388_write_reg(bus, 28, 0x08); // enable digital click free power up and down
    es8388_write_reg(bus, 29, 0x00);
    es8388_write_reg(bus, 38, 0x00); // DAC CTRL16
    es8388_write_reg(bus, 39, 0xB8); // LEFT Ch MIX
    es8388_write_reg(bus, 42, 0xB8); // RIGHTCh MIX
    es8388_write_reg(bus, 43, 0x08); // ADC and DAC separate
    es8388_write_reg(bus, 45, 0x00); // 0x00=1.5k VREF analog output / 0x10=40kVREF analog output
    es8388_write_reg(bus, 46, 0x21);
    es8388_write_reg(bus, 47, 0x21);
    es8388_write_reg(bus, 48, 0x21);
    es8388_write_reg(bus, 48, 0x21);

    LOG_INFO("ES8388 Audio initialized.");
}

#endif
#endif
