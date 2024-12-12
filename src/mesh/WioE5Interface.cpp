#include "WioE5Interface.h"
#include "NodeDB.h"
#include "PowerMon.h"
#include "configuration.h"
#include "error.h"
#include <Lora-E5.h>

#ifndef WIOE5_MAX_POWER
#define WIOE5_MAX_POWER 22
#endif

#ifdef USE_WIOE5

LoRaE5Class _lora;
size_t length;

WioE5Interface::WioE5Interface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq,
                                           RADIOLIB_PIN_TYPE rst, RADIOLIB_PIN_TYPE busy)
    : SX126xInterface(hal, cs, irq, rst, busy)
{
}

bool WioE5Interface::init()
{
    LOG_DEBUG("WioE5 init");
    RadioLibInterface::init();

    if (power > WIOE5_MAX_POWER) // This chip has lower power limits than some
        power = WIOE5_MAX_POWER;

    limitPower();

    int res = 0;
    _lora.init(SERIAL_TX, SERIAL_RX);
    _lora.setDeviceBaudRate(BR_115200);
    _lora.Debug(lora_DEBUG);
    int tm = _lora.initP2PMode(getFreq(), _spreading_factor_t(sf), _band_width_t(bw), preambleLength, preambleLength, power);

    LOG_INFO("WioE5 init result %d", res);

    LOG_INFO("Frequency set to %f", getFreq());
    LOG_INFO("Bandwidth set to %f", bw);
    LOG_INFO("Power output set to %d", power);

    if (res == RADIOLIB_ERR_NONE)
        startReceive(); // start receiving

    return res == RADIOLIB_ERR_NONE;
}

bool WioE5Interface::reconfigure()
{
    LOG_DEBUG("WioE5 reconfigure");
    RadioLibInterface::reconfigure();

    return true;
}

bool WioE5Interface::sleep()
{
    LOG_DEBUG("WioE5 sleep");
    _lora.setDeviceLowPowerAutomode(true);

    return true;
}

void WioE5Interface::disableInterrupt()
{
    LOG_DEBUG("WioE5 disableInterrupt");
    return;
}

bool WioE5Interface::isChannelActive()
{
    LOG_DEBUG("WioE5 isChannelActive");
    return true;
}

bool WioE5Interface::isActivelyReceiving()
{
    LOG_DEBUG("WioE5 isActivelyReceiving");
    return false;
}

void WioE5Interface::startReceive()
{
    LOG_DEBUG("WioE5 startReceive");
    RadioLibInterface::startReceive();
    _lora.setDeviceWakeUp();
    int tm = _lora.initP2PMode(getFreq(), _spreading_factor_t(sf), _band_width_t(bw), preambleLength, preambleLength, power);
    return;
}

void WioE5Interface::configHardwareForSend()
{
    LOG_DEBUG("WioE5 configHardwareForSend");
    RadioLibInterface::configHardwareForSend();
    return;
}

void WioE5Interface::addReceiveMetadata(meshtastic_MeshPacket *mp)
{
    LOG_DEBUG("WioE5 addReceiveMetadata");
    return;
}

void WioE5Interface::setStandby()
{
    LOG_DEBUG("WioE5 setStandby");
    _lora.setDeviceLowPowerAutomode(true);

    return;
}

void WioE5Interface::loop()
{
    short rssi = 0;
//    length = _lora.receivePacketP2PMode((uint8_t *)&radioBuffer, 255, &rssi, 100);
    length = _lora.receivePacket((char *)&radioBuffer, 255, &rssi, 100);
    if (length > 0)
    {
        handleReceiveInterrupt();
    }

    return;
}

/** start an immediate transmit */
bool WioE5Interface::startSend(meshtastic_MeshPacket *txp)
{
    printPacket("Start low level send", txp);
    if (disabled || !config.lora.tx_enabled) {
        LOG_WARN("Drop Tx packet because LoRa Tx disabled");
        packetPool.release(txp);
        return false;
    } else {
        configHardwareForSend(); // must be after setStandby

        size_t numbytes = beginSending(txp);

        bool res = _lora.transferPacketP2PMode((uint8_t *)&radioBuffer, numbytes);
        if (!res) {
            LOG_ERROR("startTransmit failed");
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_RADIO_SPI_BUG);

            // This send failed, but make sure to 'complete' it properly
            completeSending();
            powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn); // Transmitter off now
            startReceive(); // Restart receive mode (because startTransmit failed to put us in xmit mode)
        }

        return res == RADIOLIB_ERR_NONE;
    }
}

void WioE5Interface::handleReceiveInterrupt()
{
    uint32_t xmitMsec;

    // when this is called, we should be in receive mode - if we are not, just jump out instead of bombing. Possible Race
    // Condition?
    if (!isReceiving) {
        LOG_ERROR("handleReceiveInterrupt called when not in rx mode, which shouldn't happen");
        return;
    }

    isReceiving = false;

    xmitMsec = getPacketTime(length);

#ifndef DISABLE_WELCOME_UNSET
    if (config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_UNSET) {
        LOG_WARN("lora rx disabled: Region unset");
        airTime->logAirtime(RX_ALL_LOG, xmitMsec);
        return;
    }
#endif

    // Skip the 4 headers that are at the beginning of the rxBuf
    int32_t payloadLen = length - sizeof(PacketHeader);

    // check for short packets
    if (payloadLen < 0) {
        LOG_WARN("Ignore received packet too short");
        rxBad++;
        airTime->logAirtime(RX_ALL_LOG, xmitMsec);
    } else {
        rxGood++;
        // altered packet with "from == 0" can do Remote Node Administration without permission
        if (radioBuffer.header.from == 0) {
            LOG_WARN("Ignore received packet without sender");
            return;
        }

        // Note: we deliver _all_ packets to our router (i.e. our interface is intentionally promiscuous).
        // This allows the router and other apps on our node to sniff packets (usually routing) between other
        // nodes.
        meshtastic_MeshPacket *mp = packetPool.allocZeroed();

        mp->from = radioBuffer.header.from;
        mp->to = radioBuffer.header.to;
        mp->id = radioBuffer.header.id;
        mp->channel = radioBuffer.header.channel;
        assert(HOP_MAX <= PACKET_FLAGS_HOP_LIMIT_MASK); // If hopmax changes, carefully check this code
        mp->hop_limit = radioBuffer.header.flags & PACKET_FLAGS_HOP_LIMIT_MASK;
        mp->hop_start = (radioBuffer.header.flags & PACKET_FLAGS_HOP_START_MASK) >> PACKET_FLAGS_HOP_START_SHIFT;
        mp->want_ack = !!(radioBuffer.header.flags & PACKET_FLAGS_WANT_ACK_MASK);
        mp->via_mqtt = !!(radioBuffer.header.flags & PACKET_FLAGS_VIA_MQTT_MASK);

        addReceiveMetadata(mp);

        mp->which_payload_variant =
            meshtastic_MeshPacket_encrypted_tag; // Mark that the payload is still encrypted at this point
        assert(((uint32_t)payloadLen) <= sizeof(mp->encrypted.bytes));
        memcpy(mp->encrypted.bytes, radioBuffer.payload, payloadLen);
        mp->encrypted.size = payloadLen;

        printPacket("Lora RX", mp);

        airTime->logAirtime(RX_LOG, xmitMsec);

        deliverToReceiver(mp);
    }
}

#endif // USE_WIOE5
