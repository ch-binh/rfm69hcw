

#include "../inc/rfm69.h"
#include "../inc/rfm69_hw.h"
#include "../inc/rfm69_reg.h"

volatile uint8_t _mode; // current transceiver state
volatile uint8_t PAYLOADLEN;
uint8_t _address;


bool rfm69_ready_to_send(void)
{
    if (_mode == RF69_MODE_RX && PAYLOADLEN == 0 && rfm_read_RSSI(false) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
    {
        rfm69_set_mode(RF69_MODE_STANDBY);
        return true;
    }
    return false;
}

void rfm69_send_msg(uint8_t toAddress, void *buffer, uint8_t bufferSize, bool requestACK)
{
    uint8_t _size = 2;
    uint8_t _values[_size];
    rfm69_spi_read_reg(REG_PACKETCONFIG2, _values, _size);
    rfm69_spi_write_reg(REG_PACKETCONFIG2, (_values[1] & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
    // while (!rfm69_ready_to_send())
    //     receiveDone();
    rfm69_send_msg_frame(toAddress, buffer, bufferSize, requestACK, false);
}

void rfm69_send_msg_frame(uint8_t toAddress, void *buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{
    uint8_t _size = 2;
    uint8_t _values[_size];
    rfm69_set_mode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
    do
    {
        rfm69_spi_read_reg(REG_IRQFLAGS1, _values, _size);
    } while ((_values[1] & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
    rfm69_spi_write_reg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
    if (bufferSize > RF69_MAX_DATA_LEN)
        bufferSize = RF69_MAX_DATA_LEN;

    // control byte
    uint8_t CTLbyte = 0x00;
    if (sendACK)
        CTLbyte = RFM69_CTL_SENDACK;
    else if (requestACK)
        CTLbyte = RFM69_CTL_REQACK;

    // write to FIFO

    uint8_t _buffer[bufferSize + 5];

    _buffer[0] = (REG_FIFO | 0x80);
    _buffer[1] = (bufferSize + 3);
    _buffer[2] = (toAddress);
    _buffer[3] = (_address);
    _buffer[4] = (CTLbyte);

    for (uint8_t i = 5; i < bufferSize + 5; i++)
        _buffer[i] = ((uint8_t *)buffer)[i - 5];

    rfm69_spi_write_frame(_buffer, bufferSize);

    // no need to wait for transmit mode to be ready since its handled by the radio
    rfm69_set_mode(RF69_MODE_TX);
    while (rfm69_gpio_dio0_read() == 0)
        ;
    // wait for DIO0 to turn HIGH signalling transmission finish
    // while (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady
    rfm69_set_mode(RF69_MODE_STANDBY);
}

void rfm69_set_mode(uint8_t mode)
{
    uint8_t _size = 2;
    uint8_t _values[_size];

    rfm69_spi_read_reg(REG_OPMODE, _values, _size);

    switch (mode)
    {
    case RF69_MODE_TX:
        rfm69_spi_write_reg(REG_OPMODE, (_values[1] & 0xE3) | RF_OPMODE_TRANSMITTER);
        if (IS_RFM69HW)
            rfm_set_high_power_reg(true);
        break;
    case RF69_MODE_RX:
        rfm69_spi_write_reg(REG_OPMODE, (_values[1] & 0xE3) | RF_OPMODE_RECEIVER);
    case RF69_MODE_SYNTH:
        rfm69_spi_write_reg(REG_OPMODE, (_values[1] & 0xE3) | RF_OPMODE_SYNTHESIZER);
        break;
    case RF69_MODE_STANDBY:
        rfm69_spi_write_reg(REG_OPMODE, (_values[1] & 0xE3) | RF_OPMODE_STANDBY);
        break;
    case RF69_MODE_SLEEP:
        rfm69_spi_write_reg(REG_OPMODE, (_values[1] & 0xE3) | RF_OPMODE_SLEEP);
        break;
    default:
        return;
    }
}

void rfm69_set_high_power(void)
{
    uint8_t _size = 2;
    uint8_t _values[_size];

    // rfm69_spi_write_reg(REG_OCP, RF_OCP_OFF);

    if (IS_RFM69HW)
    {
        rfm69_spi_read_reg(REG_PALEVEL, _values, _size);                                               // turning ON
        rfm69_spi_write_reg(REG_PALEVEL, (_values[1] & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
    }
    else
        rfm69_spi_write_reg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | POWER_LEVEL_MAX); // enable P0 only
}

void rfm69_set_power_Level(uint8_t powerLevel)
{
    uint8_t _size = 2;
    uint8_t _values[_size];
    uint8_t _powerLevel = (powerLevel > POWER_LEVEL_MAX ? POWER_LEVEL_MAX : powerLevel);

    rfm69_spi_read_reg(REG_PALEVEL, _values, _size);
    rfm69_spi_write_reg(REG_PALEVEL, (_values[1] & 0xE0) | _powerLevel);
}

void rfm_set_high_power_reg(bool onOff)
{
    rfm69_spi_write_reg(REG_TESTPA1, onOff ? 0x5D : 0x55);
    rfm69_spi_write_reg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

int16_t rfm_read_RSSI(bool force_trigger)
{
    int16_t rssi = 0;
    uint8_t _size = 2;
    uint8_t _values[_size];

    if (force_trigger)
    {
        // RSSI trigger not needed if DAGC is in continuous mode
        rfm69_spi_write_reg(REG_RSSICONFIG, RF_RSSI_START);
        do
        {
            rfm69_spi_read_reg(REG_RSSICONFIG, _values, _size);
        } while ((_values[1] & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
    }
    rfm69_spi_read_reg(REG_RSSIVALUE, _values, _size);
    rssi = -_values[1];
    rssi >>= 1;
    return rssi;
}

uint32_t rfm_get_freq(void)
{
    uint8_t _size = 2;
    uint8_t _values[_size];

    rfm69_spi_read_reg(REG_FRFMSB, _values, _size);
    uint32_t val_1 = _values[1] << 16;
    rfm69_spi_read_reg(REG_FRFMID, _values, _size);
    uint32_t val_2 = _values[1] << 8;
    rfm69_spi_read_reg(REG_FRFLSB, _values, _size);
    uint32_t val_3 = _values[1];
    return (uint32_t)RF69_FSTEP * (val_1 + val_2 + val_3);
}
