#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "../inc/rfm69.h"
#include "../inc/RFM69registers.h"


volatile uint8_t _mode; // current transceiver state
volatile uint8_t PAYLOADLEN;
uint8_t _address;

static struct spi_dt_spec spi_dev;
static struct gpio_dt_spec irq_pin;

void rfm_set_spi_dev(struct spi_dt_spec spispec)
{
    spi_dev = spispec;
}

void rfm_set_irq_pin(struct gpio_dt_spec gpiospec)
{
    irq_pin = gpiospec;
}

int rfm_read_reg(uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t tx_buffer = reg;
    struct spi_buf tx_spi_buf = {.buf = (void *)&tx_buffer, .len = 1};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_bufs = {.buf = data, .len = size};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_bufs, .count = 1};

    int err = spi_transceive_dt(&spi_dev, &tx_spi_buf_set, &rx_spi_buf_set);
    if (err < 0)
    {
        return err;
    }

    return 0;
}
int rfm_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[] = {(reg | 0x80), value};
    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    int err = spi_write_dt(&spi_dev, &tx_spi_buf_set);
    if (err < 0)
    {
        return err;
    }

    return 0;
}

int rfm_write_frame(uint8_t *buffer, uint8_t len)
{
    // Comment 28/11/24: I forgot why the len is like that, must be doing with spi data frame, it worked btw
    struct spi_buf tx_spi_buf = {.buf = buffer, .len = ((len >> 2) + 2) * sizeof(buffer)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    int err = spi_write_dt(&spi_dev, &tx_spi_buf_set);
    if (err < 0)
    {
        return err;
    }

    return 0;
}

bool rfm_can_send(struct spi_dt_spec spi_dev)
{
    if (_mode == RF69_MODE_RX && PAYLOADLEN == 0 && rfm_read_RSSI(false) < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
    {
        rfm_set_mode(RF69_MODE_STANDBY);
        return true;
    }
    return false;
}

void rfm_send(struct gpio_dt_spec dio_pin, uint8_t toAddress, void *buffer, uint8_t bufferSize, bool requestACK)
{
    uint8_t _size = 2;
    uint8_t _values[_size];
    rfm_read_reg(REG_PACKETCONFIG2, _values, _size);
    rfm_write_reg(REG_PACKETCONFIG2, (_values[1] & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
    // while (!rfm_can_send(spi_dev))
    //     receiveDone();
    rfm_send_frame(dio_pin, toAddress, buffer, bufferSize, requestACK, false);
}

void rfm_send_frame(struct gpio_dt_spec dio_pin, uint8_t toAddress, void *buffer, uint8_t bufferSize, bool requestACK, bool sendACK)
{
    uint8_t _size = 2;
    uint8_t _values[_size];
    rfm_set_mode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
    do
    {
        rfm_read_reg(REG_IRQFLAGS1, _values, _size);
    } while ((_values[1] & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
    rfm_write_reg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
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

    rfm_write_frame(_buffer, bufferSize);

    // no need to wait for transmit mode to be ready since its handled by the radio
    rfm_set_mode(RF69_MODE_TX);
    while (gpio_pin_get_dt(&dio_pin) == 0)
        ;
    // wait for DIO0 to turn HIGH signalling transmission finish
    // while (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // wait for ModeReady
    rfm_set_mode(RF69_MODE_STANDBY);
}

void rfm_reset(struct gpio_dt_spec rst_pin)
{
    gpio_pin_set_dt(&rst_pin, 1);
    k_msleep(1);
    gpio_pin_set_dt(&rst_pin, 0);
    k_msleep(10);
}



void rfm_set_mode(uint8_t mode)
{
    uint8_t _size = 2;
    uint8_t _values[_size];

    rfm_read_reg(REG_OPMODE, _values, _size);

    switch (mode)
    {
    case RF69_MODE_TX:
        rfm_write_reg(REG_OPMODE, (_values[1] & 0xE3) | RF_OPMODE_TRANSMITTER);
        if (IS_RFM69HW)
            rfm_set_high_power_reg(true);
        break;
    case RF69_MODE_RX:
        rfm_write_reg(REG_OPMODE, (_values[1] & 0xE3) | RF_OPMODE_RECEIVER);
    case RF69_MODE_SYNTH:
        rfm_write_reg(REG_OPMODE, (_values[1] & 0xE3) | RF_OPMODE_SYNTHESIZER);
        break;
    case RF69_MODE_STANDBY:
        rfm_write_reg(REG_OPMODE, (_values[1] & 0xE3) | RF_OPMODE_STANDBY);
        break;
    case RF69_MODE_SLEEP:
        rfm_write_reg(REG_OPMODE, (_values[1] & 0xE3) | RF_OPMODE_SLEEP);
        break;
    default:
        return;
    }
}

void rfm_set_high_power(void)
{
    uint8_t _size = 2;
    uint8_t _values[_size];

    //rfm_write_reg(REG_OCP, RF_OCP_OFF);

    if (IS_RFM69HW)
    {
        rfm_read_reg(REG_PALEVEL, _values, _size);                                               // turning ON
        rfm_write_reg(REG_PALEVEL, (_values[1] & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
    }
    else
        rfm_write_reg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | POWER_LEVEL_MAX); // enable P0 only
}

void rfm_set_power_Level(uint8_t powerLevel)
{
    uint8_t _size = 2;
    uint8_t _values[_size];
    uint8_t _powerLevel = (powerLevel > POWER_LEVEL_MAX ? POWER_LEVEL_MAX : powerLevel);

    
    rfm_read_reg(REG_PALEVEL, _values, _size);
    rfm_write_reg(REG_PALEVEL, (_values[1] & 0xE0) | _powerLevel);
}

void rfm_set_high_power_reg(bool onOff)
{
    rfm_write_reg(REG_TESTPA1, onOff ? 0x5D : 0x55);
    rfm_write_reg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

int16_t rfm_read_RSSI(bool force_trigger)
{
    int16_t rssi = 0;
    uint8_t _size = 2;
    uint8_t _values[_size];

    if (force_trigger)
    {
        // RSSI trigger not needed if DAGC is in continuous mode
        rfm_write_reg(REG_RSSICONFIG, RF_RSSI_START);
        do
        {
            rfm_read_reg(REG_RSSICONFIG, _values, _size);
        } while ((_values[1] & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
    }
    rfm_read_reg(REG_RSSIVALUE, _values, _size);
    rssi = -_values[1];
    rssi >>= 1;
    return rssi;
}

uint32_t rfm_get_freq(void)
{
    uint8_t _size = 2;
    uint8_t _values[_size];

    rfm_read_reg(REG_FRFMSB, _values, _size);
    uint32_t val_1 = _values[1] << 16;
    rfm_read_reg(REG_FRFMID, _values, _size);
    uint32_t val_2 = _values[1] << 8;
    rfm_read_reg(REG_FRFLSB, _values, _size);
    uint32_t val_3 = _values[1];
    return (uint32_t)RF69_FSTEP * (val_1 + val_2 + val_3);
}
