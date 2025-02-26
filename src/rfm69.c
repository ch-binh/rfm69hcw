#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "../inc/rfm69.h"


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

bool rfm_fsk_reg_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID)
{
    uint8_t _size = 2;
    uint8_t _values[_size];

    const uint8_t CONFIG[][2] =
        {
            /* 0x01 */ {REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY},
            /* 0x02 */ {REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00}, // no shaping
            /* 0x03 */ {REG_BITRATEMSB, RF_BITRATEMSB_55555},                                                                               // default: 4.8 KBPS
            /* 0x04 */ {REG_BITRATELSB, RF_BITRATELSB_55555},
            /* 0x05 */ {REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
            /* 0x06 */ {REG_FDEVLSB, RF_FDEVLSB_50000},

            /* 0x07 */ {REG_FRFMSB, (uint8_t)(freqBand == RF69_315MHZ ? RF_FRFMSB_315 : (freqBand == RF69_433MHZ ? RF_FRFMSB_433 : (freqBand == RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915)))},
            /* 0x08 */ {REG_FRFMID, (uint8_t)(freqBand == RF69_315MHZ ? RF_FRFMID_315 : (freqBand == RF69_433MHZ ? RF_FRFMID_433 : (freqBand == RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915)))},
            /* 0x09 */ {REG_FRFLSB, (uint8_t)(freqBand == RF69_315MHZ ? RF_FRFLSB_315 : (freqBand == RF69_433MHZ ? RF_FRFLSB_433 : (freqBand == RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915)))},

            // for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
            /* 0x25 */ {REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01},    // DIO0 is the only IRQ we're using
            /* 0x26 */ {REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF}, // DIO5 ClkOut disable for power saving
            /* 0x28 */ {REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN},    // writing to this bit ensures that the FIFO & status flags are reset
            /* 0x29 */ {REG_RSSITHRESH, 220},                        // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm

            /* 0x2E */ {REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0},
            /* 0x2F */ {REG_SYNCVALUE1, 0x2D},      // attempt to make this compatible with sync1 byte of RFM12B lib
            /* 0x30 */ {REG_SYNCVALUE2, networkID}, // NETWORK ID
            /* 0x37 */ {REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF},
            /* 0x38 */ {REG_PAYLOADLENGTH, 66},                                                                                 // in variable length mode: the max frame size, not used in TX
                                                                                                                                ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
            /* 0x3C */ {REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE},                              // TX on FIFO not empty
            /* 0x3D */ {REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF}, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
                                                                                                                                // for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
            /* 0x6F */ {REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0},                                                               // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
            {255, 0}};

    do
    {
        rfm_write_reg(REG_SYNCVALUE1, 0xAA);
        rfm_read_reg(REG_SYNCVALUE1, _values, _size);
    } while (_values[1] != 0xaa);

    do
    {
        rfm_write_reg(REG_SYNCVALUE1, 0x55);
        rfm_read_reg(REG_SYNCVALUE1, _values, _size);
    } while (_values[1] != 0x55);

    for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
        rfm_write_reg(CONFIG[i][0], CONFIG[i][1]);

    // setHighPower(IS_RFM69HW); // called regardless if it's a RFM69W or RFM69HW
    rfm_set_mode(RF69_MODE_STANDBY);

    do
    {
        rfm_read_reg(REG_IRQFLAGS1, _values, _size);
    } while ((_values[1] & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

    _address = nodeID;
    return true;
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
