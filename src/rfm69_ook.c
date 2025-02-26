#include <stdio.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "../inc/rfm69_ook.h"
#include "../inc/rfm69.h"
#include "../inc/RFM69registers.h"
#include "../inc/rfm69_hw.h"

/* variables declaration */
static struct spi_dt_spec spi_dev;
static struct gpio_dt_spec irq_pin;

/* function declaration*/
/**
 * @brief Sends an OOK signal through the specified GPIO pin.
 *
 * @param dio_pin The GPIO pin to send the signal through.
 * @param duration The duration of the signal in microseconds.
 * @param offset The offset time in microseconds to adjust the signal timing.
 * @param is_high Boolean indicating whether to set the pin high (true) or low (false).
 */
static void rfm_ook_send(struct gpio_dt_spec dio_pin, uint16_t duration, uint16_t offset, bool is_high);

/* Functions */

void rfm_ook_init(struct spi_dt_spec spispec,
                  struct gpio_dt_spec gpiospec,
                  uint8_t freqBand,
                  uint8_t pwr_lvl,
                  uint8_t mode)
{
    rfm_ook_set_spi_dev(spispec);
    rfm_ook_set_irq_pin(gpiospec);
    rfm_ook_reg_init(freqBand);
    rfm_set_mode(mode);
    rfm_set_power_Level(pwr_lvl);
}

void rfm_ook_deinit(void)
{
    rfm_set_mode(RF69_MODE_SLEEP);
    gpio_pin_configure_dt(&irq_pin, GPIO_DISCONNECTED);
}

void rfm_ook_set_spi_dev(struct spi_dt_spec spispec)
{
    rfm_set_spi_dev(spispec);
    spi_dev = spispec;
}

void rfm_ook_set_irq_pin(struct gpio_dt_spec gpiospec)
{
    rfm_set_irq_pin(gpiospec);
    irq_pin = gpiospec;
    gpio_pin_configure_dt(&irq_pin, GPIO_OUTPUT_INACTIVE);
}

bool rfm_ook_reg_init(uint8_t freqBand)
{
    uint8_t _size = 2;
    uint8_t _values[_size];

    const uint8_t CONFIG[][2] =
        {
            /* 0x01 */ {REG_OPMODE, RF_OPMODE_SEQUENCER_OFF | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY},
            /* 0x02 */ {REG_DATAMODUL, RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00}, // no shaping
            /* 0x03 */ {REG_BITRATEMSB, RF69_BITRATE_MSB_VAL},                                                                                         // bitrate: 32768 Hz
            /* 0x04 */ {REG_BITRATELSB, RF69_BITRATE_LSB_VAL},

            /* 0x07 */ {REG_FRFMSB, (uint8_t)(freqBand == RF69_315MHZ ? RF_FRFMSB_315 : (freqBand == RF69_433MHZ ? RF_FRFMSB_433 : (freqBand == RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915)))},
            /* 0x08 */ {REG_FRFMID, (uint8_t)(freqBand == RF69_315MHZ ? RF_FRFMID_315 : (freqBand == RF69_433MHZ ? RF_FRFMID_433 : (freqBand == RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915)))},
            /* 0x09 */ {REG_FRFLSB, (uint8_t)(freqBand == RF69_315MHZ ? RF_FRFLSB_315 : (freqBand == RF69_433MHZ ? RF_FRFLSB_433 : (freqBand == RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915)))},
            /* 0x11 */ {REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON | POWER_LEVEL_MAX}, // enable P1 & P2 amplifier stages
            /* 0x13 */ {REG_OCP, RF_OCP_OFF},                                                                      // default: On, 120 mA
                                                                                                                   /* ================== This is for communication using OOK, not important for WURX ================*/
            /* 0x19 */ {REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_4},                          // BW: 10.4 kHz
            /* 0x1B */ {REG_OOKPEAK, RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000},
            /* 0x1D */ {REG_OOKFIX, RF_OOKFIX_FIXEDTHRESH_VALUE}, // Fixed threshold value (in dB) in the OOK demodulator
            /* 0x29 */ {REG_RSSITHRESH, RF_RSSITHRESH_VALUE},     // RSSI threshold in dBm = -(REG_RSSITHRESH / 2)
            /* 0x6F */ {REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0}, // run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
            {255, 0}};

    for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
        rfm_write_reg(CONFIG[i][0], CONFIG[i][1]);

    // rfm_set_high_power(); // called regardless if it's a RFM69W or RFM69HW
    rfm_set_high_power_reg(true);
    rfm_set_mode(RF69_MODE_STANDBY);

    do
    {
        rfm_read_reg(REG_IRQFLAGS1, _values, _size);
    } while ((_values[1] & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

    return true;
}

void rfm_ook_set_freq_mhz(float f)
{
    uint32_t freqHz = f * 1000000;
    freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
    rfm_write_reg(REG_FRFMSB, freqHz >> 16);
    rfm_write_reg(REG_FRFMID, freqHz >> 8);
    rfm_write_reg(REG_FRFLSB, freqHz);
}

void rfm_ook_send_bt_inq(uint16_t on_time, uint16_t off_time, uint32_t interval, uint32_t density)
{
    for (int i = 0; i < density; i++)
    {
        // Send the signal high for the specified on_time
        rfm_ook_send(irq_pin, on_time, 10, 1);
        // Send the signal low for the specified off_time
        rfm_ook_send(irq_pin, off_time, 10, 0);

        // Repeat the high and low signal
        rfm_ook_send(irq_pin, on_time, 10, 1);
        rfm_ook_send(irq_pin, off_time, 10, 0);

        // Wait for the remaining interval time
        k_busy_wait(interval - on_time * 2 - off_time * 2);
    }
}
void rfm_ook_send_ibeacon(uint16_t on_time, uint16_t off_time, uint32_t interval, uint32_t density)
{
    uint8_t offset = 10;
    for (int i = 0; i < density; i++)
    {
        rfm_ook_send(irq_pin, on_time, offset, ON);
        rfm_ook_send(irq_pin, off_time, offset, OFF);

        rfm_ook_send(irq_pin, on_time, offset, ON);
        rfm_ook_send(irq_pin, off_time, offset, OFF);

        rfm_ook_send(irq_pin, on_time, offset, ON);
        rfm_ook_send(irq_pin, off_time, offset, OFF);

        k_busy_wait(interval - on_time * 3 - off_time * 3);
    }
}

/**
 * @brief Sends a custom RXID signal using OOK modulation.
 *
 * @param rxid The RXID to be sent.
 * @param bit_len The duration in microseconds for each bit.
 * @param time_interval The total interval time in microseconds for one cycle of the signal.
 * @param repeat The number of times to repeat the signal pattern.
 */
void rfm_ook_send_custom_rxid(uint16_t rxid, uint16_t bit_len, uint32_t time_interval, uint32_t repeat)
{
    // Initialize the bit mask to check each bit of the RXID
    uint32_t rxid_bit_mask = 1 << (RXID_BIT_SIZE - 1);
    uint8_t offset = OFFSET_VAL; // offset because of time delay from previous cmd/calculations

    // Repeat the signal pattern for the specified number of times
    for (int i = 0; i < repeat; i++)
    {
        rxid_bit_mask = 1 << (RXID_BIT_SIZE - 1);
        // Loop through each bit of the RXID
        for (int j = 0; j < RXID_BIT_SIZE; j++)
        {
            if ((rxid & rxid_bit_mask) == rxid_bit_mask)
            {
                rfm_ook_send(irq_pin, bit_len, offset, ON);
            }
            else
            {
                rfm_ook_send(irq_pin, bit_len, offset, OFF);
            }
            rxid_bit_mask >>= 1;
        }
        // Ensure the last bit to be 0
        rfm_ook_send(irq_pin, bit_len, offset, OFF);

        k_busy_wait(time_interval - bit_len * RXID_BIT_SIZE);
    }
}

static void rfm_ook_send(struct gpio_dt_spec dio_pin, uint16_t duration, uint16_t offset, bool is_high)
{
    if (is_high)
    {
        gpio_pin_configure_dt(&dio_pin, GPIO_OUTPUT_ACTIVE);
    }
    else
    {
        gpio_pin_configure_dt(&dio_pin, GPIO_OUTPUT_INACTIVE);
    }

    k_busy_wait(duration - offset);
}