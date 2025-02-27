#include "../inc/rfm69_ook.h"
#include "../inc/rfm69.h"
#include "../inc/RFM69registers.h"
#include "../inc/rfm69_hw.h"

/* function declaration*/
/**
 * @brief Sends an OOK signal through the specified GPIO pin.
 *
 * @param dio_pin The GPIO pin to send the signal through.
 * @param duration The duration of the signal in microseconds.
 * @param offset The offset time in microseconds to adjust the signal timing.
 * @param is_high Boolean indicating whether to set the pin high (true) or low (false).
 */
void rfm69_ook_send(uint16_t duration, uint16_t offset, bool is_high);

/* Functions */

void rfm69_ook_init(uint8_t freqBand, uint8_t pwr_lvl, uint8_t mode)
{
    rfm69_ook_reg_init(freqBand);
    rfm69_set_mode(mode);
    rfm69_set_power_Level(pwr_lvl);
}

void rfm69_ook_deinit(void)
{
    rfm69_set_mode(RF69_MODE_SLEEP);
}

bool rfm69_ook_reg_init(uint8_t freqBand)
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
        rfm69_spi_write_reg(CONFIG[i][0], CONFIG[i][1]);

    // rfm69_set_high_power(); // called regardless if it's a RFM69W or RFM69HW
    rfm_set_high_power_reg(true);
    rfm69_set_mode(RF69_MODE_STANDBY);

    do
    {
        rfm69_spi_read_reg(REG_IRQFLAGS1, _values, _size);
    } while ((_values[1] & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

    return true;
}

void rfm_ook_set_freq_mhz(float f)
{
    uint32_t freqHz = f * 1000000;
    freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
    rfm69_spi_write_reg(REG_FRFMSB, freqHz >> 16);
    rfm69_spi_write_reg(REG_FRFMID, freqHz >> 8);
    rfm69_spi_write_reg(REG_FRFLSB, freqHz);
}

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
                rfm69_ook_send(bit_len, offset, ON);
            }
            else
            {
                rfm69_ook_send(bit_len, offset, OFF);
            }
            rxid_bit_mask >>= 1;
        }
        // Ensure the last bit to be 0
        rfm69_ook_send(bit_len, offset, OFF);
        rfm69_delay_us(time_interval - bit_len * RXID_BIT_SIZE);
    }
}

void rfm69_ook_send(uint16_t duration, uint16_t offset, bool val)
{
    rfm69_gpio_dio2_write(val);
    rfm69_delay_us(duration - offset);
}

/*======================================================*/
/*==================== EXAMPLES ========================*/
/*======================================================*/
void rfm_ook_send_bt_inq(uint16_t on_time, uint16_t off_time, uint32_t interval, uint32_t density)
{
    for (int i = 0; i < density; i++)
    {
        // Send the signal high for the specified on_time
        rfm69_ook_send(on_time, 10, 1);
        // Send the signal low for the specified off_time
        rfm69_ook_send(off_time, 10, 0);

        // Repeat the high and low signal
        rfm69_ook_send(on_time, 10, 1);
        rfm69_ook_send(off_time, 10, 0);

        // Wait for the remaining interval time
        rfm69_delay_us(interval - on_time * 2 - off_time * 2);
    }
}
void rfm_ook_send_ibeacon(uint16_t on_time, uint16_t off_time, uint32_t interval, uint32_t density)
{
    uint8_t offset = 10;
    for (int i = 0; i < density; i++)
    {
        rfm69_ook_send(on_time, offset, ON);
        rfm69_ook_send(off_time, offset, OFF);

        rfm69_ook_send(on_time, offset, ON);
        rfm69_ook_send(off_time, offset, OFF);

        rfm69_ook_send(on_time, offset, ON);
        rfm69_ook_send(off_time, offset, OFF);

        rfm69_delay_us(interval - on_time * 3 - off_time * 3);
    }
}
