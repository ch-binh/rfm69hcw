#ifndef __RFM69_OOK_H
#define __RFM69_OOK_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/**
 *  @brief Call a sequence of init
 */
void rfm69_ook_init(uint8_t freqBand, uint8_t pwr_lvl, uint8_t mode);

/**
 * @brief Deinitializes the RFM module.
 *
 * This function sets the RFM module to standby mode.
 */
void rfm69_ook_deinit(void);

/**
 * @brief Initializes the RFM module for OOK (On-Off Keying) modulation.
 *
 * This function configures the RFM module with the necessary settings to operate in
 * OOK mode
 *
 * @param freqBand The frequency band for operation, specified by one of the RF69_xxx frequency values.
 *
 * @return bool Returns `true` if the configuration is successful, otherwise returns `false`.
 */
bool rfm69_ook_reg_init(uint8_t freqBand);

/**
 * @brief Sets the frequency for OOK modulation in MHz.
 *
 * @param spispec  SPI specification for communicating with the RFM69 module.
 * @param f        Frequency in MHz to set for OOK modulation.
 *
 * @return void
 */
void rfm_ook_set_freq_mhz(float f);

/**
 * @brief Sends a custom RXID signal using OOK modulation.
 *
 * This function encodes and sends a custom 16-bit receiver ID (RXID) using OOK modulation.
 * The function sends each bit of the RXID in the form of ON/OFF pulses based on whether
 * the bit is 1 or 0. It repeats the transmission `repeat` times with a `time_interval`
 * between each repetition. The `bit_len` defines the length of each bit in the signal.
 *
 * @param rfm_int_pin  GPIO pin connected to the RFM69 interrupt line.
 * @param rxid         The 16-bit receiver ID (RXID) to be transmitted.
 * @param bit_len      The length of each bit in the signal in microseconds.
 * @param time_interval Time interval between each repetition of the transmission in microseconds.
 * @param repeat       The number of times to repeat the RXID transmission.
 */
void rfm_ook_send_custom_rxid(uint16_t rxid,
                              uint16_t bit_len,
                              uint32_t time_interval,
                              uint32_t repeat);

void rfm_ook_send_bt_inq(uint16_t on_time,
                         uint16_t off_time,
                         uint32_t interval,
                         uint32_t density);

void rfm_ook_send_ibeacon(uint16_t on_time,
                          uint16_t off_time,
                          uint32_t interval,
                          uint32_t density);

#endif //__RFM69_OOK_H