#ifndef RFM69_H
#define RFM69_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*======================= CONFIGURATION PARAMETERS ===========================*/
#define OFFSET_VAL 15 // delay us before sending the signals
/* Bitrate settings */
#define RF69_BITRATE_MSB_VAL 0x03 // max value
#define RF69_BITRATE_LSB_VAL 0xD1 // max value

/* Default parameters */
#define IS_RFM69HW true
#define RXID_BIT_SIZE 16

/* Data length and SPI settings */
#define RF69_MAX_DATA_LEN 61 // to take advantage of the built-in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
#define RF69_SPI_CS SS       // SS is the SPI slave select pin, for instance D10 on ATmega328

/* IRQ settings */
#define RF69_IRQ_PIN 10
#define RF69_IRQ_NUM 10

/* Power settings */
#define POWER_LEVEL_MIN 0
#define POWER_LEVEL_MAX 31

/* CSMA settings */
#define CSMA_LIMIT -90 // upper RX signal sensitivity threshold in dBm for carrier sense access

/* Operational modes */
#define RF69_MODE_SLEEP 0   // XTAL OFF
#define RF69_MODE_STANDBY 1 // XTAL ON
#define RF69_MODE_SYNTH 2   // PLL ON
#define RF69_MODE_RX 3      // RX MODE
#define RF69_MODE_TX 4      // TX MODE

/* Frequency bands */
#define RF69_315MHZ 31 // non-trivial values to avoid misconfiguration
#define RF69_433MHZ 43
#define RF69_868MHZ 86
#define RF69_915MHZ 91

/* Miscellaneous */
#define null 0
#define ON 1
#define OFF 0
#define COURSE_TEMP_COEF -90 // puts the temperature reading in the ballpark, user can fine-tune the returned value
#define RF69_BROADCAST_ADDR 255
#define RF69_CSMA_LIMIT_MS 1000
#define RF69_TX_LIMIT_MS 1000
#define RF69_FSTEP 61.03515625 // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

/* Control byte bits */
#define RFM69_CTL_SENDACK 0x80
#define RFM69_CTL_REQACK 0x40
/*======================= COMMON VALUES =====================*/

/*======================= FOR FSK =========================== */

/**
 * @brief Reads data from a register of an RFM module using SPI communication.
 *
 * @param spispec   SPI device specification fetched from device tree.
 * @param reg       SPI Register address to read from.
 * @param data      Pointer to the buffer where the read data will be stored.
 * @param size      Number of bytes to read.
 * @return int      0 on success, error code on failure.
 */
int rfm_read_reg(uint8_t reg, uint8_t *data, uint8_t size);

/**
 * @brief Writes data to a register of an RFM module using SPI communication.
 *
 * @param spispec   SPI device specification fetched from the device tree.
 * @param reg       SPI register address to write to.
 * @param value     Value to be written to the specified register.
 * @return int      0 on success, error code on failure.
 */
int rfm_write_reg(uint8_t reg, uint8_t value);

/**
 * @brief Writes a frame of data to an RFM module using SPI communication.
 *
 * @param spispec   SPI device specification fetched from the device tree.
 * @param buffer    Pointer to the buffer containing the frame to be written.
 * @param len       Length of the frame in bytes.
 * @return int      0 on success, error code on failure.
 */
int rfm_write_frame(uint8_t *buffer, uint8_t len);

/**
 * @brief Sets the operation mode of the RFM module.
 *
 * @param spispec   SPI device specification fetched from the device tree.
 * @param newMode   The new mode to set. Valid modes include TX, RX, SYNTH, STANDBY, and SLEEP.
 */

/**
 * @brief Sends a data frame to a specified address using the RFM module.
 *
 * @param toAddress     Address of the recipient device.
 * @param buffer        Pointer to the data to be sent.
 * @param bufferSize    Size of the data buffer in bytes.
 * @param requestACK    If true, requests an acknowledgment from the recipient.
 */
void rfm69_send_msg(uint8_t toAddress, void *buffer, uint8_t bufferSize, bool requestACK);

/**
 * @brief Sends a frame of data using the RFM module.
 *
 * @param toAddress     Address of the recipient device.
 * @param buffer        Pointer to the data to be sent.
 * @param bufferSize    Size of the data buffer in bytes.
 * @param requestACK    If true, requests an acknowledgment from the recipient.
 * @param sendACK       If true, the frame being sent is an acknowledgment.
 */
void rfm69_send_msg_frame(uint8_t toAddress, void *buffer, uint8_t bufferSize, bool requestACK, bool sendACK);

/**
 * @brief Sets the operational mode of the RFM module.
 *
 * @param spispec   SPI device specification fetched from the device tree.
 * @param mode      Desired operational mode (e.g., RF69_MODE_TX, RF69_MODE_RX).
 */
void rfm69_set_mode(uint8_t mode);

/**
 * @brief Configures the RFM module for high power transmission.
 *
 * This function adjusts the power amplifier settings based on the RFM module type.
 * For the RFM69HW variant, additional amplifier stages are enabled.
 *
 * @param spispec   SPI device specification fetched from the device tree.
 */
void rfm69_set_high_power(void);

/**
 * @brief Sets the output power level of the RFM module.
 *
 * This function adjusts the power level for the RFM module's power amplifier.
 * The power level value is clamped to a maximum of 31 to prevent invalid settings.
 *
 * @param spispec      SPI device specification fetched from the device tree.
 * @param powerLevel   Desired power level (0 to 31). Values greater than 31 are clamped to 31.
 */
void rfm69_set_power_Level(uint8_t powerLevel);

/**
 * @brief Enables or disables the high power mode for the RFM module.
 *
 * This function controls the power amplifier stages (PA1 and PA2) of the RFM module.
 * When enabled, it sets bit high for the PA1 and PA2 registers to
 * increase the output power of the module. When disabled, it restores the default
 * lower power settings.
 *
 * @param spispec  SPI device specification fetched from the device tree.
 * @param onOff    Boolean flag to enable (true) or disable (false) high power mode.
 */
void rfm_set_high_power_reg(bool onOff);

/**
 * @brief Reads the Received Signal Strength Indicator (RSSI) value from the RFM module.
 *
 * This function reads the RSSI value from the RFM module, which indicates the strength
 * of the received signal.
 *
 * @param spispec        SPI device specification fetched from the device tree.
 * @param force_trigger  Boolean flag to trigger a new RSSI measurement (true) or use the
 *                       previously measured value (false).
 *
 * @return int16_t       The RSSI value in dBm, negative value indicates signal strength.
 *                       The value is right-shifted by 1 for scaling.
 */
int16_t rfm_read_RSSI(bool force_trigger);

/**
 * @brief Retrieves the current frequency from the RFM module.
 *
 * This function reads the frequency registers (FRFMSB, FRFMID, FRFLSB) of the RFM module
 *
 * @param spispec   SPI device specification fetched from the device tree.
 *
 * @return uint32_t The current frequency in Hz.
 */
uint32_t rfm_get_freq(void);

#endif //__RFM_69_H