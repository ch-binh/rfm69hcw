#ifndef RFM69_FSK_H
#define RFM69_FSK_H

#include <stdint.h>
#include <stdbool.h>
/**
 * @brief Initializes the RFM module with FSK settings and configuration parameters.
 *
 * @param spispec   SPI device specification fetched from the device tree.
 * @param freqBand  Frequency band for operation (e.g., RF69_315MHZ, RF69_433MHZ).
 * @param nodeID    The unique node ID for this device.
 * @param networkID The network ID for communication grouping.
 * @return bool     True if initialization is successful, false otherwise.
 */
bool rfm69_fsk_reg_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID);

#endif // RFM69_FSK_H