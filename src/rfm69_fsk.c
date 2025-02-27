#include "../inc/rfm69_fsk.h"
#include "../inc/rfm69.h"
#include "../inc/RFM69registers.h"

bool rfm69_fsk_reg_init(uint8_t freqBand, uint8_t nodeID, uint8_t networkID)
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
    rfm69_set_mode(RF69_MODE_STANDBY);

    do
    {
        rfm_read_reg(REG_IRQFLAGS1, _values, _size);
    } while ((_values[1] & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

    //_address = nodeID;
    return true;
}