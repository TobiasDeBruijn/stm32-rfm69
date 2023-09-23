#include <string.h>
#include "include/rfm69.h"
#include "include/registers.h"
#include "stm32l1xx_hal.h"

void set_nss(RFM69* rfm, GPIO_PinState state);

#define ADDR_WRITE (0b1 << 7)

RFM69 get_rfm(SPI_HandleTypeDef* hspi, GPIO_TypeDef* nss_port, uint16_t nss_pin, uint16_t own_address) {
  RFM69 rfm;
  rfm.mode = RfModeNone;
  rfm.own_address = own_address;
  rfm.power_level = 32;
  rfm.datalen = 0;
  rfm.is_rfm69hw = false;
  rfm.hspi = hspi;
  rfm.nss_port = nss_port;
  rfm.nss_pin = nss_pin;

  return rfm;
}

HAL_StatusTypeDef init(RFM69* rfm, uint8_t network_id) {
  rfm->network_address = network_id;

  HAL_GPIO_WritePin(rfm->nss_port, rfm->nss_pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(rfm->nss_port, rfm->nss_pin, GPIO_PIN_RESET);
  HAL_Delay(5);

  const uint8_t CONFIG[][2] =
          {
                  /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
                  /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
                  /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_2400 },
                  /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_2400 },
                  /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000 }, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
                  /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000 },

                  /* 0x07 */ { REG_FRFMSB, RF_FRFMSB_433 },
                  /* 0x08 */ { REG_FRFMID, RF_FRFMID_433 },
                  /* 0x09 */ { REG_FRFLSB, RF_FRFLSB_433 },

                  // looks like PA1 and PA2 are not implemented on RFM69W/CW, hence the max output power is 13dBm
                  // +17dBm and +20dBm are possible on RFM69HW
                  // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
                  // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
                  // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
                  ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
                  ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

                  // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
                  /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
                  //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
                  /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
                  /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
                  /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
                  /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
                  ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
                  /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
                  /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
                  /* 0x30 */ { REG_SYNCVALUE2, network_id }, // NETWORK ID
                  //* 0x31 */ { REG_SYNCVALUE3, 0xAA },
                  //* 0x31 */ { REG_SYNCVALUE4, 0xBB },
                  /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
                  /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
                  ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
                  /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
                  /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_OFF | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
                  //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
                  /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
                             {255, 0}
          };

  set_nss(rfm, GPIO_PIN_SET);

  uint32_t start = HAL_GetTick();
  uint32_t timeout = 10000;

  HAL_StatusTypeDef result;
  uint8_t regv;

  do {
    result = set_reg(rfm, REG_SYNCVALUE1, 0xAA);
    if(result != HAL_OK) {
      return result;
    }

    result = get_reg(rfm, REG_SYNCVALUE1, &regv);
    if(result != HAL_OK) {
      return result;
    }
  } while(regv != 0xAA && HAL_GetTick() - start < timeout);

  if(HAL_GetTick() - start >= timeout) {
    return HAL_TIMEOUT;
  }

  start = HAL_GetTick();

  do {
    result = set_reg(rfm, REG_SYNCVALUE1, 0x55);
    if(result != HAL_OK) {
      return result;
    }

    result = get_reg(rfm, REG_SYNCVALUE1, &regv);
    if(result != HAL_OK) {
      return result;
    }
  } while(regv != 0x55 && HAL_GetTick() - start < timeout);

  if(HAL_GetTick() - start >= timeout) {
    return HAL_TIMEOUT;
  }

  for(uint8_t i = 0; CONFIG[i][0] != 255; i++) {
    result = set_reg(rfm, CONFIG[i][0], CONFIG[i][1]);
    if(result != HAL_OK) {
      return result;
    }
  }

  // Encryption is persistent between resets and can trip you up during debugging.
  // Disable it during initialization so we always start from a known state.
  result = set_encrypt(rfm, 0);
  if(result != HAL_OK) {
    return result;
  }

  // set_high_power(rfm);
  result = set_mode(rfm, RfModeSleep);
  if(result != HAL_OK) {
    return result;
  }

  start = HAL_GetTick();

  // Wait for ModeReady
  do {
    result = get_reg(rfm, REG_IRQFLAGS1, &regv);
    if(result != HAL_OK) {
      return result;
    }
  } while((regv & RF_IRQFLAGS1_MODEREADY) == 0 && HAL_GetTick() - start < timeout);

  if(HAL_GetTick() - start >= timeout) {
    return HAL_TIMEOUT;
  }

  return HAL_OK;
}

HAL_StatusTypeDef send(RFM69* rfm, uint16_t own_address, uint16_t to_address, const void* buffer, uint8_t buffer_size) {
  uint8_t regv;
  HAL_StatusTypeDef result;

  result = get_reg(rfm, REG_PACKETCONFIG2, &regv);
  if(result != HAL_OK) {
    return result;
  }

  result = set_reg(rfm, REG_PACKETCONFIG2, (regv & 0xFB) | RF_PACKET2_RXRESTART);
  if(result != HAL_OK) {
    return result;
  }

  return send_frame(rfm, own_address, to_address, buffer, buffer_size);
}

HAL_StatusTypeDef send_frame(RFM69* rfm, uint16_t own_address, uint16_t to_address, const void* buffer, uint8_t buffer_size) {
  HAL_StatusTypeDef result;
  uint8_t regv;

  result = set_mode(rfm, RfModeStandby);
  if(result != HAL_OK) {
    return result;
  }

  // Wait for ModeReady
  do {
    result = get_reg(rfm, REG_IRQFLAGS1, &regv);
    if(result != HAL_OK) {
      return result;
    }
  } while((regv & RF_IRQFLAGS1_MODEREADY) == 0);

  if(buffer_size > RfMaxDataLen) {
    buffer_size = RfMaxDataLen;
  }

  uint8_t control_byte = 0x00;
  if(to_address > 0xFF) {
    control_byte |= (to_address & 0x300) >> 6; // Assign last 2 bits of address if address > 255
  }

  if(own_address > 0xFF) {
    control_byte |= (own_address & 0x300) >> 8; // Assign last 2 bits of address if address > 255
  }

  set_nss(rfm, GPIO_PIN_RESET);

  uint8_t tx_buffer[1] = { 0 };
  uint8_t rx_buffer[1] = { 0 };

  tx_buffer[0] = REG_FIFO | ADDR_WRITE;
  result = HAL_SPI_TransmitReceive(rfm->hspi, tx_buffer, rx_buffer, 1, HAL_MAX_DELAY);
  if(result != HAL_OK) {
    return result;
  }

  tx_buffer[0] = buffer_size + 3;
  result = HAL_SPI_TransmitReceive(rfm->hspi, tx_buffer, rx_buffer, 1, HAL_MAX_DELAY);
  if(result != HAL_OK) {
    return result;
  }

  tx_buffer[0] = (uint8_t) to_address;
  result = HAL_SPI_TransmitReceive(rfm->hspi, tx_buffer, rx_buffer, 1, HAL_MAX_DELAY);
  if(result != HAL_OK) {
    return result;
  }

  tx_buffer[0] = (uint8_t) own_address;
  result = HAL_SPI_TransmitReceive(rfm->hspi, tx_buffer, rx_buffer, 1, HAL_MAX_DELAY);
  if(result != HAL_OK) {
    return result;
  }

  tx_buffer[0] = control_byte;
  result = HAL_SPI_TransmitReceive(rfm->hspi, tx_buffer, rx_buffer, 1, HAL_MAX_DELAY);
  if(result != HAL_OK) {
    return result;
  }

  for(uint8_t i = 0; i < buffer_size; i++) {
    tx_buffer[0] = ((uint8_t*) buffer)[i];
    result = HAL_SPI_TransmitReceive(rfm->hspi, tx_buffer, rx_buffer, 1, HAL_MAX_DELAY);
    if(result != HAL_OK) {
      return result;
    }
  }

  set_nss(rfm, GPIO_PIN_SET);

  result = set_mode(rfm, RfModeTx);
  if(result != HAL_OK) {
    return result;
  }

  // Wait for PacketSent
  do {
    result = get_reg(rfm, REG_IRQFLAGS2, &regv);
    if(result != HAL_OK) {
      return result;
    }
  } while((regv & RF_IRQFLAGS2_PACKETSENT) == 0);

  return set_mode(rfm, RfModeSleep);
}

HAL_StatusTypeDef set_high_power(RFM69* rfm) {
  HAL_StatusTypeDef result = set_reg(rfm, REG_OCP, RF_OCP_OFF);
  if(result != HAL_OK) {
    return result;
  }

  return set_power_level(rfm, rfm->power_level);
}

HAL_StatusTypeDef set_power_level(RFM69* rfm, uint8_t power_level) {
  uint8_t pa_setting;
  if(rfm->is_rfm69hw) {
    if(power_level > 23) {
      power_level = 23;
    }

    rfm->power_level = power_level;

    if(rfm->power_level < 16) {
      power_level += 16;
      pa_setting = RF_PALEVEL_PA1_ON;
    } else {
      if(rfm->power_level < 20) {
        power_level += 10;
      } else {
        power_level += 8;
      }

      pa_setting = RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON;
    }

    HAL_StatusTypeDef result = set_high_power_regs(rfm, true); // Always call this in case we're crossing power boundaries in TX mode
    if(result != HAL_OK) {
      return result;
    }
  } else { // This is a W/CW, register value is the same as rfm->power_level
    if(power_level > 31) {
      power_level = 31;
    }

    rfm->power_level = power_level;
    pa_setting = RF_PALEVEL_PA0_ON;
  }

  return set_reg(rfm, REG_PALEVEL, pa_setting | power_level);
}

HAL_StatusTypeDef set_encrypt(RFM69* rfm, const char* key) {
  HAL_StatusTypeDef result;

  result = set_mode(rfm, RfModeStandby);
  if(result != HAL_OK) {
    return result;
  }

  uint8_t valid_key = key != 0 && strlen(key) != 0;
  if (valid_key) {
    set_nss(rfm, GPIO_PIN_RESET);

    uint8_t tx[1] = { REG_AESKEY1 | ADDR_WRITE };
    result = HAL_SPI_Transmit(rfm->hspi, tx, 1, HAL_MAX_DELAY);
    if(result != HAL_OK) {
      return result;
    }

    for(uint8_t i = 0; i < 16; i++) {
      tx[0] = key[i];
      result = HAL_SPI_Transmit(rfm->hspi, tx, 1, HAL_MAX_DELAY);
      if(result != HAL_OK) {
        return result;
      }
    }

    set_nss(rfm, GPIO_PIN_SET);
  }

  uint8_t current_packet_config2;
  result = get_reg(rfm, REG_PACKETCONFIG2, &current_packet_config2);
  if(result != HAL_OK) {
    return result;
  }

  return set_reg(rfm, REG_PACKETCONFIG2, (current_packet_config2 & 0xFE) | (valid_key ? 1 : 0));
}

HAL_StatusTypeDef rfm_sleepmode(RFM69* rfm) {
  uint8_t regv;
  HAL_StatusTypeDef result;

  do {
    result = set_mode(rfm, RfModeSleep);
    if(result != HAL_OK) {
      return result;
    }

    result = get_reg(rfm, REG_OPMODE, &regv);
    if(result != HAL_OK) {
      return result;
    }
  } while((regv & 0xE3) != RF_OPMODE_SLEEP);

  return HAL_OK;
}

HAL_StatusTypeDef set_mode(RFM69* rfm, uint8_t mode) {
  rfm->mode = mode;

  uint8_t current_opmode;
  HAL_StatusTypeDef result;

  result = get_reg(rfm, REG_OPMODE, &current_opmode);
  if(result != HAL_OK) {
    return result;
  }

  switch(mode) {
    case RfModeTx:
      result = set_reg(rfm, REG_OPMODE, (current_opmode & 0xE3) | RF_OPMODE_TRANSMITTER);
      if(result != HAL_OK) {
        return result;
      }

      if(rfm->is_rfm69hw) {
        return set_high_power_regs(rfm, true);
      }

      break;
    case RfModeRx:
      result = set_reg(rfm, REG_OPMODE, (current_opmode & 0xE3) | RF_OPMODE_RECEIVER);
      if(result != HAL_OK) {
        return result;
      }

      if(rfm->is_rfm69hw) {
        return set_high_power_regs(rfm, false);
      }

      break;
    case RfModeSynth:
      return set_reg(rfm, REG_OPMODE, (current_opmode & 0xE3) | RF_OPMODE_SYNTHESIZER);
    case RfModeStandby:
      return set_reg(rfm, REG_OPMODE, (current_opmode & 0xE3) | RF_OPMODE_STANDBY);
    case RfModeSleep:
      return set_reg(rfm, REG_OPMODE, (current_opmode & 0xE3) | RF_OPMODE_SLEEP);
    default:
      break;
  }

  return HAL_OK;
}

HAL_StatusTypeDef set_high_power_regs(RFM69* rfm, bool enable) {
  if(!rfm->is_rfm69hw || rfm->power_level < 20) {
    enable = false;
  }

  HAL_StatusTypeDef result = set_reg(rfm, REG_TESTPA1, enable ? 0x5D : 0x55);
  if(result != HAL_OK) {
    return result;
  }

  return set_reg(rfm, REG_TESTPA2, enable ? 0x7C : 0x70);
}

void set_nss(RFM69* rfm, GPIO_PinState pin_state) {
  HAL_GPIO_WritePin(rfm->nss_port, rfm->nss_pin, pin_state);
}

HAL_StatusTypeDef get_reg(RFM69* rfm, uint8_t address, uint8_t *out) {
  set_nss(rfm, GPIO_PIN_RESET);
  uint8_t tx[1] = { address };
  HAL_StatusTypeDef result = HAL_SPI_Transmit(rfm->hspi, tx, 1, HAL_MAX_DELAY);
  if(result != HAL_OK) {
    set_nss(rfm, GPIO_PIN_SET);
    return result;
  }

  uint8_t rx[1] = { 0x00 };
  result = HAL_SPI_Receive(rfm->hspi, rx, 1, HAL_MAX_DELAY);
  set_nss(rfm, GPIO_PIN_SET);
  if(result != HAL_OK) {

    return result;
  }

  *out = rx[0];
  return HAL_OK;
}

HAL_StatusTypeDef set_reg(RFM69* rfm, uint8_t address, uint8_t value) {
  set_nss(rfm, GPIO_PIN_RESET);
  uint8_t tx[2] = { address | ADDR_WRITE, value};
  HAL_StatusTypeDef result = HAL_SPI_Transmit(rfm->hspi, tx, 2, HAL_MAX_DELAY);
  set_nss(rfm, GPIO_PIN_SET);

  return result;
}

void interrupt_handler(RFM69* rfm) {
  uint8_t regv;
  HAL_StatusTypeDef result = get_reg(rfm, REG_IRQFLAGS2, &regv);
  if(result != HAL_OK) {
    return;
  }

  if(rfm->mode == RfModeRx && (regv & RF_IRQFLAGS2_PAYLOADREADY)) {
    result = set_mode(rfm, RfModeStandby);
    if(result != HAL_OK) {
      set_mode(rfm, RfModeRx);
      return;
    }

    set_nss(rfm, GPIO_PIN_RESET);

    uint8_t tx[1] = { REG_FIFO & 0x7F };
    result = HAL_SPI_Transmit(rfm->hspi, tx, 1, HAL_MAX_DELAY);
    if(result != HAL_OK) {
      set_mode(rfm, RfModeRx);
      return;
    }

    uint8_t rx[1] = { 0x00 };

    result = HAL_SPI_Receive(rfm->hspi, rx, 1, HAL_MAX_DELAY);
    if(result != HAL_OK) {
      set_mode(rfm, RfModeRx);
      return;
    }
    uint8_t payload_length = rx[0];
    payload_length = payload_length > 66 ? 66 : payload_length;

    result = HAL_SPI_Receive(rfm->hspi, rx, 1, HAL_MAX_DELAY);
    if(result != HAL_OK) {
      set_mode(rfm, RfModeRx);
      return;
    }
    uint16_t target_id = rx[0];

    result = HAL_SPI_Receive(rfm->hspi, rx, 1, HAL_MAX_DELAY);
    if(result != HAL_OK) {
      set_mode(rfm, RfModeRx);
      return;
    }
    uint16_t sender_id = rx[0];

    result = HAL_SPI_Receive(rfm->hspi, rx, 1, HAL_MAX_DELAY);
    if(result != HAL_OK) {
      set_mode(rfm, RfModeRx);
      return;
    }
    uint8_t ctl_byte = rx[0];

    target_id |= (((uint16_t) ctl_byte) & 0x0C) << 6;
    sender_id |= (((uint16_t) ctl_byte) & 0x03) << 8;

    if(target_id != rfm->own_address) {
      set_nss(rfm, GPIO_PIN_SET);
      receive_begin(rfm);
      return;
    }

    rfm->datalen = payload_length - 3;

    // Transfer the data in FIFO to memory
    for(uint8_t idx = 0; idx < rfm->datalen; idx++) {
      result = HAL_SPI_Receive(rfm->hspi, rx, 1, HAL_MAX_DELAY);
      if(result != HAL_OK) {
        set_mode(rfm, RfModeRx);
        return;
      }
      rfm->data[idx] = rx[0];
    }

    set_nss(rfm, GPIO_PIN_SET);
    set_mode(rfm, RfModeRx);
  }
}

HAL_StatusTypeDef receive_begin(RFM69* rfm) {
  uint8_t regv;
  HAL_StatusTypeDef result = get_reg(rfm, REG_IRQFLAGS2, &regv);
  if(result != HAL_OK) {
    return result;
  }

  if(regv & RF_IRQFLAGS2_PAYLOADREADY) {
    result = get_reg(rfm, REG_PACKETCONFIG2, &regv);
    if(result != HAL_OK) {
      return result;
    }

    result = set_reg(rfm, REG_PACKETCONFIG2, (regv & 0xFB) | RF_PACKET2_RXRESTART);
    if(result != HAL_OK) {
      return result;
    }
  }

  result = set_reg(rfm, REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // Set DIO0 to PayloadReady in RX mode
  if(result != HAL_OK) {
    return result;
  }

  return set_mode(rfm, RfModeRx);
  // Potential TODO optimization:
  //set_reg(rfm, REG_TESTLNA, RF_TESTLNA_HIGH_SENSITIVITY);
}