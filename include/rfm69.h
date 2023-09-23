#ifndef STM32_RFM69_RFM69_H
#define STM32_RFM69_RFM69_H

#include "stdint.h"
#include "stdbool.h"

#include <stm32l1xx_hal_conf.h>

#define RfMaxDataLen 61

#define RfModeTx 4
#define RfModeRx 3
#define RfModeSynth 2
#define RfModeStandby 1
#define RfModeSleep 0
#define RfModeNone 10

typedef struct RFM69 {
    uint8_t mode;
    uint8_t power_level; // Default: 31
    uint16_t own_address;
    uint8_t network_address;
    uint8_t datalen;
    bool is_rfm69hw;
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* nss_port;
    uint16_t nss_pin;
    uint8_t data[61];
} RFM69;

RFM69 get_rfm(SPI_HandleTypeDef* hspi, GPIO_TypeDef* nss_port, uint16_t nss_pin, uint16_t own_address);

HAL_StatusTypeDef set_high_power_regs(RFM69* rfm, bool enable);
HAL_StatusTypeDef rfm_sleepmode(RFM69* rfm);
HAL_StatusTypeDef set_mode(RFM69* rfm, uint8_t mode);
HAL_StatusTypeDef set_encrypt(RFM69* rfm, const char* key);
HAL_StatusTypeDef set_power_level(RFM69* rfm, uint8_t power_level);
HAL_StatusTypeDef set_high_power(RFM69* rfm);
HAL_StatusTypeDef init(RFM69* rfm, uint8_t network_id);
HAL_StatusTypeDef set_reg(uint8_t address, uint8_t value);
HAL_StatusTypeDef get_reg(uint8_t address, uint8_t *out);
void interrupt_handler(RFM69* rfm);
HAL_StatusTypeDef send(RFM69* rfm, uint16_t own_address, uint16_t to_address, const void* buffer, uint8_t buffer_size);
HAL_StatusTypeDef send_frame(RFM69* rfm, uint16_t own_address, uint16_t to_address, const void* buffer, uint8_t buffer_size);
HAL_StatusTypeDef receive_begin(RFM69* rfm);
int16_t read_rssi();

#endif //STM32_RFM69_RFM69_H
