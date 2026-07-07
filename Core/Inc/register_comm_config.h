/*
 * Bao cao: Khai bao ham cau hinh USART, SPI va I2C bang thanh ghi STM32.
 */

#ifndef __REGISTER_COMM_CONFIG_H
#define __REGISTER_COMM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"


void    Reg_USART1_Init(void);


void    Reg_SPI1_Init(void);
uint8_t Reg_SPI1_transfer(uint8_t tx);


void    Reg_I2C2_Init(void);
uint8_t Reg_I2C2_cmd_write(uint8_t addr7, uint8_t cmd);
uint8_t Reg_I2C2_cmd_read(uint8_t addr7, uint8_t cmd, uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
