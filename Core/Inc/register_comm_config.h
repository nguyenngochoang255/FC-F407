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
void    Reg_USART1_write(uint8_t byte);
void    Reg_USART1_print(const char *s);
uint8_t Reg_USART1_read(void);


void    Reg_SPI1_Init(void);
uint8_t Reg_SPI1_transfer(uint8_t tx);
uint8_t Reg_ICM42688_read_reg(uint8_t reg);
void    Reg_ICM42688_write_reg(uint8_t reg, uint8_t val);
uint8_t Reg_ICM42688_who_am_i(void);


void    Reg_I2C1_Init(void);


uint8_t Reg_I2C1_mem_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t Reg_I2C1_mem_write(uint8_t addr7, uint8_t reg, uint8_t val);


void    Reg_I2C2_Init(void);
uint8_t Reg_I2C2_cmd_read(uint8_t addr7, uint8_t cmd, uint8_t *buf, uint16_t len);
uint8_t Reg_MS5611_reset(void);
uint16_t Reg_MS5611_read_prom(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif
