/*
 * Bao cao: Hang so lenh va API cua driver barometer MS5611.
 */

#ifndef INC_MS5611_H_
#define INC_MS5611_H_

#include "main.h"
#include <stdint.h>


/*
 * MS5611 dung dia chi I2C 7-bit do datasheet quy dinh. Bit dia chi thap nhat
 * phu thuoc chan CSB: neu CSB noi GND thi dia chi la 0x77, neu CSB noi VDD thi
 * dia chi la 0x76. Trong ham HAL I2C, dia chi 7-bit nay thuong duoc dich trai
 * 1 bit khi dua len bus de them bit R/W.
 */
#define MS5611_ADDR_CSB_GND     0x77U
#define MS5611_ADDR_CSB_VDD     0x76U


/*
 * Cac so ben duoi la ma lenh cua MS5611, khong phai dia chi bo nho STM32.
 * STM32 gui cac byte lenh nay qua I2C de yeu cau cam bien reset, bat dau
 * chuyen doi ADC, doc ADC hoac doc PROM hieu chuan.
 */
#define MS5611_CMD_RESET        0x1EU  /* Lenh reset cam bien theo datasheet. */
#define MS5611_CMD_CONV_D1_4096 0x48U  /* Bat dau chuyen doi D1 pressure voi OSR 4096. */
#define MS5611_CMD_CONV_D2_4096 0x58U  /* Bat dau chuyen doi D2 temperature voi OSR 4096. */
#define MS5611_CMD_ADC_READ     0x00U  /* Lenh doc ket qua ADC 24-bit sau khi chuyen doi xong. */
#define MS5611_PROM_BASE        0xA0U  /* Dia chi/lenh doc PROM dau tien; cac he so C1..C6 nam cach nhau 2 byte. */

#define MS5611_INIT_OK          0U
#define MS5611_INIT_ERR_I2C     1U
#define MS5611_INIT_ERR_CRC     2U

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t  address;
    uint16_t C[8];
    uint8_t  raw[3];
    uint32_t D1;
    uint32_t D2;
    int32_t  TEMP_centi_C;
    int32_t  P_pa;
    float    pressure_pa;
    float    temperature_c;
} MS5611_t;


uint8_t MS5611_Init(MS5611_t *baro);


HAL_StatusTypeDef MS5611_StartConvertD1(MS5611_t *baro);
HAL_StatusTypeDef MS5611_StartConvertD2(MS5611_t *baro);


HAL_StatusTypeDef MS5611_StartReadADC_DMA(MS5611_t *baro);


void MS5611_LatchD1(MS5611_t *baro);
void MS5611_LatchD2(MS5611_t *baro);


void MS5611_ComputePressure(MS5611_t *baro);

#endif
