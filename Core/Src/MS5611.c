/*
 * Bao cao: Driver barometer MS5611: khoi tao, doc ADC va tinh ap suat/nhiet do tu he so PROM.
 */

#include "MS5611.h"
#include "register_comm_config.h"

static uint8_t ms5611_crc4(const uint16_t *prom_in)
{
    uint16_t prom[8];
    uint16_t rem = 0;

    for (int i = 0; i < 8; i++) {
        prom[i] = prom_in[i];
    }

    /* CRC4 nam o 4 bit thap cua PROM[7], phai xoa truoc khi tu tinh CRC. */
    prom[7] &= 0xFF00U;
    for (int cnt = 0; cnt < 16; cnt++) {
        if ((cnt & 1) != 0) {
            rem ^= (uint16_t)(prom[cnt >> 1] & 0x00FFU);
        }
        else {
            rem ^= (uint16_t)(prom[cnt >> 1] >> 8);
        }

        for (uint8_t n = 8; n > 0; n--) {
            if ((rem & 0x8000U) != 0U) {
                rem = (uint16_t)((rem << 1) ^ 0x3000U);
            }
            else {
                rem = (uint16_t)(rem << 1);
            }
        }
    }

    return (uint8_t)((rem >> 12) & 0x000FU);
}

uint8_t MS5611_Init(MS5611_t *baro)
{
    /* Ban thanh ghi: gui lenh reset MS5611 bang I2C2, khong dung HAL_I2C_Master_Transmit. */
    if (Reg_I2C2_cmd_write(baro->address, MS5611_CMD_RESET) != 0U) {
        return MS5611_INIT_ERR_I2C;
    }
    HAL_Delay(5);

    for (uint8_t i = 0; i < 8; i++) {
        uint8_t buf[2] = {0, 0};
        uint16_t prom_word = 0xFFFFU;

        /* PROM co 8 word, moi word cach nhau 2 dia chi: 0xA0, 0xA2, ... */
        for (uint8_t attempt = 0; attempt < 4; attempt++) {
            uint8_t cmd = (uint8_t)(MS5611_PROM_BASE + (i << 1));
            if (Reg_I2C2_cmd_read(baro->address, cmd, buf, 2) == 0U) {
                prom_word = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
                break;
            }
            HAL_Delay(2);
        }

        if (prom_word == 0xFFFFU) {
            return MS5611_INIT_ERR_I2C;
        }
        baro->C[i] = prom_word;
    }

    if (ms5611_crc4(baro->C) != (uint8_t)(baro->C[7] & 0x000FU)) {
        return MS5611_INIT_ERR_CRC;
    }

    return MS5611_INIT_OK;
}

HAL_StatusTypeDef MS5611_StartConvertD1(MS5611_t *baro)
{
    /* D1 la ap suat raw, OSR 4096 cho do phan giai cao nhat. */
    return (Reg_I2C2_cmd_write(baro->address, MS5611_CMD_CONV_D1_4096) == 0U)
         ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MS5611_StartConvertD2(MS5611_t *baro)
{
    /* D2 la nhiet do raw, dung de bu nhiet cho phep tinh ap suat. */
    return (Reg_I2C2_cmd_write(baro->address, MS5611_CMD_CONV_D2_4096) == 0U)
         ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MS5611_ReadADC_Register(MS5611_t *baro)
{
    /* Doc ADC 24 bit bang I2C polling muc thanh ghi, khong dung DMA. */
    return (Reg_I2C2_cmd_read(baro->address, MS5611_CMD_ADC_READ, baro->raw, 3) == 0U)
         ? HAL_OK : HAL_ERROR;
}

void MS5611_LatchD1(MS5611_t *baro)
{
    baro->D1 = ((uint32_t)baro->raw[0] << 16)
             | ((uint32_t)baro->raw[1] << 8)
             |  (uint32_t)baro->raw[2];
}

void MS5611_LatchD2(MS5611_t *baro)
{
    baro->D2 = ((uint32_t)baro->raw[0] << 16)
             | ((uint32_t)baro->raw[1] << 8)
             |  (uint32_t)baro->raw[2];
}

void MS5611_ComputePressure(MS5611_t *baro)
{
    /* Cong thuc bu nhiet va tinh ap suat lay tu datasheet MS5611. */
    int32_t dT = (int32_t)baro->D2 - ((int32_t)baro->C[5] << 8);
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * (int64_t)baro->C[6]) >> 23);
    int64_t OFF = ((int64_t)baro->C[2] << 16) + (((int64_t)baro->C[4] * (int64_t)dT) >> 7);
    int64_t SENS = ((int64_t)baro->C[1] << 15) + (((int64_t)baro->C[3] * (int64_t)dT) >> 8);

    if (TEMP < 2000) {
        int32_t T2 = (int32_t)(((int64_t)dT * (int64_t)dT) >> 31);
        int64_t d20 = (int64_t)(TEMP - 2000);
        int64_t OFF2 = (5 * d20 * d20) >> 1;
        int64_t SENS2 = OFF2 >> 1;

        if (TEMP < -1500) {
            int64_t dm15 = (int64_t)(TEMP + 1500);
            int64_t d2 = dm15 * dm15;

            OFF2 += 7 * d2;
            SENS2 += (11 * d2) >> 1;
        }

        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    int64_t P = ((((int64_t)baro->D1 * SENS) >> 21) - OFF) >> 15;

    baro->TEMP_centi_C = TEMP;
    baro->P_pa = (int32_t)P;
    baro->pressure_pa = (float)P;
    baro->temperature_c = (float)TEMP / 100.0f;
}
