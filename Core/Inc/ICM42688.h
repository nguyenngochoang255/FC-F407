/*
 * Bao cao: Thanh ghi, cau hinh va API cua driver IMU ICM42688.
 */

#ifndef INC_ICM42688_H_
#define INC_ICM42688_H_
#include "stm32f4xx.h"
#include "arm_math.h"
#include <stdbool.h>

/*
 * Cac so 0x.. ben duoi la dia chi thanh ghi 8-bit ben trong ICM42688,
 * lay theo register map trong datasheet cua cam bien. Day khong phai dia chi
 * bo nho STM32. Khi giao tiep SPI, STM32 gui byte dia chi nay; neu doc thi
 * driver OR them 0x80 de set bit read, vi du doc PWR_MGMT0 se gui 0x4E | 0x80.
 */
#define ICM42688_PWR_MGMT0       0x4E  /* Bank 0, PWR_MGMT0: bat/tat gyro, accel va chon che do low-noise/standby. */
#define ICM42688_DEVICE_CONFIG   0x11  /* Bank 0, DEVICE_CONFIG: reset thiet bi va cau hinh ban dau. */
#define ICM42688_REG_BANK_SEL    0x76  /* Chon register bank; ICM42688 co nhieu bank nen phai chon bank truoc khi doc/ghi. */
#define ICM42688_INTF_CONFIG1    0x4D  /* Cau hinh giao tiep SPI/I2C va cach hoat dong cua bus. */
#define ICM42688_INT_CONFIG      0x14  /* Cau hinh kieu chan ngat INT: push-pull/open-drain, active high/low. */
#define ICM42688_INT_CONFIG0     0x63  /* Cau hinh cach xoa co ngat va che do phat ngat. */
#define ICM42688_INT_CONFIG1     0x64  /* Cau hinh bo sung cho mach ngat cua cam bien. */
#define ICM42688_INT_SOURCE0     0x65  /* Chon nguon phat ngat, vi du bat ngat data-ready cho accel/gyro. */
#define ICM42688_GYRO_CONFIG0    0x4F  /* Chon full-scale va output data rate cua gyro. */
#define ICM42688_ACCEL_CONFIG0   0x50  /* Chon full-scale va output data rate cua accelerometer. */
#define ICM42688_GYRO_CONFIG1    0x51  /* Cau hinh loc so noi bo cua gyro. */
#define ICM42688_GYRO_ACCEL_CONFIG0 0x52 /* Cau hinh bo loc dung chung cho gyro/accel. */
#define ICM42688_ACCEL_CONFIG1   0x53  /* Cau hinh loc so noi bo cua accelerometer. */
#define ICM42688_ACCEL_DATA_X1   0x1F  /* Thanh ghi dau tien cua block du lieu accel; doc lien tiep lay accel va gyro. */
#define ICM42688_WHO_AM_I        0x75  /* Thanh ghi nhan dang chip; doc ra gia tri co dinh de kiem tra ket noi SPI. */

/* He so doi raw data: accel 8G = 4096 LSB/g, gyro 500 dps = 65.5 LSB/(deg/s). */
#define ACCEL_FACTOR 1.0f / 4096.0f
#define IN_ACCEL_FACTOR 4096.0f
#define GYRO_FACTOR 1.0f / 65.5f

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 180.0f/PI
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD PI/180.0f
#endif


/*
 * Cac thanh ghi cau hinh bo loc noi bo va self-test nam o register bank phu.
 * Truoc khi truy cap nhom nay, code phai ghi ICM42688_REG_BANK_SEL de chon
 * dung bank theo datasheet; cung mot dia chi 0x03 co the co y nghia khac nhau
 * neu dang o bank khac.
 */
#define ICM42688_SENSOR_CONFIG0         0x03  /* Bank phu: cau hinh cam bien/tuy chon self-test. */
#define ICM42688_GYRO_CONFIG_STATIC2    0x0B  /* Bank phu: tham so anti-alias/notch noi bo cua gyro. */
#define ICM42688_GYRO_CONFIG_STATIC3    0x0C
#define ICM42688_GYRO_CONFIG_STATIC4    0x0D
#define ICM42688_GYRO_CONFIG_STATIC5    0x0E
#define ICM42688_GYRO_CONFIG_STATIC6    0x0F
#define ICM42688_GYRO_CONFIG_STATIC7    0x10
#define ICM42688_GYRO_CONFIG_STATIC8    0x11
#define ICM42688_GYRO_CONFIG_STATIC9    0x12
#define ICM42688_GYRO_CONFIG_STATIC10   0x13

#define ICM42688_ACCEL_CONFIG_STATIC2   0x03  /* Bank phu: tham so anti-alias/notch noi bo cua accel. */
#define ICM42688_ACCEL_CONFIG_STATIC3   0x04
#define ICM42688_ACCEL_CONFIG_STATIC4   0x05
#define ICM42688_XA_ST_DATA             0x3B  /* Gia tri self-test truc X accel theo datasheet. */
#define ICM42688_YA_ST_DATA             0x3C
#define ICM42688_ZA_ST_DATA             0x3D


/* OFFSET_USER0..8 la cac thanh ghi lien tiep 0x77..0x7F de luu offset nguoi dung cua cam bien. */
#define OFFSET_USER0 0x77
#define OFFSET_USER1 0x78
#define OFFSET_USER2 0x79
#define OFFSET_USER3 0x7A
#define OFFSET_USER4 0x7B
#define OFFSET_USER5 0x7C
#define OFFSET_USER6 0x7D
#define OFFSET_USER7 0x7E
#define OFFSET_USER8 0x7F

/* Day la ma bit-field ghi vao ACCEL_CONFIG0, khong phai dia chi thanh ghi. */
#define ACCEL_16G 0x00
#define ACCEL_8G 0x01
#define ACCEL_4G 0x02
#define ACCEL_2G 0x03

/* Day la ma bit-field output data rate ghi vao ACCEL_CONFIG0. */
#define ACCEL_ODR_32 0x01
#define ACCEL_ODR_16 0x02
#define ACCEL_ODR_8 0x03
#define ACCEL_ODR_4 0x04
#define ACCEL_ODR_2 0x05
#define ACCEL_ODR_1 0x06
#define ACCEL_ODR_500HZ 0x0F

/* Day la ma bit-field full-scale ghi vao GYRO_CONFIG0. */
#define GYRO_2000 0x00
#define GYRO_1000 0x01
#define GYRO_500 0x02
#define GYRO_250 0x03
#define GYRO_125 0x04

/* Day la ma bit-field output data rate ghi vao GYRO_CONFIG0. */
#define GYRO_ODR_32 0x01
#define GYRO_ODR_16 0x02
#define GYRO_ODR_8 0x03
#define GYRO_ODR_4 0x04
#define GYRO_ODR_2 0x05
#define GYRO_ODR_1 0x06
#define GYRO_ODR_500HZ 0x0F

typedef struct {
	uint8_t TEMP_FILT_BW;

	uint8_t GYRO_DEC2_M2_ORD;
    uint8_t GYRO_ODR,GYRO_FS_SEL;
    uint8_t GYRO_UI_FILT_BW,GYRO_UI_FILT_ORD;
    uint8_t GYRO_NF_DIS;
	uint8_t GYRO_AAF_DELT,GYRO_AAF_DIS;
    uint16_t GYRO_AAF_DELTSQR;
    uint8_t GYRO_AAF_BITSHIFT;

    uint8_t ACCEL_DEC2_M2_ORD;
    uint8_t ACCEL_ODR,ACCEL_FS_SEL;
    uint8_t ACCEL_UI_FILT_BW,ACCEL_UI_FILT_ORD;
	uint8_t ACCEL_AAF_DELT,ACCEL_AAF_DIS;
	uint16_t ACCEL_AAF_DELTSQR;
	uint8_t ACCEL_AAF_BITSHIFT;
	uint8_t ready;

} ICM42688_Config_t;

extern const ICM42688_Config_t CFG_ICM42688P_Default;

typedef struct {
	SPI_HandleTypeDef * spi_imu;
	GPIO_TypeDef * cs_bank_imu;
	uint16_t cs_pin;
	ICM42688_Config_t cfg;
	float32_t gyro_bias[3];
	float32_t accel_bias[3];
	float32_t data[6];
	float32_t bias_data[6];
	float32_t convert_factor[6];
	float32_t Accel[3],Gyroscope[3];
	int16_t raw_data[6];
    uint8_t rx_data[12];
    int16_t accel[3];
    int16_t gyro[3];
    uint8_t axis;
    int number_cal;
    float32_t max[3],min[3];
    float32_t scale[3];
    float32_t off_set[3];

} ICM42688;

typedef struct {


}ICM42688_CALIBRATE_t;


void ICM_42688_Init(ICM42688 *imu ,SPI_HandleTypeDef *hspi , GPIO_TypeDef * _cs_bank_imu , uint16_t _cs_pin);
void ICM_42688_Config(ICM42688 *imu);
void ICM_Write_Register(ICM42688 *imu, uint8_t reg, uint8_t data);
uint8_t ICM_Read_Register(ICM42688 *imu, uint8_t reg);
void ICM_RAW_DATA(ICM42688 *imu);
void ICM_Calib_Gyro(ICM42688 * imu,TIM_HandleTypeDef * htim , int32_t dt_fred_main,int n);
void ICM_Calib(ICM42688 * imu,TIM_HandleTypeDef * htim , int32_t dt_fred_main,bool calib_accel,int n);
void ICM_RAW_DATA_Ver2(ICM42688 *imu);
void ICM_NORM(ICM42688 * imu,TIM_HandleTypeDef * htim , int32_t dt_fred_main);
void ICM_Process_Data(ICM42688 *imu);
uint8_t ICM_RAW_DATA_DMA(ICM42688 *imu);
#endif 
