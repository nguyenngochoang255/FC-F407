/*
 * Bao cao: Driver IMU ICM42688: cau hinh cam bien va doi du lieu raw sang don vi vat ly.
 */

#include "ICM42688.h"
#include "register_comm_config.h"

static inline void ICM_CS_Low(ICM42688 *imu)
{
    imu->cs_bank_imu->BSRR = ((uint32_t)imu->cs_pin << 16);
}

static inline void ICM_CS_High(ICM42688 *imu)
{
    imu->cs_bank_imu->BSRR = imu->cs_pin;
}

const ICM42688_Config_t CFG_ICM42688P_Default ={
		
			/* ACCEL_ODR/GYRO_ODR chon output data rate; FS_SEL chon thang do cua cam bien. */
			.ACCEL_ODR = ACCEL_ODR_1,
			.ACCEL_FS_SEL = ACCEL_16G,
			.ACCEL_DEC2_M2_ORD = 0x02,
			.ACCEL_AAF_DIS = 0x00,
			.ACCEL_AAF_DELT = 1,
			.ACCEL_AAF_DELTSQR = 1,
			.ACCEL_AAF_BITSHIFT = 15,
			.ACCEL_UI_FILT_ORD = 0x00,
		    .ACCEL_UI_FILT_BW = 6,
		
			.GYRO_ODR = GYRO_ODR_1,
			.GYRO_FS_SEL = GYRO_2000,
			.GYRO_DEC2_M2_ORD = 0x02,
			.GYRO_AAF_DIS = 0x00,
			.GYRO_AAF_DELT = 2,
			.GYRO_AAF_DELTSQR = 4,
			.GYRO_AAF_BITSHIFT = 13,
			.GYRO_UI_FILT_ORD = 0x00,
			.GYRO_UI_FILT_BW = 6,
			.GYRO_NF_DIS = 0x00,
			.TEMP_FILT_BW = 0x00
};


void ICM_42688_Init(ICM42688 *imu, GPIO_TypeDef *_cs_bank_imu, uint16_t _cs_pin){
     imu ->cs_bank_imu = _cs_bank_imu;
     imu ->cs_pin = _cs_pin;
}


void ICM_Write_Register(ICM42688 *imu, uint8_t reg, uint8_t data){
    /* SPI write cua ICM42688 yeu cau bit 7 cua dia chi = 0. */
    ICM_CS_Low(imu);
	(void)Reg_SPI1_transfer((uint8_t)(reg & 0x7F));
	(void)Reg_SPI1_transfer(data);
	ICM_CS_High(imu);
	HAL_Delay(2);
}
uint8_t ICM_Read_Register(ICM42688 *imu, uint8_t reg){
        /* SPI read cua ICM42688 yeu cau bit 7 cua dia chi = 1 nen OR 0x80. */
	    uint8_t tx_data = reg | 0x80;
        uint8_t rx_data;
        ICM_CS_Low(imu);
		(void)Reg_SPI1_transfer(tx_data);
		rx_data = Reg_SPI1_transfer(0x00U);
		ICM_CS_High(imu);
		return rx_data;
}
void ICM_42688_Config(ICM42688 *imu){
	ICM42688_Config_t config = imu->cfg;
	float32_t acc_fac;
	float32_t gyro_fac;
	uint8_t data;
	switch(config.GYRO_FS_SEL){
	case 0x00 :
		gyro_fac = 16.38;
		break;
	case 0x01 :
		gyro_fac = 32.8;
		break;
	case 0x02 :
		gyro_fac = 65.5 ;
		break;
	case 0x03 :
		gyro_fac = 131;
		break;
	case 0x04 :
	    gyro_fac = 262;
	    break;
	default :
		gyro_fac = 0.0;
		break;
	}
	/* Gia tri gyro_fac la so LSB tren 1 deg/s ung voi tung thang do gyro. */
	switch(config.ACCEL_FS_SEL){
		case 0x00 :
			acc_fac = 2048.0;
			break;
		case 0x01 :
			acc_fac = 4096.0;
			break;
		case 0x02 :
			acc_fac = 8192.0 ;
			break;
		case 0x03 :
			acc_fac = 16384.0;
			break;
		default :
			acc_fac = 0.0;
			break;
		}
	/* Gia tri acc_fac la so LSB tren 1 g ung voi tung thang do accelerometer. */
	for(int i = 0 ; i < 3 ; i ++){
		imu ->convert_factor[i] = 1.0/acc_fac;
		imu ->convert_factor[i + 3] = 1.0/gyro_fac;
	}

	/* DEVICE_CONFIG=0x01 reset thiet bi theo datasheet. */
	ICM_Write_Register(imu, ICM42688_DEVICE_CONFIG, 0x01);
	HAL_Delay(100);
	/* PWR_MGMT0: 0x0C bat gyro+accel low-noise, 0x0F dua ca hai vao che do hoat dong. */
	ICM_Write_Register(imu, ICM42688_PWR_MGMT0, 0x0C);
	HAL_Delay(100);
	ICM_Write_Register(imu, ICM42688_PWR_MGMT0, 0x0F);
	HAL_Delay(100);
    /* GYRO_CONFIG0/ACCEL_CONFIG0: bit 7..5 la full-scale, bit 3..0 la ODR. */
    data = (config.GYRO_FS_SEL << 5)|(config.GYRO_ODR);
	ICM_Write_Register(imu, ICM42688_GYRO_CONFIG0, data);
	data = (config.ACCEL_FS_SEL << 5)|(config.ACCEL_ODR);
	ICM_Write_Register(imu, ICM42688_ACCEL_CONFIG0, data);
    data =((config.TEMP_FILT_BW) << 5)|(config.GYRO_UI_FILT_ORD << 2)|(config.GYRO_DEC2_M2_ORD);
	ICM_Write_Register(imu, ICM42688_GYRO_CONFIG1, data);
	data = (config.ACCEL_UI_FILT_ORD << 3)|((config.ACCEL_DEC2_M2_ORD) << 1);
	ICM_Write_Register(imu, ICM42688_ACCEL_CONFIG1, data);
	HAL_Delay(10);
	
	/* REG_BANK_SEL doi sang bank 1 de cau hinh bo loc/noise filter cua gyro. */
	ICM_Write_Register(imu, ICM42688_REG_BANK_SEL, 0x01);
    data = (config.GYRO_AAF_DIS << 1)|(config.GYRO_NF_DIS);
	ICM_Write_Register(imu, ICM42688_GYRO_CONFIG_STATIC2, data);
	data = (config.GYRO_AAF_DELT);
	ICM_Write_Register(imu, ICM42688_GYRO_CONFIG_STATIC3, data);
	data = config.GYRO_AAF_DELTSQR & 0xFF;
	ICM_Write_Register(imu, ICM42688_GYRO_CONFIG_STATIC4, data);
	data = (config.GYRO_AAF_BITSHIFT << 4) |((config.GYRO_AAF_DELTSQR >> 8) & 0x0F);
	ICM_Write_Register(imu, ICM42688_GYRO_CONFIG_STATIC5, data);
	HAL_Delay(10);
	/* Bank 2 chua cac thanh ghi bo loc AAF cua accelerometer. */
	ICM_Write_Register(imu, ICM42688_REG_BANK_SEL, 0x02);
	data = ((config.ACCEL_AAF_DELT) << 1) | config.ACCEL_AAF_DIS;
	ICM_Write_Register(imu, ICM42688_ACCEL_CONFIG_STATIC2, data);
    data = config.ACCEL_AAF_DELTSQR & 0xFF;
	ICM_Write_Register(imu, ICM42688_ACCEL_CONFIG_STATIC3, data);
	data = (config.ACCEL_AAF_BITSHIFT << 4) | ((config.ACCEL_AAF_DELTSQR >> 8) & 0x0F);
	ICM_Write_Register(imu, ICM42688_ACCEL_CONFIG_STATIC4, data);
	HAL_Delay(10);
	
	/* Quay ve bank 0 de cau hinh data-ready interrupt va doc data. */
	ICM_Write_Register(imu, ICM42688_REG_BANK_SEL, 0x00);
	data = (config.ACCEL_UI_FILT_BW << 4)|(config.GYRO_UI_FILT_BW);
	ICM_Write_Register(imu, ICM42688_GYRO_ACCEL_CONFIG0, data);
	HAL_Delay(10);
	
	uint8_t INT1_MODE = 0;  
	uint8_t INT1_DRIVE_CIRCUIT = 1; 
	uint8_t INT1_POLARITY = 1; 
	/* INT_CONFIG: push-pull, active-high de PE0 nhan rising edge data-ready. */
	data = (INT1_MODE << 2)|(INT1_DRIVE_CIRCUIT << 1)|INT1_POLARITY;
	ICM_Write_Register(imu, ICM42688_INT_CONFIG,data);
	uint8_t UI_DRDY_INT_CLEAR = 2; 
	data = UI_DRDY_INT_CLEAR << 4;
	ICM_Write_Register(imu, ICM42688_INT_CONFIG0, data); 
	


	ICM_Write_Register(imu, ICM42688_INT_CONFIG1, 0x00);
	/* INT_SOURCE0 bit 3 = UI data-ready interrupt. */
	ICM_Write_Register(imu, ICM42688_INT_SOURCE0, 0x08);
	HAL_Delay(10);
	ICM_Write_Register(imu, ICM42688_REG_BANK_SEL, 0x00);

}

