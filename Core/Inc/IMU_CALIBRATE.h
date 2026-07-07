/*
 * Bao cao: Kieu du lieu va API cho qua trinh hieu chuan IMU.
 */

#ifndef INC_IMU_CALIBRATE_H_
#define INC_IMU_CALIBRATE_H_

#include "main.h"
#include "arm_math.h"
#include "STATES_SENSOR.h"
#include <string.h>

#define CALIB_AXIS 6
#define G_Scale 1.0

#define G_SCALE  1.0  
#define NUM_POS 6         
#define NUM_EQ (NUM_POS * 3)  
#define NUM_PARAMS 12     


typedef enum {
	START_CALI,
	CALIBARTE,
	STOP_CALI,
	Caculate_Calibrate,
	CALIBRATE_OKAY,
	RESET_CALI
} IMU_CALIBRATE_STATUS_t;


typedef enum {
	Start,
	Stop,
	Running
} IMU_Sample_t;


typedef enum {
	X_P,
	X_N,
	Y_P,
	Y_N,
	Z_P,
	Z_N
} IMU_CALIBRATE_AXIS_t;


typedef struct {
	IMU_CALIBRATE_STATUS_t status;
	IMU_CALIBRATE_AXIS_t axis;
	IMU_Sample_t sample;
	
	float32_t sum_x, sum_y, sum_z;
	float32_t acc_positive[3], acc_negative[3]; 
	uint32_t num_cal;
	uint32_t count;
	
	float32_t S[3], bias[3];  
    float32_t M[3 * 3],b[3];  
	
	float32_t acc_x[6];  
	float32_t acc_y[6];
	float32_t acc_z[6];
	uint8_t axis_done[6];
	uint8_t axis_done_count;
	
	IMU_Sample_t sample_gyro;
	float32_t sum_gyro_x, sum_gyro_y, sum_gyro_z;
	uint32_t num_cal_gyro;
	uint32_t count_gyro;
	float32_t offset_gyro[3];

} IMU_CALIBRATE_t;


void IMU_ApplyCrossAxis(float32_t raw[3], float32_t M[9], float32_t b[3], float32_t out[3]);
void IMU_ApplySimpleCalibrate(float32_t raw[3], float32_t S[3], float32_t bias[3], float32_t out[3]);
void IMU_CalcCrossAxisFrom6Face(float32_t acc_x[6], float32_t acc_y[6], float32_t acc_z[6],float32_t M[9], float32_t b[3]);
void IMU_CalcSimpleCalibrate(float32_t acc_positive[3], float32_t acc_negative[3],float32_t S[3], float32_t bias[3]);

void IMU_PROCESS_CALIBRATE(IMU_CALIBRATE_t * imu, float32_t* acc,float32_t* gyro);
void IMU_Sampling(IMU_CALIBRATE_t * imu, float32_t* acc);
void RESET_IMU_CALIBRATE(IMU_CALIBRATE_t * imu);
IMU_CALIBRATE_AXIS_t IMU_DetectAxis(float32_t* acc);


typedef enum {
    MAG_CAL_IDLE = 0,       
    MAG_CAL_START,          
    MAG_CAL_COLLECTING,     
    MAG_CAL_COMPUTE,        
    MAG_CAL_DONE            
} MagCal_State_t;
typedef struct {
    MagCal_State_t state;
    float32_t S;               
    float32_t min[3];
    float32_t max[3];
    float32_t offset[3];       
    float32_t scale[3];        
    uint32_t samples;      
    uint32_t samples_target; 
} MagCal_Simple_t;

void Mag_ApplyCalibration(MagCal_Simple_t* c,MAG_DATA_t* raw,MAG_DATA_t* out);
void MagCal_Update(MagCal_Simple_t* c, MAG_DATA_t* raw,MAG_DATA_t* out);


#endif 

