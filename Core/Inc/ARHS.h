/*
 * Bao cao: Kieu du lieu va ham public cua bo uoc luong tu the.
 */

#ifndef INC_ARHS_H_
#define INC_ARHS_H_

#include "main.h"
#include "arm_math.h"
#include "STATES_SENSOR.h"


#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943
#endif


typedef enum {
	Fusion_START,
	Fusion_RUN,
	Fusion_STOP,
	Fusion_RESET
} Fusion_Status_t;

typedef struct{
	float32_t alpha[3];
	float32_t Euler_Angle_Deg[3];
	float32_t Euler_Angle_Rad[3];
	float32_t q[4];
	uint32_t update_count,predict_count;
	Fusion_Status_t status;
	uint8_t Fusion_OK;
	float32_t ypr[3];
}Complimentary_Filter_t;

typedef enum {
	ARHS_START,
	ARHS_RUN,
	ARHS_RESET
}ARHS_Status_t;
typedef struct {
	float32_t q[4];
	float32_t RPY_RAD[3],RPY_DEG[3];
	float32_t B_madgwick;
	float32_t B_ARHS_Default;
	uint32_t fusion_count;
	uint8_t Fusion_OK;
	float32_t vel_fixed_frame[3];
	float32_t dv;
    float32_t acc_linear_fixed_frame[3];
}ARHS_t;
void ARHS_Predict(ARHS_t* arhs,IMU_Data_t * imu_data);
void ARHS_Update(ARHS_t* arhs,IMU_Data_t * imu_data,MAG_DATA_t * mag);

void Complimentary_Filter_Reset(Complimentary_Filter_t * imu);
void Complimentary_Filter_Predict(Complimentary_Filter_t * imu, IMU_Data_t * imu_data);
void Complimentary_Filter_Update(Complimentary_Filter_t * imu, MAG_DATA_t * mag);

void EQ_ZYX(float32_t* ypr, float32_t *q);
void Norm_Q(float32_t *q);

#endif 
