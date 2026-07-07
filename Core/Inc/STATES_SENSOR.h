/*
 * Bao cao: Cac cau truc du lieu gom trang thai cam bien va trang thai uoc luong.
 */

#ifndef INC_STATES_SENSOR_H_
#define INC_STATES_SENSOR_H_

typedef enum {
	REQUEST_DATA,
	WAIT_DATA_RDY,
	PROCESS_DATA,
	DATA_READY
}STATES_PROCESS_SENSOR_t;

typedef enum {
	GET_SAMPLE,
	START_CALIBRATE,
	STOP_CALIBRATE,
	CALIBRATE_SUCESS
}STATES_CALIBRATE_SENSOR_t;

typedef enum {
    WAIT,
	SET_REF,
	DONE
} STATE_GPS_t;

typedef struct {
	float32_t gyro[3],acc[3];
}IMU_RAW_DATA_NOT_ROT_t;
typedef struct {
	float32_t gyro[3],acc[3];
	uint32_t timestamp;
}IMU_RAW_DATA_t;
typedef struct {
	float32_t mag[3];
	uint32_t timestamp;
}MAG_RAW_DATA_t;

typedef struct {
	float32_t mag_uT[3];
	uint32_t timestamp;
}MAG_DATA_t;


typedef struct {
	float32_t alt;
	uint32_t timestamp;
}BARO_RAW_DATA_t;

typedef enum {
	START,
	RUN,
}IMU_STATES_t;


typedef enum {
	BAD,
	GOOD,
	STAND,
}HEALTH_SENSOR_t;
typedef struct {
	HEALTH_SENSOR_t health;
	uint32_t count_error;
}SENSOR_STATES_t;


typedef struct {
    int32_t lon;            
    int32_t lat;            
    int32_t height;         
    int32_t hMSL;           
    uint32_t hAcc;          
    uint32_t vAcc;          
    int32_t velN;           
    int32_t velE;           
    int32_t velD;           
    int32_t gSpeed;         
    int32_t headMot;        
    uint32_t sAcc;          
    uint32_t headAcc;       
    uint16_t pDOP;          
    int32_t headVeh;        
    uint8_t sat;
    uint8_t fixType;        
    uint8_t flags;          
    uint8_t flags2;         
    uint32_t timestamp;
}GPS_RAW_DATA_t;
typedef struct{
    float32_t   delAng[3];         
    float32_t   delVel[3];         
    float32_t   delAngDT;       
    float32_t   delVelDT;       
    uint32_t    time_ms;        
}IMU_Delta_t;
typedef struct {
	float32_t w[3];
	float32_t acc[3];
	float32_t dt;
	uint32_t timestamp;
}IMU_Data_t;


#endif 
