/*
 * Bao cao: Chuong trinh chinh cua flight controller: doc RC, doc cam bien, uoc luong tu the, quan ly trang thai bay va xuat PWM cho 4 ESC.
 */

#include "main.h"

#include "ICM42688.h"
#include "MS5611.h"
#include "arm_math.h"
#include "STATES_SENSOR.h"
#include "Filter.h"
#include "IMU_CALIBRATE.h"
#include "ARHS.h"
#include "PID_Controller.h"
#include "register_board_config.h"
#include "register_comm_config.h"
#include <math.h>


ICM42688 ICM42688P;

IMU_Data_t ICM42688P_DATA;

/* Khoi tao cac doi tuong cam bien va bo loc dung trong vong lap bay. */
/* IMU du kien cap nhat moi 1000 us, tuong duong 1 kHz. */
#define GYRO_RATE_US 1000
biquadFilter_t gyro_lpf[3];
biquadFilter_t gyro_notch[3];
biquadFilter_t acc_lpf[3];
biquadFilter_t acc_notch[3];
IMU_CALIBRATE_t IMU_CALIBRATE ={
		.status = CALIBRATE_OKAY,
		.sample_gyro = Stop,
		.num_cal_gyro = 5000,
		.num_cal = 4000,

		.S[0] = 1.00229514f,
		.S[1] = 1.00136149f,
		.S[2] = 1.00597465f,
		.bias[0] = -0.00122478604f,
		.bias[1] = -0.00463348627f,
		.bias[2] = 0.00582873821f,
		.M[0] = 1.00200748f,
		.M[1] = 0.0214256998f,
		.M[2] = -0.00227226061f,
		.M[3] = -0.0153374802f,
		.M[4] = 1.00100935f,
		.M[5] = -0.00400897907f,
		.M[6] = -0.0187641308f,
		.M[7] = 0.0056234519f,
		.M[8] = 1.00599289f,
		.b[0] = 0.00102425925f,
		.b[1] = 0.000808518904f,
		.b[2] = 0.0068582478f,
};


MS5611_t MS5611_BARO = {
    /* Dia chi 7-bit MS5611 khi chan CSB noi GND la 0x77. */
    .address = MS5611_ADDR_CSB_GND,   
};


ARHS_t ARHS ={
		.B_ARHS_Default = 0.012,
		.B_madgwick     = 0.012f * 100.0f,  
		.q[0] = 1.0,
		.q[1] = 0,
		.q[2] = 0,
		.q[3] = 0,
};

const pidControllerParam_t pidRateRollParam = {
    .kP  = 1.45f,
    .kI  = 1.1f,
    .kD  = 0.0105f,
    .kFF = 0.0f
};

const pidControllerParam_t pidRatePitchParam = {
	.kP  = 1.45f,
	.kI  = 1.1f,
	.kD  = 0.0105f,
	.kFF = 0.0f
};

const pidControllerParam_t pidRateYawParam = {
    .kP  = 1.55f,
    .kI  = 0.7f,
    .kD  = 0.010f,
    .kFF = 0.0f
};
const pidControllerParam_t pidRollParam = {
    .kP  = 1.8f,
    .kI  = 0.4f,
    .kD  = 0.0f,
    .kFF = 0.0f
};

const pidControllerParam_t pidPitchParam = {
    .kP  = 1.8f,
    .kI  = 0.4f,
    .kD  = 0.0f,
    .kFF = 0.0f
};

const pidControllerParam_t pidYawParam = {
    .kP  = 2.5f,
    .kI  = 0.0f,
    .kD  = 0.0f,
    .kFF = 0.0f
};


pidController_t PID_RATE_ROLL = {
    .param       = pidRateRollParam,
    .dTermLpfHz  = 80.0f,
    .errorLpfHz  = 0.0f,
};

pidController_t PID_RATE_PITCH = {
    .param       = pidRatePitchParam,
    .dTermLpfHz  = 80.0f,
    .errorLpfHz  = 0.0f,
};

pidController_t PID_RATE_YAW = {
    .param       = pidRateYawParam,
    .dTermLpfHz  = 50.0f,
    .errorLpfHz  = 0.0f,
};

pidController_t PID_ROLL = {
    .param       = pidRollParam,
    .dTermLpfHz  = 80.0f,
    .errorLpfHz  = 0.0f,
};

pidController_t PID_PITCH = {
    .param       = pidPitchParam,
    .dTermLpfHz  = 80.0f,
    .errorLpfHz  = 0.0f,
};

pidController_t PID_YAW = {
    .param       = pidYawParam,
    .dTermLpfHz  = 50.0f,
    .errorLpfHz  = 0.0f,
};

ARM_Status_t ARM_Status = NOT_ARM;

/* Nguong an toan cho arm/disarm, tinh tren gia tri kenh CRSF da doi ve khoang dieu khien noi bo. */
#define MIN_ARM 250                  /* Nguong nho de coi nhu stick/switch dang o muc thap, tranh arm nham. */

/* Ly do chua cho ARM; dung de debug LED/trang thai va giai thich vi sao motor chua duoc phep quay. */
#define ARM_BLOCK_NONE       0U      /* Khong bi chan arm. */
#define ARM_BLOCK_SWITCH_OFF 1U      /* Cong tac arm tren remote dang tat. */
#define ARM_BLOCK_WAIT_EDGE  2U      /* Dang doi nguoi dung gat switch lai de tranh auto-arm sau khi boot/loi. */
#define ARM_BLOCK_THROTTLE   3U      /* Throttle chua ve thap nen khong an toan de arm. */
#define ARM_BLOCK_GYRO       4U      /* Gyro/IMU chua hieu chuan xong hoac chua san sang. */
#define ARM_BLOCK_RC_LOST    5U      /* Mat tin hieu RC/ELRS nen khong cho arm. */
#define ARM_BLOCK_BAD_RX     6U      /* Frame RC khong hop le hoac kenh nhan bi loi. */
#define ARM_REQUIRE_SWITCH_CYCLE 0U  /* =1 thi bat buoc gat switch arm off-on sau loi; =0 thi khong bat buoc. */

/* Ly do bi disarm; khac voi ARM_BLOCK vi day la nguyen nhan da tat motor/roi khoi trang thai bay. */
#define DISARM_REASON_NONE        0U /* Chua co ly do disarm. */
#define DISARM_REASON_SWITCH_OFF  1U /* Nguoi dung tat switch arm. */
#define DISARM_REASON_BAD_RX      2U /* Du lieu receiver khong hop le. */
#define DISARM_REASON_RC_LOST     3U /* Qua thoi gian failsafe khong co frame CRSF hop le. */
#define DISARM_REASON_WAIT_ARM    4U /* Dang cho dieu kien arm hop le. */
#define DISARM_REASON_THROTTLE    5U /* Throttle cao, disarm/khong arm de bao ve. */
#define DISARM_REASON_GYRO        6U /* Gyro chua san sang hoac hieu chuan chua dat. */
#define DISARM_REASON_BAD_STATE   7U /* State machine bay roi vao trang thai khong hop le. */

#define ARM_THROTTLE_MAX     350     /* Throttle phai nho hon nguong nay moi duoc arm/takeoff. */
/* Qua 1.5 s khong co frame CRSF hop le thi coi la mat RC. */
#define RC_FAILSAFE_TIMEOUT  1500000U 


/* Gesture tren stick de yeu cau hieu chuan gyro lai khi dang an toan. */
#define GESTURE_THROTTLE_MAX  300    /* Throttle phai thap de chac chan motor khong dang bay. */
#define GESTURE_YAW_MAX       300    /* Yaw keo ve mot dau theo mau gesture. */
#define GESTURE_PITCH_MAX     300    /* Pitch keo ve mot dau theo mau gesture. */
#define GESTURE_ROLL_MIN      1700   /* Roll keo ve dau con lai de phan biet voi thao tac bay binh thuong. */
#define GESTURE_DEBOUNCE_US   200000U /* Gesture phai on dinh 200 ms moi chap nhan, tranh nhieu/nham stick. */


/* Doi 3 s sau khi boot de cam bien on dinh roi moi tu dong hieu chuan gyro. */
#define BOOT_SETTLE_US        3000000U


/* MOTOR_IDLE = 0.15 tuong duong xung 1150 us trong cong thuc 1000 + cmd*1000. */
#define MOTOR_IDLE            0.15f
#define AIRMODE_FULL_THROTTLE 0.35f


/* Mapping kenh CRSF trong mang CH[]/Desired_Value[]; so nay la index kenh, khong phai chan GPIO. */
#define RX_ROLL_CH            0U    /* CH0: roll, lenh nghieng trai/phai. */
#define RX_PITCH_CH           1U    /* CH1: pitch, lenh chui len/chui xuong. */
#define RX_THROTTLE_CH        2U    /* CH2: throttle, lenh ga/nang tong luc motor. */
#define RX_YAW_CH             3U    /* CH3: yaw, lenh xoay trai/phai quanh truc dung. */
#define RX_ARM_CH             4U    /* CH4: cong tac ARM, cho phep/khong cho phep motor quay. */
#define RX_FLIGHT_SW_CH       5U    /* CH5: cong tac che do bay, dung cho auto takeoff/landing trong code. */
#define RX_ALT_HOLD_CH        8U    /* CH8: bat/tat giu do cao bang barometer. */
#define RX_ALT_HOLD_BLOCK_CH  10U   /* CH10: khoa alt-hold, dung de chan che do giu do cao khi can bay tay. */


/* SPI doc IMU 13 byte = 1 byte dia chi bat dau + 12 byte du lieu accel/gyro. */
volatile uint8_t imu_spi_rx[13];
volatile uint8_t imu_spi_tx[13];
volatile uint8_t ICM42688P_DATA_RDY = 0;
float32_t raw_adc_acc_imu[3],raw_adc_gyro_imu[3];
float32_t acc_adc_imu_filtered[3],gyro_adc_imu_filtered[3];

/* FSM doc MS5611: moi chu ky doc ap suat D1 va nhiet do D2 bang I2C2 thanh ghi. */
typedef enum {
    MS5611_S_START_D1,
    MS5611_S_WAIT_D1_CONVERT,
    MS5611_S_WAIT_D1_READ,
    MS5611_S_START_D2,
    MS5611_S_WAIT_D2_CONVERT,
    MS5611_S_WAIT_D2_READ,
} MS5611_FSM_t;

MS5611_FSM_t baro_state = MS5611_S_START_D1;
volatile uint8_t baro_read_done = 0;        /* Co bao da doc xong ADC cua MS5611 trong state hien tai. */
uint32_t baro_convert_start_us = 0;         /* Moc thoi gian bat dau convert D1/D2, don vi us theo TIM2->CNT. */
pt1Filter_t baro_alt_lpf;                   /* Bo loc thong thap PT1 lam muot do cao tu barometer. */
float32_t baro_pressure_ref_pa = 0.0f;      /* Ap suat moc mat dat, don vi Pa, dung lam P0 khi tinh do cao. */
uint8_t  baro_ref_locked = 0;               /* =1 khi da lay du mau ap suat moc va khong cap nhat P0 nua. */
/* Lay trung binh 50 mau ap suat dau lam moc mat dat de tinh do cao tuong doi. */
uint16_t baro_ref_sample_n = 0;             /* So mau ap suat da cong vao qua trinh lay moc. */
float64_t baro_ref_sample_acc = 0.0;        /* Tong ap suat 50 mau dau, dung double de giam sai so khi cong. */
float32_t baro_alt_raw = 0.0f;              /* Do cao tho tinh truc tiep tu cong thuc ap suat, don vi m. */
float32_t baro_alt_filtered = 0.0f;         /* Do cao sau khi gioi han toc do thay doi va loc PT1, don vi m. */
BARO_RAW_DATA_t BARO_DATA;                  /* Goi du lieu baro da xu ly: do cao va timestamp moi nhat. */
uint8_t  baro_init_result = 0xFF;           /* Ket qua khoi tao MS5611; 0xFF la chua khoi tao/chua co ket qua. */
float32_t BARO_ALT_SLEW_MPS = 0.55f;        /* Gioi han do cao baro chi duoc doi toi da 0.55 m/s de loc spike. */
float32_t baro_alt_sane = 0.0f;             /* Do cao baro sau khi gioi han buoc nhay bat thuong. */
static uint8_t baro_alt_sane_init = 0U;     /* Co khoi tao gia tri dau cho bo gioi han buoc nhay. */
static uint32_t baro_alt_sane_prev_us = 0U; /* Moc thoi gian lan cap nhat baro_alt_sane truoc do. */

/* Observer do cao ket hop barometer va gia toc truc Z. */
float32_t vz_est   = 0.0f;                 /* Van toc dung uoc luong, don vi m/s; duong la bay len. */
float32_t   alt_est         = 0.0f;        /* Do cao uoc luong sau khi tron baro + gia toc Z, don vi m. */
float32_t az_up = 0.0f;                    /* Gia toc theo truc thang dung huong len, don vi m/s^2. */
uint8_t     alt_obs_init    = 0;           /* =1 khi observer da duoc nap gia tri dau tu barometer. */
float32_t   ALT_OBS_W1      = 2.1f;        /* He so sua loi do cao: baro_alt_filtered - alt_est. */
float32_t   ALT_OBS_W2      = 2.25f;       /* He so sua loi van toc dung dua tren sai lech do cao baro. */


float32_t   ALT_OBS_W3      = 1.0f;        /* He so uoc luong bias gia toc Z de giam troi van toc/do cao. */
float32_t accel_bias_z_est = 0.0f;         /* Bias gia toc Z uoc luong, don vi m/s^2. */
float32_t   ALT_OBS_BIAS_CLAMP = 0.5f;     /* Gioi han bias gia toc Z trong khoang +-0.5 m/s^2. */

const pidControllerParam_t pidVzParam = {
    .kP  = 0.10f,      
    .kI  = 0.03f,
    .kD  = 0.0f,
    .kFF = 0.0f
};
pidController_t PID_VZ = {
    .param      = pidVzParam,
    .dTermLpfHz = 0.0f,
    .errorLpfHz = 0.0f,
};


float32_t ALT_HOLD_HOVER_BASE = 0.45f;     
float32_t ALT_KP_ALT          = 0.55f;     


float32_t ALT_KI_ALT          = 0.04f;     
float32_t ALT_I_VZ_CLAMP      = 0.10f;     
float32_t ALT_I_UNWIND_GAIN   = 3.0f;      
float32_t alt_vz_i   = 0.0f;      
float32_t ALT_VZ_CLAMP        = 0.5f;      
float32_t ALT_DT_CLAMP        = 0.25f;     
float32_t ALT_T_MIN_MARGIN    = 0.06f;     
float32_t ALT_STICK_DB        = 0.05f;     


float32_t ALT_EXIT_RAMP_PILOT_S = 0.4f;    
float32_t ALT_EXIT_RAMP_FAULT_S = 0.9f;    
float32_t ALT_EXIT_FAULT_RISE_MARGIN = 0.03f; 
float32_t ALT_EXIT_FAULT_DROP_MARGIN = 0.05f; 
float32_t ALT_CEIL_REL          = 3.0f;
float32_t ALT_CEIL_TAPER        = 0.3f;    
float32_t ALT_ENGAGE_HOLD_S     = 0.15f;
float32_t ALT_ENGAGE_MAX_VZ     = 0.8f;

/* Trang thai giu do cao: tat, dang giu, hoac dang tra lai quyen dieu khien. */
typedef enum { ALT_OFF = 0, ALT_ENGAGED, ALT_EXIT_RAMP } AltHoldState_t;
AltHoldState_t althold_state = ALT_OFF;

uint8_t  althold_engaged   = 0;
float32_t alt_setpoint     = 0.0f;
float32_t vz_sp            = 0.0f;
float32_t althold_T        = 0.0f;
uint8_t  althold_fault = 0;
static   uint32_t althold_prev_us   = 0;
static   uint32_t althold_exit_start_us = 0;   
static   uint32_t althold_want_start_us = 0;   
static   float32_t althold_exit_from_T  = 0.0f;
static   float32_t althold_neutral_thr  = 0.0f;
static   float32_t alt_hold_origin_m    = 0.0f;
float32_t g_manual_T           = MOTOR_IDLE; 

static uint32_t i2c2_last_start_us = 0;

uint16_t CH[16];
float32_t OFFSET_CH[4] = {987.0,994.0,172.0,994.0};
float32_t Desired_Value[4];
float32_t alpha_lpf_rx = 0.96;  
extern volatile uint16_t crsf_channel[16];
extern volatile uint8_t crsf_frame_done;
extern volatile uint32_t crsf_last_frame_us;
uint8_t arm_block_reason = ARM_BLOCK_GYRO;
uint8_t arm_cycle_required = 1;
uint8_t disarm_reason = DISARM_REASON_NONE;
uint32_t rc_lost_delta_us = 0;
uint8_t alt_hold_block = 0;

float32_t Throttle;
float32_t T;  
float32_t angle_desired[3];
float32_t angle_rate_desired[3];
float32_t pos_desired[3];
float32_t vel_desired[3];

float32_t u_roll,u_pitch,u_yaw; 

float32_t m[4];
float32_t cmd[4];
float32_t pid_rate_scale = 1e-3;

float32_t pwm[4];


uint8_t boot_gyro_calib_done = 0;


/* Trang thai bay chinh cua quadcopter. */
typedef enum {
    FLIGHT_DISARM     = 0,
    FLIGHT_ARMED_IDLE = 1,
    FLIGHT_TAKING_OFF = 2,
    FLIGHT_IN_AIR     = 3,
    FLIGHT_LANDING    = 4
} flight_state_t;

flight_state_t flight_state = FLIGHT_DISARM;

float32_t TAKEOFF_THROTTLE = 0.45f;   
float32_t AUTO_TAKEOFF_TARGET_M = 0.80f; 
float32_t AUTO_TAKEOFF_CLIMB_T = 0.57f;  
float32_t AUTO_TAKEOFF_VZ_MAX = 0.35f;   
float32_t AUTO_TAKEOFF_VZ_MIN = -0.15f;  
float32_t AUTO_TAKEOFF_KP_ALT = 0.80f;   
float32_t AUTO_TAKEOFF_LIFTOFF_M = 0.15f;
float32_t AUTO_TAKEOFF_COMPLETE_ERR_M = 0.10f;
float32_t AUTO_TAKEOFF_COMPLETE_VZ = 0.20f;
float32_t AUTO_TAKEOFF_MAX_TILT_DEG = 10.0f;
float32_t AUTO_TAKEOFF_ABORT_TILT_DEG = 25.0f;
float32_t AUTO_TAKEOFF_OVERSHOOT_M = 0.30f;
float32_t AUTO_TAKEOFF_THRUST_FAIL_T = 0.68f;
float32_t AUTO_TAKEOFF_THRUST_FAIL_VZ = -0.10f;
float32_t LANDING_THROTTLE = 0.36f;   
float32_t LANDING_DESCENT_VZ = -0.20f; 
float32_t LANDING_T_MAX = 0.70f;       
float32_t LANDING_KI = 0.10f;          
float32_t LANDING_MAX_TILT_DEG = 7.0f; 
float32_t in_air_throttle_floor = MOTOR_IDLE;
static uint32_t flight_transition_us = 0;
static uint32_t takeoff_start_us = 0;
static float32_t takeoff_entry_throttle = 0.0f;
static uint32_t landing_start_us = 0;
static uint32_t flight_sw_high_start_us = 0;
static uint32_t flight_sw_low_start_us = 0;
static float32_t landing_start_throttle = 0.0f;
static uint32_t landing_ctrl_last_baro_ts = 0;
static uint32_t landing_ctrl_prev_us = 0;
static uint32_t landing_fallback_start_us = 0;
static float32_t landing_fallback_from_T = 0.0f;
static float32_t landing_ctrl_T_max = 0.70f;
static uint8_t landing_ctrl_initialized = 0;
static uint8_t landing_closed_loop = 0;
static uint8_t landing_fallback = 0;
static uint8_t landing_abort_inhibit = 0;
static uint8_t landing_timeout_exceeded = 0;

#define AUTO_TAKEOFF_RAMP_US          1500000U
#define AUTO_TAKEOFF_TIMEOUT_US       8000000U
#define AUTO_TAKEOFF_LIFTOFF_TIMEOUT_US 4000000U
#define AUTO_TAKEOFF_STABLE_US         500000U
#define AUTO_TAKEOFF_TILT_HOLD_US      300000U
#define AUTO_TAKEOFF_THRUST_FAIL_HOLD_US 1000000U
#define LANDING_FALLBACK_RAMP_US 9000000U
#define LANDING_TIMEOUT_US     18000000U
#define LANDING_VZ_FAST_LIMIT  (-0.35f)
#define LANDING_VZ_SLOW_LIMIT  (-0.05f)
#define GROUND_REF_MIN_SAMPLES 25U
#define TOUCHDOWN_MIN_LANDING_US 2000000U
#define TOUCHDOWN_STABLE_US    1000000U
#define TOUCHDOWN_MIN_BARO_SAMPLES 10U
#define FLIGHT_SW_LOW_THRESHOLD      800U
#define FLIGHT_SW_HIGH_THRESHOLD     1600U
#define FLIGHT_SW_TRIGGER_HOLD_US    200000U
#define LANDING_TRIGGER_US     500000U


float32_t TOUCHDOWN_T_MARGIN = 0.02f;
float32_t TOUCHDOWN_ALT_STABLE_M = 0.10f;
float32_t TOUCHDOWN_VZ_MIN_MPS = -0.08f;
float32_t TOUCHDOWN_VZ_MAX_MPS = 0.15f;
float32_t TOUCHDOWN_TILT_MAX_DEG = 15.0f;
float32_t TOUCHDOWN_GYRO_MAX_RAD_S = 0.35f;
float32_t TOUCHDOWN_YAW_GYRO_MAX_RAD_S = 0.52f;
float32_t TOUCHDOWN_ACCEL_MAX_MPS2 = 2.0f;
static float32_t ground_alt_ref = 0.0f;
static uint16_t ground_ref_samples = 0;
static uint8_t ground_ref_valid = 0;
static float32_t ground_ref_accum = 0.0f;
static uint32_t ground_ref_last_baro_ts = 0;
static uint32_t touchdown_candidate_start_us = 0;
static uint32_t touchdown_landing_start_seen_us = 0;
static uint32_t touchdown_candidate_last_baro_ts = 0;
static uint16_t touchdown_candidate_baro_samples = 0;
static float32_t touchdown_candidate_alt_min = 0.0f;
static float32_t touchdown_candidate_alt_max = 0.0f;

typedef enum {
    TAKEOFF_PHASE_IDLE = 0,
    TAKEOFF_PHASE_RAMP,
    TAKEOFF_PHASE_CLIMB
} takeoff_phase_t;

static takeoff_phase_t takeoff_phase = TAKEOFF_PHASE_IDLE;
static uint8_t takeoff_liftoff = 0U;
static uint8_t takeoff_complete = 0U;
static uint8_t takeoff_fault = 0U; 
static float32_t takeoff_origin_alt = 0.0f;
static float32_t takeoff_target_alt = 0.0f;
static float32_t takeoff_alt_error = 0.0f;
static uint8_t takeoff_ctrl_initialized = 0U;
static uint32_t takeoff_ctrl_last_baro_ts = 0U;
static uint32_t takeoff_ctrl_prev_us = 0U;
static uint32_t takeoff_stable_start_us = 0U;
static uint32_t takeoff_tilt_start_us = 0U;
static uint32_t takeoff_thrust_fail_start_us = 0U;
static uint32_t althold_last_baro_ts = 0U;


uint32_t freq_pid_rate = 2500; 
uint32_t current_time_pid_rate,prev_time_pid_rate,dt_pid_rate;

uint32_t freq_pid_angle = 4000; 
uint32_t current_time_pid_angle,prev_time_pid_angle,dt_pid_angle;

uint16_t Oneshot_Value[4] = {1000,1000,1000,1000};


void IMU_PROCESS(void);
void Filter_Init(void);
void PID_Init(void);
void RESET_ALL_PID(void);
void RESET_ANGLE_PID(void);
void RESET_RATE_PID(void);
void MPC_RATE_MODE(void);
void MPC_ANGLE_MODE(void);
void MPC(void);
void RX_PROCESS(void);
void Control_Motor(void);
void ALTITUDE_HOLD(void);   
void TAKEOFF_CONTROL(void);
void LANDING_CONTROL(void);
void update_led_status(void);
void boot_gyro_auto_calib(void);
void check_gyro_recalib_gesture(void);
void update_flight_state(void);
static void lock_level_setpoint(void);
static void set_landing_attitude_setpoint(void);
static void freeze_all_pid_integrators(void);
static void set_motor_cmd_all(float32_t value);
static uint8_t landing_control_finished(uint32_t now);
static void ground_reference_reset(void);
static void ground_reference_update(uint32_t now);
static uint8_t touchdown_detector_update(uint32_t now);
static uint8_t baro_control_ready(uint32_t now);
static uint8_t auto_takeoff_ready(uint32_t now);
static void altitude_hold_seed(float32_t seed_T, float32_t setpoint_m,
                               float32_t origin_m, uint32_t now);
static inline uint8_t is_rc_lost(void);
static inline void LED_RED_set(uint8_t on);
static inline void LED_BLUE_set(uint8_t on);
static inline void LED_YELLOW_set(uint8_t on);
static uint32_t althold_fault_seen_us = 0U;
static uint8_t  althold_fault_prev    = 0U;

static void imu_latch_spi_sample(void)
{
    ICM42688P_DATA_RDY = 1;

    for(int i = 0 ; i < 3 ; i ++){
        raw_adc_acc_imu[i]  = (float32_t)((int16_t)((imu_spi_rx[2*i + 1] << 8) | imu_spi_rx[2*i + 2])) * ICM42688P.convert_factor[i];
        raw_adc_gyro_imu[i] = (float32_t)((int16_t)((imu_spi_rx[2*i + 7] << 8) | imu_spi_rx[2*i + 8])) * ICM42688P.convert_factor[i + 3] * DEG_TO_RAD;

        gyro_adc_imu_filtered[i] = biquadFilterApply(&gyro_notch[i], raw_adc_gyro_imu[i]);
        gyro_adc_imu_filtered[i] = biquadFilterApply(&gyro_lpf[i], gyro_adc_imu_filtered[i]) - IMU_CALIBRATE.offset_gyro[i];
        acc_adc_imu_filtered[i] = biquadFilterApply(&acc_lpf[i], raw_adc_acc_imu[i]);
    }
}

void IMU_DataReady_IRQHandler(void){
	if(ICM42688P.cfg.ready){
		       uint32_t prev_timestamp = ICM42688P_DATA.timestamp;
		       ICM42688P_DATA.timestamp = TIM2->CNT;
		       /* Ban thanh ghi: doc block IMU ngay trong su kien data-ready bang SPI polling. */
		       GPIOA->BSRR = ((uint32_t)CS_ICM42688P_Pin << 16);
		       for(int i = 0 ; i < 13 ; i ++){
		           imu_spi_rx[i] = Reg_SPI1_transfer(imu_spi_tx[i]);
		       }
		       GPIOA->BSRR = CS_ICM42688P_Pin;
		       imu_latch_spi_sample();
				ICM42688P_DATA.dt = (prev_timestamp == 0U)
				                   ? 0.001f
				                   : (float32_t)(uint32_t)(ICM42688P_DATA.timestamp - prev_timestamp) * 1e-6f;
    }
}

/* Doc MS5611 theo FSM khong chan CPU; ket qua la do cao da loc. */
void BARO_PROCESS(void){
    if (baro_init_result != MS5611_INIT_OK) {
        static uint32_t baro_init_retry_us = 0;
        if ((uint32_t)(TIM2->CNT - baro_init_retry_us) >= 100000U) {
            baro_init_retry_us = TIM2->CNT;
            
            baro_init_result = MS5611_Init(&MS5611_BARO);
            if (baro_init_result == MS5611_INIT_OK) {
                baro_state = MS5611_S_START_D1;   
            }
        }
        return;
    }

    switch (baro_state) {
    case MS5611_S_START_D1:
        if (MS5611_StartConvertD1(&MS5611_BARO) == HAL_OK) {
            baro_convert_start_us = TIM2->CNT;
            baro_state = MS5611_S_WAIT_D1_CONVERT;
        }
        break;

    case MS5611_S_WAIT_D1_CONVERT:
        if ((TIM2->CNT - baro_convert_start_us) >= 10000U) {   
            baro_read_done = 0;
            if (MS5611_ReadADC_Register(&MS5611_BARO) == HAL_OK) {
                i2c2_last_start_us = TIM2->CNT;
                baro_read_done = 1;
                baro_state = MS5611_S_WAIT_D1_READ;
            }
        }
        break;

    case MS5611_S_WAIT_D1_READ:
        if (baro_read_done) {
            MS5611_LatchD1(&MS5611_BARO);
            baro_state = MS5611_S_START_D2;
        } else if ((TIM2->CNT - i2c2_last_start_us) > 50000U) {
            Reg_I2C2_Init();
            baro_state = MS5611_S_START_D1;
        }
        break;

    case MS5611_S_START_D2:
        if (MS5611_StartConvertD2(&MS5611_BARO) == HAL_OK) {
            baro_convert_start_us = TIM2->CNT;
            baro_state = MS5611_S_WAIT_D2_CONVERT;
        }
        break;

    case MS5611_S_WAIT_D2_CONVERT:
        if ((TIM2->CNT - baro_convert_start_us) >= 10000U) {
            baro_read_done = 0;
            if (MS5611_ReadADC_Register(&MS5611_BARO) == HAL_OK) {
                i2c2_last_start_us = TIM2->CNT;
                baro_read_done = 1;
                baro_state = MS5611_S_WAIT_D2_READ;
            }
        }
        break;

    case MS5611_S_WAIT_D2_READ:
        if (baro_read_done) {
            MS5611_LatchD2(&MS5611_BARO);
            MS5611_ComputePressure(&MS5611_BARO);

            
            if (!baro_ref_locked) {
                baro_ref_sample_acc += (float64_t)MS5611_BARO.pressure_pa;
                baro_ref_sample_n++;
                if (baro_ref_sample_n >= 50U) {
                    baro_pressure_ref_pa = (float32_t)(baro_ref_sample_acc / (float64_t)baro_ref_sample_n);
                    baro_ref_locked = 1;
                }
            } else if (baro_pressure_ref_pa > 0.0f) {
                float32_t ratio = MS5611_BARO.pressure_pa / baro_pressure_ref_pa;
                baro_alt_raw = 44330.0f * (1.0f - powf(ratio, 0.1903f));
                {
                    uint32_t now_us = TIM2->CNT;
                    if (!baro_alt_sane_init) {
                        baro_alt_sane = baro_alt_raw;
                        baro_alt_sane_init = 1U;
                    } else {
                        float32_t dt = (float32_t)(uint32_t)(now_us - baro_alt_sane_prev_us) * 1e-6f;
                        if (dt < 0.005f || dt > 0.1f) dt = 0.02f;
                        float32_t max_step = BARO_ALT_SLEW_MPS * dt;
                        float32_t delta = baro_alt_raw - baro_alt_sane;
                        if (delta > max_step) {
                            baro_alt_sane += max_step;
                        } else if (delta < -max_step) {
                            baro_alt_sane -= max_step;
                        } else {
                            baro_alt_sane = baro_alt_raw;
                        }
                    }
                    baro_alt_sane_prev_us = now_us;
                }
                baro_alt_filtered = pt1FilterApply(&baro_alt_lpf, baro_alt_sane);
                BARO_DATA.alt = baro_alt_filtered;
                BARO_DATA.timestamp = TIM2->CNT;

                


            }
            baro_state = MS5611_S_START_D1;
        } else if ((TIM2->CNT - i2c2_last_start_us) > 50000U) {
            Reg_I2C2_Init();
            baro_state = MS5611_S_START_D1;
        }
        break;
    }
}


/* Thu tu khoi tao ban chinh: HAL core, sau do cau hinh ngoai vi bang thanh ghi Reg_... */
int main(void)
{
  HAL_Init();
  Reg_SystemClock_Config();
  HAL_InitTick(TICK_INT_PRIORITY);
  Reg_GPIO_Init();
  Reg_TIM2_Init();
  Reg_TIM_PWM_Init();
  Reg_USART1_Init();
  Reg_SPI1_Init();
  Reg_I2C2_Init();

  LED_RED_set(0);
  LED_BLUE_set(0);
  LED_YELLOW_set(0);

  /* TIM3/TIM4 da duoc Reg_TIM_PWM_Init() start; chi nap muc xung an toan ban dau. */
  TIM4->CCR1 = Oneshot_Value[0];
  TIM4->CCR2 = Oneshot_Value[1];
  TIM3->CCR3 = Oneshot_Value[2];
  TIM3->CCR4 = Oneshot_Value[3];

  baro_init_result = MS5611_Init(&MS5611_BARO);

  ICM42688P.cfg = CFG_ICM42688P_Default;
  ICM_42688_Init(&ICM42688P, CS_ICM42688P_GPIO_Port, CS_ICM42688P_Pin);

  /* ICM42688 SPI read: bit 7 = 1 nen OR 0x80; doc block tu ACCEL_DATA_X1. */
  imu_spi_tx[0] = 0x80 | ICM42688_ACCEL_DATA_X1;
  imu_spi_rx[0] = 0x00;
  for(int i = 1 ; i < 13 ; i ++){
    imu_spi_tx[i] = 0x00;
    imu_spi_rx[i] = 0x00;
  }
  ICM_42688_Config(&ICM42688P);

  Filter_Init();
  PID_Init();

  ICM42688P.cfg.ready = 1;

  while (1)
  {
    /* Vong lap chinh: doc cam bien, cap nhat trang thai bay, tinh dieu khien va xuat motor. */
    IMU_PROCESS();
    RX_PROCESS();
    boot_gyro_auto_calib();
    check_gyro_recalib_gesture();
    BARO_PROCESS();     
    update_flight_state();
    ALTITUDE_HOLD();    
    TAKEOFF_CONTROL();  
    LANDING_CONTROL();  
    MPC();
    update_led_status();

  }
}


static void lock_level_setpoint(void)
{
    for(int i = 0 ; i < 3 ; i ++){
        angle_rate_desired[i] = 0.0f;
        angle_desired[i] = 0.0f;
        pos_desired[i] = 0.0f;
        vel_desired[i] = 0.0f;
    }
}

static void set_landing_attitude_setpoint(void)
{
    for(int i = 0; i < 3; i++){
        pos_desired[i] = 0.0f;
        vel_desired[i] = 0.0f;
    }
    angle_desired[2] = 0.0f;
    angle_rate_desired[2] = 0.0f;

    if(is_rc_lost()){
        angle_desired[0] = 0.0f;
        angle_desired[1] = 0.0f;
        return;
    }

    float32_t max_tilt = constrainf(LANDING_MAX_TILT_DEG, 0.0f, 12.0f);
    float32_t roll_cmd = (Desired_Value[RX_ROLL_CH] - OFFSET_CH[RX_ROLL_CH])
                       * (max_tilt / 819.0f);
    float32_t pitch_cmd = -1.0f
                        * (Desired_Value[RX_PITCH_CH] - OFFSET_CH[RX_PITCH_CH])
                        * (max_tilt / 819.0f);
    angle_desired[0] = constrainf(roll_cmd, -max_tilt, max_tilt);
    angle_desired[1] = constrainf(pitch_cmd, -max_tilt, max_tilt);
}

static void freeze_all_pid_integrators(void)
{
    PidFreezeIntegrator(&PID_RATE_ROLL);
    PidFreezeIntegrator(&PID_RATE_PITCH);
    PidFreezeIntegrator(&PID_RATE_YAW);
    PidFreezeIntegrator(&PID_ROLL);
    PidFreezeIntegrator(&PID_PITCH);
    PidFreezeIntegrator(&PID_YAW);
}

static void set_motor_cmd_all(float32_t value)
{
    value = constrainf(value, 0.0f, 1.0f);
    for(int i = 0 ; i < 4 ; i ++){
        cmd[i] = value;
    }
}

static void ground_reference_reset(void)
{
    ground_ref_accum = 0.0f;
    ground_ref_last_baro_ts = 0U;
    ground_ref_samples = 0U;
    ground_ref_valid = 0U;
}

static void ground_reference_update(uint32_t now)
{
    if(flight_state != FLIGHT_DISARM && flight_state != FLIGHT_ARMED_IDLE){
        return;
    }

    uint8_t baro_ok = baro_ref_locked
                   && (baro_init_result == MS5611_INIT_OK)
                   && (BARO_DATA.timestamp != 0U)
                   && ((uint32_t)(now - BARO_DATA.timestamp) < 200000U);
    if(!baro_ok || BARO_DATA.timestamp == ground_ref_last_baro_ts){
        return;
    }
    ground_ref_last_baro_ts = BARO_DATA.timestamp;

    float32_t gyro_peak = fabsf(ICM42688P_DATA.w[0]);
    if(fabsf(ICM42688P_DATA.w[1]) > gyro_peak) gyro_peak = fabsf(ICM42688P_DATA.w[1]);
    if(fabsf(ICM42688P_DATA.w[2]) > gyro_peak) gyro_peak = fabsf(ICM42688P_DATA.w[2]);
    float32_t ax = ARHS.acc_linear_fixed_frame[0];
    float32_t ay = ARHS.acc_linear_fixed_frame[1];
    float32_t az = ARHS.acc_linear_fixed_frame[2];
    float32_t accel_norm = sqrtf(ax * ax + ay * ay + az * az);

    
    if(gyro_peak > TOUCHDOWN_GYRO_MAX_RAD_S || accel_norm > TOUCHDOWN_ACCEL_MAX_MPS2){
        return;
    }

    if(ground_ref_samples < GROUND_REF_MIN_SAMPLES){
        ground_ref_accum += baro_alt_filtered;
        ground_ref_samples++;
        ground_alt_ref = ground_ref_accum / (float32_t)ground_ref_samples;
        if(ground_ref_samples >= GROUND_REF_MIN_SAMPLES){
            ground_ref_valid = 1U;
        }
    }
    else{
        
        ground_alt_ref += 0.02f * (baro_alt_filtered - ground_alt_ref);
        if(ground_ref_samples < 0xFFFFU){
            ground_ref_samples++;
        }
    }
}

static uint8_t baro_control_ready(uint32_t now)
{
    return (baro_ref_locked &&
            (baro_init_result == MS5611_INIT_OK) &&
            (BARO_DATA.timestamp != 0U) &&
            ((uint32_t)(now - BARO_DATA.timestamp) < 200000U) &&
            alt_obs_init) ? 1U : 0U;
}

static uint8_t auto_takeoff_ready(uint32_t now)
{
    uint8_t mask = 0U;
    if(baro_control_ready(now)) mask |= (1U << 0);
    if(ground_ref_valid) mask |= (1U << 1);
    if(CH[RX_ALT_HOLD_CH] > 1650U && alt_hold_block == 0U) mask |= (1U << 2);
    if(CH[RX_THROTTLE_CH] < ARM_THROTTLE_MAX) mask |= (1U << 3);
    if(fabsf(ARHS.RPY_DEG[0]) <= AUTO_TAKEOFF_MAX_TILT_DEG &&
       fabsf(ARHS.RPY_DEG[1]) <= AUTO_TAKEOFF_MAX_TILT_DEG){
        mask |= (1U << 4);
    }
    if(fabsf(vz_est) <= 0.30f) mask |= (1U << 5);
    return (mask == 0x3FU) ? 1U : 0U;
}

static float32_t althold_feedback_alt(void)
{
    return alt_obs_init ? alt_est : baro_alt_filtered;
}

static void altitude_hold_seed(float32_t seed_T, float32_t setpoint_m,
                               float32_t origin_m, uint32_t now)
{
    seed_T = constrainf(seed_T, MOTOR_IDLE, 1.0f);
    (void)setpoint_m; (void)origin_m;   
    

    althold_neutral_thr = constrainf(Throttle, 0.0f, 1.0f);
    float32_t hold_alt  = althold_feedback_alt();
    alt_hold_origin_m   = hold_alt - althold_neutral_thr * ALT_CEIL_REL;
    alt_setpoint        = hold_alt;
    alt_vz_i = 0.0f;
    PidReset(&PID_VZ);
    PID_VZ.last_input = vz_est;
    
    float32_t initial_alt_err = alt_setpoint - hold_alt;   
    float32_t initial_vz_sp = constrainf(ALT_KP_ALT * initial_alt_err,
                                         -ALT_VZ_CLAMP, ALT_VZ_CLAMP);
    float32_t initial_p = PID_VZ.param.kP * (initial_vz_sp - vz_est);
    PID_VZ.integrator = constrainf((seed_T - ALT_HOLD_HOVER_BASE) - initial_p,
                                   -ALT_T_MIN_MARGIN, ALT_DT_CLAMP);
    althold_T = seed_T;
    althold_prev_us = now;
    althold_last_baro_ts = BARO_DATA.timestamp;
    althold_fault = 0U;
    althold_state = ALT_ENGAGED;
    althold_engaged = 1U;
    althold_want_start_us = 0U;
    T = seed_T;
}

static uint8_t touchdown_detector_update(uint32_t now)
{
    if(touchdown_landing_start_seen_us != landing_start_us){
        touchdown_landing_start_seen_us = landing_start_us;
        touchdown_candidate_start_us = 0U;
        touchdown_candidate_last_baro_ts = 0U;
        touchdown_candidate_baro_samples = 0U;
        touchdown_candidate_alt_min = 0.0f;
        touchdown_candidate_alt_max = 0.0f;
        landing_timeout_exceeded = 0U;
    }

    uint8_t mask = 0U;
    uint8_t baro_ok = baro_ref_locked
                   && (baro_init_result == MS5611_INIT_OK)
                   && (BARO_DATA.timestamp != 0U)
                   && ((uint32_t)(now - BARO_DATA.timestamp) < 200000U);
    if(baro_ok) mask |= (1U << 0);
    if((uint32_t)(now - landing_start_us) >= TOUCHDOWN_MIN_LANDING_US) mask |= (1U << 1);

    if(T <= LANDING_THROTTLE + TOUCHDOWN_T_MARGIN){
        mask |= (1U << 2);
    }
    if(vz_est >= TOUCHDOWN_VZ_MIN_MPS && vz_est <= TOUCHDOWN_VZ_MAX_MPS){
        mask |= (1U << 3);
    }
    if(fabsf(ARHS.RPY_DEG[0]) <= TOUCHDOWN_TILT_MAX_DEG &&
       fabsf(ARHS.RPY_DEG[1]) <= TOUCHDOWN_TILT_MAX_DEG){
        mask |= (1U << 4);
    }

    if(fabsf(ICM42688P_DATA.w[0]) <= TOUCHDOWN_GYRO_MAX_RAD_S &&
       fabsf(ICM42688P_DATA.w[1]) <= TOUCHDOWN_GYRO_MAX_RAD_S &&
       fabsf(ICM42688P_DATA.w[2]) <= TOUCHDOWN_YAW_GYRO_MAX_RAD_S){
        mask |= (1U << 5);
    }

    float32_t ax = ARHS.acc_linear_fixed_frame[0];
    float32_t ay = ARHS.acc_linear_fixed_frame[1];
    float32_t az = ARHS.acc_linear_fixed_frame[2];
    float32_t touchdown_accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    if(touchdown_accel_norm <= TOUCHDOWN_ACCEL_MAX_MPS2){
        mask |= (1U << 6);
    }
    if(mask == 0x7FU){
        if(touchdown_candidate_start_us == 0U){
            touchdown_candidate_start_us = now;
            touchdown_candidate_last_baro_ts = BARO_DATA.timestamp;
            touchdown_candidate_baro_samples = 1U;
            touchdown_candidate_alt_min = alt_est;
            touchdown_candidate_alt_max = alt_est;
        }
        else if(BARO_DATA.timestamp != touchdown_candidate_last_baro_ts){
            touchdown_candidate_last_baro_ts = BARO_DATA.timestamp;
            if(touchdown_candidate_baro_samples < 0xFFFFU){
                touchdown_candidate_baro_samples++;
            }
            if(alt_est < touchdown_candidate_alt_min){
                touchdown_candidate_alt_min = alt_est;
            }
            if(alt_est > touchdown_candidate_alt_max){
                touchdown_candidate_alt_max = alt_est;
            }
        }
        float32_t touchdown_alt_stability = touchdown_candidate_alt_max - touchdown_candidate_alt_min;

        if(touchdown_alt_stability > TOUCHDOWN_ALT_STABLE_M){
            touchdown_candidate_start_us = 0U;
            touchdown_candidate_last_baro_ts = 0U;
            touchdown_candidate_baro_samples = 0U;
        }
        else if((uint32_t)(now - touchdown_candidate_start_us) >= TOUCHDOWN_STABLE_US &&
                touchdown_candidate_baro_samples >= TOUCHDOWN_MIN_BARO_SAMPLES){
            return 1U;
        }
    }
    else{
        touchdown_candidate_start_us = 0U;
        touchdown_candidate_last_baro_ts = 0U;
        touchdown_candidate_baro_samples = 0U;
        touchdown_candidate_alt_min = 0.0f;
        touchdown_candidate_alt_max = 0.0f;
    }

    return 0U;
}

static uint8_t landing_control_finished(uint32_t now)
{
    if(landing_fallback && landing_fallback_start_us != 0U &&
       (uint32_t)(now - landing_fallback_start_us) >= LANDING_FALLBACK_RAMP_US){
        return 1U;
    }
    if(touchdown_detector_update(now)){
        return 1U;
    }
    if((uint32_t)(now - landing_start_us) >= LANDING_TIMEOUT_US){
        if(!landing_timeout_exceeded){
            landing_timeout_exceeded = 1U;
        }
        

    }
    return 0U;
}

static float32_t pid_mix_scale_from_throttle(void)
{
    float32_t span = AIRMODE_FULL_THROTTLE - MOTOR_IDLE;
    if(span <= 1e-6f){
        return 1.0f;
    }
    return constrainf((T - MOTOR_IDLE) / span, 0.0f, 1.0f);
}

static inline uint8_t is_rc_lost(void)
{
    uint32_t last_frame_us = crsf_last_frame_us;
    uint32_t now = TIM2->CNT;

    if(last_frame_us == 0U){
        rc_lost_delta_us = RC_FAILSAFE_TIMEOUT + 1U;
        return 1U;
    }

    rc_lost_delta_us = (uint32_t)(now - last_frame_us);
    return (rc_lost_delta_us > RC_FAILSAFE_TIMEOUT) ? 1U : 0U;
}

static inline uint8_t is_gyro_ready(void)
{
    return (boot_gyro_calib_done &&
            IMU_CALIBRATE.sample_gyro == Stop &&
            IMU_CALIBRATE.status == CALIBRATE_OKAY) ? 1U : 0U;
}

static uint16_t rx_channel_default(uint8_t index)
{
    if (index == RX_THROTTLE_CH) return 172U;
    if (index < 4U) return (uint16_t)OFFSET_CH[index];
    return 0U;
}

static uint16_t sanitize_rx_channel(uint8_t index, uint16_t value)
{
    if (value <= 2047U) return value;
    return rx_channel_default(index);
}

static void set_not_arm(uint8_t reason)
{
    if(ARM_Status == ARM){
        disarm_reason = reason;
    }
    ARM_Status = NOT_ARM;
}

static uint8_t update_rc_lost_state(uint32_t now)
{
    (void)now;
    return is_rc_lost();
}

/* Xu ly frame CRSF va chuyen gia tri kenh RC sang lenh dieu khien noi bo. */
void RX_PROCESS(void){
    if (crsf_frame_done)
    {
        uint16_t crsf_snapshot[16];
        uint8_t has_frame = 0U;

        NVIC_DisableIRQ(USART1_IRQn);
        if (crsf_frame_done)
        {
            crsf_frame_done = 0U;
            for(int i = 0 ; i < 16 ; i ++) {
                crsf_snapshot[i] = crsf_channel[i];
            }
            has_frame = 1U;
        }
        NVIC_EnableIRQ(USART1_IRQn);

        if (!has_frame) {
            return;
        }

        uint8_t rx_bad = 0;
    	for(int i = 0 ; i < 16 ; i ++) {
            uint16_t raw = crsf_snapshot[i];
            if (raw > 2047U) rx_bad = 1U;
            CH[i] = sanitize_rx_channel((uint8_t)i, raw);
        }
       for(int i = 0 ; i < 4 ; i ++){
        Desired_Value[i] = alpha_lpf_rx * Desired_Value[i] + (1.0 - alpha_lpf_rx) * CH[i];
       }
             uint8_t arm_switch_on = (CH[RX_ARM_CH] > 1500U) ? 1U : 0U;
             uint8_t throttle_safe = (CH[RX_THROTTLE_CH] < ARM_THROTTLE_MAX) ? 1U : 0U;
             uint8_t gyro_ready = is_gyro_ready();
             uint8_t rc_lost = is_rc_lost();

             if(!arm_switch_on){
                 arm_cycle_required = 0U;
             }
             uint8_t arm_request = arm_switch_on;
#if ARM_REQUIRE_SWITCH_CYCLE
             arm_request = (arm_switch_on && !arm_cycle_required) ? 1U : 0U;
#endif

             alt_hold_block = (CH[RX_ALT_HOLD_BLOCK_CH] > 1500U) ? 1U : 0U;

             if(arm_switch_on){
                 if(ARM_Status == ARM){
                     ARM_Status = ARM;
                     disarm_reason = DISARM_REASON_NONE;
                     arm_block_reason = ARM_BLOCK_NONE;
                 }
                 else if(rx_bad){
                     set_not_arm(DISARM_REASON_BAD_RX);
                     arm_block_reason = ARM_BLOCK_BAD_RX;
                     lock_level_setpoint();
                 }
                 else if(rc_lost){
                     set_not_arm(DISARM_REASON_RC_LOST);
                     arm_block_reason = ARM_BLOCK_RC_LOST;
                     lock_level_setpoint();
                 }
                 else if(!arm_request){
                     set_not_arm(DISARM_REASON_WAIT_ARM);
                     arm_block_reason = ARM_BLOCK_WAIT_EDGE;
                     lock_level_setpoint();
                 }
                 else if(!throttle_safe){
                     if(ARM_Status == ARM){
                         set_not_arm(DISARM_REASON_THROTTLE);
                     }
                     arm_block_reason = ARM_BLOCK_THROTTLE;
                     lock_level_setpoint();
                 }
                 else if(!gyro_ready){
                     set_not_arm(DISARM_REASON_GYRO);
                     arm_block_reason = ARM_BLOCK_GYRO;
                     lock_level_setpoint();
                 }
                 else{
                     
                     RESET_RATE_PID();
                     RESET_ANGLE_PID();
                     current_time_pid_angle = TIM2->CNT - freq_pid_angle;
                     current_time_pid_rate  = TIM2->CNT - freq_pid_rate;
                     ARM_Status = ARM;
                     disarm_reason = DISARM_REASON_NONE;
                     arm_block_reason = ARM_BLOCK_NONE;
                 }
             }
             else{
            	 set_not_arm(DISARM_REASON_SWITCH_OFF);
                 arm_block_reason = ARM_BLOCK_SWITCH_OFF;
            	 lock_level_setpoint();
             }

             
             
             
             Throttle = (Desired_Value[RX_THROTTLE_CH] - OFFSET_CH[RX_THROTTLE_CH])*6.101281269066504e-4;
             if(flight_state == FLIGHT_IN_AIR){
                 float32_t pilot_throttle = constrainf(Throttle, MOTOR_IDLE, 1.0f);
                 float32_t manual_T = (pilot_throttle > in_air_throttle_floor) ? pilot_throttle : in_air_throttle_floor;
                 
                 
                 
                 g_manual_T = constrainf(manual_T, MOTOR_IDLE, 1.0f);
                 T = g_manual_T;
                 angle_desired[0]      = (Desired_Value[RX_ROLL_CH] - OFFSET_CH[RX_ROLL_CH]) * (15.0f/819.0f);
                 angle_desired[1]      = -1.0f*(Desired_Value[RX_PITCH_CH] - OFFSET_CH[RX_PITCH_CH]) * (15.0f/819.0f);
                 angle_rate_desired[2] = (Desired_Value[RX_YAW_CH] - OFFSET_CH[RX_YAW_CH]) * (45.0f/819.0f);
             }
             else if(flight_state == FLIGHT_TAKING_OFF){
                 lock_level_setpoint();
             }
             else if(flight_state == FLIGHT_LANDING){
                 
                 set_landing_attitude_setpoint();
             }
             else{
                 
                 
                 lock_level_setpoint();
             }

		  }
	  }


/* Quan ly cac trang thai DISARM, IDLE, TAKEOFF, IN_AIR va LANDING. */
void update_flight_state(void){
    uint32_t now = TIM2->CNT;

    if(flight_state > FLIGHT_LANDING){
        flight_state = FLIGHT_DISARM;
        set_not_arm(DISARM_REASON_BAD_STATE);
        arm_block_reason = ARM_BLOCK_BAD_RX;
    }

    ground_reference_update(now);

    uint8_t rc_lost = update_rc_lost_state(now);
    if(rc_lost){
        arm_cycle_required = 1U;
        arm_block_reason = ARM_BLOCK_RC_LOST;

        
        
        
        
        
        
        if(flight_state == FLIGHT_TAKING_OFF || flight_state == FLIGHT_IN_AIR){
            float32_t entry_T = (flight_state == FLIGHT_IN_AIR && althold_engaged)
                              ? althold_T : T;
            landing_start_throttle = constrainf(entry_T, MOTOR_IDLE, 1.0f);
            flight_state = FLIGHT_LANDING;
            flight_transition_us = now;
            landing_start_us = now;
            landing_abort_inhibit = 1U;
            flight_sw_low_start_us = 0;
            flight_sw_high_start_us = 0;
            current_time_pid_angle = now - freq_pid_angle;
            current_time_pid_rate  = now - freq_pid_rate;
            return;
        }
        if(flight_state == FLIGHT_LANDING){
            
            if(landing_control_finished(now)){
                flight_state = FLIGHT_ARMED_IDLE;
                flight_transition_us = now;
                flight_sw_high_start_us = 0;
                flight_sw_low_start_us = 0;
                in_air_throttle_floor = MOTOR_IDLE;
                landing_abort_inhibit = 0U;
                ground_reference_reset();
            }
            return;
        }
        set_not_arm(DISARM_REASON_RC_LOST);
    }

    if(ARM_Status != ARM){
        if(flight_state != FLIGHT_DISARM){
            flight_transition_us = now;
            ground_reference_reset();
        }
        flight_state = FLIGHT_DISARM;
        flight_sw_high_start_us = 0;
        flight_sw_low_start_us = 0;
        takeoff_start_us = 0;
        landing_start_us = 0;
        in_air_throttle_floor = MOTOR_IDLE;
        takeoff_ctrl_initialized = 0U;
        takeoff_phase = TAKEOFF_PHASE_IDLE;
        takeoff_complete = 0U;
        landing_abort_inhibit = 0U;
        return;
    }

    switch(flight_state){
    case FLIGHT_DISARM:
        flight_state = FLIGHT_ARMED_IDLE;
        flight_transition_us = now;
        flight_sw_high_start_us = 0;
        flight_sw_low_start_us = 0;
        in_air_throttle_floor = MOTOR_IDLE;
        landing_abort_inhibit = 0U;
        break;

    case FLIGHT_ARMED_IDLE:
        if(CH[RX_FLIGHT_SW_CH] > FLIGHT_SW_HIGH_THRESHOLD &&
           auto_takeoff_ready(now)){
            if(flight_sw_high_start_us == 0){
                flight_sw_high_start_us = now;
            }
            else if((uint32_t)(now - flight_sw_high_start_us) >= FLIGHT_SW_TRIGGER_HOLD_US){
                flight_state = FLIGHT_TAKING_OFF;
                flight_transition_us = now;
                takeoff_start_us = now;
                takeoff_entry_throttle = constrainf(Throttle, 0.0f, 1.0f);
                takeoff_ctrl_initialized = 0U;
                takeoff_phase = TAKEOFF_PHASE_IDLE;
                takeoff_liftoff = 0U;
                takeoff_complete = 0U;
                takeoff_fault = 0U;
                takeoff_tilt_start_us = 0U;
                takeoff_thrust_fail_start_us = 0U;
                current_time_pid_angle = now - freq_pid_angle;
                current_time_pid_rate  = now - freq_pid_rate;
                flight_sw_high_start_us = 0;
            }
        }
        else{
            flight_sw_high_start_us = 0;
            auto_takeoff_ready(now);
        }
        break;

    case FLIGHT_TAKING_OFF:
        if(!baro_control_ready(now) ||
           CH[RX_ALT_HOLD_CH] <= 1650U ||
           alt_hold_block != 0U){
            takeoff_fault = 1U;
        }
        if((uint32_t)(now - takeoff_start_us) >= AUTO_TAKEOFF_TIMEOUT_US){
            takeoff_fault = 3U;
        }
        else if(!takeoff_liftoff &&
                (uint32_t)(now - takeoff_start_us) >= AUTO_TAKEOFF_LIFTOFF_TIMEOUT_US){
            takeoff_fault = 2U;
        }

        if(CH[RX_FLIGHT_SW_CH] < FLIGHT_SW_LOW_THRESHOLD ||
           takeoff_fault != 0U){
            flight_state = FLIGHT_LANDING;
            flight_transition_us = now;
            landing_start_us = now;
            landing_start_throttle = constrainf(T, MOTOR_IDLE, 1.0f);
            landing_abort_inhibit = (takeoff_fault != 0U) ? 1U : 0U;
            current_time_pid_angle = now - freq_pid_angle;
            current_time_pid_rate  = now - freq_pid_rate;
        }
        else if(takeoff_complete){
            flight_state = FLIGHT_IN_AIR;
            flight_transition_us = now;
            in_air_throttle_floor = TAKEOFF_THROTTLE;
            g_manual_T = constrainf(TAKEOFF_THROTTLE, MOTOR_IDLE, 1.0f);
            altitude_hold_seed(T, takeoff_target_alt,
                               takeoff_origin_alt, now);
        }
        break;

    case FLIGHT_IN_AIR:
        if(CH[RX_FLIGHT_SW_CH] < FLIGHT_SW_LOW_THRESHOLD){
            if(flight_sw_low_start_us == 0){
                flight_sw_low_start_us = now;
            }
            else if((uint32_t)(now - flight_sw_low_start_us) >= LANDING_TRIGGER_US){
                flight_state = FLIGHT_LANDING;
                flight_transition_us = now;
                landing_start_us = now;
                landing_start_throttle = constrainf(althold_engaged ? althold_T : T,
                                                     MOTOR_IDLE, 1.0f);
                landing_abort_inhibit = 0U;
                current_time_pid_angle = now - freq_pid_angle;
                current_time_pid_rate  = now - freq_pid_rate;
                flight_sw_low_start_us = 0;
            }
        }
        else{
            flight_sw_low_start_us = 0;
        }
        break;

    case FLIGHT_LANDING:
        if(landing_abort_inhibit &&
           CH[RX_FLIGHT_SW_CH] < FLIGHT_SW_LOW_THRESHOLD){
            landing_abort_inhibit = 0U;
        }
        else if(!landing_abort_inhibit && !rc_lost &&
                CH[RX_FLIGHT_SW_CH] > FLIGHT_SW_HIGH_THRESHOLD){
            flight_state = FLIGHT_IN_AIR;
            flight_transition_us = now;
            flight_sw_low_start_us = 0;
            in_air_throttle_floor = TAKEOFF_THROTTLE;
        }
        else if(landing_control_finished(now)){
            flight_state = FLIGHT_ARMED_IDLE;
            flight_transition_us = now;
            flight_sw_high_start_us = 0;
            flight_sw_low_start_us = 0;
            in_air_throttle_floor = MOTOR_IDLE;
            landing_abort_inhibit = 0U;
            ground_reference_reset();
        }
        break;

    default:
        flight_state = FLIGHT_DISARM;
        set_not_arm(DISARM_REASON_BAD_STATE);
        arm_block_reason = ARM_BLOCK_BAD_RX;
        break;
    }
}

/* Dieu phoi cac bo dieu khien theo trang thai bay hien tai. */
void MPC(void){
    switch(flight_state){
    case FLIGHT_DISARM:
        T = 0.0f;
        u_roll = 0.0f;
        u_pitch = 0.0f;
        u_yaw = 0.0f;
        lock_level_setpoint();
        set_motor_cmd_all(0.0f);
        Control_Motor();
        RESET_ALL_PID();
        break;

    case FLIGHT_ARMED_IDLE:
        T = 0.0f;
        u_roll = 0.0f;
        u_pitch = 0.0f;
        u_yaw = 0.0f;
        lock_level_setpoint();
        freeze_all_pid_integrators();
        set_motor_cmd_all(MOTOR_IDLE);
        Control_Motor();
        break;

    case FLIGHT_TAKING_OFF:
        MPC_ANGLE_MODE();
        MPC_RATE_MODE();
        Control_Motor();
        break;

    case FLIGHT_IN_AIR:
        MPC_ANGLE_MODE();
        MPC_RATE_MODE();
        Control_Motor();
        break;

    case FLIGHT_LANDING:
        set_landing_attitude_setpoint();
        MPC_ANGLE_MODE();
        MPC_RATE_MODE();
        Control_Motor();
        break;

    default:
        flight_state = FLIGHT_DISARM;
        set_not_arm(DISARM_REASON_BAD_STATE);
        T = 0.0f;
        u_roll = 0.0f;
        u_pitch = 0.0f;
        u_yaw = 0.0f;
        lock_level_setpoint();
        set_motor_cmd_all(0.0f);
        Control_Motor();
        RESET_ALL_PID();
        break;
    }
}
void PID_Init(void){
	PidInit(&PID_RATE_ROLL);
	PidInit(&PID_RATE_PITCH);
	PidInit(&PID_RATE_YAW);
	PidInit(&PID_ROLL);
	PidInit(&PID_PITCH);
	PidInit(&PID_YAW);
	PidInit(&PID_VZ);   
}
void RESET_ALL_PID(void){
	PidReset(&PID_RATE_ROLL);
	PidReset(&PID_RATE_PITCH);
	PidReset(&PID_RATE_YAW);
	PidReset(&PID_ROLL);
	PidReset(&PID_PITCH);
	PidReset(&PID_YAW);
	PidReset(&PID_VZ);              
	althold_state   = ALT_OFF;
	althold_engaged = 0;
}

void RESET_ANGLE_PID(void){
	PidReset(&PID_ROLL);
	PidReset(&PID_PITCH);
	PidReset(&PID_YAW);
}
void RESET_RATE_PID(void){
	PidReset(&PID_RATE_ROLL);
	PidReset(&PID_RATE_PITCH);
	PidReset(&PID_RATE_YAW);
}
/* Doi lenh motor 0..1 sang xung PWM/OneShot cho 4 ESC. */
void Control_Motor(void){
    for(int i = 0 ; i < 4 ; i ++){
        /* cmd 0..1 duoc doi sang xung 1000..2000 us cho ESC. */
        pwm[i] = 1000 + cmd[i] * 1000;
        Oneshot_Value[i] = pwm[i];
    }
    TIM4->CCR1 = Oneshot_Value[0];  
    TIM4->CCR2 = Oneshot_Value[1];  
    TIM3->CCR3 = Oneshot_Value[2];  
    TIM3->CCR4 = Oneshot_Value[3];  
}
void MPC_ANGLE_MODE(void){
	float32_t feedback[3];
	feedback[0] = ARHS.RPY_DEG[0];
	feedback[1] = ARHS.RPY_DEG[1];
	feedback[2] = ARHS.RPY_DEG[2];

	if(TIM2->CNT - current_time_pid_angle >=(freq_pid_angle-100)){
		prev_time_pid_angle = current_time_pid_angle;
		current_time_pid_angle = TIM2->CNT;
		dt_pid_angle = current_time_pid_angle - prev_time_pid_angle;

		angle_rate_desired[0] = Pid_Calculate(&PID_ROLL, angle_desired[0], feedback[0], 0.004, -100, 100);
		angle_rate_desired[1] = Pid_Calculate(&PID_PITCH, angle_desired[1], feedback[1], 0.004, -100, 100);
		
	}
}


/* Giu do cao bang vong ngoai altitude va vong trong vertical speed. */
void ALTITUDE_HOLD(void){
    uint32_t now = TIM2->CNT;

    

    if (flight_state != FLIGHT_IN_AIR) {
        althold_state         = ALT_OFF;
        althold_engaged       = 0;
        althold_fault     = 0U;
        althold_want_start_us = 0;
        return;
    }

    uint8_t baro_ok = baro_ref_locked
                   && (baro_init_result == MS5611_INIT_OK)
                   && (BARO_DATA.timestamp != 0U)
                   && ((uint32_t)(now - BARO_DATA.timestamp) < 200000U);   

    uint8_t alt_hold_switch = (CH[RX_ALT_HOLD_CH] > 1650U) ? 1U : 0U;
    uint8_t want_raw = (alt_hold_switch != 0U) && (alt_hold_block == 0U) && baro_ok;
    uint8_t want = want_raw;
    if ((althold_state == ALT_OFF) || (althold_state == ALT_EXIT_RAMP)) {
        float32_t vz_abs = (vz_est >= 0.0f) ? vz_est : -vz_est;
        uint8_t vz_ok = ((ALT_ENGAGE_MAX_VZ <= 0.0f) || (vz_abs <= ALT_ENGAGE_MAX_VZ)) ? 1U : 0U;
        uint32_t hold_us = (ALT_ENGAGE_HOLD_S <= 0.0f) ? 0U : (uint32_t)(ALT_ENGAGE_HOLD_S * 1000000.0f);

        if (want_raw && vz_ok) {
            if (althold_want_start_us == 0U) althold_want_start_us = now;
        } else {
            althold_want_start_us = 0;
        }

        want = want_raw && vz_ok &&
               ((hold_us == 0U) ||
                ((althold_want_start_us != 0U) && ((uint32_t)(now - althold_want_start_us) >= hold_us)));
    } else if (!want_raw) {
        althold_want_start_us = 0;
    }

    switch (althold_state) {

    case ALT_OFF:
        if (want) {
            

            althold_neutral_thr = constrainf(Throttle, 0.0f, 1.0f);   
            float32_t hold_alt  = althold_feedback_alt();
            alt_hold_origin_m   = hold_alt - althold_neutral_thr * ALT_CEIL_REL;
            alt_setpoint        = hold_alt;
            alt_vz_i            = 0.0f;                                 
            PidReset(&PID_VZ);
            PID_VZ.last_input = vz_est;
            PID_VZ.integrator = constrainf(g_manual_T - ALT_HOLD_HOVER_BASE,
                                           -ALT_T_MIN_MARGIN, ALT_DT_CLAMP);  
            althold_T         = g_manual_T;   
            althold_prev_us   = now;
            althold_last_baro_ts = BARO_DATA.timestamp;
            althold_fault = 0;
            althold_state     = ALT_ENGAGED;
            althold_want_start_us = 0;
            T = althold_T;
        }
        
        break;

    case ALT_ENGAGED:
        if (!want) {
            
            althold_exit_from_T   = althold_T;
            althold_exit_start_us = now;
            althold_fault     = baro_ok ? 0U : 1U;
            althold_state         = ALT_EXIT_RAMP;
            T = althold_T;            
            break;
        }
        
        if (BARO_DATA.timestamp != althold_last_baro_ts) {
            althold_last_baro_ts = BARO_DATA.timestamp;

            float32_t dt = (float32_t)(now - althold_prev_us) * 1e-6f;
            if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;
            althold_prev_us = now;

            


            {
                float32_t ch2 = constrainf(Throttle, 0.0f, 1.0f);
                if      (ch2 > althold_neutral_thr + ALT_STICK_DB) althold_neutral_thr = ch2 - ALT_STICK_DB;
                else if (ch2 < althold_neutral_thr - ALT_STICK_DB) althold_neutral_thr = ch2 + ALT_STICK_DB;
            }
            alt_setpoint = alt_hold_origin_m + althold_neutral_thr * ALT_CEIL_REL;

            float32_t feedback_alt = althold_feedback_alt();

            

            float32_t alt_err = alt_setpoint - feedback_alt;
            float32_t alt_i_gain = ALT_KI_ALT;
            if (((alt_vz_i > 0.0f) && (alt_err < 0.0f)) ||
                ((alt_vz_i < 0.0f) && (alt_err > 0.0f))) {
                alt_i_gain *= ALT_I_UNWIND_GAIN;
            }
            alt_vz_i += alt_i_gain * alt_err * dt;
            alt_vz_i  = constrainf(alt_vz_i, -ALT_I_VZ_CLAMP, ALT_I_VZ_CLAMP);
            vz_sp = constrainf(ALT_KP_ALT * alt_err + alt_vz_i,
                               -ALT_VZ_CLAMP, ALT_VZ_CLAMP);

            

            alt_setpoint = constrainf(alt_setpoint,
                                      alt_hold_origin_m,
                                      alt_hold_origin_m + ALT_CEIL_REL);

            

            {
                float32_t head_up = (alt_hold_origin_m + ALT_CEIL_REL) - feedback_alt;
                float32_t head_dn = feedback_alt - alt_hold_origin_m;
                float32_t taper = (ALT_CEIL_TAPER > 0.05f) ? ALT_CEIL_TAPER : 0.05f;
                if (vz_sp > 0.0f) vz_sp *= constrainf(head_up / taper, 0.0f, 1.0f);
                if (vz_sp < 0.0f) vz_sp *= constrainf(head_dn / taper, 0.0f, 1.0f);
            }

            


            float32_t dT = Pid_Calculate(&PID_VZ, vz_sp, vz_est, dt, -ALT_T_MIN_MARGIN, ALT_DT_CLAMP);

            althold_T = constrainf(ALT_HOLD_HOVER_BASE + dT, MOTOR_IDLE, 1.0f);  
        }
        T = althold_T;                 
        break;

    case ALT_EXIT_RAMP: {
        if (want) {
            

            althold_neutral_thr = constrainf(Throttle, 0.0f, 1.0f);   
            float32_t hold_alt  = althold_feedback_alt();
            alt_hold_origin_m   = hold_alt - althold_neutral_thr * ALT_CEIL_REL;
            alt_setpoint        = hold_alt;
            alt_vz_i            = 0.0f;                                 
            PidReset(&PID_VZ);
            PID_VZ.last_input = vz_est;
            PID_VZ.integrator = constrainf(althold_T - ALT_HOLD_HOVER_BASE,
                                           -ALT_T_MIN_MARGIN, ALT_DT_CLAMP);  
            althold_prev_us   = now;
            althold_last_baro_ts = BARO_DATA.timestamp;
            althold_fault = 0;
            althold_state     = ALT_ENGAGED;
            althold_want_start_us = 0;
            T = althold_T;            
            break;
        }
        if (althold_fault && (alt_hold_switch == 0U)) {
            

            althold_fault = 0U;
            althold_exit_from_T = althold_T;
            althold_exit_start_us = now;
        }

        float32_t ramp_dur = althold_fault ? ALT_EXIT_RAMP_FAULT_S : ALT_EXIT_RAMP_PILOT_S;
        if (ramp_dur < 0.05f) ramp_dur = 0.05f;   
        float32_t a = (float32_t)(now - althold_exit_start_us) * 1e-6f / ramp_dur;
        a = constrainf(a, 0.0f, 1.0f);

        float32_t ramp_target_T = g_manual_T;
        uint8_t keep_fault_override = 0U;
        if (althold_fault) {
            

            float32_t fault_drop_margin = (ALT_EXIT_FAULT_DROP_MARGIN > 0.0f) ? ALT_EXIT_FAULT_DROP_MARGIN : 0.0f;
            float32_t fault_rise_margin = (ALT_EXIT_FAULT_RISE_MARGIN > 0.0f) ? ALT_EXIT_FAULT_RISE_MARGIN : 0.0f;
            float32_t fault_min_T = althold_exit_from_T - fault_drop_margin;
            float32_t fault_max_T = althold_exit_from_T + fault_rise_margin;
            ramp_target_T = constrainf(g_manual_T, fault_min_T, fault_max_T);
            keep_fault_override = 1U;
        }

        


        T         = constrainf(althold_exit_from_T + (ramp_target_T - althold_exit_from_T) * a,
                               MOTOR_IDLE, 1.0f);
        althold_T = T;
        if ((a >= 1.0f) && !keep_fault_override) {
            althold_state = ALT_OFF;    
        }
        break;
    }

    default:
        althold_state = ALT_OFF;
        break;
    }

    althold_engaged = (althold_state == ALT_ENGAGED) ? 1U : 0U;
}


/* Dieu khien tu dong cat canh den do cao muc tieu. */
void TAKEOFF_CONTROL(void){
    uint32_t now = TIM2->CNT;

    if(flight_state != FLIGHT_TAKING_OFF){
        takeoff_ctrl_initialized = 0U;
        takeoff_phase = TAKEOFF_PHASE_IDLE;
        takeoff_stable_start_us = 0U;
        takeoff_tilt_start_us = 0U;
        takeoff_thrust_fail_start_us = 0U;
        return;
    }

    if(!takeoff_ctrl_initialized){
        float32_t target_rel = constrainf(AUTO_TAKEOFF_TARGET_M, 0.30f, 2.00f);
        takeoff_ctrl_initialized = 1U;
        takeoff_phase = TAKEOFF_PHASE_RAMP;
        takeoff_liftoff = 0U;
        takeoff_complete = 0U;
        takeoff_fault = 0U;
        takeoff_origin_alt = alt_est;
        takeoff_target_alt = takeoff_origin_alt + target_rel;
        takeoff_alt_error = target_rel;
        takeoff_ctrl_last_baro_ts = BARO_DATA.timestamp;
        takeoff_ctrl_prev_us = now;
        takeoff_stable_start_us = 0U;
        takeoff_tilt_start_us = 0U;
        takeoff_thrust_fail_start_us = 0U;
        alt_setpoint = takeoff_target_alt;
        vz_sp = 0.0f;
        T = MOTOR_IDLE;
        PidReset(&PID_VZ);
        PID_VZ.last_input = vz_est;
        return;
    }

    if(!baro_control_ready(now)){
        takeoff_fault = 1U;
        return;
    }

    uint32_t elapsed = (uint32_t)(now - takeoff_start_us);
    if(elapsed >= AUTO_TAKEOFF_TIMEOUT_US){
        takeoff_fault = 3U;
        return;
    }

    takeoff_alt_error = takeoff_target_alt - alt_est;
    alt_setpoint = takeoff_target_alt;

    float32_t takeoff_tilt = fabsf(ARHS.RPY_DEG[0]);
    if(fabsf(ARHS.RPY_DEG[1]) > takeoff_tilt){
        takeoff_tilt = fabsf(ARHS.RPY_DEG[1]);
    }
    if(takeoff_tilt >= AUTO_TAKEOFF_ABORT_TILT_DEG){
        if(takeoff_tilt_start_us == 0U){
            takeoff_tilt_start_us = now;
        }
        else if((uint32_t)(now - takeoff_tilt_start_us) >= AUTO_TAKEOFF_TILT_HOLD_US){
            takeoff_fault = 4U;
            return;
        }
    }
    else{
        takeoff_tilt_start_us = 0U;
    }

    if(takeoff_phase == TAKEOFF_PHASE_CLIMB &&
       alt_est >= takeoff_target_alt + AUTO_TAKEOFF_OVERSHOOT_M){
        takeoff_complete = 1U;
        return;
    }

    if(takeoff_phase == TAKEOFF_PHASE_RAMP){
        float32_t climb_T = constrainf(AUTO_TAKEOFF_CLIMB_T,
                                       TAKEOFF_THROTTLE + 0.02f,
                                       ALT_HOLD_HOVER_BASE + ALT_DT_CLAMP);
        float32_t progress = (float32_t)elapsed / (float32_t)AUTO_TAKEOFF_RAMP_US;
        progress = constrainf(progress, 0.0f, 1.0f);
        T = MOTOR_IDLE + (climb_T - MOTOR_IDLE) * progress;
        vz_sp = 0.0f;

        float32_t climbed = alt_est - takeoff_origin_alt;
        float32_t liftoff_min_T = constrainf(TAKEOFF_THROTTLE + 0.03f,
                                             MOTOR_IDLE,
                                             climb_T - 0.02f);
        if(T >= liftoff_min_T && climbed >= AUTO_TAKEOFF_LIFTOFF_M){
            float32_t initial_vz_sp = constrainf(AUTO_TAKEOFF_KP_ALT * takeoff_alt_error,
                                                 AUTO_TAKEOFF_VZ_MIN,
                                                 AUTO_TAKEOFF_VZ_MAX);
            float32_t initial_p = PID_VZ.param.kP * (initial_vz_sp - vz_est);
            PidReset(&PID_VZ);
            PID_VZ.last_input = vz_est;
            PID_VZ.integrator = constrainf((T - ALT_HOLD_HOVER_BASE) - initial_p,
                                           -ALT_T_MIN_MARGIN, ALT_DT_CLAMP);
            takeoff_ctrl_last_baro_ts = BARO_DATA.timestamp;
            takeoff_ctrl_prev_us = now;
            takeoff_phase = TAKEOFF_PHASE_CLIMB;
            takeoff_liftoff = 1U;
        }
        else if(elapsed >= AUTO_TAKEOFF_LIFTOFF_TIMEOUT_US){
            takeoff_fault = 2U;
        }
        return;
    }

    if(takeoff_phase == TAKEOFF_PHASE_CLIMB &&
       BARO_DATA.timestamp != takeoff_ctrl_last_baro_ts){
        takeoff_ctrl_last_baro_ts = BARO_DATA.timestamp;
        float32_t dt = (float32_t)(now - takeoff_ctrl_prev_us) * 1e-6f;
        if(dt <= 0.0f || dt > 0.1f) dt = 0.02f;
        takeoff_ctrl_prev_us = now;

        vz_sp = constrainf(AUTO_TAKEOFF_KP_ALT * takeoff_alt_error,
                           AUTO_TAKEOFF_VZ_MIN,
                           AUTO_TAKEOFF_VZ_MAX);
        float32_t dT = Pid_Calculate(&PID_VZ, vz_sp, vz_est, dt,
                                     -ALT_T_MIN_MARGIN, ALT_DT_CLAMP);
        T = constrainf(ALT_HOLD_HOVER_BASE + dT,
                       MOTOR_IDLE, ALT_HOLD_HOVER_BASE + ALT_DT_CLAMP);
    }

    if(takeoff_phase == TAKEOFF_PHASE_CLIMB &&
       T >= AUTO_TAKEOFF_THRUST_FAIL_T &&
       vz_est <= AUTO_TAKEOFF_THRUST_FAIL_VZ){
        if(takeoff_thrust_fail_start_us == 0U){
            takeoff_thrust_fail_start_us = now;
        }
        else if((uint32_t)(now - takeoff_thrust_fail_start_us) >= AUTO_TAKEOFF_THRUST_FAIL_HOLD_US){
            takeoff_fault = 5U;
            return;
        }
    }
    else{
        takeoff_thrust_fail_start_us = 0U;
    }

    if(takeoff_phase == TAKEOFF_PHASE_CLIMB &&
       fabsf(takeoff_alt_error) <= AUTO_TAKEOFF_COMPLETE_ERR_M &&
       fabsf(vz_est) <= AUTO_TAKEOFF_COMPLETE_VZ){
        if(takeoff_stable_start_us == 0U){
            takeoff_stable_start_us = now;
        }
        else if((uint32_t)(now - takeoff_stable_start_us) >= AUTO_TAKEOFF_STABLE_US){
            takeoff_complete = 1U;
        }
    }
    else{
        takeoff_stable_start_us = 0U;
    }
}


/* Dieu khien ha canh theo toc do thang dung va phat hien cham dat. */
void LANDING_CONTROL(void){
    uint32_t now = TIM2->CNT;

    if(flight_state != FLIGHT_LANDING){
        landing_ctrl_initialized = 0U;
        landing_closed_loop = 0U;
        landing_fallback = 0U;
        landing_fallback_start_us = 0U;
        landing_ctrl_T_max = LANDING_T_MAX;
        touchdown_candidate_start_us = 0U;
        touchdown_candidate_last_baro_ts = 0U;
        touchdown_candidate_baro_samples = 0U;
        touchdown_candidate_alt_min = 0.0f;
        touchdown_candidate_alt_max = 0.0f;
        touchdown_landing_start_seen_us = 0U;
        return;
    }

    uint8_t baro_ok = baro_ref_locked
                   && (baro_init_result == MS5611_INIT_OK)
                   && (BARO_DATA.timestamp != 0U)
                   && ((uint32_t)(now - BARO_DATA.timestamp) < 200000U);
    float32_t landing_vz_cmd = constrainf(LANDING_DESCENT_VZ,
                                           LANDING_VZ_FAST_LIMIT,
                                           LANDING_VZ_SLOW_LIMIT);

    if(!landing_ctrl_initialized){
        float32_t entry_T = constrainf(landing_start_throttle, MOTOR_IDLE, 1.0f);
        landing_ctrl_initialized = 1U;
        landing_ctrl_last_baro_ts = BARO_DATA.timestamp;
        landing_ctrl_prev_us = now;
        landing_fallback_from_T = entry_T;
        landing_ctrl_T_max = constrainf((entry_T > LANDING_T_MAX) ? entry_T : LANDING_T_MAX,
                                        LANDING_THROTTLE, 1.0f);
        alt_setpoint = baro_alt_filtered;
        althold_T = entry_T;
        T = entry_T;

        if(baro_ok && entry_T >= LANDING_THROTTLE){
            float32_t out_min = LANDING_THROTTLE - ALT_HOLD_HOVER_BASE;
            float32_t out_max = landing_ctrl_T_max - ALT_HOLD_HOVER_BASE;
            PidReset(&PID_VZ);
            PID_VZ.last_input = vz_est;
            PID_VZ.integrator = constrainf(entry_T - ALT_HOLD_HOVER_BASE,
                                           out_min, out_max);
            vz_sp = landing_vz_cmd;
            landing_closed_loop = 1U;
            landing_fallback = 0U;
        }
        else{
            vz_sp = 0.0f;
            landing_closed_loop = 0U;
            landing_fallback = 1U;
            landing_fallback_start_us = now;
        }
        return;
    }

    if(landing_closed_loop && !baro_ok){
        landing_closed_loop = 0U;
        landing_fallback = 1U;
        landing_fallback_start_us = now;
        landing_fallback_from_T = constrainf(T, MOTOR_IDLE, 1.0f);
        vz_sp = 0.0f;
    }

    if(landing_fallback){
        float32_t progress = (float32_t)(uint32_t)(now - landing_fallback_start_us)
                           / (float32_t)LANDING_FALLBACK_RAMP_US;
        progress = constrainf(progress, 0.0f, 1.0f);
        float32_t target = constrainf(LANDING_THROTTLE, MOTOR_IDLE, landing_fallback_from_T);
        T = landing_fallback_from_T + (target - landing_fallback_from_T) * progress;
        althold_T = T;
        return;
    }

    vz_sp = landing_vz_cmd;
    if(BARO_DATA.timestamp != landing_ctrl_last_baro_ts){
        landing_ctrl_last_baro_ts = BARO_DATA.timestamp;

        float32_t dt = (float32_t)(now - landing_ctrl_prev_us) * 1e-6f;
        if(dt <= 0.0f || dt > 0.1f) dt = 0.02f;
        landing_ctrl_prev_us = now;

        float32_t out_min = LANDING_THROTTLE - ALT_HOLD_HOVER_BASE;
        float32_t out_max = landing_ctrl_T_max - ALT_HOLD_HOVER_BASE;
        float32_t landing_extra_ki = LANDING_KI - PID_VZ.param.kI;
        if(landing_extra_ki > 0.0f){
            float32_t vz_error = vz_sp - vz_est;
            PID_VZ.integrator += vz_error * landing_extra_ki * dt;
            PID_VZ.integrator = constrainf(PID_VZ.integrator, out_min, out_max);
        }
        float32_t dT = Pid_Calculate(&PID_VZ, vz_sp, vz_est, dt, out_min, out_max);
        T = constrainf(ALT_HOLD_HOVER_BASE + dT,
                       LANDING_THROTTLE, landing_ctrl_T_max);
        althold_T = T;
    }
}

/* Vong PID toc do goc tao lenh roll, pitch, yaw cho mixer. */
void MPC_RATE_MODE(void){
	float32_t feedback[3];
	if(TIM2->CNT - current_time_pid_rate >=(freq_pid_rate - 100)){
		prev_time_pid_rate = current_time_pid_rate;
		current_time_pid_rate = TIM2->CNT;
		dt_pid_rate = current_time_pid_rate - prev_time_pid_rate;
		feedback[0] = ICM42688P_DATA.w[0] * RAD_TO_DEG;
		feedback[1] = ICM42688P_DATA.w[1] * RAD_TO_DEG;
		feedback[2] = ICM42688P_DATA.w[2] * RAD_TO_DEG;
    u_roll  = Pid_Calculate(&PID_RATE_ROLL, angle_rate_desired[0], feedback[0], 0.0025, -1e+3f, 1e+3f) * pid_rate_scale;
    u_pitch = Pid_Calculate(&PID_RATE_PITCH, angle_rate_desired[1], feedback[1], 0.0025, -1e+3f, 1e+3f) * pid_rate_scale;
    u_yaw   = Pid_Calculate(&PID_RATE_YAW, angle_rate_desired[2], feedback[2], 0.0025, -1e+3f, 1e+3f) * pid_rate_scale;

    m[0] = -u_roll  + u_pitch + u_yaw;  
    m[1] = -u_roll  - u_pitch - u_yaw;  
    m[2] = u_roll  - u_pitch + u_yaw;   
    m[3] = u_roll  + u_pitch - u_yaw;   
	}
    float32_t mix_scale = pid_mix_scale_from_throttle();
    if(mix_scale <= 1e-6f){
        freeze_all_pid_integrators();
    }

    
    
    
    float32_t T_eff = T;
    float32_t tilt_comp = 1.0f;
    if(flight_state == FLIGHT_IN_AIR){
        float32_t cos_roll  = cosf(ARHS.RPY_DEG[0] * DEG_TO_RAD);
        float32_t cos_pitch = cosf(ARHS.RPY_DEG[1] * DEG_TO_RAD);
        float32_t tilt_cos  = cos_roll * cos_pitch;
        if(tilt_cos > 0.707f){    
            tilt_comp = 1.0f / tilt_cos;
            T_eff = T * tilt_comp;
        }
        
    }

    for (int i = 0; i < 4; i++) {
        cmd[i] = T_eff + (m[i] * mix_scale);
        cmd[i] = constrainf(cmd[i], MOTOR_IDLE, 1.0f);
    }
}
void Filter_Init(void){
    for (int i = 0; i < 3; i++) {
        
        biquadFilterInitLPF(&gyro_lpf[i], 80, GYRO_RATE_US);
        
        biquadFilterInitNotch(&gyro_notch[i], GYRO_RATE_US, 200, 70);
        
        biquadFilterInitLPF(&acc_lpf[i], 5.0, GYRO_RATE_US);
        
        biquadFilterInitNotch(&acc_notch[i], GYRO_RATE_US, 200, 70);
    }
    

    pt1FilterInit(&baro_alt_lpf, 3.0f, 0.02f);
    
}
/* Xu ly mau IMU: hieu chuan, loc, doi truc toa do va cap nhat uoc luong tu the. */
void IMU_PROCESS(void){
	  if(ICM42688P_DATA_RDY){
		  ICM42688P_DATA_RDY = 0;
		IMU_PROCESS_CALIBRATE(&IMU_CALIBRATE,acc_adc_imu_filtered,gyro_adc_imu_filtered);
		if(IMU_CALIBRATE.status == CALIBRATE_OKAY){

		    IMU_ApplyCrossAxis(acc_adc_imu_filtered, IMU_CALIBRATE.M, IMU_CALIBRATE.b, acc_adc_imu_filtered);
		}
    	ICM42688P_DATA.w[0] = gyro_adc_imu_filtered[0];
    	ICM42688P_DATA.w[1] = -gyro_adc_imu_filtered[1];
    	ICM42688P_DATA.w[2] = -gyro_adc_imu_filtered[2];
    	ICM42688P_DATA.acc[0] = acc_adc_imu_filtered[0] * 9.81;
    	ICM42688P_DATA.acc[1] = -acc_adc_imu_filtered[1] * 9.81;
    	ICM42688P_DATA.acc[2] = -acc_adc_imu_filtered[2] * 9.81;
		  ARHS_Predict(&ARHS, &ICM42688P_DATA);   
		  


		  {
	      float32_t dt_imu = ICM42688P_DATA.dt;
	      if (dt_imu > 1e-5f && dt_imu < 0.02f) {
	          az_up = -ARHS.acc_linear_fixed_frame[2];
	          uint8_t baro_obs_fresh = (baro_init_result == MS5611_INIT_OK)
	                                && baro_ref_locked
	                                && (BARO_DATA.timestamp != 0U)
	                                && ((uint32_t)(TIM2->CNT - BARO_DATA.timestamp) < 200000U);
	          if (baro_obs_fresh) {
		              if (!alt_obs_init) {
		                  alt_est = baro_alt_filtered;   
		                  vz_est  = 0.0f;
		                  alt_obs_init = 1;
		              }
		              float32_t err = baro_alt_filtered - alt_est;   
		              

		              alt_est += (vz_est + ALT_OBS_W1 * err) * dt_imu;
		              vz_est  += (az_up - accel_bias_z_est + ALT_OBS_W2 * err) * dt_imu;
		              accel_bias_z_est += (-ALT_OBS_W3 * err) * dt_imu;
		              accel_bias_z_est = constrainf(accel_bias_z_est,
		                                            -ALT_OBS_BIAS_CLAMP, ALT_OBS_BIAS_CLAMP);
		          } else {
		              alt_obs_init = 0;                 
		              vz_est = 0.0f;
		              
		          }
		      }
		  }


	  }
}


static inline void LED_RED_set(uint8_t on){
    LED_RED_GPIO_Port->BSRR = on ? LED_RED_Pin : ((uint32_t)LED_RED_Pin << 16);
}
static inline void LED_BLUE_set(uint8_t on){
    LED_BLUE_GPIO_Port->BSRR = on ? LED_BLUE_Pin : ((uint32_t)LED_BLUE_Pin << 16);
}
static inline void LED_YELLOW_set(uint8_t on){
    LED_YELLOW_GPIO_Port->BSRR = on ? LED_YELLOW_Pin : ((uint32_t)LED_YELLOW_Pin << 16);
}


/* Bao trang thai he thong bang LED do/xanh/vang. */
void update_led_status(void){
    uint32_t now = TIM2->CNT;

    uint8_t rc_lost       = is_rc_lost();
    uint8_t calib_running = (IMU_CALIBRATE.sample_gyro == Running);
    uint8_t calib_ok      = is_gyro_ready();
    uint8_t throttle_safe = (CH[2] < ARM_THROTTLE_MAX);

    uint8_t blink_fast = (now / 100000U) & 1U;  
    uint8_t blink_med  = (now / 250000U) & 1U;  
    uint8_t blink_slow = (now / 500000U) & 1U;  

    uint8_t red_on  = 0;
    uint8_t blue_on = 0;

    if(calib_running){
        blue_on = blink_fast;
    }
    else if(rc_lost){
        red_on = blink_fast;
    }
    else{
        switch(flight_state){
        case FLIGHT_DISARM:
            if(calib_ok && throttle_safe){
                red_on = 1;
                blue_on = 1;
            }
            else{
                blue_on = blink_slow;
            }
            break;

        case FLIGHT_ARMED_IDLE:
            red_on = 1;
            blue_on = blink_med;
            break;

        case FLIGHT_TAKING_OFF:
            red_on = 1;
            blue_on = blink_fast;
            break;

        case FLIGHT_IN_AIR:
            red_on = 1;
            blue_on = 1;
            break;

        case FLIGHT_LANDING:
            red_on = landing_timeout_exceeded ? blink_fast : blink_med;
            blue_on = 1;
            break;
        }
    }

    


    if(!calib_running && !rc_lost && flight_state == FLIGHT_IN_AIR){
        if(althold_fault && !althold_fault_prev){
            althold_fault_seen_us = now ? now : 1U;           
        }
        if(althold_fault_seen_us != 0U &&
           (uint32_t)(now - althold_fault_seen_us) < 3000000U){
            blue_on = blink_fast;                             
        }
        else{
            althold_fault_seen_us = 0U;
            

            if(althold_engaged){
                blue_on = blink_slow;
            }
        }
    }
    else{
        althold_fault_seen_us = 0U;
    }
    althold_fault_prev = althold_fault;

    LED_RED_set(red_on);
    LED_BLUE_set(blue_on);
    LED_YELLOW_set(0);
}


static void init_arhs_from_accel(void){
    float32_t ax = ICM42688P_DATA.acc[0];
    float32_t ay = ICM42688P_DATA.acc[1];
    float32_t az = ICM42688P_DATA.acc[2];
    float32_t roll  = atan2f(-ay, -az);
    float32_t pitch = atan2f(ax, sqrtf(ay*ay + az*az));
    float32_t ypr[3] = {0.0f, pitch, roll};
    EQ_ZYX(ypr, ARHS.q);
    ARHS.fusion_count = 0;
    ARHS.B_madgwick = ARHS.B_ARHS_Default;
}


void boot_gyro_auto_calib(void){
    static uint8_t fired = 0;
    static IMU_Sample_t prev_gyro_state = Stop;

    if(!fired && TIM2->CNT > BOOT_SETTLE_US && IMU_CALIBRATE.sample_gyro == Stop){
        IMU_CALIBRATE.sample_gyro = Start;
        fired = 1;
    }

    if(prev_gyro_state == Running && IMU_CALIBRATE.sample_gyro == Stop){
        init_arhs_from_accel();
        boot_gyro_calib_done = 1;
    }
    prev_gyro_state = IMU_CALIBRATE.sample_gyro;
}


void check_gyro_recalib_gesture(void){
    static uint32_t at_position_start_us = 0;
    static uint8_t  gesture_armed = 0;

    if(ARM_Status == ARM || IMU_CALIBRATE.sample_gyro == Running){
        at_position_start_us = 0;
        gesture_armed = 0;
        return;
    }

    uint8_t at_position = (CH[2] < GESTURE_THROTTLE_MAX) &&
                          (CH[3] < GESTURE_YAW_MAX) &&
                          (CH[1] < GESTURE_PITCH_MAX) &&
                          (CH[0] > GESTURE_ROLL_MIN);

    if(at_position){
        if(at_position_start_us == 0){
            at_position_start_us = TIM2->CNT;
        }
        else if(!gesture_armed &&
                (uint32_t)(TIM2->CNT - at_position_start_us) > GESTURE_DEBOUNCE_US){
            gesture_armed = 1;
        }
    }
    else{
        if(gesture_armed){
            IMU_CALIBRATE.sample_gyro = Start;
        }
        at_position_start_us = 0;
        gesture_armed = 0;
    }
}


void Error_Handler(void)
{
  
  __disable_irq();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIOA->MODER = (GPIOA->MODER & ~((3U << (0U * 2U)) | (3U << (1U * 2U)))) |
                 (1U << (0U * 2U)) | (1U << (1U * 2U));
  GPIOA->OTYPER &= ~(LED_BLUE_Pin | LED_RED_Pin);
  GPIOA->BSRR = (uint32_t)LED_BLUE_Pin << 16U;
  while (1)
  {
    GPIOA->ODR ^= LED_RED_Pin;
    for (volatile uint32_t delay = 0; delay < 1200000U; delay++) {
      __NOP();
    }
  }
  
}

#ifdef  USE_FULL_ASSERT


void assert_failed(uint8_t *file, uint32_t line)
{
  
  

  
}
#endif
