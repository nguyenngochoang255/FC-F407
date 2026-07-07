/*
 * Bao cao: Kieu du lieu va API cua bo dieu khien PID.
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#include "main.h"
#include "arm_math.h"
#include "filter.h"
#include "math.h"
#include <stdbool.h>

typedef enum {
  ARM,
  NOT_ARM
} ARM_Status_t;

typedef struct {
    float kP;
    float kI;
    float kD;
    float kT;   
    float kFF;  
} pidControllerParam_t;

typedef struct {
    bool reset;
    pidControllerParam_t param;
    pt1Filter_t error_filter_state;
    pt1Filter_t dterm_filter_state;     
    float errorLpfHz;
    float dTermLpfHz;                   
    float integrator;                   
    float last_input;                   
    float deadband_error;
    float deadband_integral;
    float integral;                     
    float proportional;                 
    float derivative;                   
    float feedForward;                  
    float output_constrained;           
} pidController_t;
float Pid_Calculate(pidController_t *pid,float setpoint,float measurement,float dt,float outMin,float outMax);
void PidInit(pidController_t *pid);
void PidReset(pidController_t *pid);
void PidFreezeIntegrator(pidController_t *pid);

#endif 
