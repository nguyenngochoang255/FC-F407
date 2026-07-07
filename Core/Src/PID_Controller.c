/*
 * Bao cao: Bo dieu khien PID dung cho vong goc, vong toc do goc va vong toc do thang dung.
 */

#include"PID_Controller.h"

float Pid_Calculate(pidController_t *pid,float setpoint,float measurement,float dt,float outMin,float outMax){
    float newProportional, newDerivative, newFeedForward;
    float error = 0.0f;
    if (pid->errorLpfHz > 0.0f) {
        error = pt1FilterApply4(&pid->error_filter_state, setpoint - measurement, pid->errorLpfHz, dt);
    } else {
        error = setpoint - measurement;
    }
    
    newProportional = error * pid->param.kP;
    
    if (pid->reset) {
        pid->last_input = measurement;
        pid->reset = false;
    }
        
        if (dt > 1e-6f) {
            newDerivative = -(measurement - pid->last_input) / dt;
        } else {
            newDerivative = 0.0f;
        }
        pid->last_input = measurement;

    if (pid->dTermLpfHz > 0.0f) {
        newDerivative = pid->param.kD * pt1FilterApply4(&pid->dterm_filter_state, newDerivative, pid->dTermLpfHz, dt);
    } else {
        newDerivative = pid->param.kD * newDerivative;
    }


    


    newFeedForward = setpoint * pid->param.kFF;

    const float outVal = newProportional + pid->integrator + newDerivative + newFeedForward; 

    const float outValConstrained = constrainf(outVal, outMin, outMax); 
    float backCalc = outValConstrained - outVal;
    if (SIGN(backCalc) == SIGN(pid->integrator)) {
        backCalc = 0.0f;
    }

    pid->proportional = newProportional;
    pid->integral = pid->integrator;
    pid->derivative = newDerivative;
    pid->feedForward = newFeedForward;
    pid->output_constrained = outValConstrained;

    
      const float newIntegrator = pid->integrator + (error * pid->param.kI * dt) + (backCalc * pid->param.kT * dt);
      pid->integrator = newIntegrator;
      pid->integrator = constrainf(pid->integrator, outMin, outMax);
    return outValConstrained;
}


void PidFreezeIntegrator(pidController_t *pid){
    
    pid->integrator = 0.0f;
    pid->integral   = 0.0f;
}

void PidReset(pidController_t *pid){
    pid->reset = true;
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->integrator = 0.0f;
    pid->last_input = 0.0f;
    pid->feedForward = 0.0f;
    pt1FilterReset(&pid->dterm_filter_state, 0.0f);
    pt1FilterReset(&pid->error_filter_state, 0.0f);
    pid->output_constrained = 0.0f;
}

void PidInit(pidController_t *pid){
    float _kP = pid->param.kP;
    float _kI = pid->param.kI;
    float _kD = pid->param.kD;
    if (_kI > 1e-6f && _kP > 1e-6f) {
        float Ti = _kP / _kI;
        float Td = _kD/ _kP;
        pid->param.kT = 2.0f / (Ti + Td);
    }
    else if (_kI > 1e-6f) {
        pid->param.kI = _kI;
        pid->param.kT = 0.0f;
    }
    else {
        pid->param.kI = 0.0;
        pid->param.kT = 0.0;
    }
    PidReset(pid);
}
