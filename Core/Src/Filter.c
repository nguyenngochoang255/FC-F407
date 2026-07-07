/*
 * Bao cao: Cac bo loc so dung cho tin hieu IMU, barometer va lenh dieu khien.
 */

#include"Filter.h"

float sin_approx(float x){
    return sinf(x);
}

float cos_approx(float x){
    return cosf(x);
}

int constrain(int x, int min, int max){
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

float constrainf(float x, float min, float max){
    if (x < min) return min;
    if (x > max) return max;
    return x;
}


static float pt1ComputeRC(const float f_cut){
    return 1.0f / (2.0f * M_PIf * f_cut);
}


void pt1FilterInitRC(pt1Filter_t *filter, float tau, float dT){
    filter->state = 0.0f;
    filter->RC = tau;
    filter->dT = dT;
    filter->alpha = filter->dT / (filter->RC + filter->dT);
}

void pt1FilterInit(pt1Filter_t *filter, float f_cut, float dT){
    pt1FilterInitRC(filter, pt1ComputeRC(f_cut), dT);
}

void pt1FilterSetTimeConstant(pt1Filter_t *filter, float tau){
    filter->RC = tau;
}

float pt1FilterGetLastOutput(pt1Filter_t *filter){
    return filter->state;
}

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float f_cut){
    filter->RC = pt1ComputeRC(f_cut);
    filter->alpha = filter->dT / (filter->RC + filter->dT);
}

float pt1FilterApply(pt1Filter_t *filter, float input){
    filter->state = filter->state + filter->alpha * (input - filter->state);
    return filter->state;
}

float pt1FilterApply3(pt1Filter_t *filter, float input, float dT){
    filter->dT = dT;
    filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);
    return filter->state;
}

float pt1FilterApply4(pt1Filter_t *filter, float input, float f_cut, float dT){
    
    if (!filter->RC) {
        filter->RC = pt1ComputeRC(f_cut);
    }

    filter->dT = dT;    
    filter->alpha = filter->dT / (filter->RC + filter->dT);
    filter->state = filter->state + filter->alpha * (input - filter->state);
    return filter->state;
}

void pt1FilterReset(pt1Filter_t *filter, float input){
    filter->state = input;
}


float pt2FilterGain(float f_cut, float dT){
    const float order = 2.0f;
    const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f / order) - 1);
    float RC = 1 / (2 * orderCutoffCorrection * M_PIf * f_cut);
    
    
    return dT / (RC + dT);
}

void pt2FilterInit(pt2Filter_t *filter, float k){
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->k = k;
}

void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k)
{
    filter->k = k;
}

 float pt2FilterApply(pt2Filter_t *filter, float input){
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state = filter->state + filter->k * (filter->state1 - filter->state);
    return filter->state;
}


float pt3FilterGain(float f_cut, float dT){
    const float order = 3.0f;
    const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f / order) - 1);
    float RC = 1 / (2 * orderCutoffCorrection * M_PIf * f_cut);
    
    
    return dT / (RC + dT);
}

void pt3FilterInit(pt3Filter_t *filter, float k){
    filter->state = 0.0f;
    filter->state1 = 0.0f;
    filter->state2 = 0.0f;
    filter->k = k;
}

void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k){
    filter->k = k;
}

 float pt3FilterApply(pt3Filter_t *filter, float input){
    filter->state1 = filter->state1 + filter->k * (input - filter->state1);
    filter->state2 = filter->state2 + filter->k * (filter->state1 - filter->state2);
    filter->state = filter->state + filter->k * (filter->state2 - filter->state);
    return filter->state;
}


void rateLimitFilterInit(rateLimitFilter_t *filter){
    filter->state = 0;
}

float rateLimitFilterApply4(rateLimitFilter_t *filter, float input, float rate_limit, float dT){
    if (rate_limit > 0) {
        const float rateLimitPerSample = rate_limit * dT;
        filter->state = constrainf(input, filter->state - rateLimitPerSample, filter->state + rateLimitPerSample);
    }
    else {
        filter->state = input;
    }

    return filter->state;
}

float filterGetNotchQ(float centerFrequencyHz, float cutoffFrequencyHz){
    return centerFrequencyHz * cutoffFrequencyHz / (centerFrequencyHz * centerFrequencyHz - cutoffFrequencyHz * cutoffFrequencyHz);
}

void biquadFilterInitNotch(biquadFilter_t *filter, uint32_t samplingIntervalUs, uint16_t filterFreq, uint16_t cutoffHz){
    float Q = filterGetNotchQ(filterFreq, cutoffHz);
    biquadFilterInit(filter, filterFreq, samplingIntervalUs, Q, FILTER_NOTCH);
}


void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs){
    biquadFilterInit(filter, filterFreq, samplingIntervalUs, BIQUAD_Q, FILTER_LPF);
}


static void biquadFilterSetupPassthrough(biquadFilter_t *filter){
    
    filter->b0 = 1.0f;
    filter->b1 = 0.0f;
    filter->b2 = 0.0f;
    filter->a1 = 0.0f;
    filter->a2 = 0.0f;
}

void biquadFilterInit(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs, float Q, biquadFilterType_e filterType){
    
    if (filterFreq < (1000000 / samplingIntervalUs / 2)) {
        
        const float sampleRate = 1.0f / ((float)samplingIntervalUs * 0.000001f);
        const float omega = 2.0f * M_PIf * ((float)filterFreq) / sampleRate;
        const float sn = sin_approx(omega);
        const float cs = cos_approx(omega);
        const float alpha = sn / (2 * Q);

        float b0, b1, b2;
        switch (filterType) {
            case FILTER_LPF:
                b0 = (1 - cs) / 2;
                b1 = 1 - cs;
                b2 = (1 - cs) / 2;
                break;
            case FILTER_NOTCH:
                b0 = 1;
                b1 = -2 * cs;
                b2 = 1;
                break;
            default:
                biquadFilterSetupPassthrough(filter);
                return;
        }
        const float a0 =  1 + alpha;
        const float a1 = -2 * cs;
        const float a2 =  1 - alpha;

        
        filter->b0 = b0 / a0;
        filter->b1 = b1 / a0;
        filter->b2 = b2 / a0;
        filter->a1 = a1 / a0;
        filter->a2 = a2 / a0;
    } else {
        biquadFilterSetupPassthrough(filter);
    }

    
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

 float biquadFilterApplyDF1(biquadFilter_t *filter, float input){
    
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

    
    filter->x2 = filter->x1;
    filter->x1 = input;

    
    filter->y2 = filter->y1;
    filter->y1 = result;

    return result;
}


float biquadFilterApply(biquadFilter_t *filter, float input){
    const float result = filter->b0 * input + filter->x1;
    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;
    return result;
}

float biquadFilterReset(biquadFilter_t *filter, float value){
    filter->x1 = value - (value * filter->b0);
    filter->x2 = (filter->b2 - filter->a2) * value;
    return value;
}

 void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType){
    
    float x1 = filter->x1;
    float x2 = filter->x2;
    float y1 = filter->y1;
    float y2 = filter->y2;

    biquadFilterInit(filter, filterFreq, refreshRate, Q, filterType);

    
    filter->x1 = x1;
    filter->x2 = x2;
    filter->y1 = y1;
    filter->y2 = y2;
}

