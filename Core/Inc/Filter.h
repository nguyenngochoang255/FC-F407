/*
 * Bao cao: Kieu du lieu va prototype cho cac bo loc so.
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_
#include "main.h"
#include <stdint.h>
#include <math.h>
#include "arm_math.h"

#ifndef M_PIf
#define M_PIf 3.14159265358979323846f
#endif
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

float sin_approx(float x);
float cos_approx(float x);
int   constrain(int x, int min, int max);
float constrainf(float x, float min, float max);
#ifndef SIGN
#define SIGN(a) ((a >= 0) ? 1 : -1)
#endif


typedef struct rateLimitFilter_s {
    float state;
} rateLimitFilter_t;

typedef struct pt1Filter_s {
    float state;
    float RC;
    float dT;
    float alpha;
} pt1Filter_t;
typedef struct pt2Filter_s {
    float state;
    float state1;
    float k;
} pt2Filter_t;
typedef struct pt3Filter_s {
    float state;
    float state1;
    float state2;
    float k;
} pt3Filter_t;


typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;


typedef enum {
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;

typedef struct firFilter_s {
    float *buf;
    const float *coeffs;
    uint8_t bufLength;
    uint8_t coeffsLength;
} firFilter_t;


typedef float (*filterApplyFnPtr)(void *filter, float input);
typedef float (*filterApply4FnPtr)(void *filter, float input, float f_cut, float dt);

#define BIQUAD_BANDWIDTH 1.9f     
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     

void pt1FilterInit(pt1Filter_t *filter, float f_cut, float dT);
void pt1FilterInitRC(pt1Filter_t *filter, float tau, float dT);
void pt1FilterSetTimeConstant(pt1Filter_t *filter, float tau);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float f_cut);
float pt1FilterGetLastOutput(pt1Filter_t *filter);
float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply3(pt1Filter_t *filter, float input, float dT);
float pt1FilterApply4(pt1Filter_t *filter, float input, float f_cut, float dt);
void pt1FilterReset(pt1Filter_t *filter, float input);


float pt2FilterGain(float f_cut, float dT);
void pt2FilterInit(pt2Filter_t *filter, float k);
void pt2FilterUpdateCutoff(pt2Filter_t *filter, float k);
float pt2FilterApply(pt2Filter_t *filter, float input);


float pt3FilterGain(float f_cut, float dT);
void pt3FilterInit(pt3Filter_t *filter, float k);
void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k);
float pt3FilterApply(pt3Filter_t *filter, float input);

void rateLimitFilterInit(rateLimitFilter_t *filter);
float rateLimitFilterApply4(rateLimitFilter_t *filter, float input, float rate_limit, float dT);

void biquadFilterInitNotch(biquadFilter_t *filter, uint32_t samplingIntervalUs, uint16_t filterFreq, uint16_t cutoffHz);
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs);
void biquadFilterInit(biquadFilter_t *filter, uint16_t filterFreq, uint32_t samplingIntervalUs, float Q, biquadFilterType_e filterType);
float biquadFilterApply(biquadFilter_t *filter, float sample);
float biquadFilterReset(biquadFilter_t *filter, float value);
float biquadFilterApplyDF1(biquadFilter_t *filter, float input);
float filterGetNotchQ(float centerFrequencyHz, float cutoffFrequencyHz);
void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);

#endif 
