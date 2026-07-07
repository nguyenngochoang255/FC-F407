/*
 * Bao cao: Khai bao ham cau hinh clock, GPIO, timer va PWM bang thanh ghi STM32.
 */

#ifndef __REGISTER_BOARD_CONFIG_H
#define __REGISTER_BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx.h"


void Reg_GPIO_Init(void);


void Reg_TIM2_Init(void);


void Reg_SystemClock_Config(void);


void Reg_TIM_PWM_Init(void);


static inline void Reg_Motor1_us(uint16_t us){ TIM3->CCR3 = us; }
static inline void Reg_Motor2_us(uint16_t us){ TIM3->CCR4 = us; }
static inline void Reg_Motor3_us(uint16_t us){ TIM4->CCR1 = us; }
static inline void Reg_Motor4_us(uint16_t us){ TIM4->CCR2 = us; }


static inline void Reg_LED_BLUE(uint8_t on){
    GPIOA->BSRR = on ? (1U << 0) : (1U << (0 + 16));
}
static inline void Reg_LED_RED(uint8_t on){
    GPIOA->BSRR = on ? (1U << 1) : (1U << (1 + 16));
}
static inline void Reg_LED_YELLOW(uint8_t on){
    GPIOA->BSRR = on ? (1U << 2) : (1U << (2 + 16));
}


static inline void Reg_CS_IMU_low(void){  GPIOA->BSRR = (1U << (4 + 16)); }
static inline void Reg_CS_IMU_high(void){ GPIOA->BSRR = (1U << 4);        }


static inline uint32_t Reg_micros(void){
    return TIM2->CNT;
}

#ifdef __cplusplus
}
#endif

#endif
