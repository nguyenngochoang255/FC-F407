/*
 * Bao cao: Khai bao chan GPIO va cac dinh nghia dung chung cho chuong trinh chinh.
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"

#include "stm32f4xx_ll_exti.h"


void Error_Handler(void);


#define LED_BLUE_Pin GPIO_PIN_0
#define LED_BLUE_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOA
#define LED_YELLOW_Pin GPIO_PIN_2
#define LED_YELLOW_GPIO_Port GPIOA
#define CS_ICM42688P_Pin GPIO_PIN_4
#define CS_ICM42688P_GPIO_Port GPIOA
#define CS_FLASH_DISABLE_Pin GPIO_PIN_15
#define CS_FLASH_DISABLE_GPIO_Port GPIOA
#define INT_ICM42688P_Pin GPIO_PIN_0
#define INT_ICM42688P_GPIO_Port GPIOE
#define INT_ICM42688P_EXTI_IRQn EXTI0_IRQn


#ifdef __cplusplus
}
#endif

#endif 
