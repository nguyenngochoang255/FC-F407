/*
 * Bao cao: Prototype cac ham phuc vu ngat.
 */

#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif


void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);
void DMA1_Stream0_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void SPI1_IRQHandler(void);
void USART1_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);


void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);


#ifdef __cplusplus
}
#endif

#endif 
