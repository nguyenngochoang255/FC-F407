/*
 * Bao cao: Cau hinh DMA cho SPI/I2C phuc vu doc cam bien toc do cao.
 */

#include "dma.h"


void MX_DMA_Init(void)
{

  
  /* DMA2 dung cho SPI1, DMA1 dung cho I2C1/I2C2 tren STM32F407. */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  
  
  /* DMA1 Stream0: I2C1_RX; Stream2: I2C2_RX. */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  
  /* DMA2 Stream0/Stream5: SPI1 RX/TX de doc IMU bang SPI DMA. */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  

  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}


