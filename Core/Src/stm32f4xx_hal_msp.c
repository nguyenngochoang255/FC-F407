/*
 * Bao cao: Khoi tao MSP cho cac ngoai vi HAL duoc su dung trong project.
 */

#include "main.h"


void HAL_MspInit(void)
{

  

  

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  

  

  
}


