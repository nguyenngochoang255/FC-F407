/*
 * Bao cao: Trinh phuc vu ngat: doc CRSF, bat data-ready cua IMU va xu ly loi he thong.
 * Cac handler DMA/I2C/SPI ben duoi duoc giu nhu fallback generated; duong chinh
 * cua ban thanh ghi dung EXTI0, USART1 va polling SPI/I2C.
 */

#include "main.h"
#include "stm32f4xx_it.h"


/* Cac gia tri nay thuoc giao thuc CRSF, khong phai dia chi thanh ghi STM32. */
#define CRSF_MAX_FRAME 64
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_LEN_RC_CHANNELS_PACKED       24U


uint8_t datarx;
uint8_t crsf_buf[CRSF_MAX_FRAME];
uint8_t crsf_cnt = 0;
uint8_t crsf_len = 0;
uint32_t crsf_last_time = 0;

volatile uint16_t crsf_channel[16];
volatile uint8_t crsf_frame_done = 0;
volatile uint32_t crsf_frame_count = 0;
volatile uint32_t crsf_last_frame_us = 0;


  static uint8_t crsf_crc8(uint8_t crc, uint8_t data)
    {
        crc ^= data;
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0xD5;
            else
                crc <<= 1;
        }
        return crc;
    }

  static uint8_t crsf_calc_crc(uint8_t *buf, uint8_t len)
    {
        uint8_t crc = 0;

        
        crc = crsf_crc8(crc, buf[2]);

        for (uint8_t i = 0; i < (uint8_t)(len - 2U); i++)
        {
            crc = crsf_crc8(crc, buf[3 + i]);
        }

        return crc;
    }

  static void crsf_reset_parser(void)
    {
        crsf_cnt = 0;
        crsf_len = 0;
    }

void Decode_RxElrs(USART_TypeDef * uart){
    uint8_t uart_error = 0;

    if (LL_USART_IsActiveFlag_ORE(uart)) {
        LL_USART_ClearFlag_ORE(uart);
        uart_error = 1U;
    }
    if (LL_USART_IsActiveFlag_FE(uart)) {
        LL_USART_ClearFlag_FE(uart);
        uart_error = 1U;
    }
    if (LL_USART_IsActiveFlag_NE(uart)) {
        LL_USART_ClearFlag_NE(uart);
        uart_error = 1U;
    }
    if (LL_USART_IsActiveFlag_PE(uart)) {
        LL_USART_ClearFlag_PE(uart);
        uart_error = 1U;
    }
    if (uart_error) {
        crsf_reset_parser();
        return;
    }

    if (!LL_USART_IsActiveFlag_RXNE(uart)) {
        return;
    }

    datarx = LL_USART_ReceiveData8(uart);
    if ((uint32_t)(TIM2->CNT - crsf_last_time) > 2000U) {
        crsf_reset_parser();
    }
    crsf_last_time = TIM2->CNT;

    switch (crsf_cnt) {
    case 0:
        if (datarx != CRSF_ADDRESS_FLIGHT_CONTROLLER) {
            crsf_reset_parser();
            break;
        }
        crsf_buf[0] = datarx;
        crsf_cnt = 1;
        break;

    case 1:
        crsf_len = datarx;
        if (crsf_len < 2U || crsf_len > (CRSF_MAX_FRAME - 2U)) {
            crsf_reset_parser();
            break;
        }
        crsf_buf[1] = datarx;
        crsf_cnt = 2;
        break;

    default:
        if (crsf_cnt >= CRSF_MAX_FRAME) {
            crsf_reset_parser();
            break;
        }
        crsf_buf[crsf_cnt++] = datarx;
        if (crsf_cnt < (uint8_t)(crsf_len + 2U)) {
            break;
        }

        uint8_t crc_rx = crsf_buf[crsf_cnt - 1];
        uint8_t crc_calc = crsf_calc_crc(crsf_buf, crsf_len);
        if (crc_rx == crc_calc &&
            crsf_buf[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED &&
            crsf_len == CRSF_LEN_RC_CHANNELS_PACKED) {
            uint8_t *p = &crsf_buf[3];

            /* CRSF packed 16 kenh, moi kenh 11 bit nen mask 0x07FF. */
            crsf_channel[0]  = ((p[0] | p[1]<<8) & 0x07FF);
            crsf_channel[1]  = ((p[1]>>3 | p[2]<<5) & 0x07FF);
            crsf_channel[2]  = ((p[2]>>6 | p[3]<<2 | p[4]<<10) & 0x07FF);
            crsf_channel[3]  = ((p[4]>>1 | p[5]<<7) & 0x07FF);
            crsf_channel[4]  = ((p[5]>>4 | p[6]<<4) & 0x07FF);
            crsf_channel[5]  = ((p[6]>>7 | p[7]<<1 | p[8]<<9) & 0x07FF);
            crsf_channel[6]  = ((p[8]>>2 | p[9]<<6) & 0x07FF);
            crsf_channel[7]  = ((p[9]>>5 | p[10]<<3) & 0x07FF);
            crsf_channel[8]  = ((p[11] | p[12]<<8) & 0x07FF);
            crsf_channel[9]  = ((p[12]>>3 | p[13]<<5) & 0x07FF);
            crsf_channel[10] = ((p[13]>>6 | p[14]<<2 | p[15]<<10) & 0x07FF);
            crsf_channel[11] = ((p[15]>>1 | p[16]<<7) & 0x07FF);
            crsf_channel[12] = ((p[16]>>4 | p[17]<<4) & 0x07FF);
            crsf_channel[13] = ((p[17]>>7 | p[18]<<1 | p[19]<<9) & 0x07FF);
            crsf_channel[14] = ((p[19]>>2 | p[20]<<6) & 0x07FF);
            crsf_channel[15] = ((p[20]>>5 | p[21]<<3) & 0x07FF);

            crsf_frame_done = 1;
            crsf_last_frame_us = TIM2->CNT;
            crsf_frame_count++;
        }
        crsf_reset_parser();
        break;
    }
}
static void Fault_Loop(uint32_t type)
{
    (void)type;
    __disable_irq();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIOA->MODER = (GPIOA->MODER & ~((3U << (0U * 2U)) | (3U << (1U * 2U)))) |
                   (1U << (0U * 2U)) | (1U << (1U * 2U));
    GPIOA->OTYPER &= ~(LED_BLUE_Pin | LED_RED_Pin);
    GPIOA->BSRR = (uint32_t)LED_BLUE_Pin << 16U;

    while (1) {
        GPIOA->ODR ^= LED_RED_Pin;
        for (volatile uint32_t delay = 0; delay < 1200000U; delay++) {
            __NOP();
        }
    }
}


extern DMA_HandleTypeDef hdma_i2c1_rx;
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi1;

extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;


void NMI_Handler(void)
{
  

  
  HAL_RCC_NMI_IRQHandler();
  
   while (1)
  {
  }
  
}


void HardFault_Handler(void)
{
  
  Fault_Loop(1U);
  
  while (1)
  {
    
    
  }
}


void MemManage_Handler(void)
{
  
  Fault_Loop(2U);
  
  while (1)
  {
    
    
  }
}


void BusFault_Handler(void)
{
  
  Fault_Loop(3U);
  
  while (1)
  {
    
    
  }
}


void UsageFault_Handler(void)
{
  
  Fault_Loop(4U);
  
  while (1)
  {
    
    
  }
}


void SVC_Handler(void)
{
  

  
  

  
}


void DebugMon_Handler(void)
{
  

  
  

  
}


void PendSV_Handler(void)
{
  

  
  

  
}


void SysTick_Handler(void)
{
  

  
  HAL_IncTick();
  

  
}


void EXTI0_IRQHandler(void)
{
  /* Ban thanh ghi: EXTI line 0 co pending thi ghi 1 vao PR de xoa co ngat. */
  if ((EXTI->PR & (1U << 0)) != 0U) {
    EXTI->PR = (1U << 0);
    IMU_DataReady_IRQHandler();
  }
}


void DMA1_Stream0_IRQHandler(void)
{
  

  
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  

  
}


void I2C1_EV_IRQHandler(void)
{
  

  
  HAL_I2C_EV_IRQHandler(&hi2c1);
  

  
}


void I2C1_ER_IRQHandler(void)
{
  

  
  HAL_I2C_ER_IRQHandler(&hi2c1);
  

  
}


void SPI1_IRQHandler(void)
{
  

  
  HAL_SPI_IRQHandler(&hspi1);
  

  
}


void USART1_IRQHandler(void)
{
  
  Decode_RxElrs(USART1);

  

  
}


void DMA2_Stream0_IRQHandler(void)
{
  

  
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  

  
}


void DMA2_Stream5_IRQHandler(void)
{
  

  
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  

  
}


void I2C2_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c2);
}

void I2C2_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c2);
}

void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
}


