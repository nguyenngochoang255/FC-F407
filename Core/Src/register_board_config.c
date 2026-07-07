/*
 * Bao cao: Cau hinh clock, GPIO, timer va PWM bang thanh ghi STM32.
 */

#include "register_board_config.h"


void Reg_GPIO_Init(void)
{

    /* RCC->AHB1ENR: bat clock cho cac port GPIO truoc khi ghi MODER/AFR/ODR. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN
                  | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOHEN;
    (void)RCC->AHB1ENR;

    /* BSRR bit 0..15 de set, bit 16..31 de reset; PA0..PA2 tat LED, PA4/PA15 keo len cao. */
    GPIOA->BSRR = (1U << (0 + 16)) | (1U << (1 + 16)) | (1U << (2 + 16));
    GPIOA->BSRR = (1U << 4) | (1U << 15);

    /* MODER moi chan chiem 2 bit: 00 input, 01 output, 10 alternate, 11 analog. */
    GPIOA->MODER &= ~((3U << (0*2)) | (3U << (1*2)) | (3U << (2*2))
                    | (3U << (4*2)) | (3U << (15*2)));
    GPIOA->MODER |=  ((1U << (0*2)) | (1U << (1*2)) | (1U << (2*2))
                    | (1U << (4*2)) | (1U << (15*2)));

    /* OTYPER = 0 la push-pull, phu hop LED va chan chip-select SPI. */
    GPIOA->OTYPER &= ~((1U << 0) | (1U << 1) | (1U << 2) | (1U << 4) | (1U << 15));

    /* PA4 la chip-select IMU nen dat very-high speed de canh xung gon hon. */
    GPIOA->OSPEEDR &= ~((3U << (0*2)) | (3U << (1*2)) | (3U << (2*2))
                      | (3U << (4*2)) | (3U << (15*2)));
    GPIOA->OSPEEDR |=  (3U << (4*2));

    /* PA4/PA15 co pull-up de giu muc cao khi chua chu dong keo xuong. */
    GPIOA->PUPDR &= ~((3U << (0*2)) | (3U << (1*2)) | (3U << (2*2))
                    | (3U << (4*2)) | (3U << (15*2)));
    GPIOA->PUPDR |=  ((1U << (4*2)) | (1U << (15*2)));

    /* PB3/PB4/PB5 khong dung nen dat analog de giam nhieu va giam dong ro. */
    GPIOB->MODER |= (3U << (3*2)) | (3U << (4*2)) | (3U << (5*2));
    GPIOB->PUPDR &= ~((3U << (3*2)) | (3U << (4*2)) | (3U << (5*2)));

    /* PE0 la input data-ready cua IMU, dung pull-down de muc mac dinh on dinh. */
    GPIOE->MODER &= ~(3U << (0*2));
    GPIOE->PUPDR &= ~(3U << (0*2));
    GPIOE->PUPDR |=  (2U << (0*2));

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* EXTICR[0] chon source cho EXTI0; 0x4 la GPIOE nen PE0 noi vao EXTI line 0. */
    SYSCFG->EXTICR[0] &= ~(0xFU << 0);
    SYSCFG->EXTICR[0] |=  (0x4U << 0);

    /* IMR bat line 0, RTSR bat canh len vi IMU bao data-ready bang rising edge. */
    EXTI->IMR  |=  (1U << 0);
    EXTI->RTSR |=  (1U << 0);
    EXTI->FTSR &= ~(1U << 0);
    EXTI->PR    =  (1U << 0);

    /* NVIC cho phep EXTI0_IRQHandler chay khi PE0 co canh len. */
    NVIC->IP[EXTI0_IRQn] = (uint8_t)(1U << 4);
    NVIC->ISER[EXTI0_IRQn >> 5] = (1U << (EXTI0_IRQn & 0x1F));


}


void Reg_TIM2_Init(void)
{

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1  = 0;
    /* TIM2 clock = 84 MHz; PSC = 84-1 tao tan so dem 1 MHz, moi count = 1 us. */
    TIM2->PSC  = 84U - 1U;
    /* TIM2 la timer 32 bit; ARR max giup moc thoi gian tran sau khoang 71 phut. */
    TIM2->ARR  = 0xFFFFFFFFU;

    /* EGR.UG nap PSC/ARR moi vao timer truoc khi bat dem. */
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR  &= ~TIM_SR_UIF;

    /* CR1.CEN = 1 bat timer dem len. */
    TIM2->CR1 |= TIM_CR1_CEN;
}


void Reg_SystemClock_Config(void)
{

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* VOS scale 1 cho phep STM32F407 chay toi 168 MHz. */
    PWR->CR |= PWR_CR_VOS;


    /* HSE la thach anh ngoai 8 MHz, lam nguon vao PLL. */
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY)) {  }


    /* PLL: 8/8=1 MHz, 1*336=336 MHz, PLLP=2 -> SYSCLK 168 MHz, PLLQ=7 -> 48 MHz. */
    RCC->PLLCFGR = (8U <<  0)
                 | (336U << 6)
                 | (0U << 16)
                 | (RCC_PLLCFGR_PLLSRC_HSE)
                 | (7U << 24);


    /* 168 MHz can 5 wait states cho Flash, dong thoi bat prefetch/cache. */
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN
               | FLASH_ACR_ICEN | FLASH_ACR_DCEN;


    /* AHB = 168 MHz, APB1 = 42 MHz, APB2 = 84 MHz. Timer tren APB1 duoc nhan doi thanh 84 MHz. */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
              | RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV4
              | RCC_CFGR_PPRE2_DIV2;


    /* Bat PLL va doi den khi PLL khoa tan so. */
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY)) {  }


    /* Chuyen SYSCLK sang PLL sau khi PLL san sang. */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {  }


    SystemCoreClock = 168000000U;
}


void Reg_TIM_PWM_Init(void)
{

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;


    /* PB0/PB1 la TIM3_CH3/CH4, PB6/PB7 la TIM4_CH1/CH2; MODER=10 la alternate function. */
    GPIOB->MODER &= ~((3U<<(0*2)) | (3U<<(1*2)) | (3U<<(6*2)) | (3U<<(7*2)));
    GPIOB->MODER |=  ((2U<<(0*2)) | (2U<<(1*2)) | (2U<<(6*2)) | (2U<<(7*2)));

    /* AF2 la alternate function cua TIM3/TIM4 tren cac chan PB0/PB1/PB6/PB7. */
    GPIOB->AFR[0] &= ~((0xFU<<(0*4)) | (0xFU<<(1*4)) | (0xFU<<(6*4)) | (0xFU<<(7*4)));
    GPIOB->AFR[0] |=  ((2U<<(0*4)) | (2U<<(1*4)) | (2U<<(6*4)) | (2U<<(7*4)));


    /* Timer tick = 84 MHz / 84 = 1 MHz; ARR=2500-1 tao chu ky 2500 us = 400 Hz. */
    TIM3->PSC = 84U - 1U;
    TIM3->ARR = 2500U - 1U;


    /* OC mode 6 la PWM mode 1; OCxPE bat preload cho CCR de cap nhat duty on dinh. */
    TIM3->CCMR2 = (6U << 4) | TIM_CCMR2_OC3PE
                | (6U << 12) | TIM_CCMR2_OC4PE;
    TIM3->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
    /* CCR = do rong xung us vi timer tick 1 us; 1000 us la muc thap an toan cho ESC. */
    TIM3->CCR3 = 1000U;
    TIM3->CCR4 = 1000U;
    TIM3->EGR  = TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->CR1 |= TIM_CR1_CEN;


    /* TIM4 dung cung tan so 400 Hz cho hai motor con lai. */
    TIM4->PSC = 84U - 1U;
    TIM4->ARR = 2500U - 1U;

    /* CH1/CH2 nam trong CCMR1, cung chon PWM mode 1. */
    TIM4->CCMR1 = (6U << 4) | TIM_CCMR1_OC1PE
                | (6U << 12) | TIM_CCMR1_OC2PE;
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM4->CCR1 = 1000U;
    TIM4->CCR2 = 1000U;
    TIM4->EGR  = TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->CR1 |= TIM_CR1_CEN;
}
