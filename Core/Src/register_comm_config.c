/*
 * Bao cao: Cau hinh USART, SPI va I2C bang thanh ghi STM32.
 */

#include "register_comm_config.h"


void Reg_USART1_Init(void)
{

    /* USART1 nam tren APB2, GPIOA nam tren AHB1; phai bat clock truoc khi cau hinh. */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;


    /* PA9/PA10 dat alternate function de lam USART1_TX/USART1_RX. */
    GPIOA->MODER &= ~((3U << (9*2)) | (3U << (10*2)));
    GPIOA->MODER |=  ((2U << (9*2)) | (2U << (10*2)));
    GPIOA->OSPEEDR |= (3U << (9*2)) | (3U << (10*2));

    /* AF7 la ma alternate function cua USART1 tren PA9/PA10 theo datasheet STM32F407. */
    GPIOA->AFR[1] &= ~((0xFU << 4) | (0xFU << 8));
    GPIOA->AFR[1] |=  ((7U << 4) | (7U << 8));


    /* PCLK2 = 84 MHz, baud CRSF = 420000 -> BRR = 84000000 / 420000 = 200. */
    USART1->BRR = 200U;


    /* 8N1 mac dinh: CR2=0 chon 1 stop bit, CR3=0 khong dung flow control. */
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    /* TE/RE bat truyen-nhan; RXNEIE cho phep ngat khi nhan du 1 byte CRSF. */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART1->CR1 |= USART_CR1_UE;

    NVIC->IP[USART1_IRQn] = (uint8_t)(0U << 4);
    NVIC->ISER[USART1_IRQn >> 5] = (1U << (USART1_IRQn & 0x1F));
}


void Reg_SPI1_Init(void)
{

    /* SPI1 nam tren APB2, chan SPI1 dung GPIOA. */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;


    /* PA5=SCK, PA6=MISO, PA7=MOSI; MODER=10 chon alternate function. */
    GPIOA->MODER &= ~((3U<<(5*2)) | (3U<<(6*2)) | (3U<<(7*2)));
    GPIOA->MODER |=  ((2U<<(5*2)) | (2U<<(6*2)) | (2U<<(7*2)));
    GPIOA->OSPEEDR |= (3U<<(5*2)) | (3U<<(6*2)) | (3U<<(7*2));

    /* AF5 la ma alternate function cua SPI1 tren PA5/PA6/PA7. */
    GPIOA->AFR[0] &= ~((0xFU<<20) | (0xFU<<24) | (0xFU<<28));
    GPIOA->AFR[0] |=  ((5U<<20) | (5U<<24) | (5U<<28));


    /* MSTR: master; SSM/SSI: NSS phan mem; BR[2:0]=3 chia PCLK2/16 -> 5.25 MHz. */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (3U << 3);
    SPI1->CR2 = 0;
    /* SPE bat SPI sau khi cau hinh xong. */
    SPI1->CR1 |= SPI_CR1_SPE;
}


uint8_t Reg_SPI1_transfer(uint8_t tx)
{
    uint8_t rx;

    /* TXE cho biet thanh ghi truyen san sang nhan byte moi. */
    while(!(SPI1->SR & SPI_SR_TXE)) {  }
    *(volatile uint8_t *)&SPI1->DR = tx;
    /* SPI full-duplex: moi byte truyen di dong thoi tao clock de nhan 1 byte ve. */
    while(!(SPI1->SR & SPI_SR_RXNE)) {  }
    rx = *(volatile uint8_t *)&SPI1->DR;
    /* Cho BSY ve 0 de dam bao giao dich SPI ket thuc truoc khi nha CS. */
    while(SPI1->SR & SPI_SR_BSY) {  }
    return rx;
}


#define I2C_TIMEOUT  100000U


static uint8_t i2c_wait_sr1(I2C_TypeDef *i2c, uint32_t flag)
{
    uint32_t t = I2C_TIMEOUT;
    /* Timeout tranh treo chuong trinh neu bus I2C bi loi hoac thiet bi khong phan hoi. */
    while(!(i2c->SR1 & flag)){ if(--t == 0U) return 1U; }
    return 0U;
}

/* Doc SR1 roi SR2 la trinh tu bat buoc de xoa co ADDR cua I2C STM32F4. */
static inline void i2c_clear_addr(I2C_TypeDef *i2c){ (void)i2c->SR1; (void)i2c->SR2; }

static void i2c_config_400khz(I2C_TypeDef *i2c)
{

    /* Reset mem I2C de dua ngoai vi ve trang thai sach truoc khi cau hinh. */
    i2c->CR1 = I2C_CR1_SWRST;
    i2c->CR1 = 0;

    /* PCLK1 = 42 MHz nen CR2=42; CCR fast-mode duty 2 tao xap xi 400 kHz; TRISE theo fast mode. */
    i2c->CR2   = 42U;
    i2c->CCR   = (1U << 15) | 35U;
    i2c->TRISE = 13U;
    /* PE bat ngoai vi I2C sau khi thiet lap timing. */
    i2c->CR1  |= I2C_CR1_PE;
}

void Reg_I2C2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* PB10/PB11 la I2C2_SCL/SDA noi MS5611; AF4 la chuc nang I2C2. */
    GPIOB->MODER &= ~((3U<<(10*2)) | (3U<<(11*2)));
    GPIOB->MODER |=  ((2U<<(10*2)) | (2U<<(11*2)));
    GPIOB->OTYPER |= (1U << 10) | (1U << 11);
    GPIOB->OSPEEDR |= (3U<<(10*2)) | (3U<<(11*2));
    GPIOB->PUPDR &= ~((3U<<(10*2)) | (3U<<(11*2)));
    GPIOB->PUPDR |=  ((1U<<(10*2)) | (1U<<(11*2)));
    GPIOB->AFR[1] &= ~((0xFU << 8) | (0xFU << 12));
    GPIOB->AFR[1] |=  ((4U << 8) | (4U << 12));

    i2c_config_400khz(I2C2);
}

static uint8_t i2c_write_bytes(I2C_TypeDef *i2c, uint8_t addr7,
                               const uint8_t *buf, uint16_t len)
{
    if(len == 0U) return 10U;


    /* START -> dia chi 7-bit + W -> ghi tung byte -> STOP. */
    i2c->CR1 |= I2C_CR1_START;
    if(i2c_wait_sr1(i2c, I2C_SR1_SB)) return 1U;


    /* Byte dia chi tren bus = addr7 << 1, bit 0 = 0 vi day la thao tac write. */
    i2c->DR = (uint8_t)(addr7 << 1);
    if(i2c_wait_sr1(i2c, I2C_SR1_ADDR)) return 2U;
    i2c_clear_addr(i2c);

    for(uint16_t i = 0U; i < len; i++){
        if(i2c_wait_sr1(i2c, I2C_SR1_TXE)) return 3U;
        i2c->DR = buf[i];
    }

    if(i2c_wait_sr1(i2c, I2C_SR1_BTF)) return 5U;
    i2c->CR1 |= I2C_CR1_STOP;
    return 0U;
}


static uint8_t i2c_cmd_read(I2C_TypeDef *i2c, uint8_t addr7, uint8_t cmd,
                            uint8_t *buf, uint16_t len)
{
    if(len == 0U || buf == 0) return 10U;


    /* Giai doan 1: gui command/register can doc bang write transfer. */
    i2c->CR1 |= I2C_CR1_ACK;
    i2c->CR1 &= ~I2C_CR1_POS;
    i2c->CR1 |= I2C_CR1_START;
    if(i2c_wait_sr1(i2c, I2C_SR1_SB)) return 1U;
    i2c->DR = (uint8_t)(addr7 << 1);
    if(i2c_wait_sr1(i2c, I2C_SR1_ADDR)) return 2U;
    i2c_clear_addr(i2c);
    if(i2c_wait_sr1(i2c, I2C_SR1_TXE)) return 3U;
    i2c->DR = cmd;
    if(i2c_wait_sr1(i2c, I2C_SR1_BTF)) return 4U;


    /* Giai doan 2: repeated START, gui dia chi voi bit read = 1 de nhan du lieu. */
    i2c->CR1 |= I2C_CR1_START;
    if(i2c_wait_sr1(i2c, I2C_SR1_SB)) return 5U;
    i2c->DR = (uint8_t)((addr7 << 1) | 1U);
    if(i2c_wait_sr1(i2c, I2C_SR1_ADDR)) return 6U;

    uint16_t i = 0U;
    if(len == 1U){
        i2c->CR1 &= ~I2C_CR1_ACK;
        i2c_clear_addr(i2c);
        i2c->CR1 |= I2C_CR1_STOP;
        if(i2c_wait_sr1(i2c, I2C_SR1_RXNE)) return 7U;
        buf[0] = (uint8_t)i2c->DR;
    }
    else if(len == 2U){
        i2c->CR1 |= I2C_CR1_POS;
        i2c->CR1 &= ~I2C_CR1_ACK;
        i2c_clear_addr(i2c);
        if(i2c_wait_sr1(i2c, I2C_SR1_BTF)) return 8U;
        i2c->CR1 |= I2C_CR1_STOP;
        buf[i++] = (uint8_t)i2c->DR;
        buf[i++] = (uint8_t)i2c->DR;
    }
    else{
        i2c_clear_addr(i2c);

        while((uint16_t)(len - i) > 3U){
            if(i2c_wait_sr1(i2c, I2C_SR1_RXNE)) return 7U;
            buf[i++] = (uint8_t)i2c->DR;
        }

        if(i2c_wait_sr1(i2c, I2C_SR1_BTF)) return 8U;
        i2c->CR1 &= ~I2C_CR1_ACK;
        buf[i++] = (uint8_t)i2c->DR;
        if(i2c_wait_sr1(i2c, I2C_SR1_BTF)) return 9U;
        i2c->CR1 |= I2C_CR1_STOP;
        buf[i++] = (uint8_t)i2c->DR;
        if(i2c_wait_sr1(i2c, I2C_SR1_RXNE)) return 11U;
        buf[i++] = (uint8_t)i2c->DR;
    }
    i2c->CR1 &= ~(I2C_CR1_ACK | I2C_CR1_POS);
    return 0U;
}

uint8_t Reg_I2C2_cmd_write(uint8_t addr7, uint8_t cmd)
{
    /* Ghi mot byte command cho cam bien tren bus I2C2, vi du lenh D1/D2/RESET cua MS5611. */
    return i2c_write_bytes(I2C2, addr7, &cmd, 1U);
}

uint8_t Reg_I2C2_cmd_read(uint8_t addr7, uint8_t cmd, uint8_t *buf, uint16_t len)
{
    return i2c_cmd_read(I2C2, addr7, cmd, buf, len);
}
