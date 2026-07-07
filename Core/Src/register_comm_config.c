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
    /* TE/RE bat truyen-nhan; UE bat USART. */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;
}


void Reg_USART1_write(uint8_t byte)
{
    /* TXE = 1 nghia la data register trong, co the ghi byte moi. */
    while(!(USART1->SR & USART_SR_TXE)) {  }
    USART1->DR = byte;
}

void Reg_USART1_print(const char *s)
{
    while(*s) Reg_USART1_write((uint8_t)*s++);
}


uint8_t Reg_USART1_read(void)
{
    /* RXNE = 1 nghia la USART da nhan du 1 byte trong DR. */
    while(!(USART1->SR & USART_SR_RXNE)) {  }
    return (uint8_t)USART1->DR;
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


/* Bit 7 cua dia chi SPI ICM42688 la bit read; WHO_AM_I o dia chi 0x75 theo datasheet. */
#define ICM42688_SPI_READ_BIT   0x80U
#define ICM42688_REG_WHO_AM_I   0x75U

uint8_t Reg_ICM42688_read_reg(uint8_t reg)
{
    uint8_t val;

    GPIOA->BSRR = (1U << (4U + 16U));
    /* Doc thanh ghi: gui reg | 0x80, sau do gui dummy 0x00 de nhan du lieu tra ve. */
    (void)Reg_SPI1_transfer((uint8_t)(reg | ICM42688_SPI_READ_BIT));
    val = Reg_SPI1_transfer(0x00U);
    GPIOA->BSRR = (1U << 4U);

    return val;
}

void Reg_ICM42688_write_reg(uint8_t reg, uint8_t val)
{
    GPIOA->BSRR = (1U << (4U + 16U));
    /* Ghi thanh ghi: clear bit 7 bang reg & ~0x80, byte tiep theo la data. */
    (void)Reg_SPI1_transfer((uint8_t)(reg & ~ICM42688_SPI_READ_BIT));
    (void)Reg_SPI1_transfer(val);
    GPIOA->BSRR = (1U << 4U);
}

uint8_t Reg_ICM42688_who_am_i(void)
{
    return Reg_ICM42688_read_reg(ICM42688_REG_WHO_AM_I);
}


/* MS5611: dia chi 7-bit 0x77 khi chan CSB noi GND; reset 0x1E; PROM bat dau tu 0xA0. */
#define I2C_TIMEOUT  100000U
#define MS5611_ADDR_CSB_GND_REG 0x77U
#define MS5611_CMD_RESET_REG    0x1EU
#define MS5611_CMD_PROM_BASE    0xA0U


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

void Reg_I2C1_Init(void)
{

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;


    /* PB8/PB9 la I2C1_SCL/SDA; I2C can open-drain va alternate function AF4. */
    GPIOB->MODER &= ~((3U<<(8*2)) | (3U<<(9*2)));
    GPIOB->MODER |=  ((2U<<(8*2)) | (2U<<(9*2)));
    GPIOB->OTYPER |= (1U << 8) | (1U << 9);
    GPIOB->OSPEEDR |= (3U<<(8*2)) | (3U<<(9*2));
    GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOB->AFR[1] |=  ((4U << 0) | (4U << 4));


    i2c_config_400khz(I2C1);
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


uint8_t Reg_I2C1_mem_write(uint8_t addr7, uint8_t reg, uint8_t val)
{
    uint8_t frame[2] = {reg, val};
    return i2c_write_bytes(I2C1, addr7, frame, 2U);
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

uint8_t Reg_I2C1_mem_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return i2c_cmd_read(I2C1, addr7, reg, buf, len);
}

uint8_t Reg_I2C2_cmd_read(uint8_t addr7, uint8_t cmd, uint8_t *buf, uint16_t len)
{
    return i2c_cmd_read(I2C2, addr7, cmd, buf, len);
}

uint8_t Reg_MS5611_reset(void)
{
    uint8_t cmd = MS5611_CMD_RESET_REG;
    return i2c_write_bytes(I2C2, MS5611_ADDR_CSB_GND_REG, &cmd, 1U);
}

uint16_t Reg_MS5611_read_prom(uint8_t index)
{
    uint8_t raw[2];

    /* MS5611 co 8 word PROM, dia chi tung word = 0xA0 + index*2. */
    if(index > 7U) return 0xFFFFU;
    if(Reg_I2C2_cmd_read(MS5611_ADDR_CSB_GND_REG,
                         (uint8_t)(MS5611_CMD_PROM_BASE + (index * 2U)),
                         raw, 2U) != 0U){
        return 0xFFFFU;
    }
    return (uint16_t)(((uint16_t)raw[0] << 8) | raw[1]);
}
