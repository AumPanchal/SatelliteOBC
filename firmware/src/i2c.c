#include "stm32f4xx.h"
#include "i2c.h"
#include "board.h"
#include "config.h"

static int wait_set(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t t = millis();
    while (!(*reg & mask)) {
        if (millis() - t > BUS_TIMEOUT_MS) return -1;
    }
    return 0;
}

static int wait_clear(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t t = millis();
    while (*reg & mask) {
        if (millis() - t > BUS_TIMEOUT_MS) return -1;
    }
    return 0;
}

static void clear_addr(void)
{
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

void i2c_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR1 = I2C_CR1_SWRST;               // reset peripheral
    I2C1->CR1 = 0;

    uint32_t mhz = g_pclk1_hz / 1000000;
    I2C1->CR2 = mhz;
    I2C1->CCR = g_pclk1_hz / (2 * 100000);   // 100 kHz standard mode
    I2C1->TRISE = mhz + 1;
    I2C1->CR1 = I2C_CR1_PE;
}

// send start + address and wait for the slave to ack
static int send_addr(uint8_t addr, uint8_t rw)
{
    I2C1->CR1 |= I2C_CR1_START;
    if (wait_set(&I2C1->SR1, I2C_SR1_SB)) return -1;
    I2C1->DR = (uint8_t)((addr << 1) | rw);

    uint32_t t = millis();
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) return -1;   // no ack
        if (millis() - t > BUS_TIMEOUT_MS) return -1;
    }
    return 0;
}

static void bus_recover(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    i2c_init();
}

int i2c_read16(uint8_t addr, uint8_t reg, uint16_t *val)
{
    uint8_t hi, lo;

    if (wait_clear(&I2C1->SR2, I2C_SR2_BUSY)) goto fail;

    // write the register pointer
    if (send_addr(addr, 0)) goto fail;
    clear_addr();
    if (wait_set(&I2C1->SR1, I2C_SR1_TXE)) goto fail;
    I2C1->DR = reg;
    if (wait_set(&I2C1->SR1, I2C_SR1_BTF)) goto fail;

    // repeated start and read 2 bytes
    I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_POS;
    if (send_addr(addr, 1)) goto fail;
    __disable_irq();
    clear_addr();
    I2C1->CR1 &= ~I2C_CR1_ACK;               // nack after the last byte
    __enable_irq();
    if (wait_set(&I2C1->SR1, I2C_SR1_BTF)) goto fail;
    __disable_irq();
    I2C1->CR1 |= I2C_CR1_STOP;
    hi = (uint8_t)I2C1->DR;
    __enable_irq();
    lo = (uint8_t)I2C1->DR;

    I2C1->CR1 &= ~I2C_CR1_POS;
    *val = (uint16_t)((hi << 8) | lo);
    return 0;

fail:
    bus_recover();
    return -1;
}

int i2c_write16(uint8_t addr, uint8_t reg, uint16_t val)
{
    if (wait_clear(&I2C1->SR2, I2C_SR2_BUSY)) goto fail;

    if (send_addr(addr, 0)) goto fail;
    clear_addr();

    if (wait_set(&I2C1->SR1, I2C_SR1_TXE)) goto fail;
    I2C1->DR = reg;
    if (wait_set(&I2C1->SR1, I2C_SR1_TXE)) goto fail;
    I2C1->DR = (uint8_t)(val >> 8);
    if (wait_set(&I2C1->SR1, I2C_SR1_TXE)) goto fail;
    I2C1->DR = (uint8_t)val;
    if (wait_set(&I2C1->SR1, I2C_SR1_BTF)) goto fail;

    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;

fail:
    bus_recover();
    return -1;
}
