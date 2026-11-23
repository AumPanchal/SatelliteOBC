#include "stm32f4xx.h"
#include "spi.h"
#include "board.h"
#include "config.h"

static const uint8_t cs_pin[] = { 0, 1, 2 };   // PB0, PB1, PB2

void spi_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    // master, mode 3, software CS
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA |
                (6u << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi_begin(spi_dev_t d)
{
    uint32_t br = (d == DEV_FLASH) ? 2 : 6;   // flash /8, IMUs /128
    SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR) | (br << SPI_CR1_BR_Pos);
    GPIOB->BSRR = 1u << (cs_pin[d] + 16);
}

void spi_end(spi_dev_t d)
{
    uint32_t t = millis();
    while ((SPI1->SR & SPI_SR_BSY) && (millis() - t <= BUS_TIMEOUT_MS)) {}
    GPIOB->BSRR = 1u << cs_pin[d];
}

int spi_xfer(uint8_t tx, uint8_t *rx)
{
    uint32_t t = millis();

    // wait for room then send
    while (!(SPI1->SR & SPI_SR_TXE)) {
        if (millis() - t > BUS_TIMEOUT_MS) return -1;
    }
    *(volatile uint8_t *)&SPI1->DR = tx;

    // wait for the byte coming back
    while (!(SPI1->SR & SPI_SR_RXNE)) {
        if (millis() - t > BUS_TIMEOUT_MS) return -1;
    }
    uint8_t r = *(volatile uint8_t *)&SPI1->DR;
    if (rx) *rx = r;
    return 0;
}
