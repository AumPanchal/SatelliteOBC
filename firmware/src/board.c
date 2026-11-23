#include "stm32f4xx.h"
#include "board.h"
#include "config.h"

uint32_t g_sysclk_hz = 16000000;
uint32_t g_pclk1_hz  = 16000000;
uint32_t g_pclk2_hz  = 16000000;

static volatile uint32_t ms_ticks;

int clock_init(void)
{
    uint32_t t;

    // start the 8 MHz crystal
    RCC->CR |= RCC_CR_HSEON;
    for (t = 0; t < 500000 && !(RCC->CR & RCC_CR_HSERDY); t++) {}
    if (!(RCC->CR & RCC_CR_HSERDY)) {
        RCC->CR &= ~RCC_CR_HSEON;
        return -1;                   // stay on internal 16 MHz
    }

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;

    // 8 MHz /4 *168 /4 = 84 MHz
    RCC->PLLCFGR = (4u << RCC_PLLCFGR_PLLM_Pos) |
                   (168u << RCC_PLLCFGR_PLLN_Pos) |
                   (1u << RCC_PLLCFGR_PLLP_Pos) |
                   RCC_PLLCFGR_PLLSRC_HSE |
                   (7u << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    for (t = 0; t < 500000 && !(RCC->CR & RCC_CR_PLLRDY); t++) {}
    if (!(RCC->CR & RCC_CR_PLLRDY)) {
        RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON);
        return -1;
    }

    // APB1 = 42 MHz, APB2 = 84 MHz
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_SW_PLL;
    for (t = 0; t < 500000 && (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL; t++) {}

    g_sysclk_hz = 84000000;
    g_pclk1_hz  = 42000000;
    g_pclk2_hz  = 84000000;
    return 0;
}

void systick_init(void)
{
    SysTick_Config(g_sysclk_hz / 1000);   // 1 ms tick
}

void SysTick_Handler(void)
{
    ms_ticks++;
}

uint32_t millis(void)
{
    return ms_ticks;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while (ms_ticks - start < ms) {
        wdg_kick();
    }
}

static void pin_mode(GPIO_TypeDef *g, uint8_t pin, uint8_t mode)
{
    g->MODER = (g->MODER & ~(3u << (pin * 2)))  | ((uint32_t)mode << (pin * 2));
}

static void pin_af(GPIO_TypeDef *g, uint8_t pin, uint8_t af)
{
    uint8_t reg = pin >> 3;
    uint8_t pos = (pin & 7) * 4;
    g->AFR[reg] = (g->AFR[reg] & ~(0xFu << pos)) | ((uint32_t)af << pos);
    pin_mode(g, pin, 2);
    g->OSPEEDR |= 3u << (pin * 2);
}

void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // PA2 / PA3 -> USART2 TX / RX
    pin_af(GPIOA, 2, 7);
    pin_af(GPIOA, 3, 7);

    // PA5 / PA6 / PA7 -> SPI1 SCK / MISO / MOSI
    pin_af(GPIOA, 5, 5);
    pin_af(GPIOA, 6, 5);
    pin_af(GPIOA, 7, 5);

    // PB6 / PB7 -> I2C1 SCL / SDA (open drain, external 4.7k pullups)
    pin_af(GPIOB, 6, 4);
    pin_af(GPIOB, 7, 4);
    GPIOB->OTYPER |= (1u << 6) | (1u << 7);

    // PB0 / PB1 / PB2 -> chip selects, idle high
    GPIOB->BSRR = (1u << 0) | (1u << 1) | (1u << 2);
    pin_mode(GPIOB, 0, 1);
    pin_mode(GPIOB, 1, 1);
    pin_mode(GPIOB, 2, 1);
    GPIOB->OSPEEDR |= (3u << 0) | (3u << 2) | (3u << 4);
}

void wdg_init(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;   // pause watchdog when debugging
    IWDG->KR = 0xCCCC;                                // start
    IWDG->KR = 0x5555;                                // unlock config
    IWDG->PR = 3;                                     // LSI / 32
    IWDG->RLR = WDG_TIMEOUT_MS;                       // ~1 kHz clock -> ~1 s
    while (IWDG->SR) {}
    IWDG->KR = 0xAAAA;
}

void wdg_kick(void)
{
    IWDG->KR = 0xAAAA;
}

uint8_t reset_by_watchdog(void)
{
    uint8_t r = (RCC->CSR & RCC_CSR_IWDGRSTF) ? 1 : 0;
    RCC->CSR |= RCC_CSR_RMVF;    // clear reset flags
    return r;
}
