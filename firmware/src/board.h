#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

extern uint32_t g_sysclk_hz;
extern uint32_t g_pclk1_hz;
extern uint32_t g_pclk2_hz;

int      clock_init(void);          // 0 = running on 8 MHz crystal PLL, -1 = fell back to HSI
void     systick_init(void);
void     gpio_init(void);
void     wdg_init(void);
void     wdg_kick(void);
uint8_t  reset_by_watchdog(void);
uint32_t millis(void);
void     delay_ms(uint32_t ms);

#endif
