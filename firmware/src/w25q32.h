#ifndef W25Q32_H
#define W25Q32_H

#include <stdint.h>

int w25_check(void);                                              // read and verify jedec id
int w25_read(uint32_t addr, uint8_t *buf, uint16_t len);
int w25_page_program(uint32_t addr, const uint8_t *buf, uint16_t len);   // must stay inside one page
int w25_sector_erase(uint32_t addr);                              // 4 KB

#endif
