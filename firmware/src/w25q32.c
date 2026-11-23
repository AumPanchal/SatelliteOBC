#include "w25q32.h"
#include "spi.h"
#include "board.h"

#define CMD_WREN  0x06
#define CMD_RDSR1 0x05
#define CMD_READ  0x03
#define CMD_PP    0x02
#define CMD_SE    0x20
#define CMD_JEDEC 0x9F

static int send_cmd_addr(uint8_t cmd, uint32_t addr)
{
    int e = 0;
    e |= spi_xfer(cmd, 0);
    e |= spi_xfer((uint8_t)(addr >> 16), 0);
    e |= spi_xfer((uint8_t)(addr >> 8), 0);
    e |= spi_xfer((uint8_t)addr, 0);
    return e;
}

static int write_enable(void)
{
    int e;
    spi_begin(DEV_FLASH);
    e = spi_xfer(CMD_WREN, 0);
    spi_end(DEV_FLASH);
    return e;
}

// wait until the busy bit clears
static int wait_ready(uint32_t timeout_ms)
{
    uint32_t t = millis();
    uint8_t sr = 1;
    do {
        spi_begin(DEV_FLASH);
        spi_xfer(CMD_RDSR1, 0);
        spi_xfer(0xFF, &sr);
        spi_end(DEV_FLASH);
        if (!(sr & 1)) return 0;
        wdg_kick();
    } while (millis() - t < timeout_ms);
    return -1;
}

int w25_check(void)
{
    uint8_t id[3] = {0, 0, 0};
    int e = 0;

    spi_begin(DEV_FLASH);
    e |= spi_xfer(CMD_JEDEC, 0);
    e |= spi_xfer(0xFF, &id[0]);
    e |= spi_xfer(0xFF, &id[1]);
    e |= spi_xfer(0xFF, &id[2]);
    spi_end(DEV_FLASH);

    // winbond (0xEF), 32 Mbit capacity (0x16)
    if (e || id[0] != 0xEF || id[2] != 0x16) return -1;
    return 0;
}

int w25_read(uint32_t addr, uint8_t *buf, uint16_t len)
{
    int e = 0;
    spi_begin(DEV_FLASH);
    e |= send_cmd_addr(CMD_READ, addr);
    for (uint16_t i = 0; i < len; i++) {
        e |= spi_xfer(0xFF, &buf[i]);
    }
    spi_end(DEV_FLASH);
    return e ? -1 : 0;
}

int w25_page_program(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    int e = 0;
    if (write_enable()) return -1;
    spi_begin(DEV_FLASH);
    e |= send_cmd_addr(CMD_PP, addr);
    for (uint16_t i = 0; i < len; i++) {
        e |= spi_xfer(buf[i], 0);
    }
    spi_end(DEV_FLASH);
    if (e) return -1;
    return wait_ready(10);
}

int w25_sector_erase(uint32_t addr)
{
    int e;
    if (write_enable()) return -1;
    spi_begin(DEV_FLASH);
    e = send_cmd_addr(CMD_SE, addr);
    spi_end(DEV_FLASH);
    if (e) return -1;
    return wait_ready(500);
}
