#include <string.h>
#include "stm32f4xx.h"
#include "comm.h"
#include "crc16.h"
#include "board.h"
#include "config.h"

#define RX_BUF_SIZE 128

static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t rx_last_ms;
static uint8_t tx_seq;

void uart_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = (g_pclk1_hz + UART_BAUD / 2) / UART_BAUD;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t b = (uint8_t)USART2->DR;           // reading clears the flags
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail) {                      // drop byte if buffer full
            rx_buf[rx_head] = b;
            rx_head = next;
        }
        rx_last_ms = millis();
    }
}

static void uart_putc(uint8_t b)
{
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = b;
}

void uart_flush(void)
{
    while (!(USART2->SR & USART_SR_TC)) {}
}

void comm_send(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    uint8_t hdr[3] = { type, seq, len };

    // crc covers type, seq, len and payload
    uint16_t crc = crc16_update(CRC16_INIT, hdr, 3);
    crc = crc16_update(crc, payload, len);

    uart_putc(SYNC1);
    uart_putc(SYNC2);
    for (uint8_t i = 0; i < 3; i++) uart_putc(hdr[i]);
    for (uint8_t i = 0; i < len; i++) uart_putc(payload[i]);
    uart_putc((uint8_t)(crc >> 8));
    uart_putc((uint8_t)crc);
}

static uint8_t *put16(uint8_t *p, uint16_t v)
{
    *p++ = (uint8_t)v;
    *p++ = (uint8_t)(v >> 8);
    return p;
}

static uint8_t *put32(uint8_t *p, uint32_t v)
{
    p = put16(p, (uint16_t)v);
    return put16(p, (uint16_t)(v >> 16));
}

static uint8_t *put_imu(uint8_t *p, const imu_data_t *d)
{
    p = put16(p, (uint16_t)d->ax);
    p = put16(p, (uint16_t)d->ay);
    p = put16(p, (uint16_t)d->az);
    p = put16(p, (uint16_t)d->gx);
    p = put16(p, (uint16_t)d->gy);
    p = put16(p, (uint16_t)d->gz);
    return p;
}

// payload is little endian, 39 bytes: <IBBB 6h 6h h H h H
void comm_send_telemetry(const telemetry_t *t)
{
    uint8_t buf[39];
    uint8_t *p = buf;

    p = put32(p, t->time_ms);
    *p++ = t->state;
    *p++ = t->fault;
    *p++ = t->flags;
    p = put_imu(p, &t->imu1);
    p = put_imu(p, &t->imu2);
    p = put16(p, (uint16_t)t->temp_cdeg);
    p = put16(p, t->bus_mv);
    p = put16(p, (uint16_t)t->current_ma);
    p = put16(p, t->log_count);

    comm_send(PKT_TELEMETRY, tx_seq++, buf, (uint8_t)(p - buf));
}

int comm_poll(frame_t *f)
{
    enum { S_SYNC1, S_SYNC2, S_TYPE, S_SEQ, S_LEN, S_PAYLOAD, S_CRC_H, S_CRC_L };
    static uint8_t st = S_SYNC1;
    static uint8_t idx;
    static uint16_t rx_crc;
    static frame_t cur;

    while (rx_tail != rx_head) {
        uint8_t b = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

        switch (st) {
        case S_SYNC1:
            if (b == SYNC1) st = S_SYNC2;
            break;
        case S_SYNC2:
            st = (b == SYNC2) ? S_TYPE : (b == SYNC1 ? S_SYNC2 : S_SYNC1);
            break;
        case S_TYPE:
            cur.type = b;
            st = S_SEQ;
            break;
        case S_SEQ:
            cur.seq = b;
            st = S_LEN;
            break;
        case S_LEN:
            cur.len = b;
            idx = 0;
            if (b > RX_MAX_PAYLOAD) st = S_SYNC1;       // too long, resync
            else st = b ? S_PAYLOAD : S_CRC_H;
            break;
        case S_PAYLOAD:
            cur.payload[idx++] = b;
            if (idx >= cur.len) st = S_CRC_H;
            break;
        case S_CRC_H:
            rx_crc = (uint16_t)(b << 8);
            st = S_CRC_L;
            break;
        case S_CRC_L:
            rx_crc |= b;
            st = S_SYNC1;
            {
                uint8_t hdr[3] = { cur.type, cur.seq, cur.len };
                uint16_t crc = crc16_update(CRC16_INIT, hdr, 3);
                crc = crc16_update(crc, cur.payload, cur.len);
                if (crc == rx_crc) {
                    memcpy(f, &cur, sizeof(frame_t));
                    return 1;
                }
                f->seq = cur.seq;
                return -1;
            }
        }
    }

    // drop a half received frame if the line goes quiet
    if (st != S_SYNC1 && millis() - rx_last_ms > 100) st = S_SYNC1;
    return 0;
}
