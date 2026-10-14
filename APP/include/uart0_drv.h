/*********************************************************************
 * File Name          : uart0_drv.h
 * Description        : High-performance UART0 driver with hardware
 *                      CTS/RTS flow control and SRAM ring buffers.
 *********************************************************************/

#ifndef __UART0_DRV_H__
#define __UART0_DRV_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UART0_RX_BUF_SIZE   1024    /* Must be power of 2 */
#define UART0_TX_BUF_SIZE   1024    /* Must be power of 2 */

#define UART0_FLOW_NONE     0
#define UART0_FLOW_HW_RTS_CTS 1

/**
 * @brief Initialize UART0 with specified baud rate and flow control.
 *
 * Pin assignments:
 * - TXD0: PB7 (Output)
 * - RXD0: PB4 (Input with pull-up)
 * - CTS0: PB0 (Input with pull-up, remote RTS connected here)
 * - RTS0: PB6 (Output, remote CTS connected here)
 *
 * @param baud_rate Baud rate (e.g. 115200, 921600, 1000000, 2000000)
 * @param flow_ctrl UART0_FLOW_NONE or UART0_FLOW_HW_RTS_CTS
 */
void uart0_init(uint32_t baud_rate, uint8_t flow_ctrl);

/**
 * @brief Write data to UART0 transmit buffer (non-blocking / buffered).
 *
 * @param buf Pointer to data
 * @param len Number of bytes to write
 * @return Number of bytes actually enqueued
 */
int uart0_write(const void *buf, int len);

/**
 * @brief Read data from UART0 receive buffer.
 *
 * @param buf Destination buffer
 * @param len Maximum bytes to read
 * @return Number of bytes read
 */
int uart0_read(void *buf, int len);

/**
 * @brief Check number of bytes available in RX buffer.
 */
int uart0_available(void);

/**
 * @brief Check free space in TX buffer.
 */
int uart0_tx_free(void);

/**
 * @brief Wait until all TX data has been transmitted.
 */
void uart0_flush(void);

/**
 * @brief Set baud rate at runtime.
 */
void uart0_set_baudrate(uint32_t baud_rate);

#ifdef __cplusplus
}
#endif

#endif /* __UART0_DRV_H__ */
