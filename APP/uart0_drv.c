/*********************************************************************
 * File Name          : uart0_drv.c
 * Description        : High-performance UART0 driver with hardware
 *                      CTS/RTS flow control and SRAM ring buffers.
 *********************************************************************/

#include "CH59x_common.h"
#include "uart0_drv.h"
#include <string.h>

/* Ring buffers for RX and TX */
static volatile uint8_t  s_rx_buf[UART0_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;

static volatile uint8_t  s_tx_buf[UART0_TX_BUF_SIZE];
static volatile uint16_t s_tx_head = 0;
static volatile uint16_t s_tx_tail = 0;

static volatile uint8_t  s_tx_active = 0;
static uint8_t           s_flow_ctrl = UART0_FLOW_NONE;
static volatile uint8_t  s_rts_throttled = 0;

/* Watermark thresholds for RTS flow control */
#define RX_HIGH_WATERMARK   (UART0_RX_BUF_SIZE - 128)  /* Stop remote when buffer has >= 896 bytes */
#define RX_LOW_WATERMARK    (UART0_RX_BUF_SIZE / 2)    /* Resume remote when buffer drops to <= 512 bytes */

static inline uint16_t rx_count(void)
{
    return (uint16_t)((s_rx_head - s_rx_tail) & (UART0_RX_BUF_SIZE - 1));
}

static inline uint16_t rx_free(void)
{
    return (uint16_t)((UART0_RX_BUF_SIZE - 1) - rx_count());
}

static inline uint16_t tx_count(void)
{
    return (uint16_t)((s_tx_head - s_tx_tail) & (UART0_TX_BUF_SIZE - 1));
}

static inline uint16_t tx_free(void)
{
    return (uint16_t)((UART0_TX_BUF_SIZE - 1) - tx_count());
}

/*********************************************************************
 * @fn      uart0_set_baudrate
 *********************************************************************/
void uart0_set_baudrate(uint32_t baud_rate)
{
    uint32_t x;
    x = 10 * GetSysClock() / 8 / baud_rate;
    x = (x + 5) / 10;
    R16_UART0_DL = (uint16_t)x;
}

/*********************************************************************
 * @fn      uart0_init
 *********************************************************************/
void uart0_init(uint32_t baud_rate, uint8_t flow_ctrl)
{
    s_flow_ctrl = flow_ctrl;
    s_rx_head = 0;
    s_rx_tail = 0;
    s_tx_head = 0;
    s_tx_tail = 0;
    s_tx_active = 0;
    s_rts_throttled = 0;

    /* Ensure UART0 uses standard pins: PB4 (RXD0), PB7 (TXD0), PB0 (CTS), PB6 (RTS) */
    R16_PIN_ALTERNATE &= ~RB_PIN_UART0;

    /* Configure GPIO Pins */
    GPIOB_SetBits(bTXD0);
    GPIOB_ModeCfg(bTXD0, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(bRXD0, GPIO_ModeIN_PU);

    if (s_flow_ctrl == UART0_FLOW_HW_RTS_CTS)
    {
        /* CTS is input with pull-up; RTS is output (asserted low initially) */
        GPIOB_ModeCfg(bCTS, GPIO_ModeIN_PU);
        GPIOB_ResetBits(bRTS);
        GPIOB_ModeCfg(bRTS, GPIO_ModeOut_PP_5mA);
    }

    /* Configure UART0 Hardware */
    uart0_set_baudrate(baud_rate);
    R8_UART0_DIV = 1;

    /* 8 data bits, 1 stop bit, no parity */
    R8_UART0_LCR = RB_LCR_WORD_SZ;

    /* Enable 8-byte FIFO, 4-byte trigger level, reset FIFOs */
    R8_UART0_FCR = (2 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;

    /* Modem & Flow Control Configuration */
    uint8_t mcr = RB_MCR_INT_OE;
    uint8_t ier = RB_IER_TXD_EN | RB_IER_RECV_RDY | RB_IER_LINE_STAT;

    if (s_flow_ctrl == UART0_FLOW_HW_RTS_CTS)
    {
        /* Enable hardware auto flow control (CTS monitors remote, RTS drives remote) */
        mcr |= RB_MCR_AU_FLOW_EN | RB_MCR_RTS;
        ier |= RB_IER_RTS_EN;
    }

    R8_UART0_MCR = mcr;
    R8_UART0_IER = ier;

    /* Enable UART0 interrupt in PFIC with high priority */
    PFIC_SetPriority(UART0_IRQn, 0xE0);
    PFIC_EnableIRQ(UART0_IRQn);
}

/*********************************************************************
 * @fn      uart0_write
 *********************************************************************/
int uart0_write(const void *buf, int len)
{
    if (len <= 0) return 0;
    const uint8_t *src = (const uint8_t *)buf;
    int written = 0;

    PFIC_DisableIRQ(UART0_IRQn);

    /* Enqueue into TX ring buffer */
    while (written < len && tx_free() > 0)
    {
        s_tx_buf[s_tx_head] = src[written++];
        s_tx_head = (s_tx_head + 1) & (UART0_TX_BUF_SIZE - 1);
    }

    /* If transmitter is currently idle, feed hardware FIFO immediately */
    if (!s_tx_active && tx_count() > 0)
    {
        while (tx_count() > 0 && R8_UART0_TFC < UART_FIFO_SIZE)
        {
            R8_UART0_THR = s_tx_buf[s_tx_tail];
            s_tx_tail = (s_tx_tail + 1) & (UART0_TX_BUF_SIZE - 1);
        }
        s_tx_active = 1;
        R8_UART0_IER |= RB_IER_THR_EMPTY;
    }

    PFIC_EnableIRQ(UART0_IRQn);
    return written;
}

/*********************************************************************
 * @fn      uart0_read
 *********************************************************************/
int uart0_read(void *buf, int len)
{
    if (len <= 0) return 0;
    uint8_t *dst = (uint8_t *)buf;
    int count = 0;

    PFIC_DisableIRQ(UART0_IRQn);

    while (count < len && rx_count() > 0)
    {
        dst[count++] = s_rx_buf[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) & (UART0_RX_BUF_SIZE - 1);
    }

    /* If remote was throttled and buffer space has recovered, reassert RTS */
    if (s_rts_throttled && (rx_count() <= RX_LOW_WATERMARK))
    {
        s_rts_throttled = 0;
        if (s_flow_ctrl == UART0_FLOW_HW_RTS_CTS)
        {
            R8_UART0_MCR |= RB_MCR_RTS;
        }
    }

    PFIC_EnableIRQ(UART0_IRQn);
    return count;
}

/*********************************************************************
 * @fn      uart0_available
 *********************************************************************/
int uart0_available(void)
{
    return rx_count();
}

/*********************************************************************
 * @fn      uart0_tx_free
 *********************************************************************/
int uart0_tx_free(void)
{
    return tx_free();
}

/*********************************************************************
 * @fn      uart0_flush
 *********************************************************************/
void uart0_flush(void)
{
    while (s_tx_active || tx_count() > 0)
    {
        __nop();
    }
    while (!(R8_UART0_LSR & RB_LSR_TX_ALL_EMP))
    {
        __nop();
    }
}

/*********************************************************************
 * @fn      UART0_IRQHandler
 * @brief   Zero-latency UART0 ISR running from SRAM (.highcode)
 *********************************************************************/
__INTERRUPT
__HIGH_CODE
void UART0_IRQHandler(void)
{
    uint8_t iir;

    while (!((iir = R8_UART0_IIR) & RB_IIR_NO_INT))
    {
        switch (iir & RB_IIR_INT_MASK)
        {
            case UART_II_RECV_RDY:
            case UART_II_RECV_TOUT:
            {
                /* Drain RX FIFO into software ring buffer */
                while (R8_UART0_RFC > 0)
                {
                    uint8_t b = R8_UART0_RBR;
                    if (rx_free() > 0)
                    {
                        s_rx_buf[s_rx_head] = b;
                        s_rx_head = (s_rx_head + 1) & (UART0_RX_BUF_SIZE - 1);
                    }
                }

                /* Flow control: deassert RTS if buffer crossed high watermark */
                if (s_flow_ctrl == UART0_FLOW_HW_RTS_CTS && !s_rts_throttled)
                {
                    if (rx_count() >= RX_HIGH_WATERMARK)
                    {
                        s_rts_throttled = 1;
                        R8_UART0_MCR &= ~RB_MCR_RTS;
                    }
                }
                break;
            }

            case UART_II_THR_EMPTY:
            {
                /* Fill TX FIFO from software ring buffer */
                while (tx_count() > 0 && R8_UART0_TFC < UART_FIFO_SIZE)
                {
                    R8_UART0_THR = s_tx_buf[s_tx_tail];
                    s_tx_tail = (s_tx_tail + 1) & (UART0_TX_BUF_SIZE - 1);
                }

                if (tx_count() == 0)
                {
                    s_tx_active = 0;
                    R8_UART0_IER &= ~RB_IER_THR_EMPTY;
                }
                break;
            }

            case UART_II_LINE_STAT:
            {
                /* Read LSR to clear line status interrupt */
                (void)R8_UART0_LSR;
                break;
            }

            case UART_II_MODEM_CHG:
            {
                /* Read MSR to clear modem status interrupt */
                (void)R8_UART0_MSR;
                break;
            }

            default:
                break;
        }
    }
}
