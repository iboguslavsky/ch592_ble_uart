/*********************************************************************
 * File Name          : uart_bridge.h
 * Description        : BLE to Hardware UART0 Bridge interface.
 *********************************************************************/

#ifndef __UART_BRIDGE_H__
#define __UART_BRIDGE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TMOS Task Events */
#define UBRIDGE_START_DEVICE_EVT       0x0001
#define UBRIDGE_PARAM_UPDATE_EVT       0x0002
#define UBRIDGE_PUMP_UART_TO_BLE_EVT   0x0004
#define UBRIDGE_BTN_UNLOCK_EVT         0x0008
#define UBRIDGE_PHY_UPDATE_EVT         0x0010

/* Default UART0 Configuration */
#define DEFAULT_UART_BAUD_RATE         115200
#define DEFAULT_UART_FLOW_CONTROL      1       /* 1 = Hardware CTS/RTS enabled */

/* Pad 2 (PB22) Diagnostic button */
#define BOOT_BUTTON_PIN                GPIO_Pin_22

void UART_Bridge_Init(void);
uint16_t UART_Bridge_ProcessEvent(uint8_t task_id, uint16_t events);
void UART_Bridge_Poll(void);
void UART_Bridge_CheckButton(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_BRIDGE_H__ */
