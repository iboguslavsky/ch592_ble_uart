/*********************************************************************
 * File Name          : uart_bridge.c
 * Description        : High-throughput BLE Nordic NUS to Hardware UART0
 *                      bridge with automatic CTS/RTS hardware flow control.
 *********************************************************************/

#include "CONFIG.h"
#include "CH59x_common.h"
#include "CH59x_sys.h"
#include "CH59x_gpio.h"
#include "ISP592.h"
#include "devinfoservice.h"
#include "ble_uart_service.h"
#include "uart0_drv.h"
#include "uart_bridge.h"
#include <stdio.h>
#include <string.h>

/*********************************************************************
 * CONSTANTS & CONFIGURATION
 *********************************************************************/
#define DEFAULT_ADVERTISING_INTERVAL        80     /* 50ms (in 625us units) */
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL   12     /* 15.0ms (in 1.25ms units, Apple compliant) */
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL   16     /* 20.0ms (in 1.25ms units) */
#define DEFAULT_DESIRED_SLAVE_LATENCY       0
#define DEFAULT_DESIRED_CONN_TIMEOUT        200    /* 2 seconds (in 10ms units) */

/*********************************************************************
 * GLOBAL & LOCAL VARIABLES
 *********************************************************************/
static uint8_t s_bridge_task_id = INVALID_TASK_ID;
static uint16_t s_conn_handle = GAP_CONNHANDLE_INIT;
static uint16_t s_conn_interval = 0;
static uint16_t s_conn_mtu = 23;
static uint8_t  s_conn_phy_tx = 1;
static uint8_t  s_conn_phy_rx = 1;

/* Traffic statistics */
static uint32_t s_stat_uart_to_ble = 0;
static uint32_t s_stat_ble_to_uart = 0;
static uint32_t s_stat_btn_presses = 0;

/* Button debouncing and diagnostics buffer */
static uint8_t  s_prev_btn_state = 1;
static uint8_t  s_btn_locked = 0;
static char     s_report_buf[384];

/* Device Name */
static const uint8_t s_device_name[] = "CH592_BLE_UART";

/* GAP Advertisement Data:
 * Flags + 128-bit Nordic NUS Service UUID (6E400001-B5A3-F393-E0A9-E50E24DCCA9F) */
static uint8_t s_advert_data[] = {
    0x02,
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    0x11,
    GAP_ADTYPE_128BIT_COMPLETE,
    0x9F, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

/* GAP Scan Response Data */
static uint8_t s_scan_rsp_data[] = {
    15,
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'C', 'H', '5', '9', '2', '_', 'B', 'L', 'E', '_', 'U', 'A', 'R', 'T',

    0x05,
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL)
};

/*********************************************************************
 * FORWARD DECLARATIONS
 *********************************************************************/
static void ubridge_gap_state_cb(gapRole_States_t new_state, gapRoleEvent_t *p_event);
static void ubridge_ble_uart_evt_cb(uint16_t conn_handle, ble_uart_evt_t *p_evt);

static gapRolesCBs_t s_gap_peripheral_cbs = {
    ubridge_gap_state_cb,
    NULL,
    NULL
};

static gapRolesBroadcasterCBs_t s_gap_broadcaster_cbs = {
    NULL,
    NULL
};

static gapBondCBs_t s_gap_bond_mgr_cbs = {
    NULL,
    NULL
};

/*********************************************************************
 * BLE UART SERVICE CALLBACK
 *********************************************************************/
static void ubridge_ble_uart_evt_cb(uint16_t conn_handle, ble_uart_evt_t *p_evt)
{
    (void)conn_handle;

    switch (p_evt->type)
    {
        case BLE_UART_EVT_BLE_DATA_RECIEVED:
        {
            /* Central -> Peripheral: Write directly to Hardware UART0 */
            if (p_evt->data.length > 0 && p_evt->data.p_data != NULL)
            {
                uart0_write(p_evt->data.p_data, (int)p_evt->data.length);
                s_stat_ble_to_uart += p_evt->data.length;
            }
            break;
        }

        case BLE_UART_EVT_TX_NOTI_ENABLED:
        {
            s_conn_mtu = ATT_GetMTU(s_conn_handle);
            tmos_set_event(s_bridge_task_id, UBRIDGE_PUMP_UART_TO_BLE_EVT);
            break;
        }

        case BLE_UART_EVT_TX_NOTI_DISABLED:
            break;

        default:
            break;
    }
}

/*********************************************************************
 * GAP STATE CHANGE CALLBACK
 *********************************************************************/
static void ubridge_gap_state_cb(gapRole_States_t new_state, gapRoleEvent_t *p_event)
{
    switch (new_state & GAPROLE_STATE_ADV_MASK)
    {
        case GAPROLE_STARTED:
            break;

        case GAPROLE_ADVERTISING:
            s_conn_handle = GAP_CONNHANDLE_INIT;
            s_conn_interval = 0;
            s_conn_mtu = 23;
            s_conn_phy_tx = 1;
            s_conn_phy_rx = 1;
            break;

        case GAPROLE_CONNECTED:
            if (p_event && p_event->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                gapEstLinkReqEvent_t *ev = (gapEstLinkReqEvent_t *)p_event;
                s_conn_handle = ev->connectionHandle;
                s_conn_interval = ev->connInterval;
                s_conn_mtu = ATT_GetMTU(s_conn_handle);
                s_conn_phy_tx = 1;
                s_conn_phy_rx = 1;

                /* Delay connection parameter update by 1.5s */
                tmos_start_task(s_bridge_task_id, UBRIDGE_PARAM_UPDATE_EVT, 2400);

                /* Delay 2M PHY upgrade by 2.5s */
                tmos_start_task(s_bridge_task_id, UBRIDGE_PHY_UPDATE_EVT, 4000);
            }
            break;

        case GAPROLE_WAITING:
            s_conn_handle = GAP_CONNHANDLE_INIT;
            s_conn_interval = 0;
            s_conn_mtu = 23;
            s_conn_phy_tx = 1;
            s_conn_phy_rx = 1;
            {
                uint8_t adv_enable = TRUE;
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_enable);
            }
            break;

        default:
            break;
    }
}

/*********************************************************************
 * BRIDGE INITIALIZATION
 *********************************************************************/
void UART_Bridge_Init(void)
{
    s_bridge_task_id = TMOS_ProcessEventRegister(UART_Bridge_ProcessEvent);

    /* Initialize Hardware UART0 with CTS/RTS Auto-Flow Control */
    uart0_init(DEFAULT_UART_BAUD_RATE, DEFAULT_UART_FLOW_CONTROL);

    /* Setup BOOT Button on Pad 2 (PB22) */
    GPIOB_ModeCfg(BOOT_BUTTON_PIN, GPIO_ModeIN_PU);

    /* Configure GAP Peripheral Role */
    uint8_t  initial_adv_enable   = TRUE;
    uint16_t desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_adv_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(s_advert_data), s_advert_data);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(s_scan_rsp_data), s_scan_rsp_data);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &desired_min_interval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &desired_max_interval);

    uint16_t adv_int = DEFAULT_ADVERTISING_INTERVAL;
    GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, adv_int);
    GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, adv_int);

    /* Setup GATT Services */
    GGS_AddService(GATT_ALL_SERVICES);
    GATTServApp_AddService(GATT_ALL_SERVICES);
    DevInfo_AddService();

    /* Nordic NUS Service */
    ble_uart_add_service(ubridge_ble_uart_evt_cb);
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, sizeof(s_device_name) - 1, (void *)s_device_name);

    GAPRole_BroadcasterSetCB(&s_gap_broadcaster_cbs);
    tmos_set_event(s_bridge_task_id, UBRIDGE_START_DEVICE_EVT);
}

/*********************************************************************
 * TMOS EVENT PROCESSOR
 *********************************************************************/
uint16_t UART_Bridge_ProcessEvent(uint8_t task_id, uint16_t events)
{
    (void)task_id;

    if (events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;
        while ((pMsg = tmos_msg_receive(s_bridge_task_id)) != NULL)
        {
            tmos_event_hdr_t *pHdr = (tmos_event_hdr_t *)pMsg;
            if (pHdr->event == GAP_MSG_EVENT)
            {
                gapRoleEvent_t *pEvent = (gapRoleEvent_t *)pMsg;
                if (pEvent->gap.opcode == GAP_PHY_UPDATE_EVENT)
                {
                    s_conn_phy_tx = pEvent->linkPhyUpdate.connTxPHYS;
                    s_conn_phy_rx = pEvent->linkPhyUpdate.connRxPHYS;
                }
            }
            tmos_msg_deallocate(pMsg);
        }
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & UBRIDGE_START_DEVICE_EVT)
    {
        GAPRole_PeripheralStartDevice(s_bridge_task_id, &s_gap_bond_mgr_cbs, &s_gap_peripheral_cbs);
        return (events ^ UBRIDGE_START_DEVICE_EVT);
    }

    if (events & UBRIDGE_PARAM_UPDATE_EVT)
    {
        if (s_conn_handle != GAP_CONNHANDLE_INIT)
        {
            GAPRole_PeripheralConnParamUpdateReq(s_conn_handle,
                                                 DEFAULT_DESIRED_MIN_CONN_INTERVAL,
                                                 DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                                                 DEFAULT_DESIRED_SLAVE_LATENCY,
                                                 DEFAULT_DESIRED_CONN_TIMEOUT,
                                                 s_bridge_task_id);
        }
        return (events ^ UBRIDGE_PARAM_UPDATE_EVT);
    }

    if (events & UBRIDGE_PUMP_UART_TO_BLE_EVT)
    {
        UART_Bridge_Poll();
        return (events ^ UBRIDGE_PUMP_UART_TO_BLE_EVT);
    }

    if (events & UBRIDGE_BTN_UNLOCK_EVT)
    {
        s_btn_locked = 0;
        return (events ^ UBRIDGE_BTN_UNLOCK_EVT);
    }

    if (events & UBRIDGE_PHY_UPDATE_EVT)
    {
        if (s_conn_handle != GAP_CONNHANDLE_INIT)
        {
            GAPRole_UpdatePHY(s_conn_handle, 0,
                              GAP_PHY_BIT_LE_2M,
                              GAP_PHY_BIT_LE_2M,
                              0);
        }
        return (events ^ UBRIDGE_PHY_UPDATE_EVT);
    }

    return 0;
}

/*********************************************************************
 * BRIDGE POLL (UART0 -> BLE NOTIFICATION)
 * Batch-pumps packets into BLE controller TX FIFO for maximum speed
 *********************************************************************/
void UART_Bridge_Poll(void)
{
    int avail = uart0_available();
    if (avail <= 0) return;

    /* If BLE is disconnected or notifications off, discard to avoid buffer stall */
    if (s_conn_handle == GAP_CONNHANDLE_INIT || !ble_uart_notify_is_ready(s_conn_handle))
    {
        uint8_t discard[64];
        uart0_read(discard, (avail > 64) ? 64 : avail);
        return;
    }

    uint16_t mtu = ATT_GetMTU(s_conn_handle);
    if (mtu < 23) mtu = 23;
    uint16_t max_payload = mtu - 3;
    if (max_payload == 0 || max_payload > 244) max_payload = 20;

    /* Batch-pump up to 8 packets per cycle */
    for (int p = 0; p < 8; p++)
    {
        avail = uart0_available();
        if (avail <= 0) break;

        uint16_t chunk = (avail > max_payload) ? max_payload : (uint16_t)avail;

        attHandleValueNoti_t noti;
        noti.len = chunk;
        noti.pValue = GATT_bm_alloc(s_conn_handle, ATT_HANDLE_VALUE_NOTI, noti.len, NULL, 0);

        if (noti.pValue == NULL)
        {
            /* BLE TX queue full; leave remainder in UART0 ring buffer (hardware RTS throttles sender) */
            break;
        }

        int read_len = uart0_read(noti.pValue, (int)chunk);
        if (read_len > 0)
        {
            noti.len = (uint16_t)read_len;
            bStatus_t status = ble_uart_notify(s_conn_handle, &noti, 0);
            if (status == SUCCESS)
            {
                s_stat_uart_to_ble += (uint32_t)read_len;
            }
            else
            {
                GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
                break;
            }
        }
        else
        {
            GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
            break;
        }
    }
}

/*********************************************************************
 * BOOT BUTTON DIAGNOSTICS (PB22)
 * Pressing BOOT prints current link status to UART0
 *********************************************************************/
void UART_Bridge_CheckButton(void)
{
    uint8_t curr = (GPIOB_ReadPortPin(BOOT_BUTTON_PIN) != 0);

    if (s_prev_btn_state == 1 && curr == 0 && !s_btn_locked)
    {
        s_btn_locked = 1;
        tmos_start_task(s_bridge_task_id, UBRIDGE_BTN_UNLOCK_EVT, MS1_TO_SYSTEM_TIME(250));
        s_stat_btn_presses++;

        uint8_t mac[6] = {0};
        GetMACAddress(mac);

        int n;
        if (s_conn_handle != GAP_CONNHANDLE_INIT)
        {
            n = snprintf(s_report_buf, sizeof(s_report_buf),
                         "\r\n--- [CH592 BLE-UART Bridge Status] ---\r\n"
                         " BLE Link : CONNECTED (Handle 0x%04X, MTU %u, Noti %s, PHY %s)\r\n"
                         " UART0    : Baud %u, HW Flow Control (CTS/RTS ON)\r\n"
                         " Pins     : TX=PB7, RX=PB4, CTS=PB0, RTS=PB6\r\n"
                         " Device   : CH592_BLE_UART (%02X:%02X:%02X:%02X:%02X:%02X)\r\n"
                         " Bridged  : UART->BLE: %lu B | BLE->UART: %lu B (Clicks: %lu)\r\n"
                         "--------------------------------------\r\n",
                         s_conn_handle,
                         ATT_GetMTU(s_conn_handle),
                         ble_uart_notify_is_ready(s_conn_handle) ? "ON" : "OFF",
                         (s_conn_phy_tx == 2) ? "2 Mbps" : "1 Mbps",
                         DEFAULT_UART_BAUD_RATE,
                         mac[5], mac[4], mac[3], mac[2], mac[1], mac[0],
                         (unsigned long)s_stat_uart_to_ble,
                         (unsigned long)s_stat_ble_to_uart,
                         (unsigned long)s_stat_btn_presses);

            if (s_conn_phy_tx == 1)
            {
                GAPRole_UpdatePHY(s_conn_handle, 0,
                                  GAP_PHY_BIT_LE_2M,
                                  GAP_PHY_BIT_LE_2M,
                                  0);
            }
        }
        else
        {
            n = snprintf(s_report_buf, sizeof(s_report_buf),
                         "\r\n--- [CH592 BLE-UART Bridge Status] ---\r\n"
                         " BLE Link : ADVERTISING (Nordic NUS: 6E400001-...)\r\n"
                         " UART0    : Baud %u, HW Flow Control (CTS/RTS ON)\r\n"
                         " Pins     : TX=PB7, RX=PB4, CTS=PB0, RTS=PB6\r\n"
                         " Device   : CH592_BLE_UART (%02X:%02X:%02X:%02X:%02X:%02X)\r\n"
                         " Bridged  : UART->BLE: %lu B | BLE->UART: %lu B (Clicks: %lu)\r\n"
                         "--------------------------------------\r\n",
                         DEFAULT_UART_BAUD_RATE,
                         mac[5], mac[4], mac[3], mac[2], mac[1], mac[0],
                         (unsigned long)s_stat_uart_to_ble,
                         (unsigned long)s_stat_ble_to_uart,
                         (unsigned long)s_stat_btn_presses);
        }

        if (n > 0)
        {
            if (n >= (int)sizeof(s_report_buf)) n = sizeof(s_report_buf) - 1;
            uart0_write(s_report_buf, n);
            uart0_flush();
        }
    }

    s_prev_btn_state = curr;
}
