/*********************************************************************
 * File Name          : uart_bridge_main.c
 * Description        : Main entry point for CH592 BLE-UART Bridge.
 *********************************************************************/

#include "CONFIG.h"
#include "CH59x_common.h"
#include "CH59x_sys.h"
#include "HAL.h"
#include "uart_bridge.h"

/* BLE protocol stack memory buffer (aligned to 4 bytes) */
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if (defined(BLE_SNV)) && (BLE_SNV == TRUE)
uint8_t BondSysFlag = 0;
#endif

/*********************************************************************
 * @fn      Main_Circulation
 *********************************************************************/
__HIGH_CODE
void Main_Circulation(void)
{
    while (1)
    {
        /* Process TMOS timers and BLE tasks */
        TMOS_SystemProcess();

        /* High-throughput UART0 -> BLE notification pump */
        UART_Bridge_Poll();

        /* Check PB22 BOOT button for status report */
        UART_Bridge_CheckButton();
    }
}

/*********************************************************************
 * @fn      main
 *********************************************************************/
int main(void)
{
    /* Set system clock to 60 MHz for maximum throughput */
    SetSysClock(CLK_SOURCE_PLL_60MHz);

    /* Initialize hardware abstraction layer */
    HAL_Init();

    /* Initialize BLE core protocol stack */
    CH59x_BLEInit();

    /* Initialize BLE to UART bridge */
    UART_Bridge_Init();

    /* Enter main polling loop */
    Main_Circulation();

    return 0;
}
