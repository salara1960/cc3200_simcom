#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "prcm.h"

//*****************************************************************************
void PinMuxConfig(void)
{
    //
    // Enable Peripheral Clocks 
    //
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);

    //Configure PIN_55 for UART0 UART0_TX
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);
    // Configure PIN_57 for UART0 UART0_RX
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);

    MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);// sim5320 RESET
    MAP_GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_OUT);
    MAP_PinTypeGPIO(PIN_18, PIN_MODE_0, false);// sim5320 PWR_KEY
    MAP_GPIODirModeSet(GPIOA3_BASE, 0x10, GPIO_DIR_MODE_OUT);

    // Configure PIN_58 for GPIOInput
    MAP_PinTypeGPIO(PIN_58, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, 0x8, GPIO_DIR_MODE_IN);

    // Configure PIN_59 for GPIOInput =====================  VIO  ========================
    MAP_PinTypeGPIO(PIN_59, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, 0x10, GPIO_DIR_MODE_IN);

    // Configure PIN_64 for GPIOOutput (GP09 RED)
    MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);

    // Configure PIN_01 for GPIOOutput (GP10 ORANGE)
    MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);
    //
    // Configure PIN_02 for GPIOOutput (GP11 GREEN)
    MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x8, GPIO_DIR_MODE_OUT);

    //************************************************
    // Configure PIN_07 for UART1 UART1_TX
    MAP_PinTypeUART(PIN_07, PIN_MODE_5);
    // Configure PIN_08 for UART1 UART1_RX
    MAP_PinTypeUART(PIN_08, PIN_MODE_5);
    //************************************************

    // Configure PIN_63 for MCASP0 McAFSX - PCM_SYNC
    MAP_PinTypeI2S(PIN_63, PIN_MODE_7);
    // Configure PIN_50 for
    MAP_PinTypeI2S(PIN_50, PIN_MODE_4);//data 0 (from 5320 to m4)
    // Configure PIN_60 for
    MAP_PinTypeI2S(PIN_60, PIN_MODE_6);//data 1 (from m4 to 5320)
    // Configure PIN_53 for MCASP0 McACLK - PCM_CLK
    MAP_PinTypeI2S(PIN_53, PIN_MODE_2);

}
