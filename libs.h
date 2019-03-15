#ifndef __func_H__
#define __func_H__

#include "hdr.h"

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_mcasp.h"
#include "hw_ints.h"
#include "hw_apps_rcm.h"

#include "prcm.h"
#include "pin.h"
#include "uart.h"
#include "rom.h"
#include "rom_map.h"
#include "debug.h"
#include "udma.h"
#include "interrupt.h"
#include "i2s.h"
#include "gpio.h"

#include "common.h"

#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
    #include "osi.h"
#endif


#ifdef __cplusplus
extern "C"
{
#endif

//************************************************************************
//************************************************************************
//************************************************************************
#define UART_BAUD_RATE    115200
#define SYSCLK            80000000
#define CONSOLE           UARTA0_BASE
#define CONSOLE_PERIPH    PRCM_UARTA0
#define UartGetChar()     MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)    MAP_UARTCharPut(CONSOLE,c)
#define GSM               UARTA1_BASE
#define GSM_PERIPH        PRCM_UARTA1
#define GSMUartGetChar()  MAP_UARTCharGet(GSM)
#define GSMUartPutChar(c) MAP_UARTCharPut(GSM,c)
#define UART_IF_BUFFER    512
#define IS_SPACE(x) (x == 32 ? 1 : 0)
//
#define WFIFO_NUMDMA_WORDS  1
#define MCASP_DMA_PORT_ADDR 0x4401E200
#define I2S_ACT_SLOT_EVEN   0x00000001
#define I2S_ACT_SLOT_ODD    0x00000002
#define I2S_MODE_RX         0x01
#define I2S_MODE_TX         0x02
#define I2S_MODE_RX_TX      0x03
//
#define GPIO_LED1           9
#define GPIO_LED2           10
#define GPIO_LED3           11
#define GPIO_LED4           22
#define GPIO_LED5           28
//------------------------------------------------------------------------
typedef enum {
    SAMPLING_FREQ_8_KHZ    = 8000,
    SAMPLING_FREQ_11_X_KHZ = 11025,
    SAMPLING_FREQ_16_KHZ   = 16000,
    SAMPLING_FREQ_22_X_KHZ = 22050,
    SAMPLING_FREQ_32_KHZ   = 32000,
    SAMPLING_FREQ_44_1_KHZ = 44100,
    SAMPLING_FREQ_48_KHZ   = 48000
} tESamplFreq;
//
typedef enum {
    CH_MONO   = 1,
    CH_STEREO = 2
} tENumChannels;
//
typedef enum {
    BIT_8_PCM  = 8,
    BIT_16_PCM = 16,
    BIT_24_PCM = 24,
    BIT_32_PCM = 32
} tESampleLen;
//------------------------------------------------------------------------
typedef enum {
    NO_LED,
    LED1 = 0x1, /* RED LED D7/GP9/Pin64 */
    LED2 = 0x2, /* ORANGE LED D6/GP10/Pin1 */
    LED3 = 0x4, /* GREEN LED D5/GP11/Pin2 */
    LED4 = 0x8,  /* ????????????/GP22/Pin15 */
    LED5 = 0x10  /* ????????????/GP28/Pin18 */
} ledEnum;
//
typedef enum {
    NO_LED_IND = NO_LED,
    MCU_SENDING_DATA_IND = LED1,
    MCU_ASSOCIATED_IND,       /* Device associated to an AP */
    MCU_IP_ALLOC_IND,         /* Device acquired an IP */
    MCU_SERVER_INIT_IND,      /* Device connected to remote server */
    MCU_CLIENT_CONNECTED_IND, /* Any client connects to device */
    MCU_ON_IND,               /* Device SLHost invoked successfully */
    MCU_EXECUTE_SUCCESS_IND,  /* Task executed sucessfully */
    MCU_EXECUTE_FAIL_IND,     /* Task execution failed */
    MCU_RED_LED_GPIO,         /* GP09 for LED RED as per LP 3.0 */
    MCU_ORANGE_LED_GPIO,      /* GP10 for LED ORANGE as per LP 3.0 */
    MCU_GREEN_LED_GPIO,       /* GP11 for LED GREEN as per LP 3.0 */
    MCU_LED4_GPIO,            /* GP22 for LED ??? as per LP ??? */
    MCU_LED5_GPIO,            /* GP28 for LED ??? as per LP ??? */
    MCU_ALL_LED_IND
} ledNames;
//------------------------------------------------------------------------------------------------------

extern unsigned char g_ucUARTBuffer[];

//------------------------------------------------------------------------------------------------------
//-------------------   modified functions CC3200SDK_1.3.0   -------------------------------------------
//------------------------------------------------------------------------------------------------------

//----------------------------------------   UART   ----------------------------------------------------

extern void DispatcherUARTConfigure();
extern void DispatcherUartSendPacket(unsigned char *inBuff, unsigned short usLength);
extern int GetCmd(char *pcBuffer, unsigned int uiBufLen);
extern void InitTerm(unsigned char prn);
extern void ClearTerm();
extern void Message(const char *format);
extern void GSM_TX_STR(char *str);
extern char GSM_RX_BYTE();
extern void Error(char *format,...);
extern int TrimSpace(char * pcInput);
extern int Report(const char *format, ...);

//-----------------------------------------   I2S   ----------------------------------------------------

extern void AudioRendererInit();
extern void AudioCapturerInit();
extern void AudioInit();
extern void McASPInit();
extern void AudioRendererSetupCPUMode(void (*pfnAppCbHndlr)(void));
extern void AudioRendererSetupDMAMode(void (*pfnAppCbHndlr)(void),
                                      unsigned long ulCallbackEvtSamples);
extern void AudioCapturerSetupCPUMode(void (*pfnAppCbHndlr)(void));
extern long AudioCapturerSetupDMAMode(void (*pfnAppCbHndlr)(void),
                                      unsigned long ulCallbackEvtSamples);
extern long AudioSetupDMAMode(void (*pfnAppCbHndlr)(void),
                              unsigned long ulCallbackEvtSamples,
                              unsigned char RxTx);
extern unsigned int* AudioRendererGetDMADataPtr();
extern unsigned int* AudioCapturerGetDMADataPtr();
extern void AudioCapturerConfigure(int iSamplingFrequency,
                                   short sNumOfChannels,
                                   short sBitsPerSample);
extern void AudioRendererConfigure(int iSamplingFrequency,
                                   short sNumOfChannels,
                                   short sBitsPerSample);
extern void AudioRendererStart();
extern void AudioCapturerStart();
extern void AudioRendererStop();
extern void AudioRendererDeInit();
extern unsigned int BitClockConfigure(int iSamplingFrequency,
                                      short sNumOfChannels,
                                      short sBitsPerSample);
extern void AudioCaptureRendererConfigure(unsigned char bitsPerSample,
                                          unsigned short bitRate,
                                          unsigned char noOfChannels,
                                          unsigned char RxTx,
                                          unsigned char dma);
extern void NotClkFront();
extern void SetClkI2S(unsigned long ulI2CClkFreq);
extern void MyAudioCaptureRendererConfigure(unsigned long clkf,
                                            unsigned long i2smode,
                                            unsigned char bitsPerSample,
                                            unsigned short bitRate,
                                            unsigned char noOfChannels,
                                            unsigned char RxTx,
                                            unsigned char dm);
extern void Audio_Start(unsigned char RxTx);
extern void Audio_Stop();

//------------------------------------   GPIO   ---------------------------------------------------------

extern void GPIO_IF_GetPortNPin(unsigned char ucPin,
                                unsigned int *puiGPIOPort,
                                unsigned char *pucGPIOPin);

extern void GPIO_IF_ConfigureNIntEnable(unsigned int uiGPIOPort,
                                        unsigned char ucGPIOPin,
                                        unsigned int uiIntType,
                                        void (*pfnIntHandler)(void));
extern void GPIO_IF_Set(unsigned char ucPin,
                        unsigned int uiGPIOPort,
                        unsigned char ucGPIOPin,
                        unsigned char ucGPIOValue);

extern unsigned char GPIO_IF_Get(unsigned char ucPin,
                                 unsigned int uiGPIOPort,
                                 unsigned char ucGPIOPin);
extern void GPIO_IF_LedConfigure(unsigned char ucPins);
extern void GPIO_IF_LedOn(char ledNum);
extern void GPIO_IF_LedOff(char ledNum);
extern unsigned char GPIO_IF_LedStatus(unsigned char ucGPIONum);
extern void GPIO_IF_LedToggle(unsigned char ucLedNum);

//------------------------------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif



