#include "libs.h"


static unsigned long __Errorlog;
unsigned int ilen = 1;

//*****************************************************************************
void InitTerm(unsigned char prn)
{
#ifndef NOTERM

    MAP_PRCMPeripheralReset(PRCM_UARTA0);
    MAP_UARTConfigSetExpClk(CONSOLE,
                            MAP_PRCMPeripheralClockGet(CONSOLE_PERIPH),
                            UART_BAUD_RATE,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    if (prn) Message("\n\r\n\rInit UARTA0 (CONSOLE) done.\n\r");
#endif

#ifdef GSM

    MAP_PRCMPeripheralReset(PRCM_UARTA1);
    MAP_UARTConfigSetExpClk(GSM,
                            MAP_PRCMPeripheralClockGet(GSM_PERIPH),
                            UART_BAUD_RATE,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    if (prn) Message("Init UARTA1 (GSM) done.\n\r");
#endif

  __Errorlog = 0;
}
//*****************************************************************************
void Message(const char *str)
{
#ifndef NOTERM
    if (!str) return;

    while (*str != '\0') {
        MAP_UARTCharPut(CONSOLE, *str++);
    }
#endif
}
//--------------------------------------------------------------------------
void GSM_TX_STR(const char *str)
{
    if (!str) return;

    while (*str != '\0') {
        MAP_UARTCharPutNonBlocking(GSM, *str++);
    }
}
//-----------------------------------------------------------------------
char GSM_RX_BYTE()
{
char cChar = 0x00;

    if (MAP_UARTCharsAvail(GSM) != false) cChar = MAP_UARTCharGetNonBlocking(GSM);

    return cChar;
}
//*****************************************************************************
void ClearTerm()
{
    Message("\33[2J\r");
}
//*****************************************************************************
void Error(char *pcFormat, ...)
{
#ifndef NOTERM
    char  cBuf[512];
    va_list list;
    va_start(list, pcFormat);
    vsnprintf(cBuf, 512, pcFormat, list);
    Message(cBuf);
#endif
    __Errorlog++;
}
//*****************************************************************************
int GetCmd(char *pcBuffer, unsigned int uiBufLen)
{
char cChar;
int iLen = 0;

    while (MAP_UARTCharsAvail(CONSOLE) == false) {
#if defined(USE_FREERTOS) || defined(USE_TI_RTOS)
        osi_Sleep(1);
#endif
    }
    cChar = MAP_UARTCharGetNonBlocking(CONSOLE);

    MAP_UARTCharPut(CONSOLE, cChar);
    iLen = 0;

    while ( (cChar != '\r') && (cChar !='\n') ) {
        if (iLen >= uiBufLen) return -1;
        if (cChar != '\b') {
            *(pcBuffer + iLen) = cChar;
            iLen++;
        } else {
            if (iLen) iLen--;
        }
        while (MAP_UARTCharsAvail(CONSOLE) == false) {
#if defined(USE_FREERTOS) || defined(USE_TI_RTOS)
            osi_Sleep(1);
#endif
        }
        cChar = MAP_UARTCharGetNonBlocking(CONSOLE);
        MAP_UARTCharPut(CONSOLE, cChar);
    }

    *(pcBuffer + iLen) = '\0';
    Report("\n\r");

    return iLen;
}
//*****************************************************************************
int TrimSpace(char * pcInput)
{
char *endStr, *strData = pcInput;
char index = 0;
size_t size = strlen(strData);

    if (!size) return 0;

    endStr = strData + size - 1;
    while (endStr >= strData && IS_SPACE(*endStr)) endStr--;
    *(endStr + 1) = '\0';

    while (*strData && IS_SPACE(*strData)) {
        strData++;
        index++;
    }
    memmove(pcInput, strData, strlen(strData) + 1);

    return strlen(pcInput);
}
//*****************************************************************************
int Report(const char *pcFormat, ...)
{
int iRet = 0;
#ifndef NOTERM

  char *pcTemp;
  int iSize = 512;
  va_list list;

  char *pcBuff = (char *)malloc(iSize);
  if(!pcBuff) return -1;

  while(1) {
      va_start(list, pcFormat);
      iRet = vsnprintf(pcBuff, iSize, pcFormat, list);
      va_end(list);
      if(iRet > -1 && iRet < iSize) {
          break;
      } else {
          iSize <<= 1;
          if (!(pcTemp = realloc(pcBuff, iSize))) {
              Message("Could not reallocate memory\n\r");
              iRet = -1;
              break;
          } else {
              pcBuff = pcTemp;
          }

      }
  }
  Message(pcBuff);
  free(pcBuff);

#endif
  return iRet;
}
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
unsigned int* AudioRendererGetDMADataPtr()
{
    return (unsigned int *)(I2S_TX_DMA_PORT);
}
//*****************************************************************************
unsigned int* AudioCapturerGetDMADataPtr()
{
    return (unsigned int *)(I2S_RX_DMA_PORT);
}
//*****************************************************************************
void AudioInit()
{
    // Initialising the McASP
    MAP_PRCMPeripheralClkEnable(PRCM_I2S, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_I2S);
}
//*****************************************************************************
long AudioSetupDMAMode(void (*pfnAppCbHndlr)(void),
                       unsigned long ulCallbackEvtSamples,
                       unsigned char RxTx)
{
    MAP_I2SIntEnable(I2S_BASE, I2S_INT_XDATA);
#ifdef USE_TIRTOS
    long lRetVal = -1;
    lRetVal = osi_InterruptRegister(INT_I2S, pfnAppCbHndlr, INT_PRIORITY_LVL_1);
    ASSERT_ON_ERROR(lRetVal);
#else
    MAP_I2SIntRegister(I2S_BASE, pfnAppCbHndlr);
#endif

    if (RxTx == I2S_MODE_RX_TX) MAP_I2SRxFIFOEnable(I2S_BASE, 8, 1);

    if (RxTx & I2S_MODE_TX) MAP_I2STxFIFOEnable(I2S_BASE, 8, 1);

    return SUCCESS;
}
//*****************************************************************************
void AudioCaptureRendererConfigure(unsigned char bitsPerSample,
                                   unsigned short bitRate,
                                   unsigned char noOfChannels,
                                   unsigned char RxTx,
                                   unsigned char dma)
{
unsigned long bitClk = bitsPerSample * bitRate * noOfChannels;

    if (dma) {
        if(bitsPerSample == 16) {
            PRCMI2SClockFreqSet(512000);
            I2SConfigSetExpClk(I2S_BASE, 512000, bitClk, I2S_SLOT_SIZE_16 | I2S_PORT_DMA);
        }
    }

    if (RxTx == I2S_MODE_RX_TX)
        MAP_I2SSerializerConfig(I2S_BASE, I2S_DATA_LINE_1, I2S_SER_MODE_RX, I2S_INACT_LOW_LEVEL);

    if (RxTx & I2S_MODE_TX)
        MAP_I2SSerializerConfig(I2S_BASE, I2S_DATA_LINE_0, I2S_SER_MODE_TX, I2S_INACT_LOW_LEVEL);

}
//*****************************************************************************
void NotClkFront()
{
unsigned long tx_rg = 0, tx_rgn = 0;

    tx_rg = tx_rgn = HWREG(I2S_BASE + MCASP_O_ACLKXCTL);
    if (tx_rg & MCASP_ACLKXCTL_CLKXP) tx_rgn &= 0xffffff7f;
                                 else tx_rgn |= 0x00000080;
    HWREG(I2S_BASE + MCASP_O_ACLKXCTL) = tx_rgn;
}
//*****************************************************************************
void SetClkI2S(unsigned long ulI2CClkFreq)
{
unsigned long long ullDiv;
unsigned short usInteger, usFrac;

    ullDiv = (((unsigned long long)240000000 * 65536) / ulI2CClkFreq);
    usInteger = (ullDiv / 65536);
    usFrac    = (ullDiv % 65536);

    HWREG(ARCM_BASE + APPS_RCM_O_MCASP_FRAC_CLK_CONFIG0) = ((usInteger & 0x3FF) << 16 | usFrac);
}
//*****************************************************************************
void MyAudioCaptureRendererConfigure(unsigned long clkf,
                                     unsigned long i2smode,
                                     unsigned char bitsPerSample,
                                     unsigned short bitRate,
                                     unsigned char noOfChannels,
                                     unsigned char RxTx,
                                     unsigned char dm)
{
unsigned long sz, prt, rx_fmt, tx_fmt, tmp;
unsigned long bitClk = bitsPerSample * bitRate * noOfChannels;

    switch (bitsPerSample) {
        case 16: sz = I2S_SLOT_SIZE_16; break;
        case 24: sz = I2S_SLOT_SIZE_24; break;
            default : sz = I2S_SLOT_SIZE_8;
    }
    if (!dm) prt = I2S_PORT_CPU; else prt = I2S_PORT_DMA;

    SetClkI2S(clkf);
    I2SConfigSetExpClk(I2S_BASE, clkf, bitClk, sz | prt | i2smode);

    NotClkFront();

    if(RxTx == I2S_MODE_RX_TX)
        I2SSerializerConfig(I2S_BASE, I2S_DATA_LINE_0, I2S_SER_MODE_RX, I2S_INACT_LOW_LEVEL);
    if(RxTx & I2S_MODE_TX)
        I2SSerializerConfig(I2S_BASE, I2S_DATA_LINE_1, I2S_SER_MODE_TX, I2S_INACT_LOW_LEVEL);

    I2SRxActiveSlotSet(I2S_BASE, I2S_ACT_SLOT_ODD | I2S_ACT_SLOT_EVEN);

    rx_fmt = HWREG(I2S_BASE + MCASP_O_RXFMT);
    tx_fmt = HWREG(I2S_BASE + MCASP_O_TXFMT);

    tmp = ~MCASP_RXFMT_RDATDLY_M;
    rx_fmt &= tmp;
    HWREG(I2S_BASE + MCASP_O_RXFMT) = rx_fmt;

    tmp = ~MCASP_TXFMT_XDATDLY_M;
    tx_fmt &= tmp;
    HWREG(I2S_BASE + MCASP_O_TXFMT) = tx_fmt;

    rx_fmt = HWREG(I2S_BASE + MCASP_O_RXFMT);
    tx_fmt = HWREG(I2S_BASE + MCASP_O_TXFMT);

}
//*****************************************************************************
void Audio_Start(unsigned char RxTx)
{
    if (RxTx == I2S_MODE_RX_TX) I2SEnable(I2S_BASE, I2S_MODE_TX_RX_SYNC);
    else
    if (RxTx & I2S_MODE_TX) I2SEnable(I2S_BASE, I2S_MODE_TX_ONLY);
}
//*****************************************************************************
void Audio_Stop()
{
    I2SDisable(I2S_BASE);
}
//*****************************************************************************

//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
//****************************************************************************

static unsigned long ulReg[] = {
    GPIOA0_BASE,
    GPIOA1_BASE,
    GPIOA2_BASE,
    GPIOA3_BASE,
    GPIOA4_BASE
};
//
unsigned int g_uiLED1Port = 0, g_uiLED2Port = 0, g_uiLED3Port = 0, g_uiLED4Port = 0, g_uiLED5Port = 0;
unsigned char g_ucLED1Pin, g_ucLED2Pin, g_ucLED3Pin, g_ucLED4Pin, g_ucLED5Pin;
//****************************************************************************
static unsigned char GetPeripheralIntNum(unsigned int uiGPIOPort)
{
    switch(uiGPIOPort) {
       case GPIOA0_BASE:
          return INT_GPIOA0;
       case GPIOA1_BASE:
          return INT_GPIOA1;
       case GPIOA2_BASE:
          return INT_GPIOA2;
       case GPIOA3_BASE:
          return INT_GPIOA3;
       default:
          return INT_GPIOA0;
    }
}
//*****************************************************************************
void GPIO_IF_LedConfigure(unsigned char ucPins)
{

  if (ucPins & LED1) GPIO_IF_GetPortNPin(GPIO_LED1, &g_uiLED1Port, &g_ucLED1Pin);

  if (ucPins & LED2) GPIO_IF_GetPortNPin(GPIO_LED2, &g_uiLED2Port, &g_ucLED2Pin);

  if (ucPins & LED3) GPIO_IF_GetPortNPin(GPIO_LED3, &g_uiLED3Port, &g_ucLED3Pin);

  if (ucPins & LED4) GPIO_IF_GetPortNPin(GPIO_LED4, &g_uiLED4Port, &g_ucLED4Pin);

  if (ucPins & LED5) GPIO_IF_GetPortNPin(GPIO_LED5, &g_uiLED5Port, &g_ucLED5Pin);

}
//*****************************************************************************
void GPIO_IF_LedOn(char ledNum)
{
    switch (ledNum) {
        case MCU_LED5_GPIO:
            GPIO_IF_Set(GPIO_LED5, g_uiLED5Port, g_ucLED5Pin, 1);
        break;
        case MCU_LED4_GPIO:
            GPIO_IF_Set(GPIO_LED4, g_uiLED4Port, g_ucLED4Pin, 1);
        break;
        case MCU_ON_IND:
        case MCU_EXECUTE_SUCCESS_IND:
        case MCU_GREEN_LED_GPIO: /* Switch ON GREEN LED */
            GPIO_IF_Set(GPIO_LED3, g_uiLED3Port, g_ucLED3Pin, 1);
        break;
        case MCU_SENDING_DATA_IND:
        case MCU_EXECUTE_FAIL_IND:
        case MCU_ORANGE_LED_GPIO: /* Switch ON ORANGE LED */
            GPIO_IF_Set(GPIO_LED2, g_uiLED2Port, g_ucLED2Pin, 1);
        break;
        case MCU_ASSOCIATED_IND:
        case MCU_IP_ALLOC_IND:
        case MCU_SERVER_INIT_IND:
        case MCU_CLIENT_CONNECTED_IND:
        case MCU_RED_LED_GPIO: /* Switch ON RED LED */
            GPIO_IF_Set(GPIO_LED1, g_uiLED1Port, g_ucLED1Pin, 1);
        break;
        case MCU_ALL_LED_IND:/* Switch ON ALL LEDs LED */
            GPIO_IF_Set(GPIO_LED3, g_uiLED3Port, g_ucLED3Pin, 1);
            GPIO_IF_Set(GPIO_LED2, g_uiLED2Port, g_ucLED2Pin, 1);
            GPIO_IF_Set(GPIO_LED1, g_uiLED1Port, g_ucLED1Pin, 1);
        break;
                default: break;
    }
}
//*****************************************************************************
void GPIO_IF_LedOff(char ledNum)
{
    switch (ledNum) {
        case MCU_LED5_GPIO:
            GPIO_IF_Set(GPIO_LED5, g_uiLED5Port, g_ucLED5Pin, 0);
        break;
        case MCU_LED4_GPIO:
            GPIO_IF_Set(GPIO_LED4, g_uiLED4Port, g_ucLED4Pin, 0);
        break;
        case MCU_ON_IND:
        case MCU_EXECUTE_SUCCESS_IND:
        case MCU_GREEN_LED_GPIO:/* Switch OFF GREEN LED */
            GPIO_IF_Set(GPIO_LED3, g_uiLED3Port, g_ucLED3Pin, 0);
        break;
        case MCU_SENDING_DATA_IND:
        case MCU_EXECUTE_FAIL_IND:
        case MCU_ORANGE_LED_GPIO:/* Switch OFF ORANGE LED */
            GPIO_IF_Set(GPIO_LED2, g_uiLED2Port, g_ucLED2Pin, 0);
        break;
        case MCU_ASSOCIATED_IND:
        case MCU_IP_ALLOC_IND:
        case MCU_SERVER_INIT_IND:
        case MCU_CLIENT_CONNECTED_IND:
        case MCU_RED_LED_GPIO: /* Switch OFF RED LED */
            GPIO_IF_Set(GPIO_LED1, g_uiLED1Port, g_ucLED1Pin, 0);
        break;
        case MCU_ALL_LED_IND: /* Switch OFF ALL LEDs LED */
            GPIO_IF_Set(GPIO_LED3, g_uiLED3Port, g_ucLED3Pin, 0);
            GPIO_IF_Set(GPIO_LED2, g_uiLED2Port, g_ucLED2Pin, 0);
            GPIO_IF_Set(GPIO_LED1, g_uiLED1Port, g_ucLED1Pin, 0);
        break;
              default: break;
    }
}
//*****************************************************************************
unsigned char GPIO_IF_LedStatus(unsigned char ucGPIONum)
{
unsigned char ucLEDStatus;

    switch (ucGPIONum) {
        case MCU_GREEN_LED_GPIO:
            ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED3Port, g_ucLED3Pin);
        break;
        case MCU_ORANGE_LED_GPIO:
            ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED2Port, g_ucLED2Pin);
        break;
        case MCU_RED_LED_GPIO:
            ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED1Port, g_ucLED1Pin);
        break;
        case MCU_LED4_GPIO:
            ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED4Port, g_ucLED4Pin);
        break;
        case MCU_LED5_GPIO:
            ucLEDStatus = GPIO_IF_Get(ucGPIONum, g_uiLED5Port, g_ucLED5Pin);
        break;
            default: ucLEDStatus = 0;
  }

  return ucLEDStatus;
}
//*****************************************************************************
void GPIO_IF_LedToggle(unsigned char ucLedNum)
{
unsigned char ucLEDStatus = GPIO_IF_LedStatus(ucLedNum);

    if (ucLEDStatus == 1)
        GPIO_IF_LedOff(ucLedNum);
    else
        GPIO_IF_LedOn(ucLedNum);

}
//****************************************************************************
void GPIO_IF_GetPortNPin(unsigned char ucPin,
                         unsigned int *puiGPIOPort,
                         unsigned char *pucGPIOPin)
{
    *pucGPIOPin = 1 << (ucPin % 8);

    *puiGPIOPort = (ucPin / 8);
    *puiGPIOPort = ulReg[*puiGPIOPort];
}
//****************************************************************************
void GPIO_IF_ConfigureNIntEnable(unsigned int uiGPIOPort,
                                 unsigned char ucGPIOPin,
                                 unsigned int uiIntType,
                                 void (*pfnIntHandler)(void))
{
    MAP_GPIOIntTypeSet(uiGPIOPort, ucGPIOPin, uiIntType);

    // Register Interrupt handler
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
    // USE_TIRTOS: if app uses TI-RTOS (either networking/non-networking)
    // USE_FREERTOS: if app uses Free-RTOS (either networking/non-networking)
    // SL_PLATFORM_MULTI_THREADED: if app uses any OS + networking(simplelink)
    osi_InterruptRegister(GetPeripheralIntNum(uiGPIOPort), pfnIntHandler, INT_PRIORITY_LVL_1);
#else
    MAP_IntPrioritySet(GetPeripheralIntNum(uiGPIOPort), INT_PRIORITY_LVL_1);
    MAP_GPIOIntRegister(uiGPIOPort,pfnIntHandler);
#endif
    // Enable Interrupt
    MAP_GPIOIntClear(uiGPIOPort, ucGPIOPin);
    MAP_GPIOIntEnable(uiGPIOPort, ucGPIOPin);
}
//****************************************************************************
void GPIO_IF_Set(unsigned char ucPin,
                 unsigned int  uiGPIOPort,
                 unsigned char ucGPIOPin,
                 unsigned char ucGPIOValue)
{

    // Set the corresponding bit in the bitmask
    ucGPIOValue = ucGPIOValue << (ucPin % 8);

    // Invoke the API to set the value
    MAP_GPIOPinWrite(uiGPIOPort, ucGPIOPin, ucGPIOValue);
}
//****************************************************************************
unsigned char GPIO_IF_Get(unsigned char ucPin,
                          unsigned int  uiGPIOPort,
                          unsigned char ucGPIOPin)
{
unsigned char ucGPIOValue;
long lGPIOStatus;

    // Invoke the API to Get the value
    lGPIOStatus =  MAP_GPIOPinRead(uiGPIOPort, ucGPIOPin);

    // Set the corresponding bit in the bitmask
    ucGPIOValue = lGPIOStatus >> (ucPin % 8);

    return ucGPIOValue;
}
//*****************************************************************************
