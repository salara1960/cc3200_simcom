#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "hw_shamd5.h"
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_udma.h"
#include "shamd5.h"
#include "interrupt.h"
#include "utils.h"
#include "pin.h"
#include "uart.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "timer.h"
#include "i2s.h"
#include "udma.h"
#include "gpio.h"
#include "utils.h"

#include "hw_mcasp.h"

#include "simplelink.h"
#include "netcfg.h"
#include "wlan.h"
#include "protocol.h"
#include "driver.h"
#include "netapp.h"

#include "osi.h"

#include "common.h"
#include "timer_if.h"
#include "libs.h"
#include "smartconfig.h"
#include "pinmux.h"
#include "circ_buff.h"

#include <http/client/httpcli.h>
#include <http/client/common.h>


#undef DMA_USED
#define DIRECT
#define PACK_COUNT_PRN
#define SET_LOAD
#define SET_SAVE

#define SSID_FILE_NAME      "www/ap.conf"
#define AUTH_FILE_NAME      "/sys/shadow.conf"
#define DEF_LOGIN		    "def_login"
#define DEF_PASSWD		    "def_passwd"
#define DEF_SECURE_KEY	    "abracadabra"
#define APPLICATION_NAME    "GSM"
#define APPLICATION_VERSION "2.9"
#define AP_SSID_LEN_MAX     (33)
#define ROLE_INVALID        (-5)

#define LED_STRING          "LED"
#define LED1_STRING         "LED1_"
#define LED2_STRING         ",LED2_"
#define LED_ON_STRING       "ON"
#define LED_OFF_STRING      "OFF"

#define OOB_TASK_PRIORITY   (1)

#define OSI_STACK_SIZE      (4096)

#define SH_GPIO_3           (3)  /* P58 - Device Mode */
#define SH_GPIO_4           (4)  /* P59 - VIO */
#define AC_TIMEOUT_COUNT    (50)   /* 5 Sec ??? */

#define UartGetChar()       MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)      MAP_UARTCharPut(CONSOLE,c)
#define GSMUartGetChar()    MAP_UARTCharGet(GSM)
#define GSMUartPutChar(c)   MAP_UARTCharPut(GSM,c)

#define GSM_MAX_BUFFER		512
#define	MAX_LP_LEN			32

#define IP_ADDR             0xc0a80002 /* 192.168.0.2 */
#define PORT_CTL            5000
#define PORT_TCP            5001
#define PORT_RTP            5002
#define BUF_SIZE            GSM_MAX_BUFFER	//1400

#define PACKET_SIZE         160
#define RECORD_BUFFER_SIZE  25 * PACKET_SIZE//24
#define PLAY_BUFFER_SIZE    50 * PACKET_SIZE//64
#define HULF_BUFFER			25 * PACKET_SIZE//32
#ifndef DIRECT
    #define RECV_BUFFER_SIZE	8
    #define RECV_BUFFER_SIZE_BT	RECV_BUFFER_SIZE*PACKET_SIZE
    #define RECV_BUFFER_MASK	RECV_BUFFER_SIZE - 1
#endif
#define INVALID_CLIENT_ADDRESS	0x00000000 //Invalid clientIP
#define ALAW				    0xD5
#define ULAW				    0xFF

#define LENS_ADR	            32

#define TOKEN_ARRAY_SIZE        6
#define STRING_TOKEN_SIZE       10
#define SCAN_TABLE_SIZE         20

#ifdef SET_LOAD
	#define PREFIX_BUFFER_DEF   "/tmp/tmp.bin"
	#define HOST_NAME_DEF       "192.168.0.103"
	#define HOST_PORT_DEF       (80)
	#define SIZE_80K            81920  /* Serial flash file size 80 KB max*/
	#define READ_SIZE           1450
	#define MAX_BUFF_SIZE       1460
	#define FILE_NAME           "/sys/tmp.bin"// File on the serial flash to be replaced
#endif
#define URL_LEN_MAX	61

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,       
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

	SOCKET_CREATE_ERROR = -0x7D0,
	BIND_ERROR = SOCKET_CREATE_ERROR - 1,
	LISTEN_ERROR = BIND_ERROR -1,
	SOCKET_OPT_ERROR = LISTEN_ERROR -1,
	CONNECT_ERROR = SOCKET_OPT_ERROR -1,
	ACCEPT_ERROR = CONNECT_ERROR - 1,
	SEND_ERROR = ACCEPT_ERROR -1,
	RECV_ERROR = SEND_ERROR -1,
	SOCKET_CLOSE_ERROR = RECV_ERROR -1,
	STATUS_CODE_MAX = -0xBB8,

	FILE_ALREADY_EXIST = -0x7D0,
	FILE_CLOSE_ERROR = FILE_ALREADY_EXIST - 1,
	FILE_NOT_MATCHED = FILE_CLOSE_ERROR - 1,
	FILE_OPEN_READ_FAILED = FILE_NOT_MATCHED - 1,
	FILE_OPEN_WRITE_FAILED = FILE_OPEN_READ_FAILED -1,
	FILE_READ_FAILED = FILE_OPEN_WRITE_FAILED - 1,
	FILE_WRITE_FAILED = FILE_READ_FAILED - 1,

	DEVICE_START_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,
	INVALID_HEX_STRING = DEVICE_START_FAILED - 1,
	TCP_RECV_ERROR = INVALID_HEX_STRING - 1,
	TCP_SEND_ERROR = TCP_RECV_ERROR - 1,
	FILE_NOT_FOUND_ERROR = TCP_SEND_ERROR - 1,
	INVALID_SERVER_RESPONSE = FILE_NOT_FOUND_ERROR - 1,
	FORMAT_NOT_SUPPORTED = INVALID_SERVER_RESPONSE - 1,
	FILE_OPEN_FAILED = FORMAT_NOT_SUPPORTED - 1,
	FILE_WRITE_ERROR = FILE_OPEN_FAILED - 1,
	INVALID_FILE = FILE_WRITE_ERROR - 1,
	SERVER_CONNECTION_FAILED = INVALID_FILE - 1,
	GET_HOST_IP_FAILED = SERVER_CONNECTION_FAILED  - 1


} e_AppStatusCodes;

#pragma pack(push,1)
typedef struct
{
	char buf[GSM_MAX_BUFFER];
	unsigned short rd_adr;
	unsigned short wr_adr;
	unsigned short len;
	unsigned done : 1;
	unsigned sym : 1;
	unsigned none : 6;
} s_uart;
typedef struct
{
	unsigned char cmd;       //command : 0 - set ssid,key,type
	char ssid[SSID_LEN_MAX]; //AP name (32 bytes)
	char key[SSID_LEN_MAX];  //password (32 bytes)
	unsigned char type;      //sec_type (0-open; 1-wep; 2-wpa,wpa2; 3-wps_pbc; 4-wps_pin; 5-wpa_ent)
} s_ctl_cmd;
typedef struct
{
	unsigned char ack; //AP name (1 byte) (0-OK, 1..255-ERROR)
} s_ctl_ack;
typedef struct
{
	unsigned char cmd;        //command : 5 rtp params	//1
	unsigned char cli_adr[4]; //IP adr of rtp client	//4
	unsigned short cli_port;  //rtp port of client//2
	unsigned short ti_port;   //rtp port of ti		//2
	unsigned char unused[57]; //unused
} s_ctl_udp;
typedef struct
{
	unsigned char cmd;     //command : 9 - set url for download file
	unsigned int len;
	char url[URL_LEN_MAX]; //full file name (61 bytes)
} s_ctl_bin;
typedef struct
{
	unsigned char cmd;       //command : 0x0b (11) - save login,passwd to file /sys/shadow.conf
	char login[MAX_LP_LEN];  //32
	char passwd[MAX_LP_LEN]; //32
	unsigned char none;
} s_ctl_auth;
typedef struct
{
	unsigned char cmd;             //command : 0x0c (12) - set ssid,key,type
	unsigned int epoch;            //4 bytes, epoch time in binary mode (without MD5)
	unsigned char str[MAX_LP_LEN]; //32 bytes, 'login'+'passwd'+'key'+'epoch' in MD5
	unsigned char mode;            //concat mode source string
	unsigned char auth_adr[4];     //ip adr of auth. tcp client (for port 5001)
	unsigned char none[24];        //not used, maybe any bytes or zero
} s_ctl_md5;

#pragma pack(pop)

typedef struct UDPSocket
{
    int iSockDesc;             //Socket FD
    struct sockaddr_in Server; //Address to Bind - INADDR_ANY for Local IP
    struct sockaddr_in Client; //Client Address or Multicast Group
    int iServerLength;
    int iClientLength;
} tUDPSocket;
#pragma pack(push,1)
typedef struct
{
	unsigned char version;
	unsigned char codec;
	unsigned short seq_num;
	unsigned int time_stamp;
	unsigned int ssrc;
} s_rtp_hdr;
typedef struct
{
	s_rtp_hdr hdr;
	unsigned char data[PACKET_SIZE];
} s_packet;
typedef struct
{
	unsigned char data[PACKET_SIZE];
} s_recv_buf;

#pragma pack(pop)

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static void TCPTask(void *pvParameters);
static void CTLTask(void *pvParameters);
static void UDPTask(void *pvParameters);
static void BroadCastTask(void *pvParameters);

unsigned char Check_VIO(unsigned char pr);
long WriteFileToDev(unsigned char * fname);
long ReadFileFromDev(unsigned char * fname);
int CheckMD5(s_ctl_md5 *md5);
void GSM_OFF(unsigned char pr);
long CreateUdpSrv(tUDPSocket *pSock, s_ctl_udp *udp_data);
void init_rtp(s_packet * pak);
#ifdef SET_LOAD
	long ReadAnyFile(unsigned char * fname, unsigned char noprn);
	static int FlushHTTPResponse(HTTPCli_Handle cli);
	static int GetData(HTTPCli_Handle cli);
	static long ServerFileDownload();
	static long frename(unsigned char * old_name, unsigned char * new_name);
	static void BINTask(void *pvParameters);
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
s_ctl_cmd ctl_cmd, ssid_ctl_cmd;
s_ctl_ack ctl_ack;
s_ctl_udp ctl_udp;
s_ctl_bin ctl_bin;
s_ctl_auth auth_ctl_cmd;
s_ctl_md5 md5_ctl_cmd;
static tBoolean rst_mode = 1; //default 1 - all
unsigned char pwr = 0, silent = ALAW, pack_count_prn = 0, restart = 0, write_ssid = 0, bin_start = 0, write_auth = 0;
long bin_result;
char SELF_ADR[LENS_ADR] = {0};
static unsigned char self_adr[4] = {0};
static unsigned char gw_adr[4] = {0};
char CLI_ADR[LENS_ADR] = {0};
static unsigned char cli_adr[4] = {0};
static unsigned char rtp_adr[4] = {0};
char SSID_NAME[SSID_LEN_MAX + 1] = {0};
char SECURITY_KEY[SSID_LEN_MAX + 1] = {0};
_u8 SECURITY_TYPE;
static unsigned char auth_tcp_adr[4] = {0};
unsigned int * uk_auth_tcp_adr = NULL;

OsiTaskHandle g_CTLTask = NULL ;
OsiTaskHandle g_TCPTask = NULL ;
OsiTaskHandle g_UDPTask = NULL ;
#ifndef DIRECT
	s_recv_buf recv_buf[RECV_BUFFER_SIZE];
#endif
static int iNewSockID = -1;
static unsigned long tmr_tcp_close = 0;
static unsigned long tmr_rtp_close = 0;
static unsigned char tcp_cli = 0, a_start = 0, at_start = 0, udp_go = 0, tcp_begin = 0;
static unsigned char WLanDone = 0, mode_done = 0, ctl_start = 0, ctl_cli = 0, hulf = 0, done_bc = 0, done_auth = 0;
static unsigned char tcp_close = 0, rtp_close = 0;
volatile unsigned long  g_ulStatus = 0; //SimpleLink Status
int fd_bc = -1;
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1] = {0}; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX] = {0}; //Connection BSSID
unsigned char POST_token[] = "__SL_P_ULD";
unsigned char GET_token[]  = "__SL_G_ULD";
int g_iSimplelinkRole = ROLE_INVALID;
unsigned int g_uiIpAddress = 0;
unsigned int g_uiIpMask = 0xffffff00;
unsigned int broadcast_adr = 0;
int broadcast_port = 5050;
const char * broadcast_name = "sim5320_cc3200";

unsigned char g_ucSSID[AP_SSID_LEN_MAX];
const char *crypto_type[7] = {"open","wep","wpa/wpa2","wps_pbc","wps_pin","wpa_ent","unknown"};
const char *new_name = "/sys/mcuimg.bin";

signed char g_cWlanSSID[AP_SSID_LEN_MAX];
signed char g_cWlanSecurityKey[50];
SlSecParams_t g_SecParams;
Sl_WlanNetworkEntry_t g_NetEntries[SCAN_TABLE_SIZE];
volatile unsigned char g_ucProfileAdded = 1;
volatile unsigned char g_ucProvisioningDone = 0;
unsigned char g_ucPriority = 0;
unsigned char g_ucConnectedToConfAP = 0;
char g_token_get [TOKEN_ARRAY_SIZE][STRING_TOKEN_SIZE] = {
    "__SL_G_US0",
    "__SL_G_US1",
    "__SL_G_US2",
    "__SL_G_US3",
    "__SL_G_US4",
    "__SL_G_US5"
};

unsigned long  g_ulDestinationIp;// = IP_ADDR;
unsigned int   g_uiPortNum0 = PORT_CTL;
unsigned int   g_uiPortNum  = PORT_TCP;
unsigned int   g_uiPortNum1 = PORT_RTP;

//-----------   BIN   ----------------
#ifdef SET_LOAD
	unsigned char 	g_buff[MAX_BUFF_SIZE + 1];
	long 			bytesReceived = 0;                // variable to store the file size
	char 			PREFIX_BUFFER[URL_LEN_MAX] = {0}; //"/tmp/tmp.bin";
	char 			HOST_NAME[URL_LEN_MAX] = {0};     //"192.168.0.103";
	unsigned int	HOST_PORT = HOST_PORT_DEF;
#endif
//---------------------------
s_uart from_uart, to_uart;

static volatile unsigned long g_ulBase;
static unsigned long tim = 0, tim2 = 0, tim2_def = 500;
static unsigned long varta = 0, one_sec = 0, tmr_gsm = 0;

tCircularBuffer *pRecordBuffer;
tCircularBuffer *pPlayBuffer;
int g_iSentCount = 0, g_iReceiveCount = 0;
unsigned long g_ulConnected = 0;
static unsigned char RecordPlay, rtp_enable = 0, ring = 0;
static unsigned long g_txd, g_rxd;
static unsigned long g_mdi = 0, g_stat = 0;
tUDPSocket g_UdpSock;

#if defined(ccs)
    extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
    extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
// Variable related to Connection status
//*****************************************************************************
volatile unsigned short g_usMCNetworkUstate = 0;
int g_uiSimplelinkRole = ROLE_INVALID;
unsigned int g_uiDeviceModeConfig = ROLE_STA; //default is STA mode
volatile unsigned char g_ucConnectTimeout = 0;

//****************************************************************************
//****************************************************************************
//****************************************************************************
long CreateUdpSrv(tUDPSocket *pSock, s_ctl_udp *udp_data)
{
long rt = -1, ti_udp, cli_udp;
_u32 l_ip, r_ip;

    if (!udp_data) {
    	ti_udp = PORT_RTP;
    	cli_udp = PORT_RTP;
    	memcpy(rtp_adr, cli_adr,4);
    	l_ip = SL_IPV4_VAL(self_adr[0], self_adr[1], self_adr[2], self_adr[3]); //localhost
    	r_ip = SL_IPV4_VAL(rtp_adr[0], rtp_adr[1], rtp_adr[2], rtp_adr[3]);     //remote host
    } else {
    	if (udp_data->ti_port > 0) ti_udp = udp_data->ti_port; else ti_udp = PORT_RTP;
    	cli_udp = udp_data->cli_port;
    	memcpy(rtp_adr, (unsigned char *)&udp_data->cli_adr[0], 4);
    	l_ip = SL_IPV4_VAL(self_adr[0], self_adr[1], self_adr[2], self_adr[3]); //localhost
    	r_ip = SL_IPV4_VAL(rtp_adr[0], rtp_adr[1], rtp_adr[2], rtp_adr[3]);     //remote host
    }

    if (pSock->iSockDesc>0) sl_Close(pSock->iSockDesc);
    pSock->iSockDesc = sl_Socket(AF_INET, SOCK_DGRAM, 0);
    pSock->Server.sin_family = AF_INET;
    pSock->Server.sin_addr.s_addr = sl_Htonl(l_ip);
    pSock->Server.sin_port = sl_Htons(ti_udp);
    pSock->iServerLength = sizeof(pSock->Server);

    pSock->Client.sin_family = AF_INET;
    pSock->Client.sin_addr.s_addr = sl_Htonl(r_ip);
    pSock->Client.sin_port = sl_Htons(cli_udp);
    pSock->iClientLength = sizeof(pSock->Client);

    rt = sl_Bind(pSock->iSockDesc, (struct sockaddr*)&(pSock->Server), pSock->iServerLength);
    ASSERT_ON_ERROR(rt);

    if ((!rt) && (pSock->iSockDesc > 0) ) {
    	rt = pSock->iSockDesc;
    	UART_PRINT("[SOC] local %d.%d.%d.%d:%d\tremote %d.%d.%d.%d:%d\n\r",
    		self_adr[0], self_adr[1], self_adr[2], self_adr[3], ti_udp,
			rtp_adr[0], rtp_adr[1], rtp_adr[2], rtp_adr[3], cli_udp);
    } else {
    	pSock->iSockDesc = -1;
    	rt = -1;
    	UART_PRINT("[SOC] create ERROR.\n\r");
    }

    return rt;

}
//****************************************************************************
static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    if (ReadFileFromDev((unsigned char *)SSID_FILE_NAME)>0) {
    	strcpy((char*)SSID_NAME, (char*)ssid_ctl_cmd.ssid);
    	strcpy((char*)SECURITY_KEY, (char*)ssid_ctl_cmd.key);
    	SECURITY_TYPE = ssid_ctl_cmd.type;
    }

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;
    lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
        GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        MAP_UtilsDelay(800000);
    }

    return SUCCESS;
}
//****************************************************************************
//*****************************************************************************
#ifdef USE_FREERTOS
//*****************************************************************************
// FreeRTOS User Hook Functions enabled in FreeRTOSConfig.h
//*****************************************************************************
//*****************************************************************************
//! \brief Application defined hook (or callback) function - assert
//! \param[in]  pcFile - Pointer to the File Name
//! \param[in]  ulLine - Line Number
//! \return none
//*****************************************************************************
void vAssertCalled( const char *pcFile, unsigned long ulLine )
{
    while (1) {}
}
//*****************************************************************************
void vApplicationIdleHook( void)
{
    //Handle Idle Hook for Profiling, Power Management etc
}
//*****************************************************************************
void vApplicationMallocFailedHook()
{
    //Handle Memory Allocation Errors
    while (1) {}
}
//*****************************************************************************
void vApplicationStackOverflowHook( OsiTaskHandle *pxTask, signed char *pcTaskName)
{
    while (1) {}
}
#endif //USE_FREERTOS
//*****************************************************************************
//*****************************************************************************

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if (!pWlanEvent) return;

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN] STA Connected to AP '%s', BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;
        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            if (SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN]Device disconnected from AP '%s',"
                           " BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID, g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1], g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3], g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            } else {
                UART_PRINT("[WLAN] ERROR: Device disconnected from AP '%s',"
                           " BSSID: %x:%x:%x:%x:%x:%x\n\r",
                           g_ucConnectionSSID, g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1], g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3], g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
                GSM_OFF(1);
                UART_PRINT("[RST] Restart device...\n\r");
                osi_Sleep(50);
                PRCMMCUReset(1);
            }
        }
        break;
        case SL_WLAN_STA_CONNECTED_EVENT:
        {
            // when device is in AP mode and any client connects to device cc3xxx
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION_FAILED);

            UART_PRINT("[WLAN] Client BSSID=%x:%x:%x:%x:%x:%x connected to AP '%s'\n\r",
                        		pWlanEvent->EventData.APModeStaConnected.mac[0],
            					pWlanEvent->EventData.APModeStaConnected.mac[1],
            					pWlanEvent->EventData.APModeStaConnected.mac[2],
            					pWlanEvent->EventData.APModeStaConnected.mac[3],
            					pWlanEvent->EventData.APModeStaConnected.mac[4],
            					pWlanEvent->EventData.APModeStaConnected.mac[5],
            					g_ucConnectionSSID);

        }
        break;
        case SL_WLAN_STA_DISCONNECTED_EVENT:
        {
            // when client disconnects from device (AP)
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            UART_PRINT("[WLAN] Client BSSID=%x:%x:%x:%x:%x:%x disconnected from AP '%s'\n\r",
                        		pWlanEvent->EventData.APModestaDisconnected.mac[0],
                        		pWlanEvent->EventData.APModestaDisconnected.mac[1],
                        		pWlanEvent->EventData.APModestaDisconnected.mac[2],
                        		pWlanEvent->EventData.APModestaDisconnected.mac[3],
                        		pWlanEvent->EventData.APModestaDisconnected.mac[4],
                        		pWlanEvent->EventData.APModestaDisconnected.mac[5],
                        		g_ucConnectionSSID);
        }
        break;
        case SL_WLAN_SMART_CONFIG_COMPLETE_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_START);
            UART_PRINT("[WLAN] Smart config start...\n\r");
         }
        break;
        case SL_WLAN_SMART_CONFIG_STOP_EVENT:
        {
            // SmartConfig operation finished
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_START);
            UART_PRINT("[WLAN] Smart config stop.\n\r");
        }
        break;
        case SL_WLAN_P2P_DEV_FOUND_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_P2P_DEV_FOUND);
            UART_PRINT("[WLAN] P2P_DEV_FOUND_EVENT.\n\r");
        }
        break;
        case SL_WLAN_P2P_NEG_REQ_RECEIVED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_P2P_REQ_RECEIVED);
            UART_PRINT("[WLAN] P2P_NEG_REQ_RECEIVED_EVENT.\n\r");
        }
        break;
        case SL_WLAN_CONNECTION_FAILED_EVENT:
        {
            // If device gets any connection failed event
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION_FAILED);
            UART_PRINT("[WLAN] CONNECTION_FAILED_EVENT.\n\r");
        }
        break;
            default: {
                UART_PRINT("[WLAN] Unexpected event [0x%x]\n\r", pWlanEvent->Event);
            }
        break;
    }
}
//*****************************************************************************
// This function handles network events such as IP acquisition, IP leased, IP released etc.
// param[in]  pNetAppEvent - Pointer to NetApp Event Info
// return None
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if (!pNetAppEvent) return;

    switch (pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
            
            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
            g_uiIpAddress = pEventData->ip;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;
            self_adr[0]=SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 3);
            self_adr[1]=SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 2);
            self_adr[2]=SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 1);
            self_adr[3]=SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip, 0);
            sprintf(SELF_ADR, "%d.%d.%d.%d", self_adr[0], self_adr[1], self_adr[2], self_adr[3]);
			gw_adr[0]=SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 3);
			gw_adr[1]=SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 2);
			gw_adr[2]=SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 1);
			gw_adr[3]=SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway, 0);
            UART_PRINT("[NET] Acquired: IP=%s, Gateway=%d.%d.%d.%d, dns=%d.%d.%d.%d\n\r",
            			SELF_ADR,
						gw_adr[0], gw_adr[1], gw_adr[2], gw_adr[3],
						SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.dns, 3),
						SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.dns, 2),
						SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.dns, 1),
						SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.dns, 0));
            WLanDone = 1;
        }
        break;
        case SL_NETAPP_IP_LEASED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);
            SlIpLeasedAsync_t *pEventData = NULL;
            pEventData = &pNetAppEvent->EventData.ipLeased;
            cli_adr[0]=SL_IPV4_BYTE(pEventData->ip_address, 3);
            cli_adr[1]=SL_IPV4_BYTE(pEventData->ip_address, 2);
            cli_adr[2]=SL_IPV4_BYTE(pEventData->ip_address, 1);
            cli_adr[3]=SL_IPV4_BYTE(pEventData->ip_address, 0);
            sprintf(CLI_ADR, "%d.%d.%d.%d", cli_adr[0], cli_adr[1], cli_adr[2], cli_adr[3]);
            UART_PRINT("[NET] Client %s [%x:%x:%x:%x:%x:%x] online (time=%d).\n\r",
            		CLI_ADR,
					pEventData->mac[0],pEventData->mac[1],
            		pEventData->mac[2],pEventData->mac[3],
            		pEventData->mac[4],pEventData->mac[5],
            		pEventData->lease_time);
            if ((rtp_adr[0]|rtp_adr[1] | rtp_adr[2] | rtp_adr[3]) == 0) memcpy(rtp_adr, cli_adr, 4);
        }
        break;
        case SL_NETAPP_IP_RELEASED_EVENT:
        {
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);
            SlIpReleasedAsync_t *pEventData = NULL;
            pEventData = &pNetAppEvent->EventData.ipReleased;
            UART_PRINT("[NET] Client %d.%d.%d.%d [%x:%x:%x:%x:%x:%x] offline (reason=%d).\n\r",
            		SL_IPV4_BYTE(pEventData->ip_address, 3),
                    SL_IPV4_BYTE(pEventData->ip_address, 2),
                    SL_IPV4_BYTE(pEventData->ip_address, 1),
                    SL_IPV4_BYTE(pEventData->ip_address, 0),
                    pEventData->mac[0], pEventData->mac[1],
					pEventData->mac[2], pEventData->mac[3],
					pEventData->mac[4], pEventData->mac[5],
					pEventData->reason);
        }
		break;
        default: {
            UART_PRINT("[NET] Unexpected event [0x%x] \n\r", pNetAppEvent->Event);
        }
        break;
    }
}
//*****************************************************************************
// brief This function handles General Events
// param[in]     pDevEvent - Pointer to General Event Info
// return None
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if (!pDevEvent) return;

    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}
//*****************************************************************************
// This function handles socket events indication
// param[in]      pSock - Pointer to Socket Event Info
// return None
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
	UART_PRINT("[EVT] SockEventHandler not used\n\r");
    //
    // This application doesn't work w/ socket - Events are not expected
    //
}
//*****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent,
                                  SlHttpServerResponse_t *pSlHttpServerResponse)
{

    switch (pSlHttpServerEvent->Event) {
        case SL_NETAPP_HTTPGETTOKENVALUE_EVENT:
        {
            if (0 == memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
                            g_token_get [0], \
                            pSlHttpServerEvent->EventData.httpTokenName.len))
            {
                if (g_ucConnectedToConfAP == 1) {
                    // Important - Connection Status
                    memcpy(pSlHttpServerResponse->ResponseData.token_value.data, "TRUE",strlen("TRUE"));
                    pSlHttpServerResponse->ResponseData.token_value.len = strlen("TRUE");
                } else {
                    // Important - Connection Status
                    memcpy(pSlHttpServerResponse->ResponseData.token_value.data, "FALSE", strlen("FALSE"));
                    pSlHttpServerResponse->ResponseData.token_value.len = strlen("FALSE");
                }
            }

            if (0 == memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
                            g_token_get [1], \
                            pSlHttpServerEvent->EventData.httpTokenName.len))
            {
                // Important - Token value len should be < MAX_TOKEN_VALUE_LEN
                memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
                        g_NetEntries[0].ssid,g_NetEntries[0].ssid_len);
                pSlHttpServerResponse->ResponseData.token_value.len = g_NetEntries[0].ssid_len;
            }
            if (0 == memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
                            g_token_get [2], \
                            pSlHttpServerEvent->EventData.httpTokenName.len))
            {
                // Important - Token value len should be < MAX_TOKEN_VALUE_LEN
                memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
                        g_NetEntries[1].ssid,g_NetEntries[1].ssid_len);
                pSlHttpServerResponse->ResponseData.token_value.len = \
                                                       g_NetEntries[1].ssid_len;
            }
            if (0 == memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
                            g_token_get [3], \
                            pSlHttpServerEvent->EventData.httpTokenName.len))
            {
                // Important - Token value len should be < MAX_TOKEN_VALUE_LEN
                memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
                        g_NetEntries[2].ssid,g_NetEntries[2].ssid_len);
                pSlHttpServerResponse->ResponseData.token_value.len = g_NetEntries[2].ssid_len;
            }
            if (0 == memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
                            g_token_get [4], \
                            pSlHttpServerEvent->EventData.httpTokenName.len))
            {
                // Important - Token value len should be < MAX_TOKEN_VALUE_LEN
                memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
                        g_NetEntries[3].ssid,g_NetEntries[3].ssid_len);
                pSlHttpServerResponse->ResponseData.token_value.len = g_NetEntries[3].ssid_len;
            }
            if (0 == memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, \
                            g_token_get [5], \
                            pSlHttpServerEvent->EventData.httpTokenName.len))
            {
                // Important - Token value len should be < MAX_TOKEN_VALUE_LEN
                memcpy(pSlHttpServerResponse->ResponseData.token_value.data, \
                        g_NetEntries[4].ssid,g_NetEntries[4].ssid_len);
                pSlHttpServerResponse->ResponseData.token_value.len = g_NetEntries[4].ssid_len;
            } else break;
        }
        break;
        case SL_NETAPP_HTTPPOSTTOKENVALUE_EVENT:
        {

            if ((0 == memcmp(pSlHttpServerEvent->EventData.httpPostData.token_name.data, \
                             "__SL_P_USC", \
                             pSlHttpServerEvent->EventData.httpPostData.token_name.len)) && \
            (0 == memcmp(pSlHttpServerEvent->EventData.httpPostData.token_value.data, \
                         "Add", \
                         pSlHttpServerEvent->EventData.httpPostData.token_value.len)))
            {
                g_ucProfileAdded = 0;

            }
            if (0 == memcmp(pSlHttpServerEvent->EventData.httpPostData.token_name.data, \
                     "__SL_P_USD", \
                     pSlHttpServerEvent->EventData.httpPostData.token_name.len))
            {
                memcpy(g_cWlanSSID,  \
                       pSlHttpServerEvent->EventData.httpPostData.token_value.data, \
                       pSlHttpServerEvent->EventData.httpPostData.token_value.len);
                g_cWlanSSID[pSlHttpServerEvent->EventData.httpPostData.token_value.len] = 0;
            }

            if (0 == memcmp(pSlHttpServerEvent->EventData.httpPostData.token_name.data, \
                            "__SL_P_USE", \
                            pSlHttpServerEvent->EventData.httpPostData.token_name.len))
            {
                if (pSlHttpServerEvent->EventData.httpPostData.token_value.data[0] == '0') {
                    g_SecParams.Type =  SL_SEC_TYPE_OPEN;
                } else if(pSlHttpServerEvent->EventData.httpPostData.token_value.data[0] == '1') {
                    g_SecParams.Type =  SL_SEC_TYPE_WEP;
                } else if(pSlHttpServerEvent->EventData.httpPostData.token_value.data[0] == '2') {
                    g_SecParams.Type =  SL_SEC_TYPE_WPA;
                } else {
                    g_SecParams.Type =  SL_SEC_TYPE_OPEN;
                }
            }
            if (0 == memcmp(pSlHttpServerEvent->EventData.httpPostData.token_name.data, \
                         "__SL_P_USF", \
                         pSlHttpServerEvent->EventData.httpPostData.token_name.len))
            {
                memcpy(g_cWlanSecurityKey, \
                       pSlHttpServerEvent->EventData.httpPostData.token_value.data, \
                       pSlHttpServerEvent->EventData.httpPostData.token_value.len);
                g_cWlanSecurityKey[pSlHttpServerEvent->EventData.httpPostData.token_value.len] = 0;
                g_SecParams.Key = g_cWlanSecurityKey;
                g_SecParams.KeyLen = pSlHttpServerEvent->EventData.httpPostData.token_value.len;
            }
            if (0 == memcmp(pSlHttpServerEvent->EventData.httpPostData.token_name.data, \
                            "__SL_P_USG", \
                            pSlHttpServerEvent->EventData.httpPostData.token_name.len))
            {
                g_ucPriority = pSlHttpServerEvent->EventData.httpPostData.token_value.data[0] - 48;
            }
            if (0 == memcmp(pSlHttpServerEvent->EventData.httpPostData.token_name.data, \
                            "__SL_P_US0", \
                            pSlHttpServerEvent->EventData.httpPostData.token_name.len))
            {
                g_ucProvisioningDone = 1;
            }
        }
        break;
      default: break;
    }
}
//*****************************************************************************
// brief This function initializes the application variables
// param    None
// return None
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_uiIpAddress = 0;

    g_ulGatewayIP = 0;

    memset(g_ucConnectionSSID,  0, sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID, 0, sizeof(g_ucConnectionBSSID));

    memcpy(SSID_NAME, SSID_NAME_DEF, SSID_LEN_MAX);
    memcpy(SECURITY_KEY, SECURITY_KEY_DEF, SSID_LEN_MAX);

    SECURITY_TYPE = SECURITY_TYPE_DEF;

    g_ulDestinationIp = IP_ADDR;

    g_uiPortNum0 = PORT_CTL;
    g_uiPortNum  = PORT_TCP;
    g_uiPortNum1 = PORT_RTP;

    memset((unsigned char *)&ctl_udp, 0, sizeof(s_ctl_udp));
    ctl_udp.ti_port = PORT_RTP;

    strcpy(PREFIX_BUFFER, PREFIX_BUFFER_DEF);
    strcpy(HOST_NAME, HOST_NAME_DEF);

}
//*****************************************************************************
// brief This function puts the device in its default state. It:
//           - Set the mode to STATION
//           - Configures connection policy to Auto and AutoSmartConfig
//           - Deletes all the stored profiles
//           - Enables DHCP
//           - Disables Scan policy
//           - Sets Tx power to maximum
//           - Sets power policy to normal
//           - Unregister mDNS services
//           - Remove all filters
// param   none
// return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState()
{
SlVersionFull   ver = {0};
_WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};
unsigned char ucVal = 1;
unsigned char ucConfigOpt = 0;
unsigned char ucConfigLen = 0;
unsigned char ucPower = 0;
long lRetVal = -1;
long lMode = -1;


    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event before doing anything
            while (!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

    // Set connection policy to Auto + SmartConfig  (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);
    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if (!lRetVal) {
        // Wait
        while (IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE, 1, 1, &ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask, sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal;
}
//****************************************************************************
// brief Connects to the Network in AP or STA Mode - If ForceAP Jumper is
//                               Placed, Force it to AP mode
// return                        0 on success else error code
//****************************************************************************
long ConnectToNetwork()
{
    char ucAPSSID[32];
    unsigned short len, config_opt;
    long lRetVal = -1;

    // staring simplelink
    g_uiSimplelinkRole =  sl_Start(NULL, NULL, NULL);

    // Device is not in STA mode and Force AP Jumper is not Connected 
    //- Switch to STA mode
    if (g_uiSimplelinkRole != ROLE_STA && g_uiDeviceModeConfig == ROLE_STA ) {
        //Switch to STA Mode
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);
        
        lRetVal = sl_Stop(SL_STOP_TIMEOUT);
        
        g_usMCNetworkUstate = 0;
        g_uiSimplelinkRole =  sl_Start(NULL, NULL, NULL);
    }

    //Device is not in AP mode and Force AP Jumper is Connected - 
    //Switch to AP mode
    if (g_uiSimplelinkRole != ROLE_AP && g_uiDeviceModeConfig == ROLE_AP ) {
         //Switch to AP Mode
        lRetVal = sl_WlanSetMode(ROLE_AP);
        ASSERT_ON_ERROR(lRetVal);
        
        lRetVal = sl_Stop(SL_STOP_TIMEOUT);
        
        g_usMCNetworkUstate = 0;
        g_uiSimplelinkRole =  sl_Start(NULL, NULL, NULL);
    }


    //No Mode Change Required
    if (g_uiSimplelinkRole == ROLE_AP) {
       //waiting for the AP to acquire IP address from Internal DHCP Server
       while (!IS_IP_ACQUIRED(g_ulStatus))
       {

       }

       //Stop Internal HTTP Server
       lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
       ASSERT_ON_ERROR( lRetVal);

       //Start Internal HTTP Server
       lRetVal = sl_NetAppStart(SL_NET_APP_HTTP_SERVER_ID);
       ASSERT_ON_ERROR( lRetVal);

       char iCount=0;
       //Read the AP SSID
       memset(ucAPSSID, '\0', AP_SSID_LEN_MAX);
       len = AP_SSID_LEN_MAX;
       config_opt = WLAN_AP_OPT_SSID;
       lRetVal = sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, (unsigned char*) ucAPSSID);
       ASSERT_ON_ERROR(lRetVal);
        
       Report("[IP] AP mode select, AP name '%s'\n\r",ucAPSSID);
       memset(g_ucConnectionSSID, 0, SSID_LEN_MAX);
       strcpy((char *)g_ucConnectionSSID, (char *)ucAPSSID);
       

       //Blink LED 3 times to Indicate AP Mode
       for (iCount = 0; iCount < 3; iCount++) {
           //Turn RED LED On
           GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           osi_Sleep(400);
           
           //Turn RED LED Off
           GPIO_IF_LedOff(MCU_RED_LED_GPIO);
           osi_Sleep(400);
       }

    } else {
        //Stop Internal HTTP Server
        lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
        ASSERT_ON_ERROR( lRetVal);

        //Start Internal HTTP Server
        lRetVal = sl_NetAppStart(SL_NET_APP_HTTP_SERVER_ID);
        ASSERT_ON_ERROR( lRetVal);
        
		//waiting for the device to Auto Connect
		while ( (!IS_IP_ACQUIRED(g_ulStatus))&& g_ucConnectTimeout < AC_TIMEOUT_COUNT ) {
			//Turn RED LED On
			GPIO_IF_LedOn(MCU_RED_LED_GPIO);
			osi_Sleep(50);

			//Turn RED LED Off
			GPIO_IF_LedOff(MCU_RED_LED_GPIO);
			osi_Sleep(50);

			g_ucConnectTimeout++;
		}
		//Couldn't connect Using Auto Profile
		if (g_ucConnectTimeout == AC_TIMEOUT_COUNT) {
			//Blink Red LED to Indicate Connection Error
			GPIO_IF_LedOn(MCU_RED_LED_GPIO);

			CLR_STATUS_BIT_ALL(g_ulStatus);

			//Report("[IP] Use Smart Config Application to configure the device.\n\r");
			//Connect Using Smart Config
			//lRetVal = SmartConfigConnect();
			Report("[IP] Connect to AP '%s' start...\n\r", SSID_NAME);
			lRetVal = WlanConnect();
			ASSERT_ON_ERROR(lRetVal);

			//Waiting for the device to Auto Connect
			while (!IS_IP_ACQUIRED(g_ulStatus))
			{
				MAP_UtilsDelay(500);
			}

		}
		//Turn RED LED Off
		GPIO_IF_LedOff(MCU_RED_LED_GPIO);
		g_uiIpMask = 0xFFFFFF00;
		broadcast_adr = ~g_uiIpMask;
		broadcast_adr |= g_uiIpAddress;
		UART_PRINT("[IP] Device connected to AP '%s'\n\r\tIP %d.%d.%d.%d\n\r\tMASK %d.%d.%d.%d\n\r\tBCAST %d.%d.%d.%d:%d\n\r",
					g_ucConnectionSSID,
					SL_IPV4_BYTE(g_uiIpAddress, 3), SL_IPV4_BYTE(g_uiIpAddress, 2),
					SL_IPV4_BYTE(g_uiIpAddress, 1), SL_IPV4_BYTE(g_uiIpAddress, 0),
					SL_IPV4_BYTE(g_uiIpMask, 3), SL_IPV4_BYTE(g_uiIpMask,2),
					SL_IPV4_BYTE(g_uiIpMask, 1), SL_IPV4_BYTE(g_uiIpMask,0),
					SL_IPV4_BYTE(broadcast_adr, 3), SL_IPV4_BYTE(broadcast_adr, 2),
					SL_IPV4_BYTE(broadcast_adr, 1), SL_IPV4_BYTE(broadcast_adr, 0),
					broadcast_port);
    }

    mode_done = 1;

    return SUCCESS;
}
//****************************************************************************
static void ReadDeviceConfiguration()
{
unsigned int uiGPIOPort;
unsigned char pucGPIOPin;
unsigned char ucPinValue;

    //Read GPIO
    GPIO_IF_GetPortNPin(SH_GPIO_3, &uiGPIOPort, &pucGPIOPin);
    ucPinValue = GPIO_IF_Get(SH_GPIO_3, uiGPIOPort, pucGPIOPin);

    //If Connected to VCC, Mode is AP
    if (ucPinValue == 1) {
        //AP Mode
        g_uiDeviceModeConfig = ROLE_AP;
    } else {
        //STA Mode
        g_uiDeviceModeConfig = ROLE_STA;
    }

}
//*****************************************************************************
unsigned char Check_VIO(unsigned char pr)
{
unsigned int uiGPIOPort;
unsigned char pucGPIOPin, ret;

    GPIO_IF_GetPortNPin(SH_GPIO_4, &uiGPIOPort, &pucGPIOPin);
    ret = GPIO_IF_Get(SH_GPIO_4, uiGPIOPort, pucGPIOPin);

    if (pr) UART_PRINT("[VIO] %d\n\r", ret);

    return ret;
}
//*****************************************************************************
//*****************************************************************************
void TimerBaseIntHandler(void)
{
unsigned char bt;
unsigned short i = 0;

	varta++;// 1ms
	tim++;
	if (tim == 1000) {// seconda
		tim = 0;
		one_sec++;
		GPIO_IF_LedToggle(MCU_ORANGE_LED_GPIO);
	}

	tim2++;
	if (tim2 == tim2_def) {// 1/2 seconda
		tim2 = 0;
		if (ring) GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
		else {
			if (rtp_enable) {
				if (!GPIO_IF_LedStatus(MCU_RED_LED_GPIO)) GPIO_IF_LedOn(MCU_RED_LED_GPIO);
			} else {
				if (GPIO_IF_LedStatus(MCU_RED_LED_GPIO)) GPIO_IF_LedOff(MCU_RED_LED_GPIO);
			}
		}
	}

    while (MAP_UARTCharsAvail(GSM) == true) {
    	bt = MAP_UARTCharGetNonBlocking(GSM);
    	if ((bt >= 0x0a) && (bt <= 0x7f)) {
    		from_uart.buf[from_uart.wr_adr] = bt;
    		from_uart.wr_adr++;
    		if (from_uart.wr_adr >= GSM_MAX_BUFFER) from_uart.wr_adr = 0;
    		if (from_uart.rd_adr == from_uart.wr_adr)
    			from_uart.done = 0;
    		else
    			from_uart.done = 1;
    	}
    }
	if (to_uart.done) {
        	i = 0;
        	while (i < to_uart.len) {
        		if (MAP_UARTSpaceAvail(GSM) == true) {
        			bt = to_uart.buf[i++];
        			MAP_UARTCharPutNonBlocking(GSM, bt);
        		} else {
        			break;
        		}
        	}
        	to_uart.done = 0;
    }

	// Clear the timer interrupt.
    Timer_IF_InterruptClear(g_ulBase);

}
//*****************************************************************************
void GSM_RX_CLEAR()
{
	//clear gsm_rx_buffer
	while (MAP_UARTCharsAvail(GSM) == true) MAP_UARTCharGet(GSM);
	memset((unsigned char *)&from_uart, 0, sizeof(s_uart));

}
//****************************************************************************
void TcpServer(unsigned short usPort)
{
SlSockAddrIn_t  sAddr, sLocalAddr;
int iAddrSize, iSockID, iStatus;
long lNonBlocking = 1;
char *stx = NULL, *at_rx_tcp = NULL, *at_rx_gsm = NULL;
unsigned char ipadr[4] = {0}, vio_pr = 1, faza = 0;
char bt;
unsigned short uk = 0, len2 = 0, lp = 1, sym = 0, i, str_done = 0;
unsigned int *uki = NULL;

    at_rx_tcp = calloc(1, GSM_MAX_BUFFER);
    at_rx_gsm = calloc(1, GSM_MAX_BUFFER);
    stx       = calloc(1, GSM_MAX_BUFFER);

    if ((!at_rx_tcp) || (!at_rx_gsm) || (!stx)) {
    	UART_PRINT("[TCP] No heap memory, end of job.\n\r");
    	while (1) {}
    }

	GSM_RX_CLEAR();

	if (!Check_VIO(vio_pr)) {
		GPIO_IF_LedOn(MCU_LED5_GPIO);// 1 - pwr
		tmr_gsm = one_sec;
		while (one_sec <= tmr_gsm);
		GPIO_IF_LedOff(MCU_LED5_GPIO);// 0 - pwr
		tmr_gsm = one_sec;
		while (one_sec <= tmr_gsm);
		GPIO_IF_LedOn(MCU_LED5_GPIO);// 1 - pwr
		UART_PRINT("[TCP] GSM ON.(%d)\n\r", pwr);
		tmr_gsm = one_sec + 10;
	} else {
		at_start = 1;
		vio_pr = 0;
	}

    sLocalAddr.sin_family = SL_AF_INET;
    sLocalAddr.sin_port = sl_Htons((unsigned short)usPort);
    sLocalAddr.sin_addr.s_addr = 0;
    iSockID = (int)sl_Socket(SL_AF_INET, SL_SOCK_STREAM, 0);
    if( iSockID < 0 ) {
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }
    iAddrSize = sizeof(SlSockAddrIn_t);
    iStatus = sl_Bind(iSockID, (SlSockAddr_t *)&sLocalAddr, iAddrSize);
    if( iStatus < 0 ) {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(BIND_ERROR);
    }
    iStatus = sl_Listen(iSockID, 0);
    if( iStatus < 0 ) {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(LISTEN_ERROR);
    }
    iStatus = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &lNonBlocking, sizeof(lNonBlocking));
    if( iStatus < 0 ) {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
    }
    UART_PRINT("[TCP] Wait auth. client...\n\r");

    //clear all var
    uk = sym = 0;
    len2 = str_done = 0;
    tcp_cli = 0; done_auth = 0; i = 0;

    if (!Check_VIO(vio_pr)) {
    	memset((unsigned char *)&to_uart, 0, sizeof(s_uart));
    	strcpy((char *)to_uart.buf, "\r\nAT\r\n");
    	to_uart.len = strlen((char *)to_uart.buf);
    	Report((char *)&to_uart.buf[2]);
    	to_uart.done = 1;
    	tmr_gsm = one_sec + 5;
    }

    tcp_begin = 1;

    while (1) {

        if (!at_start) {
            if (one_sec >= tmr_gsm) {
                tmr_gsm = one_sec + 10;
                UART_PRINT("[TCP] GSM ON.(%d)\n\r", pwr);
                if (!pwr) {
                    GPIO_IF_LedOn(MCU_LED5_GPIO);// 1 - pwr
                    tmr_gsm = one_sec + 2;
                } else {
                    GPIO_IF_LedOff(MCU_LED5_GPIO);// 0 - pwr
                }
            }
        }

        switch (faza) {
            case 0 :
                tcp_cli = 0;
                iNewSockID = SL_EAGAIN;
                iNewSockID = sl_Accept(iSockID, (struct SlSockAddr_t *)&sAddr, (SlSocklen_t*)&iAddrSize);
                if (iNewSockID >= 0) {
                    iStatus = sl_SetSockOpt(iNewSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &lNonBlocking, sizeof(lNonBlocking));
                    if( iStatus < 0 ) {
                        sl_Close(iNewSockID); iNewSockID = -1;
                        ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
                    } else {
                        memcpy(ipadr, (unsigned char *)&sAddr.sin_addr.s_addr, 4);
                        uki = (unsigned int *)&ipadr[0];
                        osi_Sleep(10);
                        if ((done_auth) && ( *(unsigned int *)uki == *(unsigned int *)uk_auth_tcp_adr)) {//AUTH CLIENT !!!
                            GSM_RX_CLEAR();
                            GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
                            tcp_cli = 1;
                            memset(at_rx_tcp, 0, GSM_MAX_BUFFER);
                            uk = 0;
                            UART_PRINT("[TCP] Auth. client %d.%d.%d.%d:%d connected.\n\r",
                                       ipadr[0], ipadr[1], ipadr[2], ipadr[3], sl_Htons(sAddr.sin_port));
                        } else {
                            ASSERT_ON_ERROR(sl_Close(iNewSockID));
                            iNewSockID = -1;
                            UART_PRINT("[TCP] Unknown client %d.%d.%d.%d - reject.\n\r",
                                       ipadr[0], ipadr[1], ipadr[2], ipadr[3]);
                        }
                    }
                }
                faza = 1;
            break;
            case 1 :
                if ((tcp_cli) && (done_auth)) {
                    iStatus = sl_Recv(iNewSockID, at_rx_tcp + uk, BUF_SIZE - 1, 0);
                    if (!iStatus) {
                        tcp_cli = 0;
                        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
                        Report ("[TCP] Client disconnected\n\r");
                        ASSERT_ON_ERROR(sl_Close(iNewSockID));
                        iNewSockID = -1;
                    } else if (iStatus > 0) {
                        uk += iStatus;
                        memset(stx, 0, GSM_MAX_BUFFER);
                        strcpy(stx, "\r\n"); strcat(stx, at_rx_tcp); strcat(stx, "\r\n");
                        memset((unsigned char *)&to_uart, 0, sizeof(s_uart));
                        strcpy((char *)to_uart.buf, stx);
                        to_uart.len = strlen(stx);
                        to_uart.done = 1;
                        Report(at_rx_tcp);
                        memset(at_rx_tcp, 0, BUF_SIZE);
                        uk = 0;
                        if (strstr(stx,"AT+CHUP") != NULL) {
                            rtp_enable = 0;
                            ring = 0;
                        }
                    }
                }
                /***********	RX from GSM		************/
                if (from_uart.done) {
                    lp = 1;
                    while (lp) {
                        if ((from_uart.rd_adr != from_uart.wr_adr) && (from_uart.done)) {
                            if (from_uart.rd_adr >= GSM_MAX_BUFFER) from_uart.rd_adr = 0;
                            bt = from_uart.buf[from_uart.rd_adr];
                            from_uart.rd_adr++;
                            if (bt > 0x0d) sym = 1;
                            if (sym) {
                                at_rx_gsm[i] = bt;
                                i++;
                            }
                            if ((bt == 0x0a) && (sym) && (i > 1)) {
                                lp = 0;
                                str_done = 1;
                            }
                        } else lp = 0;
                    }
                    if (str_done) {
                        if (!at_start) {
                            at_start = 1;
                            if (!pwr) GPIO_IF_LedOn(MCU_LED5_GPIO);// 1 - pwr
                        }
                        len2 = strlen(at_rx_gsm);
                        if ((tcp_cli) && (done_auth)) sl_Send(iNewSockID, at_rx_gsm, len2, 0);
                        UART_PRINT(at_rx_gsm);
                        if (strstr(at_rx_gsm,"RING") != NULL) {
                            rtp_enable = 0;
                            ring = 1;
                        } else if (strstr(at_rx_gsm, "VOICE CALL: BEGIN") != NULL) {
                            rtp_enable = 1;
                            ring = 0;
                            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                        } else if ( (strstr(at_rx_gsm,"VOICE CALL: END") != NULL) ||
        								(strstr(at_rx_gsm, "NO CARRIER") != NULL) ) {
                            rtp_enable = 0;
                            ring = 0;
                            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
                        } else if (len2 > 0) ring = 0;
                        memset(at_rx_gsm, 0, GSM_MAX_BUFFER);
                        i = sym = 0;
                        str_done = 0;
                    }
                }
                /*******************************************/
                if ((tcp_cli) && (done_auth))
                    faza = 1;
                else
                    faza = 0;
            break;
                default : UART_PRINT("\r\n!!! STAGE ERROR !!!\r\n");
        }//switch(faza)

        if (tcp_close) {
            if (one_sec > tmr_tcp_close) {
                tcp_close = 0;
                if (tcp_cli) {
                    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
                    Report ("[TCP] Connection close by signal 2\n\r");
                    if (iNewSockID > 0) {
                        sl_Close(iNewSockID);
                        iNewSockID = -1;
                    }
                    faza = 0;
                }
            }
        }

    }//EndOfJob

}
//****************************************************************************
void I2SIntHandler()
{
unsigned char bt, f;
unsigned long sta = 0;

	g_stat = MAP_I2SIntStatus(I2S_BASE);

	//-------------------  RX DATA  -------------------------------
	if (g_stat & I2S_STS_RDATA) {
		MAP_I2SDataGet( I2S_BASE, I2S_DATA_LINE_0, &g_rxd);
		g_rxd &= 0xff;
		bt = g_rxd;
		if (rtp_enable) {
		    *(unsigned char*)(GetWritePtr(pRecordBuffer)) = bt;
			UpdateWritePtr(pRecordBuffer, 1);
		}
		sta |= I2S_STS_RDATA;
	}
	//--------------  TX DATA  ---------------------------------------
	if (g_stat & I2S_STS_XDATA) {
		bt = silent;
		f = 0;
		if (rtp_enable) {
			if (GetBufferSize(pPlayBuffer)>=HULF_BUFFER) hulf = 1;
			if (hulf) {
				if (GetBufferSize(pPlayBuffer)>=PACKET_SIZE) {
					bt = *(unsigned char*)(GetReadPtr(pPlayBuffer));
					f = 1;
				}
			}
		}
		g_txd = bt;

		MAP_I2SDataPut(I2S_BASE, I2S_DATA_LINE_1, g_txd);
		if (f) UpdateReadPtr(pPlayBuffer, 1);

		sta |= I2S_STS_XDATA;
	}

	if (sta) MAP_I2SIntClear(I2S_BASE, sta);
}
//****************************************************************************
void init_rtp(s_packet * pak)
{
	if (!pak) return;

	memset((unsigned char *)pak, 0, sizeof(s_packet));
	pak->hdr.version = 0x80;
	pak->hdr.codec = 8; //711alaw codec
	pak->hdr.ssrc = sl_Htonl(one_sec);
}
//-----------------------------------------------------------
static void UDPTask(void *pvParameters)
{
SlFdSet_t rds;
struct SlTimeval_t timeout;
s_packet r_pack, t_pack;
unsigned char faza = 2, done_pack = 0, err = 0, first = 1;
unsigned short t_seqn = 0, r_seqn = 0, seq_now = 0, seq_first = 0;
unsigned int t_tstamp = 0, fsize = 0, r_pk = 0, t_pk = 0;
int rStatus, sStatus, pak_size=sizeof(s_packet);
#ifndef DIRECT
	unsigned short ind;
	unsigned char rx_cnt = 0;
#endif
#ifdef PACK_COUNT_PRN
	unsigned int udp_sec;
#endif

	while (!WLanDone || !mode_done || !ctl_start) osi_Sleep(40);// wait all conditions is done


#ifdef PACK_COUNT_PRN
    udp_sec = one_sec+10;
#endif

#ifndef DIRECT
    memset((unsigned char *)&recv_buf, silent, RECV_BUFFER_SIZE_BT);
    osi_Sleep(20);
#endif

    udp_go = 1;

    while (1) {

    	switch (faza) {
    		case 0 :
    			if( CreateUdpSrv(&g_UdpSock, &ctl_udp) < 0 ) {
    				osi_Sleep(2000);
    			} else {
#ifndef DIRECT
    				memset((unsigned char *)&recv_buf[0], silent, RECV_BUFFER_SIZE_BT);
    				rx_cnt = 0;
#endif
    				init_rtp(&t_pack);
    				r_seqn = t_seqn = 0;
    				t_tstamp = 0;
    				fsize = 0;
    				r_pk = t_pk = 0;
    				faza = 1;
    				first = 1;
    			}
    		break;
    		case 1 :
    			if ((rtp_enable) && (a_start)) {
    				if (g_UdpSock.iSockDesc > 0) {
    					if ( (GetBufferSize(pRecordBuffer)) >= PACKET_SIZE ) {
    						memcpy((unsigned char *)&t_pack.data[0], (char*)(pRecordBuffer->pucReadPtr), PACKET_SIZE);
    						UpdateReadPtr(pRecordBuffer, PACKET_SIZE);
    						t_pack.hdr.seq_num = sl_Htons(t_seqn);
    						t_seqn++;
    						t_pack.hdr.time_stamp = sl_Htonl(t_tstamp);
    						t_tstamp += PACKET_SIZE;
    						sStatus = sl_SendTo(g_UdpSock.iSockDesc,
    						                    (unsigned char *)&t_pack,
    											pak_size,
    											MSG_DONTWAIT,
    											(SlSockAddr_t *)&g_UdpSock.Client,
    											g_UdpSock.iClientLength);
    						if (sStatus != pak_size)
    							UART_PRINT("[UDP] SEND ERROR: pk=%d ret=%d\n\r", t_pk, sStatus);
    					    else
    					    	t_pk++;
    					}
    					timeout.tv_sec = 0; timeout.tv_usec = 16000;
    					SL_FD_ZERO(&rds); SL_FD_SET(g_UdpSock.iSockDesc, &rds);
    					if (sl_Select(g_UdpSock.iSockDesc + 1, &rds, NULL, NULL, &timeout) > 0) {
#ifndef DIRECT
    						if ( (GetBufferEmptySize(pPlayBuffer)) >= RECV_BUFFER_SIZE_BT ) {
#else
    						if ( (GetBufferEmptySize(pPlayBuffer)) >= 2*PACKET_SIZE ) {
#endif
    						    rStatus = sl_RecvFrom(g_UdpSock.iSockDesc,
    						                          (unsigned char *)&r_pack,
    								                  pak_size,
    								                  0,
												      (SlSockAddr_t *)&g_UdpSock.Server,
												      (SlSocklen_t *)&g_UdpSock.iServerLength);
    							if (rStatus > 0) {
    							    fsize += rStatus;
    								if (rStatus >= pak_size) {
    								    if (rStatus > pak_size) UART_PRINT("[UDP] RECV ERROR(big): pk=%d bytes=%d fs=%d\n\r", r_pk, rStatus, fsize);
    								    done_pack = 1;
    								} else UART_PRINT("[UDP] RECV ERROR(strip): pk=%d bytes=%d fs=%d\n\r", r_pk, rStatus, fsize);
    								if (done_pack) {
    								    r_pk++; err = 0;
    								    seq_now = sl_Htons(r_pack.hdr.seq_num);
    								    if (first) {
    								        first = 0;
    								        seq_first = seq_now;
    								        if (seq_now > 0) UART_PRINT("[UDP] RECV FIRST PACKET = %d\n\r", seq_first);
    								    }
    									if (seq_now >= seq_first) seq_now -= seq_first;
    									                     else UART_PRINT("[UDP] RECV ERROR : seq_now %d < seq_first %d\n\r", seq_now, seq_first);
#ifdef DIRECT
    									if (seq_now != r_seqn) {
    									    if (seq_now < r_seqn) err = 2;//old pack - skip
    										else if ((seq_now - r_seqn) == 1) {
    										    err = 1;
    										    if ( (GetBufferSize(pPlayBuffer)) >= PACKET_SIZE )
    													UpdateWritePtr(pPlayBuffer, PACKET_SIZE);
    										}
    										UART_PRINT("[UDP] RECV ERROR: wait=%d now=%d err=%d\n\r", r_seqn, seq_now, err);
    									}
    									if (err < 2) {
    									    memcpy((char*)(pPlayBuffer->pucWritePtr), (unsigned char *)&r_pack.data[0], PACKET_SIZE);
    										UpdateWritePtr(pPlayBuffer, PACKET_SIZE);
    										memcpy((char*)(pPlayBuffer->pucWritePtr), (unsigned char *)&r_pack.data[0], PACKET_SIZE);
    										r_seqn = seq_now + 1;
    									}
#else
    									ind = seq_now & RECV_BUFFER_MASK;
    									if (seq_now != r_seqn) UART_PRINT("[UDP] RECV ERROR: wait=%d now=%d err=%d\n\r", r_seqn, seq_now, err);
    									rx_cnt++;
    									rx_cnt &= RECV_BUFFER_MASK;
    									memcpy((unsigned char *)(&recv_buf[ind].data[0]), (unsigned char *)&r_pack.data[0], PACKET_SIZE);
    									if (!rx_cnt) {
    									    if ( (GetBufferEmptySize(pPlayBuffer)) >= RECV_BUFFER_SIZE_BT ) {
    											for (ind = 0; ind < RECV_BUFFER_SIZE; ind++) {
    											    memcpy((char*)(pPlayBuffer->pucWritePtr), (unsigned char *)&recv_buf[ind], PACKET_SIZE);
    											    UpdateWritePtr(pPlayBuffer, PACKET_SIZE);
    											}

    										} else {
    										    UART_PRINT("[UDP] RECV ERROR: no room in buffer\n\r");
    										}
    									    memset((unsigned char *)&recv_buf[0], silent, RECV_BUFFER_SIZE_BT);
    									}
    									r_seqn = seq_now + 1;
#endif
    								}//if (done_pack)
    							}//if (rStatus > 0)
    						}//----------  ren recv
    					}//end select
    				} else faza = 2;
				} else faza = 2;
    		break;
    		case 2 :
    			faza = 1;
    	    	if (rtp_enable) {
    	    		if (!a_start) {
    	    			osi_Sleep(1);
    	    			MAP_I2SIntEnable(I2S_BASE, g_mdi);
    	    			Audio_Start(RecordPlay);
    	    			a_start = 1;
    	    			faza = 0;
    	    			UART_PRINT("[I2S:%08u] Audio start\n\r", varta);
    	    		}
    	    	} else {
    	    		if (a_start) {
    	    			MAP_I2SIntDisable(I2S_BASE, g_mdi);
    	    			Audio_Stop();
    	    			a_start = 0;
    	    			InitCircularBuffer(pRecordBuffer);
    	    			InitCircularBuffer(pPlayBuffer);
    	    			osi_Sleep(1);
    	    			UART_PRINT("[I2S:%08u] Audio stop\n\r", varta);
    	    			if (g_UdpSock.iSockDesc > 0) {
    	    				sl_Close(g_UdpSock.iSockDesc);
    	    				g_UdpSock.iSockDesc = -1;
    	    			} else {
    	    			    UART_PRINT("[UDP] Socket already closed\n\r");
    	    			}
    	    		}
    	    	}
    	    	if (rtp_close) {
    	    		if (one_sec > tmr_rtp_close) rtp_close = 0;
    	    	}
    		break;
    		default : UART_PRINT("\r\n!!! UDP STAGE ERROR !!!\r\n");
    	}//switch (faza)
    	//------------------------
#ifdef PACK_COUNT_PRN
    	if (one_sec > udp_sec) {
    		if (pack_count_prn) {
    			if ((a_start) && (g_UdpSock.iSockDesc > 0))
    				UART_PRINT("[UDP:%08u] tx: pk=%d s=%u; rx[%d]: pk=%d s=%u\n\r", varta, t_pk, t_seqn, seq_first, r_pk, r_seqn);
    		}
    		udp_sec = one_sec + 1;
    	}
#endif
    	//------------------------
    }//loop
}
//****************************************************************************
static void TCPTask(void *pvParameters)
{
long lRetVal = -1;

	InitializeAppVariables();

	lRetVal = ConfigureSimpleLinkToDefaultState();
	if(lRetVal < 0) {
		if (lRetVal == DEVICE_NOT_IN_STATION_MODE)
		    UART_PRINT("Failed to configure the device in its default state\n\r");
	    LOOP_FOREVER();
	}

	memset(g_ucSSID, '\0', AP_SSID_LEN_MAX);

	ReadDeviceConfiguration();//Read Device Mode Configuration

	lRetVal = ConnectToNetwork();//Connect to Network

	//******************    TIMER    ***************************
	// Base address for timer
	g_ulBase = TIMERA0_BASE;

	// Configuring the timers
	Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

	// Setup the interrupts for the timer timeouts.
	Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
	//**********************************************************
	memset((unsigned char *)&from_uart, 0, sizeof(s_uart));
	memset((unsigned char *)&to_uart, 0, sizeof(s_uart));

	Timer_IF_Start(g_ulBase, TIMER_A, 1);//1ms

	while (!WLanDone || !mode_done || !ctl_start) osi_Sleep(20);

	UART_PRINT("[TCP] server start, listen port %d\n\r", g_uiPortNum);

	TcpServer(g_uiPortNum);

}
//****************************************************************************
//****************************************************************************
//****************************************************************************
void prn_ctl_cmd(s_ctl_cmd * uk, unsigned char label)
{
char *pole = NULL;
unsigned char cmd;
s_ctl_udp *udp;
s_ctl_bin *bin;
s_ctl_auth *au;
s_ctl_md5 *md5;

	char *st = calloc(1, sizeof(s_ctl_cmd) + 96);
	if (st) {
		switch (label) {
			case 1:
			    sprintf(st, "[FILE]");
			    break;
			default : sprintf(st,"[CTL]");
		}
		cmd = uk->cmd;
		switch (cmd) {
			case 0 :
				pole = calloc(1, SSID_LEN_MAX + 1);
				if (pole) {
					memcpy(pole, uk->ssid, SSID_LEN_MAX);
					sprintf(st+strlen(st), "\tcmd=%d\n\r\tssid='%s'\n\r", cmd, pole);
					memset(pole, 0, SSID_LEN_MAX + 1);
					memcpy(pole, uk->key, SSID_LEN_MAX);
					sprintf(st+strlen(st), "\tkey='%s'\n\r\ttype=%d", pole, uk->type);
					if (uk->type < 7) sprintf(st+strlen(st), " (%s)\n\r", crypto_type[uk->type]);
							     else sprintf(st+strlen(st), " (%s)\n\r", crypto_type[6]);
					free(pole);
				}
				break;
			case 1 : sprintf(st+strlen(st), "\tcmd=%d - reset gsm\n\r", cmd);
				break;
			case 2 : sprintf(st+strlen(st), "\tcmd=%d - close tcp,rtp sockets\n\r", cmd);
				break;
			case 3 : sprintf(st+strlen(st), "\tcmd=%d - about tcp,rtp sockets\n\r", cmd);
				break;
			case 4 : sprintf(st+strlen(st), "\tcmd=%d - audio stop\n\r", cmd);
				break;
			case 5 :
				udp = (s_ctl_udp *)uk;
				sprintf(st+strlen(st), "\tcmd=%d - udp:%d.%d.%d.%d:%d port=%d\n\r", cmd,
						udp->cli_adr[0], udp->cli_adr[1], udp->cli_adr[2], udp->cli_adr[3],
						udp->cli_port, udp->ti_port);
				break;
			case 6 :
				if (pack_count_prn) sprintf(st+strlen(st), "\tcmd=%d - start print packets\n\r", cmd);
							   else sprintf(st+strlen(st), "\tcmd=%d - stop print packets\n\r", cmd);
				break;
			case 7 : sprintf(st+strlen(st), "\tcmd=%d - check VIO\n\r", cmd);
				break;
			case 8 :
				sprintf(st+strlen(st), "\tcmd=%d - restart device\n\r", cmd);
				break;
			case 9 :
				bin = (s_ctl_bin *)uk;
				sprintf(st+strlen(st), "\tcmd=%d - url:'%s' (%d)\n\r", cmd, (char *)bin->url, bin->len);
				break;
			case 10 :
				sprintf(st+strlen(st), "\tcmd=%d - get firmware version - '%s'\n\r", cmd, (char *)APPLICATION_VERSION);
				break;
			case 11 :
				au = (s_ctl_auth *)uk;
				pole = calloc(1, MAX_LP_LEN + 1);
				if (pole) {
					memcpy(pole, au->login, MAX_LP_LEN);
					sprintf(st+strlen(st), "\tcmd=%d\n\r\tlogin='%s'\n\r", cmd, pole);
					memset(pole, 0, MAX_LP_LEN + 1); memcpy(pole, au->passwd, MAX_LP_LEN);
					sprintf(st+strlen(st), "\tpasswd='%s'\n\r", pole);
					free(pole);
				}
				break;
			case 12 :
				md5 = (s_ctl_md5 *)uk;
				pole = calloc(1, MAX_LP_LEN + 1);
				if (pole) {
					memcpy(pole, md5->str, MAX_LP_LEN);
					sprintf(st+strlen(st),
							"\tcmd=%d\n\r\ttime='%u'\n\r\tmd5='%s'\n\r\tmode=%d\n\r\tauth_ip=%d.%d.%d.%d\n\r",
							cmd, md5->epoch, pole, md5->mode,
							md5->auth_adr[0], md5->auth_adr[1], md5->auth_adr[2], md5->auth_adr[3]);
					free(pole);
				}
				break;
			case 13 :
				sprintf(st+strlen(st), "\tcmd=%d - restart wifi\n\r", cmd);
				break;
			default : sprintf(st+strlen(st), "\tUnknown cmd=%d\n\r", cmd);
		}
		UART_PRINT(st);
		free(st);
	}
}
//*****************************************************************************
long WriteFileToDev(unsigned char * fname)
{
long lRetVal = -1, lFileHandle;
int sz = sizeof(s_ctl_cmd);
_u8 *uk;

    lRetVal = sl_FsOpen(fname,
                FS_MODE_OPEN_CREATE(sz*2, _FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE),
				NULL,
				&lFileHandle);
    if (lRetVal < 0) {// File may already be created
        lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);
    } else {
        // close the user file
    	lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        if (SL_RET_CODE_OK != lRetVal) ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
    }

    //  open a user file for writing
    lRetVal = sl_FsOpen(fname, FS_MODE_OPEN_WRITE, NULL, &lFileHandle);
    if (lRetVal < 0) {
    	lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
        ASSERT_ON_ERROR(FILE_OPEN_WRITE_FAILED);
    }

    if (strstr((char *)fname, "shadow.conf") != NULL) uk = (_u8 *)&auth_ctl_cmd.cmd;
       									         else uk = (_u8 *)&ssid_ctl_cmd.cmd;
    lRetVal = sl_FsWrite(lFileHandle, 0, uk, sz);
    if (lRetVal < 0) {
    	if (lFileHandle) sl_FsClose(lFileHandle, 0, 0, 0);
    	ASSERT_ON_ERROR(FILE_WRITE_FAILED);
    }

    if (SL_RET_CODE_OK != sl_FsClose(lFileHandle, 0, 0, 0)) ASSERT_ON_ERROR(FILE_CLOSE_ERROR);

    return lRetVal;
}
//****************************************************************************
long ReadFileFromDev(unsigned char * fname)
{
long ret = -1, lFileHandle = 0, sz = sizeof(s_ctl_cmd);
SlFsFileInfo_t info;
_u8 *uk;


	// open a user file for reading
    ret = sl_FsOpen(fname, FS_MODE_OPEN_READ, NULL, &lFileHandle);
    if (ret < 0) {
    	UART_PRINT("[FILE] open file '%s' ERROR (%d)\n\r", fname, ret);
    	if (lFileHandle) sl_FsClose(lFileHandle, 0, 0, 0);
    	return ret;
    }

    ret = sl_FsGetInfo(fname, 0, &info);
    if (ret < 0) {
    	UART_PRINT("[FILE] info file '%s' ERROR (%d)\n\r", fname, ret);
    	if (lFileHandle) sl_FsClose(lFileHandle, 0, 0, 0);
    	return ret;
    } else {
    	UART_PRINT("[FILE] file '%s' info: flag=%u len=%d/%d\n\r",
    			fname,
				info.flags,
				info.FileLen,
				info.AllocatedLen);
    }
    if (info.FileLen < sz) return (-1);

    // read data from file

    if (strstr((char *)fname,"shadow.conf") != NULL) uk = (_u8 *)&auth_ctl_cmd.cmd;// /sys/shadow.conf
    									        else uk = (_u8 *)&ssid_ctl_cmd.cmd;// www/ap.conf
    ret = sl_FsRead(lFileHandle, 0, uk, sz);
    if (ret != sz) {
    	if (lFileHandle) sl_FsClose(lFileHandle, 0, 0, 0);
    	UART_PRINT("[FILE] read file '%s' ERROR (%d)\n\r", fname, ret);
    	return (-1);
    }
    if (lFileHandle) sl_FsClose(lFileHandle, 0, 0, 0);

    prn_ctl_cmd((s_ctl_cmd *)uk,1);

    return ret;
}
//****************************************************************************
long ReadAnyFile(unsigned char * fname, unsigned char noprn)
{
long ret = -1, lFileHandle = 0, sz = 1024, cnt = 0;
SlFsFileInfo_t info;
unsigned char *buf = NULL;

	// open a user file for reading
    ret = sl_FsOpen(fname, FS_MODE_OPEN_READ, NULL, &lFileHandle);
    if (ret < 0) {
    	UART_PRINT("[FILE] open file '%s' ERROR (%d)\n\r", fname, ret);
    	if (lFileHandle) sl_FsClose(lFileHandle, 0, 0, 0);
    	return ret;
    }

    ret = sl_FsGetInfo(fname, 0, &info);
    if (ret < 0) {
    	UART_PRINT("[FILE] info file '%s' ERROR (%d)\n\r", fname, ret);
    	if (lFileHandle) sl_FsClose(lFileHandle, 0, 0, 0);
    	return ret;
    } else {
    	UART_PRINT("[FILE] file '%s' info: flag=%u len=%d/%d\n\r",
    			fname,
				info.flags,
				info.FileLen,
				info.AllocatedLen);
    }

    if (!info.FileLen) return (-1);

    buf = (unsigned char *)calloc(1, sz);
    if (!buf) return (-1);

    while (1) {
    	// read data from file
    	ret = sl_FsRead(lFileHandle, cnt, buf, sz);
    	if (ret <= 0) {
    		if (lFileHandle) sl_FsClose(lFileHandle, 0, 0, 0);
    		UART_PRINT("[FILE] read file '%s' ERROR (%d)\n\r", fname, ret);
    		return (-1);
    	} else {
    		if (!noprn) {
    			UART_PRINT("%s", buf);
    			memset(buf, 0, sz);
    		}
    		cnt += ret;
    	}
    	if (cnt >= info.FileLen) break;
	}
    ret = cnt;
    if (buf) free(buf);
    if (lFileHandle) sl_FsClose(lFileHandle, 0, 0, 0);

    return ret;
}
//****************************************************************************
int CheckMD5(s_ctl_md5 *md5)
{
int ret = -1, dl;
char dst_str[MAX_LP_LEN + 1] = {0};
uint8_t dst_bin[16] = {0};

	dl = (MAX_LP_LEN << 1) + strlen((char *)DEF_SECURE_KEY) + 18;//'login'+'passwd'+'key'+'epoch'
	char *src_str = (char *)calloc(1, dl);
	if (src_str) {
		sprintf(src_str,"%s%s%s%u",
				(char *)&auth_ctl_cmd.login[0],
				(char *)&auth_ctl_cmd.passwd[0],
				(char *)DEF_SECURE_KEY,
				md5->epoch);
		dl = strlen(src_str);
		if (dl > 0) {
			SHAMD5ConfigSet(SHAMD5_BASE, SHAMD5_ALGO_MD5);
			SHAMD5DataProcess(SHAMD5_BASE, (uint8_t *)src_str, dl, (uint8_t *)&dst_bin);
			for (dl = 0; dl < 16; dl++) sprintf(dst_str+strlen(dst_str), "%02x", dst_bin[dl]);
			if (!strcmp(dst_str, (char *)md5->str)) ret = 0;
		}
		if (ret) {
			UART_PRINT("[MD5] ret=%d\n\r\tsrc='%s'\n\r\tdst='%s'\n\r\tin ='%s'\n\r",
						ret, src_str, &dst_str[0], (char *)md5->str);
		} else {
			UART_PRINT("[MD5] Access granted for %d.%d.%d.%d\n\r",
						md5->auth_adr[0], md5->auth_adr[1], md5->auth_adr[2], md5->auth_adr[3]);
		}
	}
	if (src_str) free(src_str);

	return ret;
}
//****************************************************************************
void GSM_OFF(unsigned char pr)
{
	if (Check_VIO(pr)) {
		GPIO_IF_LedOff(MCU_LED5_GPIO);// 0 - pwr
		osi_Sleep(1000);
		GPIO_IF_LedOn(MCU_LED5_GPIO);// 1 - pwr
	}
}
//****************************************************************************
static void CTLTask(void *pvParameters)
{
SlSockAddrIn_t  cAddr;
SlSockAddrIn_t  cLocalAddr;
int             cAddrSize, cSockID = -1, cStatus, cNewSockID = -1, dl = 1;
long            cNonBlocking = 1;
unsigned char	faza = 0, ipadr[4] = {0}, out[4] = {0}, prn = 0;
unsigned int 	ind = 0, tmp = 0, fsize = 0, sz = sizeof(s_ctl_cmd);
static unsigned long tmr_ctl = 0;

	while (!WLanDone || !mode_done) osi_Sleep(20);

	memset((_u8 *)&auth_ctl_cmd.cmd, 0, sz);
	if (ReadFileFromDev((unsigned char *)AUTH_FILE_NAME) <= 0) {
		auth_ctl_cmd.cmd = 11;
		strcpy((char *)&auth_ctl_cmd.login[0], (char *)DEF_LOGIN);
		strcpy((char *)&auth_ctl_cmd.passwd[0], (char *)DEF_PASSWD);
		WriteFileToDev((unsigned char *)AUTH_FILE_NAME);
	}

	uk_auth_tcp_adr = (unsigned int *)&auth_tcp_adr[0];

	cLocalAddr.sin_family      = SL_AF_INET;
    cLocalAddr.sin_port        = sl_Htons((unsigned short)g_uiPortNum0);
    cLocalAddr.sin_addr.s_addr = 0;

    cSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if (cSockID < 0) UART_PRINT("[CTL] Socket create ERROR\n\r");

    cAddrSize = sizeof(SlSockAddrIn_t);
    cStatus = sl_Bind(cSockID, (SlSockAddr_t *)&cLocalAddr, cAddrSize);
    if( cStatus < 0 ) {
    	sl_Close(cSockID);
    	UART_PRINT("[CTL] Socket bind ERROR\n\r");
    }

    cStatus = sl_Listen(cSockID, 0);
    if( cStatus < 0 ) {
    	sl_Close(cSockID);
    	UART_PRINT("[CTL] Socket listen ERROR\n\r");
    }

    cStatus = sl_SetSockOpt(cSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &cNonBlocking, sizeof(cNonBlocking));
    if( cStatus < 0 ) {
    	sl_Close(cSockID);
    	UART_PRINT("[CTL] Set socket nonblocking ERROR\n\r");
    }

    UART_PRINT("[CTL] server start, listen port %d\n\r", g_uiPortNum0);

    ctl_start=1;

    while (1) {
    	switch (faza) {
    		case 0 :
    			cNewSockID = sl_Accept(cSockID, ( struct SlSockAddr_t *)&cAddr, (SlSocklen_t*)&cAddrSize);
    	    	if (cNewSockID > 0) {
    	    		cStatus = sl_SetSockOpt(cNewSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING, &cNonBlocking, sizeof(cNonBlocking));
    	    		if( cStatus < 0 ) {
    	    			sl_Close(cNewSockID);
    	    			UART_PRINT("[CTL] Set socket nonblocking ERROR\n\r");
    	    		} else {
    	    			memcpy(ipadr,(unsigned char *)&cAddr.sin_addr.s_addr,4);
    	    			ctl_cli = 1;
    	    			ind = tmp = 0;
    	    			fsize = 0;
    	    			faza = 1;
    	    			tmr_ctl = one_sec + 10;
    	    		}
    	    	}
    	    break;
    	    case 1 :
    	    	if (ctl_cli) {
    	    		if (cNewSockID > 0) {
    	    			cStatus = sl_Recv(cNewSockID, (char*)&ctl_cmd + ind, sz - tmp, 0);
    	    			if (cStatus > 0) {
    	    				ind += cStatus; fsize += cStatus;
    	    				if (fsize <= sz) {
    	    					if (fsize == sz) {
    	    						tmp = 0;
    	    						memset(out, 0, 4);
    	    						dl = 1;
    	    						done_bc = 1;
    	    						switch (ctl_cmd.cmd) {
    	    							case 0:
    	    								memcpy((char*)&ssid_ctl_cmd, (char*)&ctl_cmd, sz);
    	    								write_ssid = 1;
    	    							break;//ssid,key,type
    	    							case 1:
    	    								tmr_gsm = one_sec + 1;
    	    								at_start = 0;
    	    							break;//reset gsm
    	    							case 2:
    	    								if (g_UdpSock.iSockDesc > 0) {
    	    									out[0] |= 0x0f;
    	    									tmr_rtp_close = one_sec;
    	    									rtp_close = 1;
    	    								}
    	    								if (iNewSockID > 0) {
    	    									out[0] |= 0xf0;
    	    									tmr_tcp_close = one_sec + 1;
    	    									tcp_close = 1;
    	    								}
    	    							break;//close all sockets
    	    							case 3:
    	    								if (iNewSockID > 0) out[0] |= 0xf0;
    	    								if (g_UdpSock.iSockDesc > 0) out[0] |= 0x0f;
    	    							break;
    	    							case 4:
    	    								if (a_start) rtp_enable = 0;
    	    							break;
    	    							case 5://ip & port for udp socket
    	    								memcpy((unsigned char *)&ctl_udp, (unsigned char *)&ctl_cmd, sizeof(s_ctl_cmd));
    	    							break;
    	    							case 6:
    	    								dl = 2;
    	    								pack_count_prn = ~pack_count_prn;
    	    								out[1] = pack_count_prn;
    	    							break;
    	    							case 7:
    	    								dl = 2;
    	    								out[1] = Check_VIO(0);
    	    							break;
    	    							case 8 :
    	    								restart = 1;
    	    								rst_mode = 1;
    	    							break;
    	    							case 9 :
    	    								memcpy((unsigned char *)&ctl_bin, (unsigned char *)&ctl_cmd, sizeof(s_ctl_cmd));
    	    								if (ctl_bin.len <= SIZE_80K) {
    	    									bin_start = 1;
    	    									prn_ctl_cmd(&ctl_cmd, 0);
    	    								} else {
    	    								    dl = 1;
    	    								    out[0] = 0xee;
    	    								}
    	    							break;
    	    							case 10 ://0x0A
    	    								dl = 4;
    	    								memcpy(&out[1], (unsigned char *)APPLICATION_VERSION, 3);
    	    							break;
    	    							case 11://0x0B
    	    								memcpy((char*)&auth_ctl_cmd, (char*)&ctl_cmd, sz);
    	    								write_auth = 1;
    	    							break;
    	    							case 12://0x0C
    	    								memcpy((char*)&md5_ctl_cmd, (char*)&ctl_cmd, sz);
    	    								if (CheckMD5(&md5_ctl_cmd)) {//ERROR auth
    	    									out[0] = 0xee;
    	    									done_auth = 0;
    	    								} else {
    	    									done_auth = 1;//auth OK !!!
    	    									memcpy(&auth_tcp_adr[0], &md5_ctl_cmd.auth_adr[0], 4);
    	    								}
    	    							break;
    	    							case 13 ://0x0D
    	    								restart = 1;
    	    								rst_mode = 0;
    	    							break;
    	    								default: out[0] = 0xee;
    	    						}
    	    						if ((!write_ssid) && (!bin_start) && (!write_auth)) {
    	    							cStatus = sl_Send(cNewSockID, &out[0], dl, 0);
    	    							prn = 1;
    	    						} else prn = 0;
    	    						faza = 2;
    	    					} else tmp += cStatus;
    	    				} else {
    	    				    tmp = 0;
    	    				    ind = 0;
    	    				}
    	    			} else if (!cStatus) faza = 2;
    	    		} else faza = 2;
    	    		//--------------------------------------------------------
    	    		if (one_sec >= tmr_ctl) faza = 2;
    	    	} else faza = 2;
    	    break;
    	    case 2 :
    	    	if (write_ssid) {
    	    		write_ssid = 0;
    	    		WriteFileToDev((unsigned char *)SSID_FILE_NAME);
    	    		if (ReadFileFromDev((unsigned char *)SSID_FILE_NAME) <= 0) out[0] = 0xee;
    	    		cStatus = sl_Send(cNewSockID, &out[0], 1, 0);
    	    	}
    	    	if (write_auth) {
    	    		write_auth = 0;
    	    		if (done_auth) {
    	    			prn = 0;
    	    			WriteFileToDev((unsigned char *)AUTH_FILE_NAME);
    	    			if (ReadFileFromDev((unsigned char *)AUTH_FILE_NAME) <= 0) out[0] = 0xee;
    	    		} else {
    	    			out[0] = 0xee;
    	    			prn_ctl_cmd((s_ctl_cmd *)&auth_ctl_cmd, 0);
    	    		}
    	    		cStatus = sl_Send(cNewSockID, &out[0], 1, 0);
    	    	}
    	    	if (bin_start) {
    	    		while (bin_start) {
    	    			osi_Sleep(10);
    	    		}
    	    		if (bin_result)
    	    		    out[0] = 0xee;
    	    		else
    	    		    out[0] = 0;
    	    		cStatus = sl_Send(cNewSockID, &out[0], 1, 0);
    	    		sl_Close(cNewSockID);
    	    		GSM_OFF(1);
    	    		UART_PRINT("[RST] Restart device...\n\r");
    	    		osi_Sleep(50);
    	    		PRCMMCUReset(1);
    	    	}
    	    	ctl_cli = 0;
    	    	faza = 0;
    	    	if (cNewSockID > 0) sl_Close(cNewSockID);
    	    	if (prn) {
    	    	    prn = 0;
    	    	    prn_ctl_cmd(&ctl_cmd, 0);
    	    	}
    	    	if (restart) {
    	    		restart = 0;
    	    		GSM_OFF(1);
    	    		UART_PRINT("[RST] Restart device...(mode %d)\n\r", rst_mode);
    	    		osi_Sleep(50);
    	    		PRCMMCUReset(rst_mode);
    	    	}
    	    break;
    	    	default : UART_PRINT("\r\n!!! CTL STAGE ERROR !!!\r\n");
    	}//switch (faza)

    }

}
//*****************************************************************************
#ifdef SET_LOAD
//*****************************************************************************
static int FlushHTTPResponse(HTTPCli_Handle cli)
{
const char *ids[2] = {
		HTTPCli_FIELD_NAME_CONNECTION, /* App will get connection header value. all others will skip by lib */
		NULL
};
char  buf[128];
int id;
int len = 1;
bool moreFlag = 0;
char **prevRespFilelds = NULL;

    prevRespFilelds = HTTPCli_setResponseFields(cli, ids);

    // Read response headers
    while ((id = HTTPCli_getResponseField(cli,buf,sizeof(buf),&moreFlag)) != HTTPCli_FIELD_ID_END) {
        if (!id) {
            if(!strncmp(buf, "close", sizeof("close")))
            			UART_PRINT("[BIN] Connection terminated by server\n\r");
        }
    }

    HTTPCli_setResponseFields(cli, (const char **)prevRespFilelds);

    while(1) {
        len = HTTPCli_readResponseBody(cli, buf, sizeof(buf) - 1, &moreFlag);
        ASSERT_ON_ERROR(len);
        if ((len - 2) >= 0 && buf[len - 2] == '\r' && buf [len - 1] == '\n') break;
        if (!moreFlag) break;
    }

    return SUCCESS;
}
//****************************************************************************
static int GetData(HTTPCli_Handle cli)
{
long lRetVal = 0, fileHandle = -1;
unsigned long Token = 0;
int id, len = 0;
bool moreFlag = 0;
HTTPCli_Field fields[3] = {
    {HTTPCli_FIELD_NAME_HOST, HOST_NAME},
	{HTTPCli_FIELD_NAME_ACCEPT, "text/html, application/xhtml+xml, */*"},
	{NULL, NULL}
};
const char *ids[4] = {
	HTTPCli_FIELD_NAME_CONTENT_LENGTH,
	HTTPCli_FIELD_NAME_TRANSFER_ENCODING,
	HTTPCli_FIELD_NAME_CONNECTION,
	NULL
};


    UART_PRINT("[BIN] Start downloading from '%s:%d%s'\r\n", HOST_NAME, HOST_PORT, PREFIX_BUFFER);

    // Set request fields
    HTTPCli_setRequestFields(cli, fields);

    memset(g_buff, 0, sizeof(g_buff));

    // Make HTTP 1.1 GET request
    lRetVal = HTTPCli_sendRequest(cli, HTTPCli_METHOD_GET, PREFIX_BUFFER, 0);
    if (lRetVal < 0) ASSERT_ON_ERROR(TCP_SEND_ERROR);

    // Test getResponseStatus: handle
    lRetVal = HTTPCli_getResponseStatus(cli);
    if (lRetVal != 200) {
        FlushHTTPResponse(cli);
        if (lRetVal == 404) ASSERT_ON_ERROR(FILE_NOT_FOUND_ERROR);
        ASSERT_ON_ERROR(INVALID_SERVER_RESPONSE);
    }

    HTTPCli_setResponseFields(cli, ids);

    // Read response headers
    while ((id = HTTPCli_getResponseField(cli, (char *)g_buff, sizeof(g_buff), &moreFlag)) != HTTPCli_FIELD_ID_END) {
        if(!id) UART_PRINT("[BIN] Content length: %s\n\r", g_buff);
        else if(id == 1) {
            if(!strncmp((const char *)g_buff, "chunked", sizeof("chunked")))
                						UART_PRINT("Chunked transfer encoding\n\r");
        } else if(id == 2) {
            if(!strncmp((const char *)g_buff, "close", sizeof("close")))
            							ASSERT_ON_ERROR(FORMAT_NOT_SUPPORTED);
        }
    }
#ifdef SET_SAVE
    // Open file to save the downloaded file
    lRetVal = sl_FsOpen((_u8 *)FILE_NAME, FS_MODE_OPEN_WRITE, &Token, &fileHandle);
    if(lRetVal < 0) {
        // File Doesn't exit create a new of 80 KB file
        lRetVal = sl_FsOpen((unsigned char *)FILE_NAME,
                            FS_MODE_OPEN_CREATE(SIZE_80K, _FS_MODE_OPEN_WRITE|_FS_MODE_OPEN_READ),
                            &Token,
                            &fileHandle);
        ASSERT_ON_ERROR(lRetVal);
    }
#endif

    while(1) {
        len = HTTPCli_readResponseBody(cli, (char *)g_buff, sizeof(g_buff) - 1, &moreFlag);
        if(len < 0) {
            // Close file without saving
#ifdef SET_SAVE
            lRetVal = sl_FsClose(fileHandle, 0, (unsigned char*) "A", 1);
            return lRetVal;
#else
            return (-1);
#endif
        }

#ifdef SET_SAVE
        lRetVal = sl_FsWrite(fileHandle, bytesReceived, (unsigned char *)g_buff, len);
        if(lRetVal < len) {
            UART_PRINT("[BIN] Failed during writing the file, Error-code: %d\r\n", FILE_WRITE_ERROR);
            // Close file without saving
            lRetVal = sl_FsClose(fileHandle, 0, (unsigned char*) "A", 1);
            return lRetVal;
        }
#endif
        bytesReceived +=len;
        if ((len - 2) >= 0 && g_buff[len - 2] == '\r' && g_buff [len - 1] == '\n') break;
        if(!moreFlag) break;
    }

    UART_PRINT("[BIN] Total bytes received: %d\n\r", bytesReceived);
#ifdef SET_SAVE
    lRetVal = sl_FsClose(fileHandle, 0, 0, 0);
    ASSERT_ON_ERROR(lRetVal);
#endif
    return SUCCESS;
}
//*****************************************************************************
static long ServerFileDownload()
{
long ret = -1, prt = 0;
struct sockaddr_in addr;
HTTPCli_Struct cli;
char srv[16] = {0};
unsigned char ipadr[4] = {0}, dl;
char *uk1 = NULL, *uk2 = NULL, *uk3 = NULL;

    if (ctl_bin.len > 0) {
    	uk1 = &ctl_bin.url[0];//begin adr
    	uk2 = strchr(uk1, ':');
    	if (uk2) {
    		uk3 = uk2+1;
    		dl = uk2 - uk1;
    		memcpy(HOST_NAME, uk1, dl);
    	}
    	uk2 = strchr(uk1, '/');
    	if (uk2) {
    		strcpy(PREFIX_BUFFER, uk2);
    		if (uk3) {
    			if (uk2 > uk3) {
    				memcpy(srv, uk3, uk2 - uk3);
    				prt = atoi(srv);
    				if ((prt > 0) && (prt < 65535)) HOST_PORT = prt;
    			}
    		}
    	}
    }

    ret = sl_NetAppDnsGetHostByName((signed char *)HOST_NAME,
                                    strlen((const char *)HOST_NAME),
                                    &g_ulDestinationIp,
                                    SL_AF_INET);
    if(ret < 0) ASSERT_ON_ERROR(GET_HOST_IP_FAILED);

    // Set up the input parameters for HTTP Connection
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(HOST_PORT);
    addr.sin_addr.s_addr = sl_Htonl(g_ulDestinationIp);
    memcpy(ipadr, (unsigned char *)&addr.sin_addr.s_addr, 4);
    sprintf(srv, "%d.%d.%d.%d", ipadr[0], ipadr[1], ipadr[2], ipadr[3]);
    UART_PRINT("[BIN] HTTP server IP is %s:%d\n\r", srv, HOST_PORT);

    // Testing HTTPCli open call: handle, address params only
    HTTPCli_construct(&cli);
        ret = HTTPCli_connect(&cli, (struct sockaddr *)&addr, 0, NULL);
        if (ret < 0) {
            UART_PRINT("[BIN] Connection to HTTP server %s:%d failed\n\r", srv, HOST_PORT);
            ASSERT_ON_ERROR(SERVER_CONNECTION_FAILED);
        }
        ret = GetData(&cli);
        if (ret < 0)
            UART_PRINT("[BIN] ERROR download from '%s:%d%s'\n\r", HOST_NAME, HOST_PORT, PREFIX_BUFFER);
    HTTPCli_destruct(&cli);

    if (bytesReceived == ctl_bin.len)
        ret = 0;
    else
        ret = -1;

    UART_PRINT("[BIN] Download '%s:%d%s' (%d/%d) ", HOST_NAME, HOST_PORT, PREFIX_BUFFER, ctl_bin.len, bytesReceived);

    if (!ret)
        UART_PRINT("OK\n\r");
    else
        UART_PRINT("BAD\n\r");

    bytesReceived = 0;

    return ret;
}
//*****************************************************************************
static long frename(unsigned char * old_name, unsigned char * new_name)
{
long ret = -1, old_fd = 0, new_fd = 0, lens = 0, cnt_rd = 0, cnt_wr = 0, rt = 0, sz = 2048;
unsigned char *buf = NULL;
SlFsFileInfo_t info;

	if (sl_FsOpen(old_name, FS_MODE_OPEN_READ, NULL, &old_fd)) return ret;

	ret = sl_FsGetInfo(old_name,0,&info);
	if (ret < 0)
	    goto outl;
	else
	    lens = info.FileLen;

	if (!sl_FsOpen(new_name, FS_MODE_OPEN_READ, NULL, &new_fd)) {
		sl_FsClose(new_fd, 0, 0, 0);
		ret = sl_FsDel(new_name, 0);
		if (!ret) {
			new_fd=-1;
		} else {
			UART_PRINT("[FILE] delete '%s' ERROR (%d)\n\r", new_name, ret);
			goto outl;
		}
	}
	//create dest file
	ret = sl_FsOpen((_u8 *)new_name, FS_MODE_OPEN_WRITE, 0, &new_fd);
	if (ret < 0) {
		ret = sl_FsOpen((unsigned char *)new_name,
		                FS_MODE_OPEN_CREATE(SIZE_80K, _FS_MODE_OPEN_WRITE|_FS_MODE_OPEN_READ),
		                0,
		                &new_fd);
		if (ret < 0) {
			UART_PRINT("[FILE] create '%s' ERROR (%d)\n\r",new_name,ret);
			goto outl;
		}
	}

	buf = (unsigned char *)calloc(1, sz);
	if (!buf) goto outl;

	while (1) {
		// read data from file
		ret = sl_FsRead(old_fd, cnt_rd, buf, sz);
		if (ret < 0) break;
	    else {
	    	cnt_rd += ret;
	    	rt = ret;
	    	//--------------
	    	ret = sl_FsWrite(new_fd, cnt_wr, buf, rt);
	    	if (ret > 0)
	    	    cnt_wr += ret;
	    	else {
	    		UART_PRINT("[FILE] Failed during writing the file, Error-code: %d\r\n", FILE_WRITE_ERROR);
	    		// Close file without saving
	    		break;
	    	}
	    	//--------------
	    }
	    if (cnt_rd >= lens) break;
	}

outl:
	info.FileLen = 0;
	if (buf) free(buf);
	if (new_fd) {
		sl_FsGetInfo(new_name, 0, &info);
		sl_FsClose(new_fd, 0, 0, 0);
	}
	if (old_fd) sl_FsClose(old_fd, 0, 0, 0);

	if (lens == info.FileLen) {
		ret = sl_FsDel(old_name, 0);
		if (ret) UART_PRINT("[FILE] delete old file '%s' ERROR (%d)\n\r", old_name, ret);
		ret = 0;
	} else {
		ret = -1;
		UART_PRINT("[FILE] Failed during writing the file, rd=%d/%d wr=%d/%d\n\r",
				   lens, cnt_rd, info.FileLen, cnt_wr);
	}
	return ret;
}
//*****************************************************************************
static void BINTask(void *pvParameters)
{
long ret = 0, err = 0;

	while (!WLanDone || !mode_done || !ctl_start || !udp_go) osi_Sleep(40);

	while (1) {
		if (bin_start) {
			UART_PRINT("[BIN:%08u] Start firmware update...\n\r", varta);
			err = 0;
			bin_result = ServerFileDownload();// 0 - all OK
			if (!bin_result)
			    UART_PRINT("[FILE] saved as '%s'\n\r", FILE_NAME);
			else
			    err++;

			if (!bin_result) {
				ret = frename((unsigned char *)FILE_NAME, (unsigned char *)new_name);
				UART_PRINT("[BIN] rename from '%s' to '%s' done (%d)\n\r", FILE_NAME, new_name, ret);
				if (ret) err++;
			}

			if ((!bin_result) && (!ret)) {
				ret = ReadAnyFile((unsigned char *)new_name, 1);// 1 - no print file content
				if (ret > 0) UART_PRINT("[BIN] read '%s' OK (%d)\n\r", (unsigned char *)new_name, ret);
			}
			bin_result = err;
			bin_start = 0;
			UART_PRINT("[BIN:%08u] Stop firmware update.\n\r", varta);
		}
		osi_Sleep(50);
	}
}
#endif
//*****************************************************************************
long SendMulticastPacket()
{
long rt = -1;
struct sockaddr_in msg;
char bcm[128] = {0};

    if (fd_bc <= 0) fd_bc = sl_Socket(AF_INET, SOCK_DGRAM, 0);

    msg.sin_family      = AF_INET;
    msg.sin_addr.s_addr = htonl(broadcast_adr);
    msg.sin_port        = htons(broadcast_port);

    sprintf(bcm, "%s:%d.%d.%d.%d", broadcast_name, self_adr[0], self_adr[1], self_adr[2], self_adr[3]);
    UART_PRINT("[BCAST] Send broadcast msg '%s'\n\r", bcm);
    rt = sendto(fd_bc, bcm, strlen(bcm), 0, (struct sockaddr*)&(msg), sizeof(msg));
    ASSERT_ON_ERROR(rt);

    rt = SUCCESS;

    return rt;
}
//*****************************************************************************
static void BroadCastTask(void *pvParameters)
{
unsigned int *uk_self_adr = NULL, *uk_s_a = NULL;
unsigned char s_a[4] = {0};

	while (!WLanDone || !mode_done || !tcp_begin) osi_Sleep(50);

	while (1) {
		memcpy(s_a, self_adr, 4);
		uk_self_adr = (unsigned int *)&self_adr[0];
		uk_s_a = (unsigned int *)&s_a[0];

		UART_PRINT("[BCAST] Start broadcast task...\n\r");
		while (!done_bc) {
			SendMulticastPacket();
			if (!done_bc) osi_Sleep(5000);
		}
		if (fd_bc > 0) {
		    close(fd_bc);
		    fd_bc = -1;
		}
		UART_PRINT("[BCAST] Stop broadcast task...\n\r");

		while (1) {
			osi_Sleep(1000);
			if ( *(unsigned int *)uk_self_adr != *(unsigned int *)uk_s_a) break;
		}

	}
}
//*****************************************************************************
// Board Initialization & Configuration
// param  None
// return None
//*****************************************************************************
static void BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    IntMasterEnable();
    IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
void main()
{
long lRetVal = -1;

    BoardInit();
    
    PinMuxConfig();
    
    //Change Pin 58 Configuration from Default to Pull Down
    PinConfigSet(PIN_58, PIN_STRENGTH_2MA|PIN_STRENGTH_4MA, PIN_TYPE_STD_PD);//WLAN MODE (AP or Sta)
    PinConfigSet(PIN_59, PIN_STRENGTH_2MA|PIN_STRENGTH_4MA, PIN_TYPE_STD);//VIO
    
    // Initialize ALL LED
    GPIO_IF_LedConfigure(LED1 | LED2 | LED3 | LED4 | LED5);

    //Turn Off the LEDs
    GPIO_IF_LedOff(MCU_ALL_LED_IND);//LED1,LED2,LED3=0

    InitTerm(1);//init uart CONSOLE and GSM with print flag

    UART_PRINT("\n\r\t\tVersion %s\n\r\n\r",APPLICATION_VERSION);

    AudioInit();//init i2s

    /****************      Create buffers      **********************************/
    RecordPlay = I2S_MODE_RX_TX;

    // Create RX and TX Buffer
    if(RecordPlay & I2S_MODE_TX) {
    	pPlayBuffer = CreateCircularBuffer(PLAY_BUFFER_SIZE);//TX
    	if(!pPlayBuffer) {
    		UART_PRINT("[MAIN] Unable to Allocate Memory for Tx Buffer (Play)\n\r");
    		LOOP_FOREVER();
        } else UART_PRINT("[MAIN] Create Tx Buffer Play Ok (%d)\n\r", PLAY_BUFFER_SIZE);
    }
    if(RecordPlay == I2S_MODE_RX_TX) {//I2S_MODE_RX_TX
    	pRecordBuffer = CreateCircularBuffer(RECORD_BUFFER_SIZE);//RX
    	if(!pRecordBuffer) {
    		UART_PRINT("[MAIN] Unable to Allocate Memory for Rx Buffer (Record)\n\r");
    		LOOP_FOREVER();
        }  else UART_PRINT("[MAIN] Create Rx Buffer Record Ok (%d)\n\r", RECORD_BUFFER_SIZE);
    }
    /***************************************************************************/

    MyAudioCaptureRendererConfigure(2048000,
    								I2S_MODE_SLAVE,
									16,        // 16 (bitsPerSample)
									8000,      // 8000 (bitRate)
									1,         // 1 (noOfChannels)
									RecordPlay,
									0);        //without DMA

    MAP_I2SIntRegister(I2S_BASE, I2SIntHandler);

    g_mdi = I2S_INT_XDATA | I2S_INT_RDATA;

    /***************************************************************************/

    // Simplelinkspawntask
    if ( VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY) < 0 ) {
        UART_PRINT("[MAIN] Unable to start simpelink spawn task\n\r");
        LOOP_FOREVER();
    }

    // Create CTL Server Task
    lRetVal = osi_TaskCreate(CTLTask, (signed char*)"CTLTask", OSI_STACK_SIZE, NULL, (1), NULL);//(0)
    if (lRetVal < 0) {
    	UART_PRINT("[MAIN] Unable to create task CTLTask\n\r");
    	LOOP_FOREVER();
    }

    // Create TCP Server Task
    lRetVal = osi_TaskCreate(TCPTask, (signed char*)"TCPTask", OSI_STACK_SIZE, NULL, (1), NULL);//(2)
    if (lRetVal < 0) {
    	UART_PRINT("[MAIN] Unable to create task TCPTask\n\r");
    	LOOP_FOREVER();
    }

    // Create UDP Server Task
    lRetVal = osi_TaskCreate(UDPTask, (signed char*)"UDPTask", OSI_STACK_SIZE, NULL, (1), NULL);
    if (lRetVal < 0) {
    	UART_PRINT("[MAIN] Unable to create task UDPTask\n\r");
    	LOOP_FOREVER();
    }

#ifdef SET_LOAD
    // Create BIN Server Task
    lRetVal = osi_TaskCreate(BINTask, (signed char*)"BINTask", OSI_STACK_SIZE, NULL, (1), NULL);
    if (lRetVal < 0) {
    	UART_PRINT("[MAIN] Unable to create task BINTask\n\r");
    	LOOP_FOREVER();
    }
#endif

    ReadDeviceConfiguration();
    if (g_uiDeviceModeConfig == ROLE_STA) {
    	// Create BroadCast Task
    	lRetVal = osi_TaskCreate(BroadCastTask, (signed char*)"STATask", OSI_STACK_SIZE, NULL, (1), NULL);
    	if(lRetVal < 0) UART_PRINT("[MAIN] Unable to create task STATask\n\r");
    }

    osi_start();// Start OS Scheduler

    while (1) {}

}
