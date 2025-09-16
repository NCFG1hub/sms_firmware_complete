
#include "custom_feature_def.h"
#include "dfota/inc/data_types.h"
#include "dfota/inc/json_parser.h"
#include "dfota/inc/tracker_functions.h"
#include "ql_stdlib.h"
#include "ql_common.h"
#include "ql_system.h"
#include "ql_type.h"
#include "ql_trace.h"
#include "ql_error.h"
#include "ql_uart.h"
#include "ql_timer.h"
#include "ril_network.h"
#include "ril_mqtt.h"
#include "ril.h"
#include "ril_util.h"
#include "ril_system.h"
#include "ril_sms.h"
#include "ril_telephony.h"
#include "ql_memory.h"
#include "string.h"
#include "sms_command.h"
#include "data_types.h"
#include "ql_fs.h"
#include "ql_power.h"
#include "ql_wtd.h"
#include "ril_location.h"
#include "math.h"


#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
#define DEBUG_PORT  UART_PORT1
#define DBG_BUF_LEN   1024
static char DBG_BUFFER[DBG_BUF_LEN];
#define APP_DEBUG(FORMAT,...) {\
    Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    Ql_sprintf(DBG_BUFFER,FORMAT,##__VA_ARGS__); \
    if (UART_PORT2 == (DEBUG_PORT)) \
    {\
        Ql_Debug_Trace(DBG_BUFFER);\
    } else {\
        Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8*)(DBG_BUFFER), Ql_strlen((const char *)(DBG_BUFFER)));\
    }\
}
#else
#define APP_DEBUG(FORMAT,...) 
#endif



/*****************************************************************
* define process state
******************************************************************/




typedef enum {
    GNSS_DATA_SET,
    GNSS_DATA_EMPTY
} GNSS_DATA_STATUS;  // used to mark when the gps data is empty of set

typedef enum {
    MQTT_LOGGED_OUT,
    MQTT_LOGGED_IN,
} MQTTP_AUTH_STATE;


static u8 m_gnss_state = STATE_GNSS_POWERON;


DeviceConfig devConfig; // holds all the device configuration
static s32 isPdpContextGotten = 0;




/*****************************************************************
* MQTT  timer param
******************************************************************/
#define MQTT_TIMER_ID         0x200
#define MQTT_TIMER_PERIOD     500


#define PIN_TIMER_ID         0x210
#define PIN_TIMER_PERIOD     5000


#define PI 3.14159265358979323846
#define EARTH_RADIUS 6371000.0  // in meters


/*****************************************************************
* GNSS timer param
******************************************************************/
#define GNSS_TIMER_ID         			0x100

#define GNSS_TIMER_PERIOD		3000
#define NMEA_TIMER_PERIOD		2000


/***********************************************************************
 * MACRO CONSTANT DEFINITIONS sms
************************************************************************/

#define CON_SMS_BUF_MAX_CNT   (1)
#define CON_SMS_SEG_MAX_CHAR  (160)
#define CON_SMS_SEG_MAX_BYTE  (4 * CON_SMS_SEG_MAX_CHAR)
#define CON_SMS_MAX_SEG       (7)


#define MSG_ID_USER_GNSS_READ   0x1001


/***********************************************************************
 * STRUCT TYPE DEFINITIONS
************************************************************************/
typedef struct
{
    u8 aData[CON_SMS_SEG_MAX_BYTE];
    u16 uLen;
} ConSMSSegStruct;

typedef struct
{
    u16 uMsgRef;
    u8 uMsgTot;

    ConSMSSegStruct asSeg[CON_SMS_MAX_SEG];
    bool abSegValid[CON_SMS_MAX_SEG];
} ConSMSStruct;

/***********************************************************************
 * FUNCTION DECLARATIONS
************************************************************************/
static bool ConSMSBuf_IsIntact(ConSMSStruct *pCSBuf,u8 uCSMaxCnt,u8 uIdx,ST_RIL_SMS_Con *pCon);
static bool ConSMSBuf_AddSeg(ConSMSStruct *pCSBuf,u8 uCSMaxCnt,u8 uIdx,ST_RIL_SMS_Con *pCon,u8 *pData,u16 uLen);
static s8 ConSMSBuf_GetIndex(ConSMSStruct *pCSBuf,u8 uCSMaxCnt,ST_RIL_SMS_Con *pCon);
static bool ConSMSBuf_ResetCtx(ConSMSStruct *pCSBuf,u8 uCSMaxCnt,u8 uIdx);
static bool SMS_Initialize(void);
static void Hdlr_RecvNewSMS(u32 nIndex, bool bAutoReply);
void SMS_TextMode_Send(void);
void loadConfig();

static void Mqtt_Callback_Timer(u32 timerId, void* param);
bool GNSS_Get_FixStatus_from_RMC(u8 *NMEA_Str);
void Callback_GNSS_APGS_Hdlr(char *str_URC);
static void Gnss_Callback_Timer(u32 timerId, void* param);
static void Pin_Check_Callback_Timer(u32 timerId, void* param);
static void Watchdog_Init(void);

u32 HexStringToInt(const char* hexStr);
static void decodeCellInfo(char* cellRawData, ST_CellInfo* cellInfo);
static void decodeCellInfo2(char* cellRawData, ST_CellInfo* cellInfo);

static void decode_gps_data(char* gpsData,GPS_LOCATION_T* decodedGps);

static s32 AT_CELL_LOCATION_HANDLER(char* line, u32 len, void* userData);
static s32 AT_CELL_INFO_HANDLER(char* line, u32 len, void* userdata);



double calculateDistance(double lat1, double lon1, double lat2, double lon2);
static double toRadians(double deg);
double convertNmeaToDecimal(double nmea, char direction);
/***********************************************************************
 * GLOBAL DATA DEFINITIONS
************************************************************************/
ConSMSStruct g_asConSMSBuf[CON_SMS_BUF_MAX_CNT];
static s32 wtdId;
static countFailedPublish = 0;

#define RESP_BUF_SIZE 1024
static char gRespBuf[RESP_BUF_SIZE];
static u16 gRespLen = 0;

typedef struct {
    int arfcn;
    int rxlev;
    int cellid;
    int lac;
} NeighborCell;

#define MAX_NEIGH 16
NeighborCell neighbors[MAX_NEIGH];
int neighCount = 0;
 


/*****************************************************************
* Server Param
******************************************************************/
#define SRVADDR_BUFFER_LEN    100
#define HOST_PORT             8500

/*****************************************************************
* Uart   param
******************************************************************/
#define SERIAL_RX_BUFFER_LEN  2048
#define SERIAL_TX_BUFFER_LEN  2048
static Enum_SerialPort m_myUartPort  = UART_PORT1;
static u8 g_RxBuf_Uart1[SERIAL_RX_BUFFER_LEN];


static u8 m_RxBuf_Uart[SERIAL_RX_BUFFER_LEN];
static u8 m_TxBuf_Uart[SERIAL_TX_BUFFER_LEN];
static u8 pre_NMEA_buf[SERIAL_TX_BUFFER_LEN];
u16 remainLen = 0;

/*****************************************************************
*  MQTT Param
******************************************************************/
MQTT_Urc_Param_t*	  mqtt_urc_param_ptr = NULL;
ST_MQTT_topic_info_t  mqtt_topic_info_t;
bool DISC_flag  = TRUE;
bool CLOSE_flag = TRUE;

/*****************************************************************
*  Sample Param
******************************************************************/
Enum_ConnectID connect_id = ConnectID_0;
u32 pub_message_id = 0;
u32 sub_message_id = 0;


u8 username[] = "NCFTrack\0";

static u8 test_data[128] =  "hello cloud,this is quectel test code!!!\0"; //<first packet data

static u8 device_topic[128] = "device/78489383830945\0"; //<topic

MQTTP_AUTH_STATE mqtt_auth = MQTT_LOGGED_OUT;

u32 lastLocationPush;

static ST_CellInfo g_cell;


static u8 m_mqtt_state = STATE_NW_QUERY_STATE;
static u8 last_m_mqtt_state = STATE_NW_QUERY_STATE;

char deviceData[1024];
char commandData[1024];
char deviceAlarm[1024];
u16 isDeviceData = 0;
u16 isDeviceAlarm = 0;
u16 iscommandData = 0;

u16 countPublishFailure = 0;
u16 countMqttResponseWaitTime = 0;
bool isConnectionOpen = FALSE;
/*****************************************************************
* Other global variable
******************************************************************/
//static s32 	ret;



/*****************************************************************
* Uart callback function
******************************************************************/
static void proc_handle(u8 *pData,s32 len);
static s32 ReadSerialPort(Enum_SerialPort port, /*[out]*/u8* pBuffer, /*[in]*/u32 bufLen);
static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara);


/*****************************************************************
* custom  function
******************************************************************/

void checkConnectionStatus(ST_MSG msg);

MQTT_CON_STATUS_T mqtt_con_status;

/*****************************************************************
* Timer callback function
******************************************************************/

/*****************************************************************
* MQTT recv callback function
******************************************************************/
static void mqtt_recv(u8* buffer,u32 length);

ST_CellInfo mainCellInfo;
ST_CellInfo secondCellInfo;
ST_CellInfo thirdCellInfo;
ST_CellInfo fourthCellInfo;

GPS_LOCATION_T gnssData;
GPS_LOCATION_T prevGnssData;

s32 sosPinLevel;
float batteryLevel;

/* mqttp topics */


/* end mqttp topics */
    
    u8 deviceTopic[] = "device/78489383830945\0";

/* mqttp actions */
    u8 trackOnDemand[] = "trackOnDemand\0";
    u8 setGprsInterval[] = "setGprsInterval\0";
    u8 setAuthPhone[] = "setAuthPhone\0";
    u8 setOverSpeedLimit[] = "setOverSpeedLimit\0";
    u8 setMovementAlarmRadius[] = "setMovementAlarmRadius\0";
    u8 setGeofenceAlarm[] = "setGeofenceAlarm\0";
    u8 initialize[] = "initialize\0";
    u8 setSleepMode[] = "setSleepMode\0";
    u8 outputControl[] = "outputControl\0";
    u8 armDisarm[] = "armDisarm\0";
    u8 setGprsIntervalOnStop[] = "setGprsIntervalOnStop\0";
    u8 setTimeZone[] = "setTimeZone\0";
    u8 setInitOrdometer[] = "setInitOrdometer\0";
    u8 rebootDevice[] = "rebootDevice\0";
    u8 setHeartBeat[] = "setHeartBeat\0";
    u8 clearDataLogger[] = "clearDataLogger\0";
    u8 getFirmwareVersion[] = "getFirmwareVersion\0";
    u8 readGprsInterval[] = "readGprsInterval\0";
    u8 setAuthorizationTag[] = "setAuthorizationTag\0";
    u8 readAuthorizationTag[] = "readAuthorizationTag\0";

/*  end mqttp actions*/



static void Watchdog_Init(void) {
    // Init watchdog
    Ql_WTD_Init(0, -1, 60);
    wtdId = Ql_WTD_Start(20);    // 60s timeout
    APP_DEBUG("[WDT] Watchdog started (60s) with id %d\r\n",wtdId);
}





// ---------------------------------------------------

// ---------------------------------------------------
// AT+QENG response handler
// ---------------------------------------------------






static s32 AT_CELL_INFO_HANDLER(char* line, u32 len, void* userdata)
{
    char *loclnfo = (char *)userdata;
    char* head = Ql_RIL_FindString(line, len, "+CCED: 1"); //continue wait
    if (head) {
        char* tempData = Ql_MEM_Alloc(Ql_strlen(line));
        Ql_memset(tempData,0,Ql_strlen(tempData));
        Ql_strcpy(tempData,line);
        tempData = tempData + 9;
        Ql_strcpy(loclnfo,tempData);

        //Ql_WTD_Feed(wtdId);
        return RIL_ATRSP_CONTINUE;
    }



    /*head = Ql_RIL_FindString(line, len, "OK"); //continue wait
    if (head) {
        return RIL_ATRSP_SUCCESS;
    }*/


    head = Ql_RIL_FindString(line, len, "+CME ERROR:");// find <CR><LF>ERROR<CR><LF>, <CR>ERROR<CR>��<LF>ERROR<LF>
    if(head)
    {  
        return  RIL_ATRSP_FAILED;
    }


    head = Ql_RIL_FindString(line, len, "ERROR");// find <CR><LF>ERROR<CR><LF>, <CR>ERROR<CR>��<LF>ERROR<LF>
    if(head)
    {  
        return  RIL_ATRSP_FAILED;
    }


    return RIL_ATRSP_SUCCESS;
}



static s32 AT_CELL_LOCATION_HANDLER(char* line, u32 len, void* userData){
      APP_DEBUG("forware version =%s\r\n", line);
      char* head = Ql_RIL_FindString(line, len, "+QCELLLOC:");
      if (head) {
        APP_DEBUG("<-- URC: %s -->\r\n", line);

        return RIL_ATRSP_CONTINUE;
     }

     head = Ql_RIL_FindString(line, len, "+CME ERROR:");// find <CR><LF>ERROR<CR><LF>, <CR>ERROR<CR>��<LF>ERROR<LF>
     if(head){  
         return  RIL_ATRSP_FAILED;
     }


     head = Ql_RIL_FindString(line, len, "ERROR");// find <CR><LF>ERROR<CR><LF>, <CR>ERROR<CR>��<LF>ERROR<LF>
     if(head){  
        return  RIL_ATRSP_FAILED;
     }

     head = Ql_RIL_FindString(line, len, "OK");// find <CR><LF>ERROR<CR><LF>, <CR>ERROR<CR>��<LF>ERROR<LF>
     if(head){  
        return  RIL_ATRSP_SUCCESS;
     }

     return RIL_ATRSP_CONTINUE;
}





void proc_main_task(s32 taskId)
{
    ST_MSG msg;
    s32 iResult = 0;
    s32 ret;

    //<Register & open UART port
    Watchdog_Init();
    Ql_UART_Register(m_myUartPort, CallBack_UART_Hdlr, NULL);
    Ql_UART_Open(m_myUartPort, 115200, FC_NONE);

    Ql_GPIO_Init(PINNAME_GPIO_0, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_DISABLE);
    Ql_GPIO_Init(PINNAME_GPIO_2, PINDIRECTION_IN, PINLEVEL_LOW, PINPULLSEL_PULLDOWN);

    

    APP_DEBUG("//<------------OpenCPU: MQTT Client.------------\r\n");

    //<register state timer 
    Ql_Timer_Register(MQTT_TIMER_ID, Mqtt_Callback_Timer, NULL);

	//register MQTT recv callback
    ret = Ql_Mqtt_Recv_Register(mqtt_recv);

    Ql_Timer_Register(GNSS_TIMER_ID, Gnss_Callback_Timer, NULL);
	Ql_Timer_Start(GNSS_TIMER_ID, GNSS_TIMER_PERIOD, TRUE);

    Ql_Timer_Register(PIN_TIMER_ID, Pin_Check_Callback_Timer, NULL);
	Ql_Timer_Start(PIN_TIMER_ID, PIN_TIMER_PERIOD, TRUE);

	APP_DEBUG("//<register recv callback,ret = %d\r\n",ret);

    while(TRUE)
    {
        Ql_WTD_Feed(wtdId);
        Ql_OS_GetMessage(&msg);
        switch(msg.message)
        {
            case MSG_ID_RIL_READY:
                APP_DEBUG("//<RIL is ready\r\n");
                Ql_RIL_Initialize();
                loadConfig();
                if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                    last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
                m_mqtt_state = STATE_NW_QUERY_STATE;
                break;
    		case MSG_ID_URC_INDICATION:
        		{     
        			switch (msg.param1)
                    {
            		    case URC_SIM_CARD_STATE_IND:
                            {
                    			APP_DEBUG("//<SIM Card Status:%d\r\n", msg.param2);
                				if(SIM_STAT_READY == msg.param2)
                				{
                                    if(devConfig.reportSpeed <= 0){
                                         Ql_Timer_Start(MQTT_TIMER_ID, MQTT_TIMER_PERIOD, TRUE);
                                    }
                                    else{
                                         Ql_Timer_Start(MQTT_TIMER_ID,(devConfig.reportSpeed * 1000), TRUE);
                                    }
                                   
                				   APP_DEBUG("//<state timer start,ret = %d\r\n",ret);
                				}
                		    }
                            break;
                        case URC_SYS_INIT_STATE_IND:
                            {
                                APP_DEBUG("<-- Sys Init Status %d -->\r\n", msg.param2);
                                if (SYS_STATE_SMSOK == msg.param2)
                                {
                                    APP_DEBUG("\r\n<-- SMS module is ready -->\r\n");
                                    APP_DEBUG("\r\n<-- Initialize SMS-related options -->\r\n");
                                    iResult = SMS_Initialize();         
                                    if (!iResult)
                                    {
                                        APP_DEBUG("Fail to initialize SMS\r\n");
                                    }

                                    SMS_TextMode_Send();
                                }
                                break;
                            }

                        case URC_GSM_NW_STATE_IND:
                            {
                                APP_DEBUG("\r\n<-- GSM Network Status:%d -->\r\n", msg.param2);
                                break;
                            }

                        case URC_GPRS_NW_STATE_IND:
                            {
                                APP_DEBUG("\r\n<-- GPRS Network Status:%d -->\r\n", msg.param2);
                                break;
                            }

                        case URC_CFUN_STATE_IND:
                            {
                                APP_DEBUG("\r\n<-- CFUN Status:%d -->\r\n", msg.param2);
                                break;
                            }

                        case URC_COMING_CALL_IND:
                            {
                                ST_ComingCall* pComingCall = (ST_ComingCall*)(msg.param2);
                                APP_DEBUG("\r\n<-- Coming call, number:%s, type:%d -->\r\n", pComingCall->phoneNumber, pComingCall->type);
                                break;
                        }

                        case URC_NEW_SMS_IND:
                            {
                                APP_DEBUG("\r\n<-- New SMS Arrives: index=%d\r\n", msg.param2);
                                Hdlr_RecvNewSMS((msg.param2), FALSE);
                                break;
                            }

                        case URC_MODULE_VOLTAGE_IND:
                            {
                                APP_DEBUG("\r\n<-- VBatt Voltage Ind: type=%d\r\n", msg.param2);
                                break;
                            }
        				case URC_MQTT_OPEN:
            				{
            					mqtt_urc_param_ptr = msg.param2;
            					if(0 == mqtt_urc_param_ptr->result)
            					{
                 					APP_DEBUG("//<Open a MQTT client successfully\r\n");
                                    if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                                         last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
                                    m_mqtt_state = STATE_MQTT_CONN;
                                    isConnectionOpen = TRUE;
            					}
            					else
            					{
            						APP_DEBUG("//<Open a MQTT client failure,error = %d\r\n",mqtt_urc_param_ptr->result);
            					}
            				}
                            break;
            		    case URC_MQTT_CONN:
            				{
            					mqtt_urc_param_ptr = msg.param2;
            					if(0 == mqtt_urc_param_ptr->result)
            					{
                    		        APP_DEBUG("//<logged in server successfully\r\n");
                                    if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                                         last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
            						m_mqtt_state = STATE_MQTT_SUB;
            					}
            					else
            					{
            						APP_DEBUG("//<Connect to MQTT server failure,error = %d\r\n",mqtt_urc_param_ptr->result);
            					}
                		    }
                            break;
                        case URC_MQTT_SUB:
            				{
            					mqtt_urc_param_ptr = msg.param2;
            					if((0 == mqtt_urc_param_ptr->result)&&(128 != mqtt_urc_param_ptr->sub_value[0]))
            					{
                    		        APP_DEBUG("//<Subscribe topics successfully\r\n");
                                    if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                                         last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
            						m_mqtt_state = STATE_MQTT_PUB;
            					}
            					else
            					{
            						APP_DEBUG("//<Subscribe topics failure,error = %d\r\n",mqtt_urc_param_ptr->result);
                                    if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                                         last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
                                    m_mqtt_state = STATE_MQTT_CONN;
            					}
                		    }
            			    break;
        				case URC_MQTT_PUB:
            				{
            					mqtt_urc_param_ptr = msg.param2;
            					if(0 == mqtt_urc_param_ptr->result)
            					{
                    		        APP_DEBUG("//<Publish messages to MQTT server successfully\r\n");
            					}
            					else
            					{
            						APP_DEBUG("//<Publish messages to MQTT server failure,error = %d\r\n",mqtt_urc_param_ptr->result);
                                    if(countPublishFailure > 10){
                                        countPublishFailure = 0;
                                        if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                                             last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
                                        m_mqtt_state = STATE_MQTT_CONN;
                                    }
            					}
                		    }
            			    break;
        			    case URC_MQTT_CLOSE:
            				{
            					mqtt_urc_param_ptr = msg.param2;
            					if(0 == mqtt_urc_param_ptr->result)
            					{
                    		        APP_DEBUG("//<Closed MQTT socket successfully\r\n");
                                    isConnectionOpen = FALSE;
            					}
            					else
            					{
            						APP_DEBUG("//<Closed MQTT socket failure,error = %d\r\n",mqtt_urc_param_ptr->result);
            					}
                		    }
            			    break;
                        case URC_MQTT_DISC:
            				{
            					mqtt_urc_param_ptr = msg.param2;
            					if(0 == mqtt_urc_param_ptr->result)
            					{
                    		        APP_DEBUG("//<Disconnect MQTT successfully\r\n");
            					}
            					else
            					{
            						APP_DEBUG("//<Disconnect MQTT failure,error = %d\r\n",mqtt_urc_param_ptr->result);
            					}
                		    }
            			    break;
        		        default:
            		        //APP_DEBUG("<-- Other URC: type=%d\r\n", msg.param1);
            	            break;
        			}
        		}
    		    break;
        	default:
                break;
        }
    }
}





static s32 ReadSerialPort(Enum_SerialPort port, /*[out]*/u8* pBuffer, /*[in]*/u32 bufLen)
{
    s32 rdLen = 0;
    s32 rdTotalLen = 0;
    if (NULL == pBuffer || 0 == bufLen)
    {
        return -1;
    }
    Ql_memset(pBuffer, 0x0, bufLen);
    while (1)
    {
        rdLen = Ql_UART_Read(port, pBuffer + rdTotalLen, bufLen - rdTotalLen);
        if (rdLen <= 0)  // All data is read out, or Serial Port Error!
        {
            break;
        }
        rdTotalLen += rdLen;
        // Continue to read...
    }
    if (rdLen < 0) // Serial Port Error!
    {
        APP_DEBUG("//<Fail to read from port[%d]\r\n", port);
        return -99;
    }
    return rdTotalLen;
}




static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
{
    char *p = NULL;
    s32 ret;
    
    switch (msg)
    {
        case EVENT_UART_READY_TO_READ:
            {
                if(m_myUartPort == port)
                {
                    s32 totalBytes = ReadSerialPort(port,m_RxBuf_Uart , sizeof(m_RxBuf_Uart));
                    Ql_memcpy(g_RxBuf_Uart1,m_RxBuf_Uart,sizeof(m_RxBuf_Uart)); // copy the received data to the gps

                }
            }
            break;
        case EVENT_UART_READY_TO_WRITE:        {
            if(m_myUartPort == port)
                {
                    if(remainLen > 0)
                    {
                        /*s32 retLen = Ql_UART_Write(m_myUartPort, m_TxBuf_Uart, remainLen);
                        if(retLen < remainLen)
                        {
                            remainLen -= ret;
                            Ql_memmove(m_TxBuf_Uart, m_TxBuf_Uart+retLen, remainLen);
                        }*/
                    }
                }
            }
            break;
        default:
            break;
    }
}





static void mqtt_recv(u8* buffer,u32 length)
{

    //+QMTRECV: 0,38899,device/23478789654577,{"action": "positionPacket","imei": "23478789654577","utcTime":-2010018745,"longitude":0,"longitudeDirection":⸮,"latitude":-618475290,"latitudeDirection":y,"speed":69,"courseOverGround":64,"deviceDate":1249320088,"magneticVariation":1082364039,"battery":78,"trip":-2118123520,"external":-1030792151}


	APP_DEBUG("//<data:%s,len:%d\r\n",buffer,length);
    char* rawMessage = my_strtok(buffer,",");
    rawMessage = rawMessage + Ql_strlen(rawMessage) + 1;
    
    char* rawMessage1 = my_strtok(rawMessage1,",");
    rawMessage1 = rawMessage1 + Ql_strlen(rawMessage1) + 1;

    char* topic = my_strtok(rawMessage1,",");

    char myTopic[25];
    Ql_memset(myTopic,0,Ql_strlen(myTopic));
    Ql_strcpy(myTopic,topic);
    u32 topicLength = Ql_strlen(topic);
    myTopic[topicLength] = '\0';

    APP_DEBUG("seen topic %s\r\n",myTopic);
    APP_DEBUG("saved topic %s\r\n",devConfig.deviceTopic);

    char *payload = topic + Ql_strlen(topic) + 1;  // Skip topic + null terminator
    APP_DEBUG("seen payload %s\r\n",payload);



    if(Ql_strcmp(myTopic,devConfig.deviceTopic) == 0){  // check if the topic is correct

           /*---------- MQTT CODE FOR USER STARTS HERE ------------------*/
            char myAction[100];
            
            char *action = extract_string(payload, "action");

            
            Ql_strcpy(myAction,action); 

            if(Ql_memcmp(myAction,trackOnDemand,sizeof(myAction)) == 0){
                 Ql_sprintf(commandData,"{action:%s,imei:%s,longitude:%d,latitude:%d,speed:%d,course:%d}",trackOnDemand,devConfig.imei,gnssData.longitude,gnssData.latitude,gnssData.speed,gnssData.courseOverGround);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,setGprsInterval,sizeof(myAction)) == 0){
                 int myInterval = extract_int(payload, "action");
                 devConfig.findIntervalMin = myInterval;
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",setGprsInterval,devConfig.imei);
                 iscommandData = 1;  
            }
            else if(Ql_memcmp(myAction,setAuthPhone,sizeof(myAction)) == 0){
                 char *phoneAuth = extract_string(payload, "phoneAuth");
                 
                 char myPhoneAuth[300];
                 Ql_strcpy(myPhoneAuth,phoneAuth); // copy out the converted json
                 Ql_strcpy(devConfig.users,myPhoneAuth); // save to the config object
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",setAuthPhone,devConfig.imei);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,setOverSpeedLimit,sizeof(myAction)) == 0){
                 int mySpeed = extract_int(payload, "speed");
                 devConfig.speedLimitKph = mySpeed;
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",setOverSpeedLimit,devConfig.imei);
                 iscommandData = 1;  
            }
            else if(Ql_memcmp(myAction,setMovementAlarmRadius,sizeof(myAction)) == 0){
                 int myMovementAlarm = extract_int(payload, "movementAlarm");
                 //devConfig.movementAlarm = myMovementAlarm;
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",setMovementAlarmRadius,devConfig.imei);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,setGeofenceAlarm,sizeof(myAction)) == 0){
                 int myGeofenceAlarm = extract_int(payload, "geofenceAlarm");
                 //devConfig.geofenceAlarm = geofenceAlarm;
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",setGeofenceAlarm,devConfig.imei);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,initialize,sizeof(myAction)) == 0){
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",initialize,devConfig.imei);
                 //publishMessage(1,myTopic,commandData); 
                 // write command for reboot
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,setSleepMode,sizeof(myAction)) == 0){
                 int mySleepMode = extract_int(payload, "mode");
                 devConfig.sleepMode = mySleepMode;
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",setSleepMode,devConfig.imei);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,outputControl,sizeof(myAction)) == 0){
                 int myOutputControl = extract_int(payload, "control");
                 devConfig.outputControl = myOutputControl;
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",outputControl,devConfig.imei);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,armDisarm,sizeof(myAction)) == 0){
                 int myArmDisarm = extract_int(payload, "arm");
                 // take action here
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",armDisarm,devConfig.imei);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,setGprsIntervalOnStop,sizeof(myAction)) == 0){
                 int myGprsIntervalOnStop = extract_int(payload, "interval");
                 devConfig.tripReporting = myGprsIntervalOnStop;
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",setGprsIntervalOnStop,devConfig.imei);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,setInitOrdometer,sizeof(myAction)) == 0){
                 int myOrdorMeter = extract_int(payload, "ordormeter");
                 devConfig.odometerKm = myOrdorMeter;
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",setInitOrdometer,devConfig.imei);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,rebootDevice,sizeof(myAction)) == 0){
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",rebootDevice,devConfig.imei);
                 //publishMessage(1,myTopic,commandData);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,setHeartBeat,sizeof(myAction)) == 0){
                 int myHeartBeat = extract_int(payload, "heartBeat");
                 devConfig.heartbeatMin = myHeartBeat;
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",setHeartBeat,devConfig.imei);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,clearDataLogger,sizeof(myAction)) == 0){
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s}",clearDataLogger,devConfig.imei);
                 //publishMessage(1,myTopic,commandData);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,getFirmwareVersion,sizeof(myAction)) == 0){
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s,version:%s}",getFirmwareVersion,devConfig.imei,devConfig.FW_VERSION);
                 //publishMessage(1,myTopic,commandData);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,readGprsInterval,sizeof(myAction)) == 0){
                 Ql_memset(commandData,0,Ql_strlen(commandData));
                 Ql_sprintf(commandData,"{action:%s,imei:%s,interval:%d}",getFirmwareVersion,devConfig.imei,devConfig.findIntervalMin);
                 iscommandData = 1;
            }
            else if(Ql_memcmp(myAction,setAuthorizationTag,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,readAuthorizationTag,sizeof(myAction)) == 0){

            }

            /*---------- MQTT CODE FOR USER ENDS HERE ------------------*/

    }
    

}




static void Pin_Check_Callback_Timer(u32 timerId, void* param){
    if(PIN_TIMER_ID == timerId){
        sosPinLevel = Ql_GPIO_GetLevel(PINNAME_GPIO_2);
        batteryLevel = get_battery_percentage(&devConfig);
        APP_DEBUG("checking pin level battery = %2f, sospinLevel = %d \r\n",batteryLevel,sosPinLevel);

        if(devConfig.trip == 1){
            Ql_GPIO_SetLevel(PINNAME_GPIO_0, PINLEVEL_HIGH); 
        }
        else{
            Ql_GPIO_SetLevel(PINNAME_GPIO_0, PINLEVEL_LOW); 
            if(devConfig.tripReporting == 1){
                Ql_memset(deviceAlarm,0,Ql_strlen(deviceAlarm));
                Ql_sprintf(deviceAlarm,
                    "{"
                        "\"action\": \"tripAlarmPacket\","
                        "\"imei\": \"%s\","
                                "\"utcTime\":%2f,"
                                "\"longitude\":%8f,"
                                "\"longitudeDirection\":\"%c\","
                                "\"latitude\":%8f,"
                                "\"latitudeDirection\":\"%c\","
                                "\"speed\":%6f,"
                                "\"courseOverGround\":%6f,"
                                "\"deviceDate\":%d,"
                                "\"magneticVariation\":%2f,"
                                "\"battery\":%f,"
                                "\"trip\":%d,"
                                "\"external\":%d,"
                                "\"ordor\":%d"
                            "}",
                            devConfig.imei,
                            gnssData.utcTime,
                            gnssData.longitude,
                            gnssData.longDirection,
                            gnssData.latitude,
                            gnssData.latDirection,
                            gnssData.speed,
                            gnssData.courseOverGround,
                            gnssData.deviceDate,
                            gnssData.magneticVariation,
                            batteryLevel,  // battery placeholder
                            devConfig.trip,  // trip placeholder
                            1,   // external placeholder
                            devConfig.odometerKm
                     );

                isDeviceAlarm = 1;
            }
        }
    } 
}



static void Mqtt_Callback_Timer(u32 timerId, void* param)
{
    s32 ret;
    
    if(MQTT_TIMER_ID == timerId)
    {
        if(devConfig.mqtt_state =  STATE_NW_QUERY_STATE){
            m_mqtt_state = STATE_NW_QUERY_STATE;
            devConfig.mqtt_state = STATE_MQTT_CFG;
        }
        switch(m_mqtt_state)
        {        
            case STATE_NW_QUERY_STATE:
            {
                s32 cgreg = 0;
                APP_DEBUG("run mode is %d --\r\n",devConfig.runMode);
                if(devConfig.runMode != 1){
                    if(isConnectionOpen == TRUE)
                          RIL_MQTT_QMTCLOSE(connect_id);
                    APP_DEBUG("run mode is zero exiting connection --\r\n");
                    break;
                }
                ret = RIL_NW_GetGPRSState(&cgreg);
                APP_DEBUG("//<Network State:cgreg = %d\r\n",cgreg);
                if((cgreg == NW_STAT_REGISTERED)||(cgreg == NW_STAT_REGISTERED_ROAMING))
                {
                    //<Set PDP context 0
                    RIL_NW_SetGPRSContext(0);
                    APP_DEBUG("//<Set PDP context 0 \r\n");
                	//<Set APN
                    APP_DEBUG("setting apn=%s, user = %s, password = %s 0 \r\n",devConfig.apn, devConfig.apnUser, devConfig.apnPass);
                	ret = RIL_NW_SetAPN(1, devConfig.apn, devConfig.apnUser, devConfig.apnPass);
                	APP_DEBUG("//<Set APN \r\n");
                    //PDP activated
                    ret = RIL_NW_OpenPDPContext();
                    if(ret == RIL_AT_SUCCESS)
                	{
                	    APP_DEBUG("//<Activate PDP context,ret = %d\r\n",ret);
                        if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                            last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
                	    m_mqtt_state = STATE_MQTT_CFG;
                	}
                }
                break;
            }
            case STATE_MQTT_CFG:
            {
                //ret = RIL_MQTT_QMTCFG_Ali(connect_id,product_key,device_name,device_secret);//<This configuration is required to connect to Ali Cloud.
                RIL_MQTT_QMTCFG_Showrecvlen(connect_id,ShowFlag_1);//<This sentence must be configured. The configuration will definitely succeed, so there is no need to care about.
				ret = RIL_MQTT_QMTCFG_Version_Select(connect_id,Version_3_1_1);
                if(RIL_AT_SUCCESS == ret)
                {
                    //APP_DEBUG("//<Ali Platform configure successfully\r\n");
                    APP_DEBUG("//<Select version 3.1.1 successfully\r\n");
                    if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                        last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
                    m_mqtt_state = STATE_MQTT_OPEN;
                }
                else
                {
                    //APP_DEBUG("//<Ali Platform configure failure,ret = %d\r\n",ret);
                    APP_DEBUG("//<Select version 3.1.1 failure,ret = %d\r\n",ret);
                }
                break;
            }
			case STATE_MQTT_OPEN:
            {
                
                APP_DEBUG("opening mqtt server = %s, port = %d\r\n",devConfig.serverHost,devConfig.serverPort);
                ret = RIL_MQTT_QMTOPEN(connect_id,devConfig.serverHost,devConfig.serverPort);
                if(RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("//<Start opening a MQTT client\r\n");
                    if(FALSE == CLOSE_flag)
                        CLOSE_flag = TRUE;
                    if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                        last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
                    m_mqtt_state = STATE_MQTT_TOTAL_NUM;
                }
                else
                {
                    APP_DEBUG("//<Open a MQTT client failure,ret = %d-->\r\n",ret);
                    
                }
                break;
            }
            case STATE_MQTT_CONN:
            {
			    APP_DEBUG("imei = %s, username = %s\r\n",devConfig.imei,username);
                ret = RIL_MQTT_QMTCONN(connect_id,devConfig.clientId,username,devConfig.imei);
	            if(RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("//<login in to mqtt serverr\n");
                    if(FALSE == DISC_flag)
                        DISC_flag = TRUE;
                    
                    if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                        last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
                    m_mqtt_state = STATE_MQTT_TOTAL_NUM;
                }
                else
                {
                    APP_DEBUG("//<failed to login = %d\r\n",ret);
                }
                break;
            }
			case STATE_MQTT_SUB:
            {				
                mqtt_topic_info_t.count = 1;
				mqtt_topic_info_t.topic[0] = (u8*)Ql_MEM_Alloc(sizeof(u8)*256);
				
				Ql_memset(mqtt_topic_info_t.topic[0],0,256);
				Ql_memcpy(mqtt_topic_info_t.topic[0],devConfig.deviceTopic,Ql_strlen(devConfig.deviceTopic));
                mqtt_topic_info_t.qos[0] = QOS1_AT_LEASET_ONCE;
				sub_message_id++;  //< 1-65535.
				
				ret = RIL_MQTT_QMTSUB(connect_id,sub_message_id,&mqtt_topic_info_t);
				
				Ql_MEM_Free(mqtt_topic_info_t.topic[0]);
	            mqtt_topic_info_t.topic[0] = NULL;
                if(RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("//<Start subscribe topic\r\n");
                    if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                        last_m_mqtt_state = m_mqtt_state; // save last mqtt state;
                    m_mqtt_state = STATE_MQTT_TOTAL_NUM;
                }
                else
                {
                    APP_DEBUG("//<Subscribe topic failure,ret = %d\r\n",ret);
                    
                }
                break;
            }
            case STATE_MQTT_PUB:
            {
				pub_message_id++;  //< The range is 0-65535. It will be 0 only when<qos>=0.

                u8 wasAnyPublish = 0; // checks if there was a publish
                
                if(isDeviceData == 1){
                     if( (Ql_GetMsSincePwrOn() - lastLocationPush) >= (devConfig.rptSec * 1000) ){
                          ret = RIL_MQTT_QMTPUB(connect_id,pub_message_id,QOS1_AT_LEASET_ONCE,0,devConfig.deviceTopic,Ql_strlen(deviceData),deviceData);
                          wasAnyPublish = 1;
                     }
                }


                if(isDeviceAlarm == 1){
                     ret = RIL_MQTT_QMTPUB(connect_id,pub_message_id,QOS1_AT_LEASET_ONCE,0,devConfig.deviceTopic,Ql_strlen(deviceAlarm),deviceAlarm);
                     wasAnyPublish = 1;
                }

                if(iscommandData == 1){
				     ret = RIL_MQTT_QMTPUB(connect_id,pub_message_id,QOS1_AT_LEASET_ONCE,0,devConfig.deviceTopic,Ql_strlen(commandData),commandData);
                     wasAnyPublish = 1;
                }

                if(wasAnyPublish == 1){
                    if(RIL_AT_SUCCESS == ret )
                    {
                        APP_DEBUG("//<Start publish a message to MQTT server\r\n");
                        m_mqtt_state = STATE_MQTT_PUB;
                        if(m_mqtt_state != STATE_MQTT_TOTAL_NUM)
                            last_m_mqtt_state = m_mqtt_state; // save last mqtt state;

                        if(isDeviceData == 1){
                            lastLocationPush = Ql_GetMsSincePwrOn();
                            isDeviceData = 0;
                        }
                        
                        if(iscommandData == 1)
                        iscommandData = 0;

                        if(isDeviceAlarm == 1)
                        isDeviceAlarm = 0;
                    }
                    else
                    {
                        APP_DEBUG("//<Publish a message to MQTT server failure,ret = %d\r\n",ret);
                        countPublishFailure = countPublishFailure + 1;
                        if(countPublishFailure >= 10){
                            RIL_MQTT_QMTCLOSE(connect_id);
                            m_mqtt_state = STATE_MQTT_OPEN;
                            countPublishFailure = 0;
                        }
                    }
                }
                else{
                    APP_DEBUG("//<no publishh yet \r\n");
                }

                
                break;
            }
			case STATE_MQTT_TOTAL_NUM:
            {
                //<do nothing
                APP_DEBUG("//mqtt doing nothing\r\n");
                countMqttResponseWaitTime = countMqttResponseWaitTime + 1;
                if(countMqttResponseWaitTime > 150){
                    countMqttResponseWaitTime = 0;
                    m_mqtt_state = last_m_mqtt_state;
                }
			    break;
            }
            default:
                break;
        }    
    }
}




bool GNSS_Get_FixStatus_from_RMC(u8 *NMEA_Str)
{
	char *p1 = NMEA_Str;
	bool fixed = FALSE;
	u8 comma = 0;
	u16 strLen = 0;

	if(NMEA_Str == NULL)
	{
		return FALSE;
	}
	strLen = Ql_strlen(NMEA_Str);
	for(u8 i = 0 ; i < strLen; i++)
	{
		if(*p1 == ',')
		{
			comma++;
			if(comma == 2)
			{
				break;
			}
		}
		p1++;
	}
	p1++;
	return (*p1 == 'A')?TRUE:FALSE;
}



void Callback_GNSS_APGS_Hdlr(char *str_URC)
{
		char* p1 = NULL;
        char* p2 = NULL;
        char strTmp[20];
        s32 len = Ql_strlen("\r\n+QGAGPS: ");
        
        if (Ql_StrPrefixMatch(str_URC, "\r\n+QGAGPS:"))
        {
            p1 = Ql_strstr(str_URC, "\r\n+QGAGPS: ");
            p1 += len;
            p2 = Ql_strstr(p1, "\r\n");
            if (p1 && p2)
            {
				Ql_memset(strTmp, 0x0, sizeof(strTmp));
				Ql_memcpy(strTmp, p1, p2 - p1);
				if(0 == Ql_atoi(strTmp))
				{
					m_gnss_state = STATE_GNSS_AGPS_AID;
					APP_DEBUG("\r\n<--Download AGPS data successful-->\r\n");
				}
				else
				{
					APP_DEBUG("\r\n<--Download AGPS data failed-->\r\n");
				}
            }
        }
}


static void Gnss_Callback_Timer(u32 timerId, void* param)
{
    s32 ret;
    if (GNSS_TIMER_ID == timerId)
    {
        switch (m_gnss_state)
        {
			case STATE_GNSS_POWERON:
			{
				ret = RIL_GNSS_Open(1);
				if(ret == RIL_AT_SUCCESS)
				{
					APP_DEBUG("\r\n<-- Open GNSS OK-->\r\n");
					m_gnss_state = STATE_GNSS_QUERY_STATE;
				}
				else
				{
					APP_DEBUG("\r\n<-- Open GNSS fail -->\r\n");
				}
				break;
			}
			case STATE_GNSS_QUERY_STATE:
            {
                s32 creg = 0;
                s32 cgreg = 0;
                ret = RIL_NW_GetGSMState(&creg);
                ret = RIL_NW_GetGPRSState(&cgreg);
                APP_DEBUG("<--Network State:creg=%d,cgreg=%d-->\r\n",creg,cgreg);
                if((cgreg == NW_STAT_REGISTERED)||(cgreg == NW_STAT_REGISTERED_ROAMING))
                {
                    m_gnss_state = STATE_GNSS_APN_CONFIG;
                }
                else{
                    m_gnss_state = STATE_GNSS_CHECK_FIX;
                }
                break;
            }
			case STATE_GNSS_APN_CONFIG:
            {
                ret = RIL_GNSS_EPO_Config_APN(0,devConfig.apn,devConfig.apnUser,devConfig.apnPass);
                if (RIL_ATRSP_SUCCESS == ret)
                {
                    APP_DEBUG("<--configure APN of GNSS context OK.-->\r\n");
					m_gnss_state = STATE_GNSS_PDP_Context;
                }
				else
                {
                    APP_DEBUG("<--configure APN of GNSS context fail,ret=%d.-->\r\n",ret);
                    m_gnss_state = STATE_GNSS_CHECK_FIX;
                }
                break;
            }
			case STATE_GNSS_PDP_Context:
			{
				ret = RIL_NW_OpenPDPContext();
                if (RIL_ATRSP_SUCCESS == ret)
                {
                    APP_DEBUG("<--GNSS PDP active sucessful.-->\r\n");
					m_gnss_state = STATE_GNSS_AGPS_START;
                }
				else
                {
                    APP_DEBUG("<--GNSS PDP active fail,ret=%d.-->\r\n",ret);
                    m_gnss_state = STATE_GNSS_CHECK_FIX;
                }
                break;
			}
            case STATE_GNSS_AGPS_START:
            {
				ret = RIL_GNSS_AGPS(Callback_GNSS_APGS_Hdlr);
				if(ret == RIL_ATRSP_SUCCESS)
				{
					m_gnss_state = STATE_GNSS_TOTAL_NUM;
                    APP_DEBUG("Start Download AGPS data, iRet = %d.\r\n", ret);
				}
                else
                {
					APP_DEBUG("<--Enable EPO download fail.-->\r\n");
                    m_gnss_state = STATE_GNSS_CHECK_FIX;
                }
                break;
            }
			case STATE_GNSS_AGPS_AID:
            {
				ret = RIL_GNSS_AGPSAID();
                if(RIL_AT_SUCCESS != ret) 
                {
                    APP_DEBUG("AGPS aiding fail, iRet = %d.\r\n", ret);
                    break;
                }
				m_gnss_state = STATE_GNSS_CHECK_FIX;
                APP_DEBUG("AGPS aiding successful, iRet = %d.\r\n", ret);
                break;
            }
			case STATE_GNSS_CHECK_FIX:
            {
				u8 rd_buf[1024] = {0};
				s32 ret = RIL_GNSS_Read("RMC", rd_buf);
				if(ret == RIL_ATRSP_SUCCESS)
				{
					APP_DEBUG("read buffer %s\r\n", rd_buf);
					if(GNSS_Get_FixStatus_from_RMC(rd_buf) == TRUE)
					{
					    APP_DEBUG("\r\n<--GNSS Successful position sucessful-->\r\n");
                        char* headCharacter = my_strtok(rd_buf,":");
                        headCharacter = headCharacter + Ql_strlen(headCharacter) + 1;
                        char* mainGpsData = my_strtok(headCharacter,":");
                        decode_gps_data(mainGpsData,&gnssData);
                        APP_DEBUG("longitude: %f\r\n", gnssData.longitude);
                        APP_DEBUG("latitude: %f\r\n", gnssData.latitude);
                        APP_DEBUG("speed: %f\r\n", gnssData.speed);
                        APP_DEBUG("status: %c\r\n", gnssData.status);
                        APP_DEBUG("GNSS Data: %s\r\n", mainGpsData);

                        devConfig.longitude = gnssData.longitude;
                        devConfig.latitude = gnssData.latitude;
                        devConfig.speed = gnssData.speed;

                        Ql_memcpy(&prevGnssData, &gnssData, sizeof(GPS_LOCATION_T));

                        double distanceCovered = calculateDistance(gnssData.latitude,gnssData.longitude,prevGnssData.latitude,prevGnssData.longitude);
                        APP_DEBUG("Distance = %d meters\r\n", (int)distanceCovered);
                        devConfig.odometerKm = devConfig.odometerKm + (u32)distanceCovered;
                        u8 ordoMeterArr[4];
                        ordoMeterArr[0] = (devConfig.odometerKm >> 24) & 0xff;
                        ordoMeterArr[1] = (devConfig.odometerKm >> 16) & 0xff;
                        ordoMeterArr[2] = (devConfig.odometerKm >> 8) & 0xff;
                        ordoMeterArr[3] = (devConfig.odometerKm >> 0) & 0xff;

                        saveBytesToFlash("odometerKm.txt",ordoMeterArr,sizeof(ordoMeterArr)); 

                        Ql_memset(deviceData,0,Ql_strlen(deviceData));
                        Ql_sprintf(deviceData,
                            "{"
                                "\"action\": \"positionPacket\","
                                "\"imei\": \"%s\","
                                "\"utcTime\":%2f,"
                                "\"longitude\":%8f,"
                                "\"longitudeDirection\":\"%c\","
                                "\"latitude\":%8f,"
                                "\"latitudeDirection\":\"%c\","
                                "\"speed\":%6f,"
                                "\"courseOverGround\":%6f,"
                                "\"deviceDate\":%d,"
                                "\"magneticVariation\":%2f,"
                                "\"battery\":%f,"
                                "\"trip\":%d,"
                                "\"external\":%d,"
                                "\"ordor\":%d"
                            "}",
                            devConfig.imei,
                            gnssData.utcTime,
                            gnssData.longitude,
                            gnssData.longDirection,
                            gnssData.latitude,
                            gnssData.latDirection,
                            gnssData.speed,
                            gnssData.courseOverGround,
                            gnssData.deviceDate,
                            gnssData.magneticVariation,
                            batteryLevel,  // battery placeholder
                            devConfig.trip,  // trip placeholder
                            1,   // external placeholder
                            devConfig.odometerKm
                        );
                        isDeviceData = 1; // set mqtt to publish when gps is set
						m_gnss_state = STATE_GNSS_CHECK_FIX;
					}
					else
					{
						APP_DEBUG("<--GPS not fixed.-->\r\n");
                        if(gnssData.status == 'A'){
                            Ql_memset(deviceData,0,Ql_strlen(deviceData));
                            Ql_sprintf(deviceData,
                                "{"
                                    "\"action\": \"positionPacket\","
                                    "\"imei\": \"%s\","
                                    "\"utcTime\":%2f,"
                                    "\"longitude\":%8f,"
                                    "\"longitudeDirection\":\"%c\","
                                    "\"latitude\":%8f,"
                                    "\"latitudeDirection\":\"%c\","
                                    "\"speed\":%6f,"
                                    "\"courseOverGround\":%6f,"
                                    "\"deviceDate\":%d,"
                                    "\"magneticVariation\":%2f,"
                                    "\"battery\":%f,"
                                    "\"trip\":%d,"
                                    "\"external\":%d,"
                                    "\"ordor\":%d"
                                "}",
                                devConfig.imei,
                                gnssData.utcTime,
                                gnssData.longitude,
                                gnssData.longDirection,
                                gnssData.latitude,
                                gnssData.latDirection,
                                gnssData.speed,
                                gnssData.courseOverGround,
                                gnssData.deviceDate,
                                gnssData.magneticVariation,
                                batteryLevel,  // battery placeholder
                                devConfig.trip,  // trip placeholder
                                1,   // external placeholder
                                devConfig.odometerKm
                            );
                            isDeviceData = 1; // set mqtt to publish when gps is set
                        }

					}
				}
                else
                {
					APP_DEBUG("<--Read RMC fail.-->\r\n");
                    if(gnssData.status == 'A'){
                        Ql_memset(deviceData,0,Ql_strlen(deviceData));
                        Ql_sprintf(deviceData,
                            "{"
                                "\"action\": \"positionPacket\","
                                "\"imei\": \"%s\","
                                "\"utcTime\":%2f,"
                                "\"longitude\":%8f,"
                                "\"longitudeDirection\":\"%c\","
                                "\"latitude\":%8f,"
                                "\"latitudeDirection\":\"%c\","
                                "\"speed\":%6f,"
                                "\"courseOverGround\":%6f,"
                                "\"deviceDate\":%d,"
                                "\"magneticVariation\":%2f,"
                                "\"battery\":%f,"
                                "\"trip\":%d,"
                                "\"external\":%d,"
                                "\"ordor\":%d"
                            "}",
                            devConfig.imei,
                            gnssData.utcTime,
                            gnssData.longitude,
                            gnssData.longDirection,
                            gnssData.latitude,
                            gnssData.latDirection,
                            gnssData.speed,
                            gnssData.courseOverGround,
                            gnssData.deviceDate,
                            gnssData.magneticVariation,
                            batteryLevel,  // battery placeholder
                            devConfig.trip,  // trip placeholder
                            1,   // external placeholder
                            devConfig.odometerKm
                        );
                        isDeviceData = 1; // set mqtt to publish when gps is set
                    }
                }
                break;
            }		
            default:
                break;
        }    
    }
}




static double toRadians(double deg) {
    return deg * PI / 180.0;
}

// Haversine formula to calculate distance
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = toRadians(lat2 - lat1);
    double dLon = toRadians(lon2 - lon1);

    lat1 = toRadians(lat1);
    lat2 = toRadians(lat2);

    double a = pow(sin(dLat / 2), 2) +
               cos(lat1) * cos(lat2) * pow(sin(dLon / 2), 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return EARTH_RADIUS * c;  // distance in meters
}










/******************************************************************************************/

/* Handling sms sending */

/******************************************************************************************/

static s8 ConSMSBuf_GetIndex(ConSMSStruct *pCSBuf,u8 uCSMaxCnt,ST_RIL_SMS_Con *pCon)
{
	u8 uIdx = 0;
	
    if(    (NULL == pCSBuf) || (0 == uCSMaxCnt) 
        || (NULL == pCon)
      )
    {
        APP_DEBUG("Enter ConSMSBuf_GetIndex,FAIL! Parameter is INVALID. pCSBuf:%x,uCSMaxCnt:%d,pCon:%x\r\n",pCSBuf,uCSMaxCnt,pCon);
        return -1;
    }

    if((pCon->msgTot) > CON_SMS_MAX_SEG)
    {
        APP_DEBUG("Enter ConSMSBuf_GetIndex,FAIL! msgTot:%d is larger than limit:%d\r\n",pCon->msgTot,CON_SMS_MAX_SEG);
        return -1;
    }
    
	for(uIdx = 0; uIdx < uCSMaxCnt; uIdx++)  //Match all exist records
	{
        if(    (pCon->msgRef == pCSBuf[uIdx].uMsgRef)
            && (pCon->msgTot == pCSBuf[uIdx].uMsgTot)
          )
        {
            return uIdx;
        }
	}

	for (uIdx = 0; uIdx < uCSMaxCnt; uIdx++)
	{
		if (0 == pCSBuf[uIdx].uMsgTot)  //Find the first unused record
		{
            pCSBuf[uIdx].uMsgTot = pCon->msgTot;
            pCSBuf[uIdx].uMsgRef = pCon->msgRef;
            
			return uIdx;
		}
	}

    APP_DEBUG("Enter ConSMSBuf_GetIndex,FAIL! No avail index in ConSMSBuf,uCSMaxCnt:%d\r\n",uCSMaxCnt);
    
	return -1;
}

/*****************************************************************************
 * FUNCTION
 *  ConSMSBuf_AddSeg
 *
 * DESCRIPTION
 *  This function is used to add segment in <pCSBuf>
 *  
 * PARAMETERS
 *  <pCSBuf>     The SMS index in storage,it starts from 1
 *  <uCSMaxCnt>  TRUE: The module should reply a SMS to the sender; FALSE: The module only read this SMS.
 *  <uIdx>       Index of <pCSBuf> which will be stored
 *  <pCon>       The pointer of 'ST_RIL_SMS_Con' data
 *  <pData>      The pointer of CON-SMS-SEG data
 *  <uLen>       The length of CON-SMS-SEG data
 *
 * RETURNS
 *  FALSE:   FAIL!
 *  TRUE: SUCCESS.
 *
 * NOTE
 *  1. This is an internal function
 *****************************************************************************/
static bool ConSMSBuf_AddSeg(ConSMSStruct *pCSBuf,u8 uCSMaxCnt,u8 uIdx,ST_RIL_SMS_Con *pCon,u8 *pData,u16 uLen)
{
    u8 uSeg = 1;
    
    if(    (NULL == pCSBuf) || (0 == uCSMaxCnt) 
        || (uIdx >= uCSMaxCnt)
        || (NULL == pCon)
        || (NULL == pData)
        || (uLen > (CON_SMS_SEG_MAX_CHAR * 4))
      )
    {
        APP_DEBUG("Enter ConSMSBuf_AddSeg,FAIL! Parameter is INVALID. pCSBuf:%x,uCSMaxCnt:%d,uIdx:%d,pCon:%x,pData:%x,uLen:%d\r\n",pCSBuf,uCSMaxCnt,uIdx,pCon,pData,uLen);
        return FALSE;
    }

    if((pCon->msgTot) > CON_SMS_MAX_SEG)
    {
        APP_DEBUG("Enter ConSMSBuf_GetIndex,FAIL! msgTot:%d is larger than limit:%d\r\n",pCon->msgTot,CON_SMS_MAX_SEG);
        return FALSE;
    }

    uSeg = pCon->msgSeg;
    pCSBuf[uIdx].abSegValid[uSeg-1] = TRUE;
    Ql_memcpy(pCSBuf[uIdx].asSeg[uSeg-1].aData,pData,uLen);
    pCSBuf[uIdx].asSeg[uSeg-1].uLen = uLen;
    
	return TRUE;
}

/*****************************************************************************
 * FUNCTION
 *  ConSMSBuf_IsIntact
 *
 * DESCRIPTION
 *  This function is used to check the CON-SMS is intact or not
 *  
 * PARAMETERS
 *  <pCSBuf>     The SMS index in storage,it starts from 1
 *  <uCSMaxCnt>  TRUE: The module should reply a SMS to the sender; FALSE: The module only read this SMS.
 *  <uIdx>       Index of <pCSBuf> which will be stored
 *  <pCon>       The pointer of 'ST_RIL_SMS_Con' data
 *
 * RETURNS
 *  FALSE:   FAIL!
 *  TRUE: SUCCESS.
 *
 * NOTE
 *  1. This is an internal function
 *****************************************************************************/
static bool ConSMSBuf_IsIntact(ConSMSStruct *pCSBuf,u8 uCSMaxCnt,u8 uIdx,ST_RIL_SMS_Con *pCon)
{
    u8 uSeg = 1;
	
    if(    (NULL == pCSBuf) 
        || (0 == uCSMaxCnt) 
        || (uIdx >= uCSMaxCnt)
        || (NULL == pCon)
      )
    {
        APP_DEBUG("Enter ConSMSBuf_IsIntact,FAIL! Parameter is INVALID. pCSBuf:%x,uCSMaxCnt:%d,uIdx:%d,pCon:%x\r\n",pCSBuf,uCSMaxCnt,uIdx,pCon);
        return FALSE;
    }

    if((pCon->msgTot) > CON_SMS_MAX_SEG)
    {
        APP_DEBUG("Enter ConSMSBuf_GetIndex,FAIL! msgTot:%d is larger than limit:%d\r\n",pCon->msgTot,CON_SMS_MAX_SEG);
        return FALSE;
    }
        
	for (uSeg = 1; uSeg <= (pCon->msgTot); uSeg++)
	{
        if(FALSE == pCSBuf[uIdx].abSegValid[uSeg-1])
        {
            APP_DEBUG("Enter ConSMSBuf_IsIntact,FAIL! uSeg:%d has not received!\r\n",uSeg);
            return FALSE;
        }
	}
    
    return TRUE;
}

/*****************************************************************************
 * FUNCTION
 *  ConSMSBuf_ResetCtx
 *
 * DESCRIPTION
 *  This function is used to reset ConSMSBuf context
 *  
 * PARAMETERS
 *  <pCSBuf>     The SMS index in storage,it starts from 1
 *  <uCSMaxCnt>  TRUE: The module should reply a SMS to the sender; FALSE: The module only read this SMS.
 *  <uIdx>       Index of <pCSBuf> which will be stored
 *
 * RETURNS
 *  FALSE:   FAIL!
 *  TRUE: SUCCESS.
 *
 * NOTE
 *  1. This is an internal function
 *****************************************************************************/
static bool ConSMSBuf_ResetCtx(ConSMSStruct *pCSBuf,u8 uCSMaxCnt,u8 uIdx)
{
    if(    (NULL == pCSBuf) || (0 == uCSMaxCnt) 
        || (uIdx >= uCSMaxCnt)
      )
    {
        APP_DEBUG("Enter ConSMSBuf_ResetCtx,FAIL! Parameter is INVALID. pCSBuf:%x,uCSMaxCnt:%d,uIdx:%d\r\n",pCSBuf,uCSMaxCnt,uIdx);
        return FALSE;
    }
    
    //Default reset
    Ql_memset(&pCSBuf[uIdx],0x00,sizeof(ConSMSStruct));

    //TODO: Add special reset here
    
    return TRUE;
}

/*****************************************************************************
 * FUNCTION
 *  SMS_Initialize
 *
 * DESCRIPTION
 *  Initialize SMS environment.
 *  
 * PARAMETERS
 *  VOID
 *
 * RETURNS
 *  TRUE:  This function works SUCCESS.
 *  FALSE: This function works FAIL!
 *****************************************************************************/
static bool SMS_Initialize(void)
{
    s32 iResult = 0;
    u8  nCurrStorage = 0;
    u32 nUsed = 0;
    u32 nTotal = 0;
    
    // Set SMS storage:
    // By default, short message is stored into SIM card. You can change the storage to ME if needed, or
    // you can do it again to make sure the short message storage is SIM card.
    #if 0
    {
        iResult = RIL_SMS_SetStorage(RIL_SMS_STORAGE_TYPE_SM,&nUsed,&nTotal);
        if (RIL_ATRSP_SUCCESS != iResult)
        {
            APP_DEBUG("Fail to set SMS storage, cause:%d\r\n", iResult);
            return FALSE;
        }
        APP_DEBUG("<-- Set SMS storage to SM, nUsed:%u,nTotal:%u -->\r\n", nUsed, nTotal);

        iResult = RIL_SMS_GetStorage(&nCurrStorage, &nUsed ,&nTotal);
        if(RIL_ATRSP_SUCCESS != iResult)
        {
            APP_DEBUG("Fail to get SMS storage, cause:%d\r\n", iResult);
            return FALSE;
        }
        APP_DEBUG("<-- Check SMS storage: curMem=%d, used=%d, total=%d -->\r\n", nCurrStorage, nUsed, nTotal);
    }
    #endif

    // Enable new short message indication
    // By default, the auto-indication for new short message is enalbed. You can do it again to 
    // make sure that the option is open.
    #if 0
    {
        iResult = Ql_RIL_SendATCmd("AT+CNMI=2,1",Ql_strlen("AT+CNMI=2,1"),NULL,NULL,0);
        if (RIL_AT_SUCCESS != iResult)
        {
            APP_DEBUG("Fail to send \"AT+CNMI=2,1\", cause:%d\r\n", iResult);
            return FALSE;
        }
        APP_DEBUG("<-- Enable new SMS indication -->\r\n");
    }
    #endif

    // Delete all existed short messages (if needed)
    //iResult = RIL_SMS_DeleteSMS(0, RIL_SMS_DEL_ALL_MSG);
    iResult = RIL_SMS_DeleteSMS(1, RIL_SMS_DEL_ALL_MSG);
    if (iResult != RIL_AT_SUCCESS)
    {
        APP_DEBUG("Fail to delete all messages, iResult=%d,cause:%d\r\n", iResult, Ql_RIL_AT_GetErrCode());
        return FALSE;
    }
    APP_DEBUG("Delete all existed messages\r\n");
    
    return TRUE;
}



void controlRelay(u8 myControl){
    if(myControl == 0x01){
        Ql_GPIO_SetLevel(PINNAME_GPIO_0, PINLEVEL_HIGH);   // set GPIO1 = HIGH
    }
    else{
       Ql_GPIO_SetLevel(PINNAME_GPIO_0, PINLEVEL_LOW);
    }
}

void SMS_TextMode_Read(u32 nIndex)
{
    s32 iResult;
    ST_RIL_SMS_TextInfo *pTextInfo = NULL;
    ST_RIL_SMS_DeliverParam *pDeliverTextInfo = NULL;
    ST_RIL_SMS_SubmitParam *pSubmitTextInfo = NULL;
    LIB_SMS_CharSetEnum eCharSet = LIB_SMS_CHARSET_GSM;
    
    pTextInfo = Ql_MEM_Alloc(sizeof(ST_RIL_SMS_TextInfo));
    if (NULL == pTextInfo)
    {
        return;
    }        

    Ql_memset(pTextInfo,0x00,sizeof(ST_RIL_SMS_TextInfo));
    iResult = RIL_SMS_ReadSMS_Text(nIndex, eCharSet, pTextInfo);
    if (iResult != RIL_AT_SUCCESS)
    {
        Ql_MEM_Free(pTextInfo);
        APP_DEBUG("< Fail to read PDU SMS, cause:%d >\r\n", iResult);
        return;
    }        
    if (RIL_SMS_STATUS_TYPE_INVALID == (pTextInfo->status))
    {
        APP_DEBUG("<-- SMS[index=%d] doesn't exist -->\r\n", nIndex);
        return;
    }

    // Resolve the read short message
    if (LIB_SMS_PDU_TYPE_DELIVER == (pTextInfo->type))
    {
        pDeliverTextInfo = &((pTextInfo->param).deliverParam);
        APP_DEBUG("<-- Read short message (index:%u) with charset %d -->\r\n", nIndex, eCharSet);

        if(FALSE == pDeliverTextInfo->conPres) //Normal SMS
        {
            APP_DEBUG(
                "short message info: \r\n\tstatus:%u \r\n\ttype:%u \r\n\talpha:%u \r\n\tsca:%s \r\n\toa:%s \r\n\tscts:%s \r\n\tdata length:%u\r\ncp:0,cy:0,cr:0,ct:0,cs:0\r\n",
                    (pTextInfo->status),
                    (pTextInfo->type),
                    (pDeliverTextInfo->alpha),
                    (pTextInfo->sca),
                    (pDeliverTextInfo->oa),
                    (pDeliverTextInfo->scts),
                    (pDeliverTextInfo->length)
           );
        }
        else
        {
            APP_DEBUG(
                "short message info: \r\n\tstatus:%u \r\n\ttype:%u \r\n\talpha:%u \r\n\tsca:%s \r\n\toa:%s \r\n\tscts:%s \r\n\tdata length:%u\r\ncp:1,cy:%d,cr:%d,ct:%d,cs:%d\r\n",
                    (pTextInfo->status),
                    (pTextInfo->type),
                    (pDeliverTextInfo->alpha),
                    (pTextInfo->sca),
                    (pDeliverTextInfo->oa),
                    (pDeliverTextInfo->scts),
                    (pDeliverTextInfo->length),
                    pDeliverTextInfo->con.msgType,
                    pDeliverTextInfo->con.msgRef,
                    pDeliverTextInfo->con.msgTot,
                    pDeliverTextInfo->con.msgSeg
           );
        }
        
        APP_DEBUG("\r\n\tmessage content:");
        APP_DEBUG("%s\r\n",(pDeliverTextInfo->data));
        APP_DEBUG("\r\n");
    }
    else if (LIB_SMS_PDU_TYPE_SUBMIT == (pTextInfo->type))
    {// short messages in sent-list of drafts-list
    } else {
        APP_DEBUG("<-- Unkown short message type! type:%d -->\r\n", (pTextInfo->type));
    }
    Ql_MEM_Free(pTextInfo);
}

void SMS_TextMode_Send(void)
{
    s32 iResult;
    u32 nMsgRef;
    char strPhNum[] = "+8610086\0";
    char strTextMsg[] = "Quectel Module SMS Test\0";
    char strConMsgSeg1[] = "Quectel Module CON-SMS Test Start:<< GSM/GPRS modules are based on open standards and meet the requirements of applicable international standards and leg\0";
    char strConMsgSeg2[] = "islation. Our leading edge products and services that cherish the environment and support sustainable development are the result of our own know-how and \0";
    char strConMsgSeg3[] = "a perfect fit to the demands on the market. With compact size, low power consumption and extended temperature, our GSM/GPRS modules are used in a wide ra\0";
    char strConMsgSeg4[] = "nge of applications, such as automobile, VTS, Smart Metering, Wireless POS, security and a multitude of embedded devices in many industries. >> Test End.\0";
    char strConMsgChSeg1[] = "79FB8FDC6A215757957F77ED4FE16D4B8BD55F0059CBFF1A300A79FB8FDC901A4FE162E567099886514876844EA754C1548C670D52A1FF0C00470053004D002F004700500052005300206A21575757FA4E8E5F00653E548C7B26540856FD9645680751C66CD589C4768489816C42751F4EA78BBE8BA1FF0C517767097D2751D176845C3A5BF8\0";
    char strConMsgChSeg2[] = "30014F4E529F8017300162695C556E295EA67B494F1852BF0020300279FB8FDC901A4FE100470053004D002F00470050005200536A2157575E7F6CDB5E9475284E8E8F668F7D3001667A80FD8BA191CF30018FDC7A0B76D163A7300172694F538DDF8E2A300165E07EBF4ED86B3E30015B89516876D163A7300179FB52A88BA17B977B494F17\0";
    char strConMsgChSeg3[] = "591A884C4E1A76845D4C51655F0F8BBE59074E2D3002300B6D4B8BD57ED3675F\0";

    ST_RIL_SMS_SendExt sExt;

    //Initialize
    Ql_memset(&sExt,0x00,sizeof(sExt));

    APP_DEBUG("< Send Normal Text SMS begin... >\r\n");
    
    iResult = RIL_SMS_SendSMS_Text(strPhNum, Ql_strlen(strPhNum), LIB_SMS_CHARSET_GSM, strTextMsg, Ql_strlen(strTextMsg), &nMsgRef);
    if (iResult != RIL_AT_SUCCESS)
    {   
        APP_DEBUG("< Fail to send Text SMS, iResult=%d, cause:%d >\r\n", iResult, Ql_RIL_AT_GetErrCode());
        return;
    }
    APP_DEBUG("< Send Text SMS successfully, MsgRef:%u >\r\n", nMsgRef);

    
}

void SMS_PDUMode_Read(u32 nIndex)
{
    s32 iResult;
    ST_RIL_SMS_PDUInfo *pPDUInfo = NULL;

    pPDUInfo = Ql_MEM_Alloc(sizeof(ST_RIL_SMS_PDUInfo));
    if (NULL == pPDUInfo)
    {
        return;
    }
    
    iResult = RIL_SMS_ReadSMS_PDU(nIndex, pPDUInfo);
    if (RIL_AT_SUCCESS != iResult)
    {
        Ql_MEM_Free(pPDUInfo);
        APP_DEBUG("< Fail to read PDU SMS, cause:%d >\r\n", iResult);
        return;
    }

    do
    {
        if (RIL_SMS_STATUS_TYPE_INVALID == (pPDUInfo->status))
        {
            APP_DEBUG("<-- SMS[index=%d] doesn't exist -->\r\n", nIndex);
            break;
        }

        APP_DEBUG("<-- Send Text SMS[index=%d] successfully -->\r\n", nIndex);
        APP_DEBUG("status:%u,data length:%u\r\n", (pPDUInfo->status), (pPDUInfo->length));
        APP_DEBUG("data = %s\r\n",(pPDUInfo->data));
    } while(0);
    
    Ql_MEM_Free(pPDUInfo);
}

void SMS_PDUMode_Send(void)
{
    s32 iResult;
    u32 nMsgRef;
    char pduStr[] = "1234923asdf";
    iResult = RIL_SMS_SendSMS_PDU(pduStr, sizeof(pduStr), &nMsgRef);
    if (RIL_AT_SUCCESS != iResult)
    {
        APP_DEBUG("< Fail to send PDU SMS, cause:%d >\r\n", iResult);
        return;
    }
    APP_DEBUG("< Send PDU SMS successfully, MsgRef:%u >\r\n", nMsgRef);

}

/*****************************************************************************
 * FUNCTION
 *  Hdlr_RecvNewSMS
 *
 * DESCRIPTION
 *  The handler function of new received SMS.
 *  
 * PARAMETERS
 *  <nIndex>     The SMS index in storage,it starts from 1
 *  <bAutoReply> TRUE: The module should reply a SMS to the sender; 
 *               FALSE: The module only read this SMS.
 *
 * RETURNS
 *  VOID
 *
 * NOTE
 *  1. This is an internal function
 *****************************************************************************/




static void Hdlr_RecvNewSMS(u32 nIndex, bool bAutoReply)
{
    s32 iResult = 0;
    u32 uMsgRef = 0;
    ST_RIL_SMS_TextInfo *pTextInfo = NULL;
    ST_RIL_SMS_DeliverParam *pDeliverTextInfo = NULL;
    char aPhNum[RIL_SMS_PHONE_NUMBER_MAX_LEN] = {0,};
    char myTextMessage[100];
    char aReplyCon[] = {"Module has received SMS dddd."};
    bool bResult = FALSE;
    
    pTextInfo = Ql_MEM_Alloc(sizeof(ST_RIL_SMS_TextInfo));
    if (NULL == pTextInfo)
    {
        APP_DEBUG("%s/%d:Ql_MEM_Alloc FAIL! size:%u\r\n", sizeof(ST_RIL_SMS_TextInfo), __func__, __LINE__);
        return;
    }
    Ql_memset(pTextInfo, 0x00, sizeof(ST_RIL_SMS_TextInfo));
    iResult = RIL_SMS_ReadSMS_Text(nIndex, LIB_SMS_CHARSET_GSM, pTextInfo);
    if (iResult != RIL_AT_SUCCESS)
    {
        Ql_MEM_Free(pTextInfo);
        APP_DEBUG("Fail to read text SMS[%d], cause:%d\r\n", nIndex, iResult);
        return;
    }        
    
    if ((LIB_SMS_PDU_TYPE_DELIVER != (pTextInfo->type)) || (RIL_SMS_STATUS_TYPE_INVALID == (pTextInfo->status)))
    {
        Ql_MEM_Free(pTextInfo);
        APP_DEBUG("WARNING: NOT a new received SMS.\r\n");    
        return;
    }
    
    pDeliverTextInfo = &((pTextInfo->param).deliverParam);    

    if(TRUE == pDeliverTextInfo->conPres)  //Receive CON-SMS segment
    {
        s8 iBufIdx = 0;
        u8 uSeg = 0;
        u16 uConLen = 0;

        iBufIdx = ConSMSBuf_GetIndex(g_asConSMSBuf,CON_SMS_BUF_MAX_CNT,&(pDeliverTextInfo->con));
        if(-1 == iBufIdx)
        {
            APP_DEBUG("Enter Hdlr_RecvNewSMS,WARNING! ConSMSBuf_GetIndex FAIL! Show this CON-SMS-SEG directly!\r\n");

            APP_DEBUG(
                "status:%u,type:%u,alpha:%u,sca:%s,oa:%s,scts:%s,data length:%u,cp:1,cy:%d,cr:%d,ct:%d,cs:%d\r\n",
                    (pTextInfo->status),
                    (pTextInfo->type),
                    (pDeliverTextInfo->alpha),
                    (pTextInfo->sca),
                    (pDeliverTextInfo->oa),
                    (pDeliverTextInfo->scts),
                    (pDeliverTextInfo->length),
                    pDeliverTextInfo->con.msgType,
                    pDeliverTextInfo->con.msgRef,
                    pDeliverTextInfo->con.msgTot,
                    pDeliverTextInfo->con.msgSeg
            );
            APP_DEBUG("data = %s\r\n",(pDeliverTextInfo->data));

            Ql_MEM_Free(pTextInfo);
        
            return;
        }

        bResult = ConSMSBuf_AddSeg(
                    g_asConSMSBuf,
                    CON_SMS_BUF_MAX_CNT,
                    iBufIdx,
                    &(pDeliverTextInfo->con),
                    (pDeliverTextInfo->data),
                    (pDeliverTextInfo->length)
        );
        if(FALSE == bResult)
        {
            APP_DEBUG("Enter Hdlr_RecvNewSMS,WARNING! ConSMSBuf_AddSeg FAIL! Show this CON-SMS-SEG directly!\r\n");

            APP_DEBUG(
                "status:%u,type:%u,alpha:%u,sca:%s,oa:%s,scts:%s,data length:%u,cp:1,cy:%d,cr:%d,ct:%d,cs:%d\r\n",
                (pTextInfo->status),
                (pTextInfo->type),
                (pDeliverTextInfo->alpha),
                (pTextInfo->sca),
                (pDeliverTextInfo->oa),
                (pDeliverTextInfo->scts),
                (pDeliverTextInfo->length),
                pDeliverTextInfo->con.msgType,
                pDeliverTextInfo->con.msgRef,
                pDeliverTextInfo->con.msgTot,
                pDeliverTextInfo->con.msgSeg
            );
            APP_DEBUG("data = %s\r\n",(pDeliverTextInfo->data));

            Ql_MEM_Free(pTextInfo);
        
            return;
        }

        bResult = ConSMSBuf_IsIntact(
                    g_asConSMSBuf,
                    CON_SMS_BUF_MAX_CNT,
                    iBufIdx,
                    &(pDeliverTextInfo->con)
        );
        if(FALSE == bResult)
        {
            APP_DEBUG(
                "Enter Hdlr_RecvNewSMS,WARNING! ConSMSBuf_IsIntact FAIL! Waiting. cp:1,cy:%d,cr:%d,ct:%d,cs:%d\r\n",
                pDeliverTextInfo->con.msgType,
                pDeliverTextInfo->con.msgRef,
                pDeliverTextInfo->con.msgTot,
                pDeliverTextInfo->con.msgSeg
            );

            Ql_MEM_Free(pTextInfo);

            return;
        }

        //Show the CON-SMS
        APP_DEBUG(
            "status:%u,type:%u,alpha:%u,sca:%s,oa:%s,scts:%s",
            (pTextInfo->status),
            (pTextInfo->type),
            (pDeliverTextInfo->alpha),
            (pTextInfo->sca),
            (pDeliverTextInfo->oa),
            (pDeliverTextInfo->scts)
        );
        
        uConLen = 0;
        for(uSeg = 1; uSeg <= pDeliverTextInfo->con.msgTot; uSeg++)
        {
            uConLen += g_asConSMSBuf[iBufIdx].asSeg[uSeg-1].uLen;
        }

        APP_DEBUG(",data length:%u",uConLen);
        APP_DEBUG("\r\n"); //Print CR LF

        for(uSeg = 1; uSeg <= pDeliverTextInfo->con.msgTot; uSeg++)
        {
            APP_DEBUG("data = %s ,len = %d",
                g_asConSMSBuf[iBufIdx].asSeg[uSeg-1].aData,
                g_asConSMSBuf[iBufIdx].asSeg[uSeg-1].uLen
            );
        }

        APP_DEBUG("\r\n"); //Print CR LF

        //Reset CON-SMS context
        bResult = ConSMSBuf_ResetCtx(g_asConSMSBuf,CON_SMS_BUF_MAX_CNT,iBufIdx);
        if(FALSE == bResult)
        {
            APP_DEBUG("Enter Hdlr_RecvNewSMS,WARNING! ConSMSBuf_ResetCtx FAIL! iBufIdx:%d\r\n",iBufIdx);
        }

        Ql_MEM_Free(pTextInfo);
        
        return;
    }
    
   // APP_DEBUG("data = %s\r\n",(pDeliverTextInfo->data));
    
    Ql_memset(myTextMessage,'0',sizeof(myTextMessage));
    Ql_strcpy(aPhNum, pDeliverTextInfo->oa);
    Ql_strcpy(myTextMessage,pDeliverTextInfo->data);


    APP_DEBUG("new text message :%s -->\r\n",myTextMessage);
    APP_DEBUG("new phone number :%s -->\r\n",aPhNum);

    sms_pump(aPhNum,myTextMessage,&devConfig); 
    Ql_MEM_Free(pTextInfo);

    u32 count = 0;
    return;
}




// load the saved device configurations
void loadConfig(){  
    u32 numberOfBytesRead = 0;
    char serverHost[MAX_HOST_LEN] = {0};
    numberOfBytesRead = readAlphabetFromFlash("serverHost.txt",serverHost,MAX_HOST_LEN);
    Ql_memset(devConfig.serverHost,"0",Ql_strlen(devConfig.serverHost));
    Ql_strcpy(devConfig.serverHost,serverHost);

    char magic[4] = {0}; 
    numberOfBytesRead = readFromFlash("magic.txt",&magic,5);
    devConfig.magic = (magic[0] << 24) | (magic[1] << 16) | (magic[2] << 8) | (magic[3] & 0xff);

    char ver[2] = {0};
    numberOfBytesRead = readFromFlash("ver.txt",ver,3);
    devConfig.ver = (ver[0] << 8) | (ver[1] & 0xff);

    char password[MAX_PASS_LEN] = {0};
    Ql_memset(devConfig.password,"0",Ql_strlen(devConfig.password));
    numberOfBytesRead = readAlphabetFromFlash("pwd.txt",password,MAX_PASS_LEN);
    Ql_strcpy(devConfig.password,password);


    char myImei[16] = {0};
    numberOfBytesRead = readFromFlashString("imei.txt",myImei,15);
    Ql_memset(devConfig.imei,"0",Ql_strlen(devConfig.imei));
    Ql_strncpy(devConfig.imei,myImei,15);
    devConfig.imei[15] = "\n";
    APP_DEBUG("device imei = %s\r\n",devConfig.imei);
    

    char allowPublic[1] = {0};
    numberOfBytesRead = readFromFlash("allowPublic.txt",allowPublic,1);
    devConfig.allowPublic = (u8)allowPublic[0];

    char unitName[MAX_NAME_LEN] = {0};
    numberOfBytesRead = readFromFlashString("name.txt",unitName,MAX_NAME_LEN);
    Ql_strcpy(devConfig.unitName,unitName);

    char apn[MAX_APN_LEN] = {0};
    Ql_memset(devConfig.apn,"0",Ql_strlen(devConfig.apn));
    numberOfBytesRead = readAlphabetFromFlash("apn.txt",apn,MAX_APN_LEN);
    Ql_strcpy(devConfig.apn,apn);

    char apnUser[32] = {0};
    Ql_memset(devConfig.apnUser,"0",Ql_strlen(devConfig.apnUser));
    numberOfBytesRead = readAlphabetFromFlash("apnUser.txt",apnUser,32);
    Ql_strcpy(devConfig.apnUser,apnUser);

    char apnPass[32] = {0};
    Ql_memset(devConfig.apnPass,"0",Ql_strlen(devConfig.apnPass));
    numberOfBytesRead = readAlphabetFromFlash("apnPass.txt",apnPass,32);
    Ql_strcpy(devConfig.apnPass,apnPass);

    

    char serverPort[2] = {0};
    s32 fileHandle = Ql_FS_Open("serverPort.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    s32 ret = Ql_FS_Read(fileHandle,serverPort,3, &numberOfBytesRead);
    devConfig.serverPort = (serverPort[0] << 8) | (serverPort[1] & 0xff);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("server poet = %i total bytes read = %i file handle = %i\r\n",devConfig.serverPort,numberOfBytesRead,fileHandle);
    

    char rptSec[2] = {0}; // report interval
    fileHandle = Ql_FS_Open("rptSec.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,rptSec,3, &numberOfBytesRead);
    devConfig.rptSec = (rptSec[0] << 8) | (rptSec[1] & 0xff);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("server poet = %i total bytes read = %i file handle = %i\r\n",devConfig.rptSec,numberOfBytesRead,fileHandle);
    

    char slpSec[2] = {0}; // sleep interval
    fileHandle = Ql_FS_Open("slpSec.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,slpSec,3, &numberOfBytesRead);
    devConfig.slpSec = (slpSec[0] << 8) | (slpSec[1] & 0xff);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("server poet = %i total bytes read = %i file handle = %i\r\n",devConfig.slpSec,numberOfBytesRead,fileHandle);


    char runMode[1] = {0}; // sleep interval
    fileHandle = Ql_FS_Open("runMode.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,runMode,2, &numberOfBytesRead);
    APP_DEBUG("allow public = %i total bytes read = %i file handle = %i\r\n",runMode[0],numberOfBytesRead,fileHandle);
    devConfig.runMode = (u8)runMode[0];
    Ql_FS_Close(fileHandle);


    char sleepMode[1] = {0}; // sleep mode
    fileHandle = Ql_FS_Open("sleepMode.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,sleepMode,2, &numberOfBytesRead);
    APP_DEBUG("allow public = %i total bytes read = %i file handle = %i\r\n",sleepMode[0],numberOfBytesRead,fileHandle);
    devConfig.sleepMode = sleepMode[0];
    Ql_FS_Close(fileHandle);



    char periodicFindMode[1] = {0};
    fileHandle = Ql_FS_Open("periodicFindMode.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,periodicFindMode,2, &numberOfBytesRead);
    APP_DEBUG("allow public = %i total bytes read = %i file handle = %i\r\n",periodicFindMode[0],numberOfBytesRead,fileHandle);
    devConfig.periodicFindMode = periodicFindMode[0];
    Ql_FS_Close(fileHandle);


    char findIntervalMin[2] = {0};
    fileHandle = Ql_FS_Open("findIntervalMin.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,findIntervalMin,3, &numberOfBytesRead);
    devConfig.findIntervalMin = (findIntervalMin[0] << 8) | (findIntervalMin[1] & 0xff);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("find interval min = %i total bytes read = %i file handle = %i\r\n",devConfig.findIntervalMin,numberOfBytesRead,fileHandle);
    
    char tripReporting[1] = {0};
    fileHandle = Ql_FS_Open("tripReporting.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,tripReporting,2, &numberOfBytesRead);
    APP_DEBUG("trip reporting = %i total bytes read = %i file handle = %i\r\n",tripReporting[0],numberOfBytesRead,fileHandle);
    devConfig.tripReporting = tripReporting[0];
    Ql_FS_Close(fileHandle);


    char alarmReporting[1] = {0};
    fileHandle = Ql_FS_Open("alarmReporting.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,alarmReporting,2, &numberOfBytesRead);
    APP_DEBUG("alarm reporting = %i total bytes read = %i file handle = %i\r\n",alarmReporting[0],numberOfBytesRead,fileHandle);
    devConfig.alarmReporting = alarmReporting[0];
    Ql_FS_Close(fileHandle);


    char tripData[1] = {0};
    fileHandle = Ql_FS_Open("trip.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,tripData,2, &numberOfBytesRead);
    APP_DEBUG("trip = %i \r\n",tripData[0]);
    devConfig.trip = tripData[0];
    Ql_FS_Close(fileHandle);



    char speedReporting[1] = {0};
    fileHandle = Ql_FS_Open("speedReporting.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,speedReporting,2, &numberOfBytesRead);
    APP_DEBUG("alarm reporting = %i total bytes read = %i file handle = %i\r\n",speedReporting[0],numberOfBytesRead,fileHandle);
    devConfig.speedReporting = speedReporting[0];
    Ql_FS_Close(fileHandle);



    char speedLimitKph[2] = {0};
    fileHandle = Ql_FS_Open("speedLimitKph.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,speedLimitKph,3, &numberOfBytesRead);
    devConfig.speedLimitKph = (speedLimitKph[0] << 8) | (speedLimitKph[1] & 0xff);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("find interval min = %i total bytes read = %i file handle = %i\r\n",devConfig.speedLimitKph,numberOfBytesRead,fileHandle);


    char mainPowerAlarm[1] = {0};
    fileHandle = Ql_FS_Open("mainPowerAlarm.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,mainPowerAlarm,2, &numberOfBytesRead);
    APP_DEBUG("alarm reporting = %i total bytes read = %i file handle = %i\r\n",mainPowerAlarm[0],numberOfBytesRead,fileHandle);
    devConfig.mainPowerAlarm = mainPowerAlarm[0];
    Ql_FS_Close(fileHandle);



    char vehicleDisabled[1] = {0};
    fileHandle = Ql_FS_Open("vehicleDisabled.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,vehicleDisabled,2, &numberOfBytesRead);
    APP_DEBUG("alarm reporting = %i total bytes read = %i file handle = %i\r\n",vehicleDisabled[0],numberOfBytesRead,fileHandle);
    devConfig.vehicleDisabled = vehicleDisabled[0];
    Ql_FS_Close(fileHandle);



    char ignitionReporting[1] = {0};
    fileHandle = Ql_FS_Open("ignitionReporting.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,ignitionReporting,2, &numberOfBytesRead);
    APP_DEBUG("alarm reporting = %i total bytes read = %i file handle = %i\r\n",ignitionReporting[0],numberOfBytesRead,fileHandle);
    devConfig.ignitionReporting = ignitionReporting[0];
    Ql_FS_Close(fileHandle);



    char odometerKm[4] = {0}; 
    fileHandle = Ql_FS_Open("odometerKm.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,odometerKm,3, &numberOfBytesRead);
    devConfig.odometerKm = (odometerKm[0] << 24) | (odometerKm[1] << 16) | (odometerKm[2] << 8) | (odometerKm[3] & 0xff);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("find interval min = %i total bytes read = %i file handle = %i\r\n",devConfig.odometerKm,numberOfBytesRead,fileHandle);



    char shockSensitivity[1] = {0};
    fileHandle = Ql_FS_Open("shockSensitivity.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,shockSensitivity,2, &numberOfBytesRead);
    APP_DEBUG("alarm reporting = %i total bytes read = %i file handle = %i\r\n",shockSensitivity[0],numberOfBytesRead,fileHandle);
    devConfig.shockSensitivity = shockSensitivity[0];
    Ql_FS_Close(fileHandle);



    char heartbeatMin[2] = {0};
    fileHandle = Ql_FS_Open("heartbeatMin.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,heartbeatMin,3, &numberOfBytesRead);
    devConfig.heartbeatMin = (heartbeatMin[0] << 8) | (heartbeatMin[1] & 0xff);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("heart beat min = %i total bytes read = %i file handle = %i\r\n",devConfig.heartbeatMin,numberOfBytesRead,fileHandle);



    char userCount[1] = {0};
    fileHandle = Ql_FS_Open("userCount.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,userCount,2, &numberOfBytesRead);
    APP_DEBUG("user count = %i total bytes read = %i file handle = %i\r\n",userCount[0],numberOfBytesRead,fileHandle);
    devConfig.userCount = userCount[0];
    Ql_FS_Close(fileHandle);



    char timezoneOffset[2] = {0};
    fileHandle = Ql_FS_Open("timezoneOffset.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,timezoneOffset,3, &numberOfBytesRead);
    devConfig.timezoneOffset = (timezoneOffset[0] << 8) | (timezoneOffset[1] & 0xff);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("heart beat min = %i total bytes read = %i file handle = %i\r\n",devConfig.timezoneOffset,numberOfBytesRead,fileHandle);


    char users[MAX_USERS * 25] = {0};
    numberOfBytesRead = readFromFlashString("user.txt",users,(MAX_USERS * 15));
    Ql_strcpy(devConfig.users,users);



    char mqttUser[MAX_MQTT_USER] = {0};
    numberOfBytesRead = readFromFlashString("mqttUser.txt",mqttUser,MAX_MQTT_USER);
    Ql_strcpy(devConfig.mqttUser,mqttUser);


    char mqttPass[MAX_MQTT_PASS] = {0};
    numberOfBytesRead = readFromFlashString("mqttPass.txt",mqttPass,MAX_MQTT_PASS);
    Ql_strcpy(devConfig.mqttPass,mqttPass);



    char myReportAlarm[1] = {0};
    fileHandle = Ql_FS_Open("reportAlarm.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,myReportAlarm,2, &numberOfBytesRead);
    APP_DEBUG("my report alarm = %i total bytes read = %i file handle = %i\r\n",myReportAlarm[0],numberOfBytesRead,fileHandle);
    devConfig.reportAlarm = myReportAlarm[0];
    Ql_FS_Close(fileHandle);



    char mReportTrip[1] = {0};
    fileHandle = Ql_FS_Open("reportTrip.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,mReportTrip,2, &numberOfBytesRead);
    APP_DEBUG("my report trip = %i total bytes read = %i file handle = %i\r\n",mReportTrip[0],numberOfBytesRead,fileHandle);
    devConfig.reportTrip = mReportTrip[0];
    Ql_FS_Close(fileHandle);


    char myReportSpeed[1] = {0};
    fileHandle = Ql_FS_Open("reportSpeed.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,myReportSpeed,2, &numberOfBytesRead);
    APP_DEBUG("my report speed = %i total bytes read = %i file handle = %i\r\n",myReportSpeed[0],numberOfBytesRead,fileHandle);
    devConfig.reportSpeed = myReportSpeed[0];
    Ql_FS_Close(fileHandle);


    char myReportPower[1] = {0};
    fileHandle = Ql_FS_Open("reportPower.txt", QL_FS_READ_ONLY);
    numberOfBytesRead = 0;
    ret = Ql_FS_Read(fileHandle,myReportPower,2, &numberOfBytesRead);
    APP_DEBUG("my report power = %i total bytes read = %i file handle = %i\r\n",myReportPower[0],numberOfBytesRead,fileHandle);
    devConfig.reportPower = myReportPower[0];
    Ql_FS_Close(fileHandle);
    



    Ql_strcpy(devConfig.imei,devConfig.imei);
    APP_DEBUG("this is the imei %s\r\n",devConfig.imei);
    
    Ql_memset(devConfig.clientId,0,Ql_strlen(devConfig.clientId));
    Ql_strcpy(devConfig.clientId,"device_/");
    Ql_strcat(devConfig.clientId,devConfig.imei);
    APP_DEBUG("Client id %s\r\n",devConfig.clientId);

    
    
    Ql_strcpy(devConfig.HW_VERSION,"1.0.0");
    Ql_strcpy(devConfig.FW_VERSION,"1.0.0");

    devConfig.isSimCardReaddy = 0;
    devConfig.isGsmNetworkGotten = 0;
    devConfig.isGpRsNetworkGotten = 0;
    devConfig.isApnSet = 0;
    devConfig.isMqttOpen = 0;
    devConfig.isMqttLoggedIn = 0;
    devConfig.isSubscribedToTopic = 0;

    devConfig.isGnssPowered = 0;
    devConfig.isGnssConfigured = 0;
    devConfig.isGnssSetupComplete = 0;
    devConfig.isGnssDowloaded = 0;
    devConfig.isGnssDataInjected = 0;
    devConfig.isFixedGnssPosition = 0;
    devConfig.isGnssRead = 0;
    
    Ql_strncat(devConfig.deviceTopic,"device/",Ql_strlen("device/"));
    Ql_strncat(devConfig.deviceTopic,devConfig.imei,Ql_strlen(devConfig.imei));
    u32 lastIndex =  Ql_strlen("device/") + Ql_strlen(devConfig.imei);
    devConfig.deviceTopic[lastIndex] = '\0';
    
}





/* take setting actions*/
    


/* end take setting actions*/


/* take alarm actions*/


/* end take alarm actions*/



/* custom general functions */



// Convert hex string to unsigned integer
u32 HexStringToInt(const char* hexStr)
{
    u32 result = 0;
    char c;
    while ((c = *hexStr++) != '\0')
    {
        result <<= 4; // shift left 4 bits for next hex digit
        if (c >= '0' && c <= '9')
            result += c - '0';
        else if (c >= 'a' && c <= 'f')
            result += c - 'a' + 10;
        else if (c >= 'A' && c <= 'F')
            result += c - 'A' + 10;
        else
            break; // stop on invalid char
    }
    return result;
}



double convertNmeaToDecimal(double nmea, char direction) {
    int degrees = (int)(nmea / 100);              // take first part as degrees
    double minutes = nmea - (degrees * 100);      // remaining is minutes
    double decimal = degrees + (minutes / 60.0);  // convert to decimal degrees

    // Apply hemisphere
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    return decimal;
}


static void decodeCellInfo2(char* cellRawData, ST_CellInfo* mainCellInfo){
        char myDataBuffer[1024];
        Ql_strcpy(myDataBuffer,cellRawData);
        char* myRawData = my_strtok(myDataBuffer,",");
        if(myRawData != NULL){
            mainCellInfo->mcc = Ql_atoi(myRawData);
        }

        myRawData = myRawData + Ql_strlen(myRawData) + 1;
        char* rawMnc =  my_strtok(myRawData,",");
        if(rawMnc != NULL){
            mainCellInfo->mnc = Ql_atoi(rawMnc);
        }
        

        rawMnc = rawMnc + Ql_strlen(rawMnc) + 1;
        char* rawLac =  my_strtok(rawMnc,",");
        if(rawLac != NULL){
            mainCellInfo->lac = HexStringToInt(rawLac);
        }

        rawLac = rawLac + Ql_strlen(rawLac) + 1;
        char* rawCellId =  my_strtok(rawLac,",");
        if(rawCellId != NULL){
            mainCellInfo->cellId = HexStringToInt(rawCellId);
        }
        

        rawCellId = rawCellId + Ql_strlen(rawCellId) + 1;
        char* rawBsic =  my_strtok(rawCellId,",");

        rawBsic = rawBsic + Ql_strlen(rawBsic) + 1;
        char* rawArfcn =  my_strtok(rawBsic,",");


        rawArfcn = rawArfcn + Ql_strlen(rawArfcn) + 1;
        char* rawRssi =  my_strtok(rawArfcn,",");
        if(rawRssi != NULL){
            mainCellInfo->rssi = Ql_atoi(rawRssi);
        }

        rawRssi = rawRssi + Ql_strlen(rawRssi) + 1;
        char* rawTimeAdd =  my_strtok(rawRssi,",");
        if(rawTimeAdd != NULL){
            mainCellInfo->timeAd = Ql_atoi(rawTimeAdd);
        }

}


static void decodeCellInfo(char* cellRawData, ST_CellInfo* mainCellInfo){

        char* myRawData = my_strtok(cellRawData,",");
        //0,621,30,3a4f,a56b,581,19,-6,4,4,0,8,x,x,x,x,x,x,x
        myRawData = myRawData + Ql_strlen(myRawData) + 1;
        char* rawMcc = my_strtok(myRawData,",");
        mainCellInfo->mcc = Ql_atoi(rawMcc);

        rawMcc = rawMcc + Ql_strlen(rawMcc) + 1;
        char* rawMnc =  my_strtok(rawMcc,",");
        mainCellInfo->mnc = Ql_atoi(rawMnc);

        rawMnc = rawMnc + Ql_strlen(rawMnc) + 1;
        char* rawLac =  my_strtok(rawMnc,",");
        mainCellInfo->lac = HexStringToInt(rawLac);

        rawLac = rawLac + Ql_strlen(rawLac) + 1;
        char* rawCellId =  my_strtok(rawLac,",");
        mainCellInfo->cellId = HexStringToInt(rawCellId);

        rawCellId = rawCellId + Ql_strlen(rawCellId) + 1;
        char* rawBsic =  my_strtok(rawCellId,",");

        rawBsic = rawBsic + Ql_strlen(rawBsic) + 1;
        char* rawArfcn =  my_strtok(rawBsic,",");


        rawArfcn = rawArfcn + Ql_strlen(rawArfcn) + 1;
        char* rawRssi =  my_strtok(rawArfcn,",");
        mainCellInfo->rssi = Ql_atoi(rawRssi);

        rawRssi = rawRssi + Ql_strlen(rawRssi) + 1;
        char* rawTimeAdd =  my_strtok(rawRssi,",");
        mainCellInfo->timeAd = Ql_atoi(rawTimeAdd);

}


static void decode_gps_data(char* gpsData,GPS_LOCATION_T* decodedGps){
    //$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

    //$GNRMC,131716.00,A,0627.45079,N,00319.01189,E,0.081,,110925,,,A,V*16


    char* myRawData =  my_strtok(gpsData,",");
    myRawData = myRawData + Ql_strlen(myRawData) + 1; // skip $GPRMC

    char* rawUtc = my_strtok(myRawData,",");
    if(rawUtc != NULL){
        decodedGps->utcTime = Ql_atof(rawUtc);
    }
    rawUtc = rawUtc + Ql_strlen(rawUtc) + 1;


    char* rawStatus = my_strtok(rawUtc,",");
    if(rawStatus != NULL){
        decodedGps->status = rawStatus[0];
    }
    rawStatus = rawStatus + Ql_strlen(rawStatus) + 1;


    char* rawLatitude = my_strtok(rawStatus,",");
    if(rawLatitude != NULL){
        decodedGps->latitude = Ql_atof(rawLatitude);
    }
    rawLatitude = rawLatitude + Ql_strlen(rawLatitude) + 1;


    char* rawLatDirection = my_strtok(rawLatitude,",");
    if(rawLatDirection != NULL){
        decodedGps->latDirection = rawLatDirection[0];
    }
    rawLatDirection = rawLatDirection + Ql_strlen(rawLatDirection) + 1;


    char* rawLongitude = my_strtok(rawLatDirection,",");
    if(rawLongitude != NULL){
        decodedGps->longitude = Ql_atof(rawLongitude);
    }
    rawLongitude = rawLongitude + Ql_strlen(rawLongitude) + 1;


    char* rawLonDirection = my_strtok(rawLongitude,",");
    if(rawLonDirection != NULL){
        decodedGps->longDirection = rawLonDirection[0];
    }
    rawLonDirection = rawLonDirection + Ql_strlen(rawLonDirection) + 1;


    char* rawSpeed = my_strtok(rawLonDirection,",");
    if(rawSpeed != NULL){
        decodedGps->speed = Ql_atof(rawSpeed);
    }
    rawSpeed = rawSpeed + Ql_strlen(rawSpeed) + 1;


    char* rawCourseOverGround = my_strtok(rawSpeed,",");
    if(rawCourseOverGround != NULL){
        decodedGps->courseOverGround = Ql_atof(rawCourseOverGround);
    }
    rawCourseOverGround = rawCourseOverGround + Ql_strlen(rawCourseOverGround) + 1;


    char* rawDeviceDate = my_strtok(rawCourseOverGround,",");
    if(rawDeviceDate != NULL){
        decodedGps->deviceDate = Ql_atoi(rawDeviceDate);
    }
    
    rawDeviceDate = rawDeviceDate + Ql_strlen(rawDeviceDate) + 1;


    char* rawMagneticVariation = my_strtok(rawDeviceDate,",");
    if(rawMagneticVariation != NULL){
        decodedGps->magneticVariation = Ql_atof(rawMagneticVariation);
        rawMagneticVariation = rawMagneticVariation + Ql_strlen(rawMagneticVariation) + 1;
    }
    
    if(decodedGps->latDirection)
        decodedGps->latitude =  convertNmeaToDecimal(decodedGps->latitude,decodedGps->latDirection);
    
    if(decodedGps->longDirection)
         decodedGps->longitude =  convertNmeaToDecimal(decodedGps->longitude,decodedGps->longDirection);
}