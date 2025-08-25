
#include "custom_feature_def.h"
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
#include "ril_flash.h"
#include "sms_command.h"
#include "data_types.h"
#include "gnss_parser.h"


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



GNSS_DATA_STATUS gnss_data_status; // checks if gnss data is gotten fully


DeviceConfig devConfig; // holds all the device configuration
static s32 isPdpContextGotten = 0;



/****************************************************************************
* Definition for APN
****************************************************************************/
#define APN      "web.gprs.mtnnigeria.net\0"
#define USERID   ""
#define PASSWD   ""

/*****************************************************************
* MQTT  timer param
******************************************************************/
#define MQTT_TIMER_ID         0x200
#define MQTT_TIMER_PERIOD     500




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
void loopThroughSms(ST_MSG taskMsg);
void loadConfig();


/***********************************************************************
 * GLOBAL DATA DEFINITIONS
************************************************************************/
ConSMSStruct g_asConSMSBuf[CON_SMS_BUF_MAX_CNT];




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

u8 product_key[] =   "your-productkey\0";   //<ali cloud needs it.
u8 device_name[]=    "your-devicename\0";   //<ali cloud needs it.
u8 device_secret[] = "your-devicesecret\0"; //<ali cloud needs it.


u8 clientID[50];
u8 username[50];
u8 passwd[50];
u8 domainName[100];

static u8 test_data[128] =  "hello cloud,this is quectel test code!!!\0"; //<first packet data
static u8 test_topic[128] = "device/78489383830945\0"; //<topic

MQTTP_AUTH_STATE mqtt_auth = MQTT_LOGGED_OUT;

u32 lastLocationPush;



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
void checkGnssConnection(ST_MSG msg);

/*****************************************************************
* Timer callback function
******************************************************************/
static void MqttCallback_Timer(u32 timerId, void* param);
static void GnssCallback_Timer(u32 timerId, void* param);
void publishMessage(u32 message_id, u8* topic,u8* out_data);

/*****************************************************************
* MQTT recv callback function
******************************************************************/
static void mqtt_recv(u8* buffer,u32 length);

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


void proc_main_task(s32 taskId)
{

    ST_MSG msg;
    s32 ret;
        

    lastLocationPush = Ql_GetMsSincePwrOn();

    //<Register & open UART port
    Ql_UART_Register(m_myUartPort, CallBack_UART_Hdlr, NULL);
    Ql_UART_Open(m_myUartPort, 115200, FC_NONE);



    APP_DEBUG("//<------------OpenCPU: MQTT Client.------------\r\n");

    loadConfig();

    //<register state timer 
    Ql_Timer_Register(MQTT_TIMER_ID, MqttCallback_Timer, NULL);

    Ql_Timer_Register(GNSS_TIMER_ID, GnssCallback_Timer, NULL);
	Ql_Timer_Start(GNSS_TIMER_ID, GNSS_TIMER_PERIOD, TRUE);

	//register MQTT recv callback
    ret = Ql_Mqtt_Recv_Register(mqtt_recv);
	APP_DEBUG("//<register recv callback,ret = %d\r\n",ret);

    while(TRUE)
    {
         Ql_memset(&msg, 0x0, sizeof(ST_MSG));
         Ql_OS_GetMessage(&msg);
         loopThroughSms(msg);

        if(devConfig.gnss_state == STATE_GNSS_READ_ALL_NMEA ){

        }
        else{
            checkGnssConnection(msg);          
        }

        if(devConfig.mqtt_state == STATE_MQTT_CONN){
            if(mqtt_auth == MQTT_LOGGED_IN){
                char* deviceData[200];
                Ql_snprintf(deviceData, sizeof(deviceData),"{'imei':'4784748474848','gpsData':%s}", pre_NMEA_buf);
                publishMessage(1,"device/4784748474848",deviceData);
            }
            else{
                
            }
        }
        else{
            checkConnectionStatus(msg);
        }
        
    }
}


void checkGnssConnection(ST_MSG msg){
    switch(msg.message)
    {
        case MSG_ID_RIL_READY:
            Ql_RIL_Initialize();
            APP_DEBUG("<-- RIL is ready -->\r\n");
        break;

        case MSG_ID_USER_START:
            break;

        default:
            break;
    }
}


void loopThroughSms(ST_MSG taskMsg){
    s32 iResult = 0;
    s32 i = 0;

    switch (taskMsg.message)
    {
    case MSG_ID_RIL_READY:
        {
            APP_DEBUG("<-- RIL is ready -->\r\n");
            Ql_RIL_Initialize(); // MUST call this function

            for(i = 0; i < CON_SMS_BUF_MAX_CNT; i++)
            {
                ConSMSBuf_ResetCtx(g_asConSMSBuf,CON_SMS_BUF_MAX_CNT,i);
            }
            
            break;
        }
    case MSG_ID_URC_INDICATION:
        switch (taskMsg.param1)
        {
        case URC_SYS_INIT_STATE_IND:
            {
                APP_DEBUG("<-- Sys Init Status %d -->\r\n", taskMsg.param2);
                if (SYS_STATE_SMSOK == taskMsg.param2)
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
        case URC_SIM_CARD_STATE_IND:
            {
                APP_DEBUG("\r\n<-- SIM Card Status:%d -->\r\n", taskMsg.param2);
            }
            break;

        case URC_GSM_NW_STATE_IND:
            {
                APP_DEBUG("\r\n<-- GSM Network Status:%d -->\r\n", taskMsg.param2);
                break;
            }

        case URC_GPRS_NW_STATE_IND:
            {
                APP_DEBUG("\r\n<-- GPRS Network Status:%d -->\r\n", taskMsg.param2);
                break;
            }

        case URC_CFUN_STATE_IND:
            {
                APP_DEBUG("\r\n<-- CFUN Status:%d -->\r\n", taskMsg.param2);
                break;
            }

        case URC_COMING_CALL_IND:
            {
                ST_ComingCall* pComingCall = (ST_ComingCall*)(taskMsg.param2);
                APP_DEBUG("\r\n<-- Coming call, number:%s, type:%d -->\r\n", pComingCall->phoneNumber, pComingCall->type);
                break;
            }

        case URC_NEW_SMS_IND:
            {
                APP_DEBUG("\r\n<-- New SMS Arrives: index=%d\r\n", taskMsg.param2);
                Hdlr_RecvNewSMS((taskMsg.param2), FALSE);
                break;
            }

        case URC_MODULE_VOLTAGE_IND:
            {
                APP_DEBUG("\r\n<-- VBatt Voltage Ind: type=%d\r\n", taskMsg.param2);
                break;
            }

        default:
            break;
        }
        break;

    default:
        break;
    }
}


void checkConnectionStatus(ST_MSG msg){
        s32 ret;
        switch(msg.message)
        {
            case MSG_ID_RIL_READY:
                APP_DEBUG("//<RIL is ready\r\n");
                Ql_RIL_Initialize();
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
                                   Ql_Timer_Start(MQTT_TIMER_ID, MQTT_TIMER_PERIOD, TRUE);
                				   APP_DEBUG("//<state timer start,ret = %d\r\n",ret);
                				}
                		    }
                            break;
        				case URC_MQTT_OPEN:
            				{
            					mqtt_urc_param_ptr = msg.param2;
            					if(0 == mqtt_urc_param_ptr->result)
            					{
                 					APP_DEBUG("//<Open a MQTT client successfully\r\n");
                                    devConfig.mqtt_state = STATE_MQTT_CONN;
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
                    		        APP_DEBUG("//<Connect to MQTT server successfully\r\n");
            						devConfig.mqtt_state = STATE_MQTT_SUB;
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
            						devConfig.mqtt_state = STATE_MQTT_PUB;
            					}
            					else
            					{
            						APP_DEBUG("//<Subscribe topics failure,error = %d\r\n",mqtt_urc_param_ptr->result);
            					}
                		    }
            			    break;
        				case URC_MQTT_PUB:
            				{
            					mqtt_urc_param_ptr = msg.param2;
            					if(0 == mqtt_urc_param_ptr->result)
            					{
                    		        APP_DEBUG("//<Publish messages to MQTT server successfully\r\n");
            						devConfig.mqtt_state = STATE_MQTT_TOTAL_NUM;
            					}
            					else
            					{
            						APP_DEBUG("//<Publish messages to MQTT server failure,error = %d\r\n",mqtt_urc_param_ptr->result);
            					}
                		    }
            			    break;
        			    case URC_MQTT_CLOSE:
            				{
            					mqtt_urc_param_ptr = msg.param2;
            					if(0 == mqtt_urc_param_ptr->result)
            					{
                    		        APP_DEBUG("//<Closed MQTT socket successfully\r\n");
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

                    if (totalBytes <= 0)
                    {
                        APP_DEBUG("//<No data in UART buffer!\r\n");
                        return;
                    }
                    p = Ql_strstr(g_RxBuf_Uart1,"DISC");
    				if(p)
    				{
    			        ret = RIL_MQTT_QMTDISC(connect_id);
                        if(RIL_AT_SUCCESS == ret)
                        {
                            APP_DEBUG("//<Start disconnect MQTT socket\r\n");
                            if(TRUE == DISC_flag)
                                DISC_flag  = FALSE;
                            devConfig.mqtt_state = STATE_MQTT_TOTAL_NUM;
                        }else
                        {
                            APP_DEBUG("//<Disconnect MQTT socket failure,ret = %d\r\n",ret); 
                            devConfig.mqtt_state = STATE_MQTT_TOTAL_NUM;
                        }
    					break;
    				}
                    p = Ql_strstr(g_RxBuf_Uart1,"CLOSE");
    				if(p)
    				{
    					ret = RIL_MQTT_QMTCLOSE(connect_id);
                        if (RIL_AT_SUCCESS == ret)
                        {
                            APP_DEBUG("//<Start closed MQTT socket\r\n");
                            if(TRUE == CLOSE_flag)
                                CLOSE_flag = FALSE;
                            devConfig.mqtt_state = STATE_MQTT_TOTAL_NUM;
                        }else
                        {
                            APP_DEBUG("//<Closed MQTT socket failure,ret = %d\r\n",ret);
                            devConfig.mqtt_state = STATE_MQTT_TOTAL_NUM;
        			    }
    					break;
    				}
                    p = Ql_strstr(g_RxBuf_Uart1,"RECONN");
    				if(p)
    				{
                        if((FALSE == DISC_flag)||(FALSE == CLOSE_flag))
                        {
                            devConfig.mqtt_state = STATE_MQTT_OPEN;
                            APP_DEBUG("\r\n");
                        }
    					break;
    				}
                    //<The rest of the UART data is published as data.
                    {
                        if((TRUE == DISC_flag)&&(TRUE == CLOSE_flag))
                        {
                            pub_message_id++;  // The range is 0-65535. It will be 0 only when<qos>=0.
            				ret = RIL_MQTT_QMTPUB(connect_id,pub_message_id,QOS1_AT_LEASET_ONCE,0,test_topic,totalBytes,g_RxBuf_Uart1);
                            if (RIL_AT_SUCCESS == ret)
                            {
                                APP_DEBUG("//<Start publish a message to server\r\n");
                            }else
                            {
                                APP_DEBUG("//<Publish a message to server failure,ret = %d\r\n",ret);
                            }
                        }
                        else
                        {
                            //<No connection to the cloud platform, just echo.
                            APP_DEBUG("\r\n//<No connection to the cloud platform, just echo.\r\n");
                            Ql_UART_Write(m_myUartPort, g_RxBuf_Uart1, totalBytes);
                        }
                    }
                }
            }
            break;
        case EVENT_UART_READY_TO_WRITE:        {
            if(m_myUartPort == port)
                {
                    if(remainLen > 0)
                    {
                        s32 retLen = Ql_UART_Write(m_myUartPort, m_TxBuf_Uart, remainLen);
                        if(retLen < remainLen)
                        {
                            remainLen -= ret;
                            Ql_memmove(m_TxBuf_Uart, m_TxBuf_Uart+retLen, remainLen);
                        }
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
	APP_DEBUG("//<data:%s,len:%d\r\n",buffer,length);
    char *topic = (char *)buffer;               // First part is topic
    char *payload = topic + strlen(topic) + 1;  // Skip topic + null terminator


    if(Ql_strcmp(deviceTopic,topic) == 0){  // check if the topic is correct

           /*---------- MQTT CODE FOR USER STARTS HERE ------------------*/
            char myAction[100];

            cJSON *paloadData; = cJSON_Parse(payload);


            if (paloadData == NULL) {
                return;
            }

            cJSON *action = cJSON_GetObjectItem(paloadData, "action");

            if (!cJSON_IsString(action) || !action->valuestring != NULL) {
                return;
            }

            
            Ql_strcpy(myAction,action->valuestring); 

            if(Ql_memcmp(myAction,trackOnDemand,sizeof(myAction)) == 0){
                  
            }
            else if(Ql_memcmp(myAction,setGprsInterval,sizeof(myAction)) == 0){
                 
            }
            else if(Ql_memcmp(myAction,setAuthPhone,sizeof(myAction)) == 0){
                 
            }
            else if(Ql_memcmp(myAction,setOverSpeedLimit,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,setMovementAlarmRadius,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,setGeofenceAlarm,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,initialize,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,setSleepMode,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,outputControl,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,armDisarm,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,setGprsIntervalOnStop,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,setInitOrdometer,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,rebootDevice,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,setHeartBeat,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,clearDataLogger,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,getFirmwareVersion,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,readGprsInterval,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,setAuthorizationTag,sizeof(myAction)) == 0){

            }
            else if(Ql_memcmp(myAction,readAuthorizationTag,sizeof(myAction)) == 0){

            }

            /*---------- MQTT CODE FOR USER ENDS HERE ------------------*/

    }
    

}




static void MqttCallback_Timer(u32 timerId, void* param)
{
    s32 ret;
    
    if(MQTT_TIMER_ID == timerId)
    {
        switch(devConfig.mqtt_state)
        {        
            case STATE_NW_QUERY_STATE:
            {

                s32 cgreg = 0;
                ret = RIL_NW_GetGPRSState(&cgreg);
                APP_DEBUG("//<Network State:cgreg = %d\r\n",cgreg);
                if((cgreg == NW_STAT_REGISTERED)||(cgreg == NW_STAT_REGISTERED_ROAMING))
                {
                    //<Set PDP context 0
                    RIL_NW_SetGPRSContext(0);
                    APP_DEBUG("//<Set PDP context 0 \r\n");
                	//<Set APN
                	ret = RIL_NW_SetAPN(1, devConfig.apn, devConfig.apnUser, devConfig.apnPass);
                	APP_DEBUG("//<Set APN \r\n");
                    //PDP activated 
                    if(isPdpContextGotten == 1){  // check if pdp has been activated and skip
                        devConfig.mqtt_state = STATE_MQTT_CFG;
                    }
                    else{
                        ret = RIL_NW_OpenPDPContext();
                        if(ret == RIL_AT_SUCCESS)
                        {
                            APP_DEBUG("//<Activate PDP context,ret = %d\r\n",ret);
                            devConfig.mqtt_state = STATE_MQTT_CFG;
                            isPdpContextGotten = 1;
                        }
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
                    devConfig.mqtt_state = STATE_MQTT_OPEN;
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
                ret = RIL_MQTT_QMTOPEN(connect_id,devConfig.serverHost,devConfig.serverPort);
                if(RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("//<Start opening a MQTT client\r\n");
                    if(FALSE == CLOSE_flag)
                        CLOSE_flag = TRUE;
                    devConfig.mqtt_state = STATE_MQTT_TOTAL_NUM;
                }
                else
                {
                    APP_DEBUG("//<Open a MQTT client failure,ret = %d-->\r\n",ret);
                }
                break;
            }
            case STATE_MQTT_CONN:
            {
			    ret = RIL_MQTT_QMTCONN(connect_id,clientID,username,passwd);
	            if(RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("//<Start connect to MQTT server\r\n");
                    if(FALSE == DISC_flag)
                        DISC_flag = TRUE;
                    devConfig.mqtt_state = STATE_MQTT_TOTAL_NUM;
                }
                else
                {
                    APP_DEBUG("//<connect to MQTT server failure,ret = %d\r\n",ret);
                }
                break;
            }
			case STATE_MQTT_SUB:
            {				
                mqtt_topic_info_t.count = 1;
				mqtt_topic_info_t.topic[0] = (u8*)Ql_MEM_Alloc(sizeof(u8)*256);
				
				Ql_memset(mqtt_topic_info_t.topic[0],0,256);
				Ql_memcpy(mqtt_topic_info_t.topic[0],test_topic,Ql_strlen(test_topic));
                mqtt_topic_info_t.qos[0] = QOS1_AT_LEASET_ONCE;
				sub_message_id++;  //< 1-65535.
				
				ret = RIL_MQTT_QMTSUB(connect_id,sub_message_id,&mqtt_topic_info_t);
				
				Ql_MEM_Free(mqtt_topic_info_t.topic[0]);
	            mqtt_topic_info_t.topic[0] = NULL;
                if(RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("//<Start subscribe topic\r\n");
                    devConfig.mqtt_state = STATE_MQTT_TOTAL_NUM;
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
				ret = RIL_MQTT_QMTPUB(connect_id,pub_message_id,QOS1_AT_LEASET_ONCE,0,test_topic,Ql_strlen(pre_NMEA_buf),pre_NMEA_buf);
                if(RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("//<Start publish a message to MQTT server\r\n");
                    devConfig.mqtt_state = STATE_MQTT_TOTAL_NUM;
                }
                else
                {
                    APP_DEBUG("//<Publish a message to MQTT server failure,ret = %d\r\n",ret);
                }
                break;
            }
			case STATE_MQTT_TOTAL_NUM:
            {
                //<do nothing
			    break;
            }
            default:
                break;
        }    
    }
}




void publishMessage(u32 message_id, u8* topic,u8* out_data){
    s32 ret;
    ret = RIL_MQTT_QMTPUB(connect_id,message_id,QOS1_AT_LEASET_ONCE,0,topic,Ql_strlen(out_data),out_data);
    if(RIL_AT_SUCCESS == ret)
    {
        APP_DEBUG("//<Start publish a message to MQTT server\r\n");
        devConfig.mqtt_state = STATE_MQTT_TOTAL_NUM;
    }
    else
    {
        APP_DEBUG("//<Publish a message to MQTT server failure,ret = %d\r\n",ret);
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
        APP_DEBUG("nmea failed to load\r\n");
		return FALSE;
	}
	strLen = Ql_strlen(NMEA_Str);
	for(u8 i = 0 ; i < strLen; i++)
	{
        APP_DEBUG("%c",*p1);
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
    APP_DEBUG("\r\n");
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
					devConfig.gnss_state = STATE_GNSS_AGPS_AID;
					APP_DEBUG("\r\n<--Download AGPS data successful-->\r\n");
				}
				else
				{
					APP_DEBUG("\r\n<--Download AGPS data failed-->\r\n");
				}
            }
        }
}




static void GnssCallback_Timer(u32 timerId, void* param)
{
    s32 ret;
    if (GNSS_TIMER_ID == timerId)
    {
        switch (devConfig.gnss_state)
        {
			case STATE_GNSS_POWERON:
			{
				ret = RIL_GNSS_Open(1);
				if(ret == RIL_AT_SUCCESS)
				{
					APP_DEBUG("\r\n<-- Open GNSS OK-->\r\n");
					devConfig.gnss_state = STATE_GNSS_QUERY_STATE;
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
                    devConfig.gnss_state = STATE_GNSS_APN_CONFIG;
                }
                break;
            }
			case STATE_GNSS_APN_CONFIG:
            {
                ret = RIL_GNSS_EPO_Config_APN(0,APN, username,passwd);
                if (RIL_ATRSP_SUCCESS == ret)
                {
                    APP_DEBUG("<--configure APN of GNSS context OK.-->\r\n");
					//devConfig.gnss_state = STATE_GNSS_PDP_Context;
                    devConfig.gnss_state = STATE_GNSS_AGPS_START;
                }
				else
                {
                    APP_DEBUG("<--configure APN of GNSS context fail,ret=%d.-->\r\n",ret);
                }
                break;
            }
			case STATE_GNSS_PDP_Context:
			{
                if(isPdpContextGotten == 1){  // check if pdp has been activated and skip
                    devConfig.gnss_state = STATE_GNSS_AGPS_START;
                }
                else{
                    ret = RIL_NW_OpenPDPContext();
                    if (RIL_ATRSP_SUCCESS == ret)
                    {
                        APP_DEBUG("<--GNSS PDP active sucessful.-->\r\n");
                        devConfig.gnss_state = STATE_GNSS_AGPS_START;
                        isPdpContextGotten = 1;
                    }
                    else
                    {
                        APP_DEBUG("<--GNSS PDP active fail,ret=%d.-->\r\n",ret);
                    }
                }
                break;
			}
            case STATE_GNSS_AGPS_START:
            {
				ret = RIL_GNSS_AGPS(Callback_GNSS_APGS_Hdlr);
				if(ret == RIL_ATRSP_SUCCESS)
				{
					devConfig.gnss_state = STATE_GNSS_TOTAL_NUM;
                    APP_DEBUG("Start Download AGPS data, iRet = %d.\r\n", ret);
				}
                else
                {
					APP_DEBUG("<--Enable EPO download fail.-->\r\n");
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
				devConfig.gnss_state = STATE_GNSS_CHECK_FIX;
                APP_DEBUG("AGPS aiding successful, iRet = %d.\r\n", ret);
                break;
            }
			case STATE_GNSS_CHECK_FIX:
            {
				u8 rd_buf[1024] = {0};
				s32 ret = RIL_GNSS_Read("ALL", rd_buf);
				if(ret == RIL_ATRSP_SUCCESS)
				{
					//APP_DEBUG("%s\r\n", rd_buf);
					if(GNSS_Get_FixStatus_from_RMC(rd_buf) == TRUE)
					{
					    APP_DEBUG("\r\n<--GNSS Successful position sucessful-->\r\n");
						devConfig.gnss_state = STATE_GNSS_READ_ALL_NMEA;
					}
					else
					{
						APP_DEBUG("<--GPS not fixed.-->\r\n");
					}
				}
                else
                {
					APP_DEBUG("<--Read RMC fail.-->\r\n");
                }
                break;
            }
			case STATE_GNSS_READ_ALL_NMEA:
			{
				Ql_memset(m_TxBuf_Uart,0, sizeof(m_TxBuf_Uart));
				ret = RIL_GNSS_Read("ALL", m_TxBuf_Uart);
				if(Ql_strcmp(pre_NMEA_buf, m_TxBuf_Uart))
				{
                    u8 deviceData[500];
					Ql_memset(pre_NMEA_buf,0, sizeof(pre_NMEA_buf));
                    Ql_memset(deviceData,0, sizeof(deviceData));
					Ql_strcpy(pre_NMEA_buf, m_TxBuf_Uart);
                    Ql_strcpy(devConfig.gpsData,pre_NMEA_buf);

					remainLen = Ql_strlen(m_TxBuf_Uart);

                    gnss_data_status = GNSS_DATA_EMPTY;
					if(RIL_ATRSP_SUCCESS == ret)
					{
						if(remainLen > 0)
						{
                            gnss_data_status = GNSS_DATA_SET;
							s32 retLen = Ql_UART_Write(m_myUartPort, m_TxBuf_Uart, remainLen);
							if(retLen < remainLen)
							{
								remainLen -= ret;
								Ql_memmove(m_TxBuf_Uart, m_TxBuf_Uart+retLen, remainLen);
							}
						}
                        else{

                        }
					}
				}
				break;
			}		
            default:
                break;
        }    
    }
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

    char magic[4] = {0}; 
    readFromStorage("magic.txt",magic);
    devConfig.magic = (magic[0] << 24) | (magic[1] << 16) | (magic[2] << 8) | (magic[3] & 0xff);

    char ver[2] = {0};
    readFromStorage("ver.txt",ver);
    devConfig.ver = (ver[0] << 8) | (ver[1] & 0xff);

    char password[MAX_PASS_LEN] = {0};
    readFromStorage("pwd.txt",password);
    Ql_strcpy(devConfig.password,password);

    char allowPublic[1] = {0};
    readFromStorage("allowPublic.txt",allowPublic);
    devConfig.allowPublic = (u8)allowPublic[0];

    char unitName[MAX_NAME_LEN] = {0};
    readFromStorage("name.txt",unitName);
    Ql_strcpy(devConfig.unitName,unitName);

    char apn[MAX_APN_LEN] = {0};
    readFromStorage("apn.txt",apn);
    Ql_strcpy(devConfig.apn,apn);

    char apnUser[32] = {0};
    readFromStorage("apnUser.txt",apnUser);
    Ql_strcpy(devConfig.apnUser,apnUser);

    char apnPass[32] = {0};
    readFromStorage("apnPass.txt",apnPass);
    Ql_strcpy(devConfig.apnPass,apnPass);

    char serverHost[MAX_HOST_LEN] = {0};
    readFromStorage("serverHost.txt",serverHost);
    Ql_strcpy(devConfig.serverHost,serverHost);

    char serverPort[2] = {0};
    readFromStorage("serverPort.txt",serverPort);
    devConfig.serverPort = (serverPort[0] << 8) | (serverPort[1] & 0xff);

    char rptSec[2] = {0}; // report interval
    readFromStorage("rptSec.txt",rptSec);
    devConfig.rptSec = (rptSec[0] << 8) | (rptSec[1] & 0xff);

    char slpSec[2] = {0}; // sleep interval
    readFromStorage("slpSec.txt",slpSec);
    devConfig.slpSec = (slpSec[0] << 8) | (slpSec[1] & 0xff);

    char runMode[1] = {0}; // sleep interval
    readFromStorage("runMode.txt",runMode);
    devConfig.runMode = runMode[0];

    char sleepMode[1] = {0}; // sleep mode
    readFromStorage("sleepMode.txt",sleepMode);
    devConfig.sleepMode = sleepMode[0];

    char periodicFindMode[1] = {0};
    readFromStorage("periodicFindMode.txt",periodicFindMode);
    devConfig.periodicFindMode = periodicFindMode[0];

    char findIntervalMin[2] = {0};
    readFromStorage("findIntervalMin.txt",findIntervalMin);
    devConfig.findIntervalMin = (findIntervalMin[0] << 8) | (servefindIntervalMinrPort[1] & 0xff);
    
    char tripReporting[1] = {0};
    readFromStorage("tripReporting.txt",peritripReportingodicFindMode);
    devConfig.tripReporting = tripReporting[0];

    char alarmReporting[1] = {0};
    readFromStorage("alarmReporting.txt",alarmReporting);
    devConfig.alarmReporting = alarmReporting[0];

    char speedReporting[1] = {0};
    readFromStorage("speedReporting.txt",speedReporting);
    devConfig.speedReporting = speedReporting[0];

    char speedLimitKph[2] = {0};
    readFromStorage("speedLimitKph.txt",speedLimitKph);
    devConfig.speedLimitKph = (speedLimitKph[0] << 8) | (speedLimitKph[1] & 0xff);

    char mainPowerAlarm[1] = {0};
    readFromStorage("mainPowerAlarm.txt",mainPowerAlarm);
    devConfig.mainPowerAlarm = mainPowerAlarm[0];

    char vehicleDisabled[1] = {0};
    readFromStorage("vehicleDisabled.txt",vehicleDisabled);
    devConfig.vehicleDisabled = vehicleDisabled[0];

    char ignitionReporting[1] = {0};
    readFromStorage("ignitionReporting.txt",ignitionReporting);
    devConfig.ignitionReporting = ignitionReporting[0];

    char odometerKm[4] = {0}; 
    readFromStorage("odometerKm.txt",odometerKm);
    devConfig.odometerKm = (odometerKm[0] << 24) | (odometerKm[1] << 16) | (odometerKm[2] << 8) | (odometerKm[3] & 0xff);

    char shockSensitivity[1] = {0};
    readFromStorage("shockSensitivity.txt",shockSensitivity);
    devConfig.shockSensitivity = shockSensitivity[0];

    char heartbeatMin[2] = {0};
    readFromStorage("heartbeatMin.txt",heartbeatMin);
    devConfig.heartbeatMin = (heartbeatMin[0] << 8) | (heartbeatMin[1] & 0xff);

    char userCount[1] = {0};
    readFromStorage("userCount.txt",userCount);
    devConfig.userCount = userCount[0];

    char timezoneOffset[2] = {0};
    readFromStorage("timezoneOffset.txt",timezoneOffset);
    devConfig.timezoneOffset = (timezoneOffset[0] << 8) | (timezoneOffset[1] & 0xff);

    char users[MAX_USERS * 15] = {0};
    readFromStorage("user.txt",users);
    Ql_strcpy(devConfig.users,users);

    char mqttUser[MAX_MQTT_USER] = {0};
    readFromStorage("mqttUser.txt",mqttUser);
    Ql_strcpy(devConfig.mqttUser,mqttUser);

    char mqttPass[MAX_MQTT_PASS] = {0};
    readFromStorage("mqttPass.txt",mqttPass);
    Ql_strcpy(devConfig.mqttPass,mqttPass);

    char myReportAlarm[1] = {0};
    readFromStorage("reportAlarm.txt",myReportAlarm);
    devConfig.reportAlarm = myReportAlarm[0];


    char mReportTrip[1] = {0};
    readFromStorage("reportTrip.txt",mReportTrip);
    devConfig.reportTrip = mReportTrip[0];


    char myReportSpeed[1] = {0};
    readFromStorage("reportSpeed.txt",myReportSpeed);
    devConfig.reportSpeed = myReportSpeed[0];


    char myReportPower[1] = {0};
    readFromStorage("reportPower.txt",myReportPower);
    devConfig.reportPower = myReportPower[0];
    
    Ql_strcpy(devConfig.HW_VERSION,"1.0.0");
    Ql_strcpy(devConfig.FW_VERSION,"1.0.0");
    
}





/* take setting actions*/
    


/* end take setting actions*/


/* take alarm actions*/


/* end take alarm actions*/



/* custom general functions */


