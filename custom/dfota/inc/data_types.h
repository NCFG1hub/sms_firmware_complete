
#ifndef DATA_TYPES
#define DATA_TYPES

#include "ql_type.h"
/* === Limits === */
#define MAX_USERS        10
#define MAX_NAME_LEN     16
#define MAX_APN_LEN      64
#define MAX_HOST_LEN     64
#define MAX_PASS_LEN     21

#define MAX_MQTT_USER 32
#define MAX_MQTT_PASS 32

#define MAX_USERS 10
#define GNSS_DATA_LENGTH 2048




typedef enum{
    STATE_NW_QUERY_STATE,
    STATE_MQTT_CFG,
    STATE_MQTT_OPEN,
    STATE_MQTT_CONN,
    STATE_MQTT_SUB,
    STATE_MQTT_PUB,
    STATE_MQTT_TUNS,
    STATE_MQTT_CLOSE,
    STATE_MQTT_DISC,
    STATE_MQTT_TOTAL_NUM
} MQTT_STATE_T;



typedef enum{
	STATE_GNSS_POWERON,
    STATE_GNSS_QUERY_STATE,
    STATE_GNSS_APN_CONFIG,
    STATE_GNSS_PDP_Context,
	STATE_GNSS_AGPS_START,
	STATE_GNSS_AGPS_AID,
	STATE_GNSS_CHECK_FIX,
	STATE_GNSS_READ_ALL_NMEA,
    STATE_GNSS_TOTAL_NUM
}Enum_GNSS_STATE;




/* === Main device configuration === */
typedef struct {
    u32   magic;        // CFG_MAGIC
    u16   ver;          // CFG_VER

    char  password[MAX_PASS_LEN];    // default "0000"
    u8    allowPublic;               // 0 = only registered users, 1 = allow all
    char  unitName[MAX_NAME_LEN];    // device name

    /* === GPRS / Server Settings === */
    char  apn[MAX_APN_LEN];
    char  apnUser[32];
    char  apnPass[32];
    char  serverHost[MAX_HOST_LEN];
    u16   serverPort;
    u16   rptSec;       // report interval in seconds
    u16   slpSec;       // sleep interval in seconds
    u8    runMode;      // 0 = off, 1 = TCP
    

    u8    sleepMode;
    u8    periodicFindMode;   // 0=off, 1=on
    u16   findIntervalMin;     // minutes between FIND reports
    u8    tripReporting;      // 0=off, 1=on
    u8    alarmReporting;
    u8    speedReporting;     // 0=off, 1=on
    u16   speedLimitKph;      // km/h
    u8    mainPowerAlarm;     // 0=off, 1=on
    u8    vehicleDisabled;    // 0=enabled, 1=disabled
    u8    ignitionReporting;  // 0=off, 1=on
    u32   odometerKm;
    u8    shockSensitivity;   // 0=off, 1-5 = sensitivity level
    u16   heartbeatMin;

    u16   reportAlarm;
    u16   reportTrip;
    u16   reportSpeed;
    u16   reportPower;

    /* === Registered users === */
    u8    userCount;
    char users[MAX_USERS * 25];
    int  timezoneOffset;     // Timezone offset from GMT
    char  mqttUser[MAX_MQTT_USER];
    char  mqttPass[MAX_MQTT_PASS];
    u8 ignationStatus;
    char gpsData[GNSS_DATA_LENGTH];
    MQTT_STATE_T mqtt_state;
    Enum_GNSS_STATE gnss_state;
    char FW_VERSION[10];
    char HW_VERSION[10];
    char imei[25];
    u8 outputControl;
    char serverIP[20];
    char deviceTopic[22];
    char clientId[30];
    u8 isSimCardReaddy;
    u8 isGsmNetworkGotten;
    u8 isGpRsNetworkGotten;
    u8 isApnSet;
    u8 isMqttOpen;
    u8 isMqttLoggedIn;
    u8 isSubscribedToTopic;
    u8 isGnssPowered;
    u8 isGnssConfigured;
    u8 isGnssSetupComplete;
    u8 isGnssDowloaded;
    u8 isGnssDataInjected;
    u8 isFixedGnssPosition;
    u8 isGnssRead;
    double longitude;
    double latitude;
    double speed;
    float batteryLevel;
    u8 trip;
    u8 lastRelayStatus;
    u8 ignationLevel;
    u8 lastIgnationLevel;
} DeviceConfig;



typedef struct {
    u16 mcc;
    u16 mnc;
    u32 lac;
    s32 cellId;
    s16 rssi;
    u16 timeAd;
} myCellInfo;


typedef struct{
    double utcTime;
    char status;
    double latitude;
    char latDirection;
    double longitude;
    char longDirection;
    double speed;
    float courseOverGround; 
    u32 deviceDate;
    float magneticVariation;
} GPS_LOCATION_T;

typedef enum {
    DISCONNECTED,
    SERVERCONNECTION,
    LOGGEDIN

} MQTT_CON_STATUS_T;

#endif

