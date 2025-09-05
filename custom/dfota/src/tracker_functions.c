

#include "ril.h"
#include "ril_network.h"
#include "ql_gpio.h"
#include "ql_type.h"
#include "ql_adc.h"
#include "ql_uart.h"
#include "ql_stdlib.h"
#include "data_types.h"
#include "ql_trace.h"
#include "ql_system.h"
#include "ql_error.h"




char *my_strtok2(char *str, const char *delim); 

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


int get_gsm_signal_strength(DeviceConfig *g_cfg) {
    s32 rssi;
    s32 ber;
    s32 ret;
    
    int rssi_dBm;
    ret = RIL_NW_GetSignalQuality(&rssi, &ber);
    if (ret == RIL_AT_SUCCESS) {
        if (rssi == 99) {
            rssi_dBm = -999; // unknown
        } else if (rssi == 0) {
            rssi_dBm = -113;
        } else if (rssi == 1) {
            rssi_dBm = -111;
        } else {
            rssi_dBm = -113 + (rssi * 2);
        }
    } else {
        APP_DEBUG("<-- Failed to get signal quality -->\r\n");
    }

    return rssi_dBm;
}

int get_battery_percentage(DeviceConfig *g_cfg){
    Enum_PinName adcPin = PIN_ADC0;
    u32 adcValue = 0;
    s32 ret;

    // Init ADC
    Ql_ADC_Open(adcPin,ADC_PERIOD_250MS);

    // Read ADC
    ret = Ql_ADC_Read(adcPin, &adcValue);
    if (ret == QL_RET_OK) {
        float vAdc = (adcValue * 2.8f) / 1023.0f;
        float vBat = vAdc * ((100.0f + 47.0f) / 47.0f); // Example R1=100k, R2=47k
        APP_DEBUG("ADC=%d, Vadc=%.3f V, Vbat=%.3f V\r\n", adcValue, vAdc, vBat);

    } else {
        APP_DEBUG("ADC Read failed\r\n");
    }
    Ql_ADC_Close(adcPin);
    return (int)adcValue;
}

int get_external_power_status(DeviceConfig *g_cfg){
    Enum_PinName adcPin = PIN_ADC0;
    u32 adcValue = 0;
    s32 ret;

    // Init ADC
    Ql_ADC_Open(adcPin,ADC_PERIOD_250MS);

    // Read ADC
    ret = Ql_ADC_Read(adcPin, &adcValue);
    if (ret == QL_RET_OK) {
        float vAdc = (adcValue * 2.8f) / 1023.0f;
        float vBat = vAdc * ((100.0f + 47.0f) / 47.0f); // Example R1=100k, R2=47k
        APP_DEBUG("ADC=%d, Vadc=%.3f V, Vbat=%.3f V\r\n", adcValue, vAdc, vBat);
    } else {
        APP_DEBUG("ADC Read failed\r\n");
    }
     Ql_ADC_Close(adcPin);
    return (int)adcValue;
}

int get_ignition_status(DeviceConfig *g_cfg){
     return (int)g_cfg->ignationStatus;
}


int get_last_known_location(double* lat, double* lon){
    int ret;

    return ret;
}


int get_mileage(){
    int ret;

    return ret;
}


char* reverse_geocode(double lat, double lon){
     char* myAddress = "";

     return myAddress;
}

char* get_nearest_places(char* places,int length){
     char* myAddress = "";

     return myAddress;
}


static s32 ATResponse_IMEI(char* line, u32 len, void* userData)
{
    char* imeiBuf = (char*)userData;   // userData is caller’s buffer
    
    if (Ql_isdigit(line[0])) {
        // Copy only 14 characters
        Ql_memset(imeiBuf, 0, 15);   // clear first
        Ql_memcpy(imeiBuf, line, 14);
        imeiBuf[14] = '\0';          // null terminate
        return RIL_ATRSP_SUCCESS;    // done
    }
    APP_DEBUG("//device imei only = %s\r\n",imeiBuf);

    if (Ql_strstr(line, "OK")) {
        return RIL_ATRSP_SUCCESS;
    } else if (Ql_strstr(line, "ERROR")) {
        return RIL_ATRSP_FAILED;
    }

    return RIL_ATRSP_CONTINUE;
}

int get_device_imei(char* out, int maxLen) {
    char imei[15] = {0};   // buffer for 14 chars + '\0'

    char strAT[] = "AT+CGSN\0";

    s32 ret = Ql_RIL_SendATCmd(strAT,Ql_strlen(strAT) , ATResponse_IMEI, (void*)imei, 0);
    if (ret == RIL_AT_SUCCESS) {
        Ql_strncpy(out,imei,maxLen);
        out[13] = '\0'; 
        APP_DEBUG("IMEI (14 digits): %s\r\n", out);
    } else {
        APP_DEBUG("Failed to get IMEI, ret=%d\r\n", ret);
    }
    return (int)ret;
}


int stop_current_trip(){

    return 1;
}


int get_trip_summary(){

    return 1;
}


int storage_clear_logs(){

    return 1;
}


char *my_strtok2(char *str, const char *delim) {
    static char *saved = NULL;   // remembers position between calls
    if (str != NULL) {
        saved = str;             // initialize new string
    }
    if (saved == NULL) {
        return NULL;             // no more tokens
    }

    // Skip leading delimiters
    char *token_start = saved;
    while (*token_start && Ql_strchr(delim, *token_start)) {
        token_start++;
    }
    if (*token_start == '\0') {
        saved = NULL;            // reached end
        return NULL;
    }

    // Find end of token
    char *token_end = token_start;
    while (*token_end && !Ql_strchr(delim, *token_end)) {
        token_end++;
    }

    if (*token_end != '\0') {
        *token_end = '\0';       // terminate token
        saved = token_end + 1;   // move past delimiter
    } else {
        saved = NULL;            // reached end of string
    }

    return token_start;
}
