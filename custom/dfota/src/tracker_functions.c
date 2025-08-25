

#include "ril.h"
#include "ril_network.h"
#include "ql_gpio.h"
#include "ql_type.h"
#include "ql_adc.h"
#include "ql_uart.h"
#include "ql_stdlib.h"



#define DBG_PORT UART_PORT1
#define APP_DEBUG(FMT, ...) do { \
    char __buf[256]; \
    Ql_sprintf(__buf, FMT, ##__VA_ARGS__); \
    Ql_UART_Write(DBG_PORT, (u8*)__buf, Ql_strlen(__buf)); \
} while(0)


int get_gsm_signal_strength(DeviceConfig *g_cfg){
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
    Ql_ADC_Init(adcPin);

    // Read ADC
    ret = Ql_ADC_Read(adcPin, &adcValue);
    if (ret == QL_RET_OK) {
        float vAdc = (adcValue * 2.8f) / 1023.0f;
        float vBat = vAdc * ((100.0f + 47.0f) / 47.0f); // Example R1=100k, R2=47k
        APP_DEBUG("ADC=%d, Vadc=%.3f V, Vbat=%.3f V\r\n", adcValue, vAdc, vBat);
    } else {
        APP_DEBUG("ADC Read failed\r\n");
    }

    return (int)adcValue;
}

int get_external_power_status(DeviceConfig *g_cfg){
    Enum_PinName adcPin = PIN_ADC0;
    u32 adcValue = 0;
    s32 ret;

    // Init ADC
    Ql_ADC_Init(adcPin);

    // Read ADC
    ret = Ql_ADC_Read(adcPin, &adcValue);
    if (ret == QL_RET_OK) {
        float vAdc = (adcValue * 2.8f) / 1023.0f;
        float vBat = vAdc * ((100.0f + 47.0f) / 47.0f); // Example R1=100k, R2=47k
        APP_DEBUG("ADC=%d, Vadc=%.3f V, Vbat=%.3f V\r\n", adcValue, vAdc, vBat);
    } else {
        APP_DEBUG("ADC Read failed\r\n");
    }

    return (int)adcValue;
}

int get_ignition_status(DeviceConfig *g_cfg){
     return (int)g_cfg->ignationStatus;
}


int get_last_known_location(&lat, &lon){
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

    // If the line starts with digits, it's the IMEI
    if (Ql_isdigit(line[0])) {
        // Copy only 14 characters
        Ql_memset(imeiBuf, 0, 15);   // clear first
        Ql_memcpy(imeiBuf, line, 14);
        imeiBuf[14] = '\0';          // null terminate
        return RIL_ATRSP_SUCCESS;    // done
    }

    if (Ql_strstr(line, "OK")) {
        return RIL_ATRSP_SUCCESS;
    } else if (Ql_strstr(line, "ERROR")) {
        return RIL_ATRSP_FAILED;
    }

    return RIL_ATRSP_CONTINUE;
}

void get_device_imei(char* out, int maxLen) {
    char imei[15] = {0};   // buffer for 14 chars + '\0'

    s32 ret = Ql_RIL_SendATCmd("AT+GSN\r\n", ATResponse_IMEI, (void*)imei, 5000);
    if (ret == RIL_AT_SUCCESS) {
        Ql_strncpy(out,imei,maxLen);
        APP_DEBUG("IMEI (14 digits): %s\r\n", imei);
    } else {
        APP_DEBUG("Failed to get IMEI, ret=%d\r\n", ret);
    }
}