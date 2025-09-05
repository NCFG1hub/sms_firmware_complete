
#ifndef tracker_functions_h
#define tracker_functions_h

#include "ql_type.h"
#include "data_types.h"

int get_gsm_signal_strength(DeviceConfig *g_cfg);
int get_battery_percentage(DeviceConfig *g_cfg);
int get_external_power_status(DeviceConfig *g_cfg);
int get_ignition_status(DeviceConfig *g_cfg);
int get_last_known_location(double* lat, double* lon);
int get_mileage();
char* reverse_geocode(double lat, double lon);
char* get_nearest_places(char* places,int length);
int get_device_imei(char* out, int maxLen);
int stop_current_trip(); 
int get_trip_summary();
int storage_clear_logs();

#endif