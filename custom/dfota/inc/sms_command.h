#ifndef SMS_COMMANDS_H
#define SMS_COMMANDS_H

#include "data_types.h"

/* === Initialize SMS subsystem === */
void sms_pump(char* smsSender, char* smsContent, DeviceConfig *devConfig);
static void handle_www_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_heartbeat_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_logger_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_shock_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_set_odo_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_ignition_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_disable_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_mainpower_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_speed_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_listen_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_alarm_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_trip_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_find_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_help_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_sleep_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_reboot_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_version_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_status_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_settings_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_sms_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_time_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_deluser_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_listuser_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_password_read_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_getreport_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_password_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_allowpublic_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_name_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_adduser_command(const char* sender, const char* body, DeviceConfig *devConfig);
static void handle_add_command(const char* sender, const char* body, DeviceConfig *devConfig);
static bool is_authorized(const char* number, DeviceConfig *g_cfg); 

s32 readFromFlashString(char* fileName,char* data,u32 length);
s32 readFromFlash(char* fileName,char* data,u32 length);
s32 readAlphabetFromFlash(char* fileName,char* data,u32 length);
void cleanString(char *input, char *output);
void cleanAlpha(char *input, char *output);


#endif /* SMS_COMMANDS_H */