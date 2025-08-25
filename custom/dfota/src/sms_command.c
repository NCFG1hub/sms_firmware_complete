#include "sms_command.h"
#include "ql_trace.h"
#include "ql_stdlib.h"
#include "ql_uart.h"
#include "storage.h"
#include "config.h"
#include "trip.h"
#include "gprs.h"
#include "cJSON.c"
#include "ril_sms.h"
#include "data_types.h"
#include "ril.h"
#include "ril_network.h"
#include "ril_gnss.h"
#include "ql_type.h"
#include "gnss_parser.h"


char *my_strtok(char *str, const char *delim);

#define DBG_PORT UART_PORT1
#define APP_DEBUG(FMT, ...) do { \
    char __buf[256]; \
    Ql_sprintf(__buf, FMT, ##__VA_ARGS__); \
    Ql_UART_Write(DBG_PORT, (u8*)__buf, Ql_strlen(__buf)); \
} while(0)

/* === Internal buffer for reading SMS === */
static char smsContent[160];
static char smsSender[24];



void sms_reply(const char* destNumber, const char* message) {
    u32 uMsgRef = 0;
    char buf[200];
    Ql_sprintf(buf, "NCFTrack: %s", message);
    iResult = RIL_SMS_SendSMS_Text(destNumber, Ql_strlen(destNumber),LIB_SMS_CHARSET_GSM,(u8*)buf,Ql_strlen(buf),&uMsgRef);
    if (iResult != RIL_AT_SUCCESS)
    {
        APP_DEBUG("[SMS] Reply failed (%d) to %s\r\n", res, destNumber);
        return;
    }
    APP_DEBUG("[SMS] Reply sent to %s\r\n", destNumber);
}


static bool is_authorized(const char* number) {
    if (g_cfg->allowPublic) return true;
    for (u8 i = 0; i < g_cfg->userCount; i++) {
        if (Ql_strcmp(number, g_cfg->users[i].msisdn) == 0) {
            return true;
        }
    }
    return false;
}

/* === Handle ADD command (Change Device Password) === */
static void handle_add_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    const char* newPass = body + 4; // skip "ADD,"
    while (*newPass == ' ') newPass++; // skip spaces

    u32 len = Ql_strlen(newPass);
    if (len < 4 || len > 20) {
        sms_reply(sender, "Failed (password length must be 4-20 chars)");
        return;
    }

    /* === Check it's Alphanumeric Only === */
    for (u32 i = 0; i < len; i++) {
        if (!( (newPass[i] >= '0' && newPass[i] <= '9') ||
               (newPass[i] >= 'A' && newPass[i] <= 'Z') ||
               (newPass[i] >= 'a' && newPass[i] <= 'z') )) {
            sms_reply(sender, "Failed (password must be letters/numbers only)");
            return;
        }
    }

    char* myNewPass = Ql_MEM_Alloc(len); // assign memory to hold the password

    Ql_strcpy(myNewPass, newPass);
    if (saveBytesToFlash("pwd.txt",myNewPass,len) == SUCCESS) {
        Ql_strcpy(g_cfg->password,myNewPass);
        char successMsg[100];
        Ql_sprintf(successMsg, "Password changed to %s", newPass);
        sms_reply(sender, successMsg);
    } else {
        sms_reply(sender, "Failed to save password");
    }
}

/* === Handle Add User SMS Command === */
static void handle_adduser_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    const char* numPtr;
    if (Ql_strncmp(body, "ADDUSER,", 8) == 0 || Ql_strncmp(body, "AddUser,", 8) == 0)
        numPtr = body + 8;
    else
        numPtr = body + 3; // "AU,"

    while (*numPtr == ' ') numPtr++; // skip spaces

    u32 len = Ql_strlen(numPtr);
    if (len < 7 || len > 20) {
        sms_reply(sender, "Failed (invalid phone number length)");
        return;
    }

    /* === Check Digits And '+' Sign At The Start === */
    if (numPtr[0] != '+' && (numPtr[0] < '0' || numPtr[0] > '9')) {
        sms_reply(sender, "Failed (invalid phone number format)");
        return;
    }
    for (u32 i = 1; i < len; i++) {
        if (numPtr[i] < '0' || numPtr[i] > '9') {
            sms_reply(sender, "Failed (phone number must be digits only)");
            return;
        }
    }



    char* splitedUsers = my_strtok(g_cfg->users,",");

    /* === Check If Already Registered === */
    for (u8 i = 0; i < g_cfg->userCount; i++) {
        char* nextUser = strtok(splitedUsers,",");
        splitedUsers = nextUser + strlen(nextUser) + 1;
        if (Ql_strcmp(nextUser, numPtr) == 0) {
            sms_reply(sender, "Failed (number already exists)");
            return;
        }
    }

    /* === Check User Limit === */
    if (g_cfg->userCount >= MAX_USERS) {
        sms_reply(sender, "Failed (user list full)");
        return;
    }

    /* === Add The User If All Is Fine === */
    g_cfg->userCount++;

    char* allNewUser = Ql_MEM_Alloc(Ql_strlen(numPtr) + Ql_strlen(users) + 1);
    Ql_strcat(allNewUser,g_cfg->users);
    Ql_strcat(allNewUser,",");
    Ql_strcat(allNewUser,numPtr);

    if (saveBytesToFlash("user.txt",allNewUser,Ql_strlen(allNewUser)) == SUCCESS) {
        char msg[80];
        Ql_sprintf(msg, "User added %s", numPtr);
        saveBytesToFlash("userCount.txt",g_cfg->userCount,2); // update the total number of user saved
        sms_reply(sender, msg);
    } else {
        sms_reply(sender, "Failed to save new user");
    }
}

/* === Handle Add Unit Name SMS Command === */
static void handle_name_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    const char* namePtr;

    if (Ql_strncmp(body, "NAME,", 5) == 0 || Ql_strncmp(body, "Name,", 5) == 0) {
        namePtr = body + 5;
    } else {
        namePtr = body + 4; // skip "N,UN"
    }

    while (*namePtr == ' ') namePtr++; /* skip spaces */

    u32 len = Ql_strlen(namePtr);
    if (len == 0 || len > 15) {
        sms_reply(sender, "Failed (unit name length must be 1-15 chars)");
        return;
    }

    for (u32 i = 0; i < len; i++) {
        if (!( (namePtr[i] >= '0' && namePtr[i] <= '9') ||
               (namePtr[i] >= 'A' && namePtr[i] <= 'Z') ||
               (namePtr[i] >= 'a' && namePtr[i] <= 'z') ||
               (namePtr[i] == ' ') )) {
            sms_reply(sender, "Failed (unit name must be letters/numbers/spaces only)");
            return;
        }
    }

    if (saveBytesToFlash("name.txt",namePtr,Ql_strlen(namePtr)) == SUCCESS) {
        char msg[60];
        Ql_strcpy(g_cfg->unitName,namePtr);
        Ql_sprintf(msg, "Unit name changed to %s", namePtr);
        sms_reply(sender, msg);
    } else {
        sms_reply(sender, "Failed to save unit name");
    }
}

/* === Handle Allow Public SMS Command === */
static void handle_allowpublic_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    const char* valPtr;

    if (Ql_strncmp(body, "SET,ALLOWPUBLIC,", 16) == 0 ||
        Ql_strncmp(body, "Set,AllowPublic,", 16) == 0) {
        valPtr = body + 16;
    } else {
        valPtr = body + 17; // "Set,AllowPubli c," short form handling
    }

    while (*valPtr == ' ') valPtr++; /* skip spaces */

    if (*valPtr != '0' && *valPtr != '1') {
        sms_reply(sender, "Failed (value must be 0 or 1)");
        return;
    }

    char allowPublic[1];
    allowPublic[0] = (*valPtr == '1') ? 1 : 0;

    if (saveBytesToFlash("allowPublic.txt",allowPublic,1) == SUCCESS) {
        char msg[50];
        g_cfg->allowPublic = allowPublic[0];
        Ql_sprintf(msg, "AllowPublic set to %d", allowPublic);
        sms_reply(sender, msg);
    } else {
        sms_reply(sender, "Failed to save AllowPublic setting");
    }
}

/* === Handle Reset Password SMS Command === */
static void handle_password_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    const char* args = body + 9; // skip "PASSWORD,"
    while (*args == ' ') args++;

    char oldPass[21] = {0};
    char newPass[21] = {0};

    // Extract old and new passwords
    if (Ql_sscanf(args, "%20[^,],%20s", oldPass, newPass) != 2) {
        sms_reply(sender, "Failed (format: PASSWORD,old,new)");
        return;
    }


    // Check if old password matches
    if (Ql_strcmp(oldPass, g_cfg->password) != 0) {
        sms_reply(sender, "Access denied (wrong old password)");
        return;
    }

    u32 len = Ql_strlen(newPass);
    if (len < 4 || len > 20) {
        sms_reply(sender, "Failed (password length must be 4-20 chars)");
        return;
    }

    for (u32 i = 0; i < len; i++) {
        if (!( (newPass[i] >= '0' && newPass[i] <= '9') ||
               (newPass[i] >= 'A' && newPass[i] <= 'Z') ||
               (newPass[i] >= 'a' && newPass[i] <= 'z') )) {
            sms_reply(sender, "Failed (password must be letters/numbers only)");
            return;
        }
    }

    if (saveBytesToFlash("pwd.txt",newPass,Ql_strlen(newPass)) == SUCCESS) {
        char msg[80];
        Ql_sprintf(msg, "Password changed to %s", newPass);
        Ql_strcpy(g_cfg->password,newPass);
        sms_reply(sender, msg);
    } else {
        sms_reply(sender, "Failed to save password");
    }
}

/* === Handle Get Report SMS Command === */  //############## need clarification
static void handle_getreport_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    u8 userIndex;
    char typeChar;
    char stateStr[6] = {0};

    // Skip GETREPORT, or GR,
    if (Ql_strncmp(body, "GETREPORT,", 10) == 0 || Ql_strncmp(body, "GetReport,", 10) == 0) {
        body += 10;
    } else {
        body += 3; // skip "GR,"
    }

    if (Ql_sscanf(body, "%hhu,%c,%5s", &userIndex, &typeChar, stateStr) != 3) {
        sms_reply(sender, "Failed (format: GR,<user>,<type>,on/off)");
        return;
    }

    if (userIndex < 1 || userIndex > MAX_USERS) {
        sms_reply(sender, "Failed (user index out of range)");
        return;
    }

    // Convert state to lower
    for (char* p = stateStr; *p; ++p) {
        if (*p >= 'A' && *p <= 'Z') *p += 32;
    }
    int enable = (Ql_strcmp(stateStr, "on") == 0) ? 1 :
                 (Ql_strcmp(stateStr, "off") == 0) ? 0 : -1;
    if (enable == -1) {
        sms_reply(sender, "Failed (state must be on/off)");
        return;
    }

    // Map type char to report flag
    switch (typeChar) {
        case 'A': case 'a': g_cfg->reportAlarm = enable; break;
        case 'T': case 't': g_cfg->reportTrip  = enable; break;
        case 'S': case 's': g_cfg->reportSpeed = enable; break;
        case 'P': case 'p': g_cfg->reportPower = enable; break;
        default:
            sms_reply(sender, "Failed (invalid report type)");
            return;
    }

    char reportAlarm[1] = {0};
    reportAlarm[0] = g_cfg->reportAlarm;
    saveBytesToFlash("reportAlarm.txt",reportAlarm,Ql_strlen(reportAlarm));

    char reportTrip[1] = {0};
    reportTrip[0] = g_cfg->reportTrip;
    saveBytesToFlash("reportTrip.txt",reportTrip,Ql_strlen(reportTrip));

    char reportSpeed[1] = {0};
    reportSpeed[0] = g_cfg->reportSpeed;
    saveBytesToFlash("reportSpeed.txt",reportSpeed,Ql_strlen(reportSpeed));

    char reportPower[1] = {0};
    reportPower[0] = g_cfg->reportPower;
    saveBytesToFlash("reportPower.txt",reportPower,Ql_strlen(reportPower));

    if (storage_save(&g_cfg)) {
        char msg[80];
        Ql_sprintf(msg, "User %d %s report %s", userIndex, 
                   (typeChar=='A'||typeChar=='a')?"Alarm":
                   (typeChar=='T'||typeChar=='t')?"Trip":
                   (typeChar=='S'||typeChar=='s')?"Speed":"Power",
                   enable ? "enabled" : "disabled");
        sms_reply(sender, msg);
    } else {
        sms_reply(sender, "Failed to save report setting");
    }
}

/* === Handle Get Device Password SMS Command === */
static void handle_password_read_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    // Insure it's exactly "PASSWORD" (case-insensitive) and nothing else
    if (Ql_stricmp(body, "PASSWORD") != 0) {
        sms_reply(sender, "Failed (invalid format)");
        return;
    }

    char msg[60];
    Ql_sprintf(msg, "Password is %s", g_cfg->password);
    sms_reply(sender, msg);
}

/* === Handle All List User SMS Command === */  //######## need clarification here
static void handle_listuser_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char arg[20] = {0};

    // Skip LISTUSER, or LU,
    if (Ql_strncmp(body, "LISTUSER", 8) == 0 || Ql_strncmp(body, "ListUser", 8) == 0) {
        if (Ql_strlen(body) > 8 && body[8] == ',')
            Ql_strcpy(arg, body + 9);
    } else {
        if (Ql_strlen(body) > 2 && body[2] == ',')
            Ql_strcpy(arg, body + 3);
    }

    char userlistMessage[300];
    Ql_strcat(userlistMessage,"NCFTRACK : list user User1: admin");

    char* splitedUsers = my_strtok(g_cfg->users,",");

    /* === Check If Already Registered === */
    for (u8 i = 0; i < g_cfg->userCount; i++) {
        char* nextUser = strtok(splitedUsers,",");
        splitedUsers = nextUser + strlen(nextUser) + 1;
        char myString[50];
        Ql_sprintf(myString,",User%d:%s",(i+2),nextUser); 
        Ql_strcat(userlistMessage,myString);
    }

    sms_reply(sender, userlistMessage);
}



/* === Handle Delete User SMS Command === */ //# need clarification
static void handle_deluser_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    int slot = -1;

    // Skip "DELUSER," or "DU,"
    if (Ql_strncmp(body, "DELETEUSER,", 11) == 0 || Ql_strncmp(body, "DeleteUser,", 11) == 0) {  
        slot = Ql_atoi(body + 8);
    } else if(Ql_strncmp(body, "du,", 3) == 0 || Ql_strncmp(body, "DU,", 3) == 0){
        slot = Ql_atoi(body + 3);
    }

    if (slot < 1 || slot > MAX_USERS) {
        sms_reply(sender, "Failed (invalid user slot)");
        return;
    }

    char* splitedUsers = my_strtok(g_cfg->users,",");
    char remainingUser[300];

    /* === Check If Already Registered === */
    for (u8 i = 0; i < g_cfg->userCount; i++) {
        char* nextUser = strtok(splitedUsers,",");
        splitedUsers = nextUser + strlen(nextUser) + 1;
        if((i+1) != slot){
            Ql_strcpy(remainingUser,nextUser);
        }
    }


    if (storage_save(saveBytesToFlash("user.txt",remainingUser,Ql_strlen(remainingUser)) == SUCCESS)) {
        char msg[40];
        Ql_sprintf(msg, "User %d deleted", slot);
        sms_reply(sender, msg);
    } else {
        sms_reply(sender, "Failed to delete user");
    }
}

/* === Handle Time Stamp SMS Command === */ 
static void handle_time_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    int offset = -100; // invalid default

    if (Ql_strncmp(body, "TIME,", 5) == 0 || Ql_strncmp(body, "Time,", 5) == 0) {
        offset = Ql_atoi(body + 5);
    }

    if (offset < -12 || offset > 12) {
        sms_reply(sender, "Failed (offset must be between -12 and +12)");
        return;
    }

    g_cfg->timezoneOffset = offset;
    char myTimeZoneOffset[2] = {(offset >> 8),(offset & 0xff)}; // convert the timezone to bytes and save
    if (saveBytesToFlash("timezoneOffset.txt",myTimeZoneOffset,Ql_strlen(myTimeZoneOffset)) == SUCCESS) {
        char msg[50];
        Ql_sprintf(msg, "Timezone offset set to %d", offset);
        sms_reply(sender, msg);
    } else {
        sms_reply(sender, "Failed to save timezone offset");
    }
}

/* === Handle SMS Receaving Phone Number SMS Command === */
static void handle_sms_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char phone[24] = {0};
    char message[161] = {0};

    if (Ql_strncmp(body, "SMS,", 4) == 0 || Ql_strncmp(body, "Sms,", 4) == 0) {
        // Expected format: SMS,<phone>,<message>
        const char* args = body + 4;
        while (*args == ' ') args++; // skip spaces

        // Extract phone and message
        const char* comma = Ql_strchr(args, ',');
        if (!comma) {
            sms_reply(sender, "Failed (format: SMS,number,message)");
            return;
        }

        int phoneLen = comma - args;
        if (phoneLen <= 0 || phoneLen >= sizeof(phone)) {
            sms_reply(sender, "Failed (invalid phone length)");
            return;
        }

        Ql_memcpy(phone, args, phoneLen);
        phone[phoneLen] = '\0';

        // Get message after the phone number
        Ql_strcpy(message, comma + 1);
        if (Ql_strlen(message) == 0) {
            sms_reply(sender, "Failed (message cannot be empty)");
            return;
        }

        // Send SMS
        u32 uMsgRef = 0;
        s32 ret = RIL_SMS_SendSMS_Text(phone,Ql_strlen(phone),LIB_SMS_CHARSET_GSM,(u8*)message,Ql_strlen(message),&uMsgRef);
        if (ret == 0) {
            char replyMsg[80];
            Ql_sprintf(replyMsg, "SMS sent to %s", phone);
            sms_reply(sender, replyMsg);
        } else {
            sms_reply(sender, "Failed to send SMS");
        }
    }
}

/* === Handle Gets The Device Settings SMS Command === */
static void handle_settings_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    if (Ql_stricmp(body, "SETTINGS") != 0) {
        sms_reply(sender, "Failed (invalid format)");
        return;
    }

    char msg[160];
    Ql_sprintf(msg,
        "PWD:%s TZ:%d\r\n"
        "SRV:%s:%d\r\n"
        "APN:%s,%s,%s\r\n"
        "RPT:%d SLP:%d\r\n"
        "Public:%d",
        g_cfg->password,
        g_cfg->timezoneOffset,
        g_cfg->serverIP,
        g_cfg->serverPort,
        g_cfg->apn,
        g_cfg->apnUser,
        g_cfg->apnPass,
        g_cfg->rptInterval,
        g_cfg->slpInterval,
        g_cfg->allowPublic
    );

    sms_reply(sender, msg);
}



/* === Handle Gets The Device Status SMS Command === */ //# need clarification
static void handle_status_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    if (!(Ql_stricmp(body, "STATUS") == 0 || Ql_stricmp(body, "ST") == 0)) {
        sms_reply(sender, "Failed (invalid format)");
        return;
    }

    int gsmSignal = get_gsm_signal_strength(g_cfg);   // 0-31
    int gpsFix    = get_gps_fix_status(g_cfg);        // 0 or 1
    double lat, lon;
   // get_last_known_location(&lat, &lon);         // Fill in last coordinates
    int battery  = get_battery_percentage(g_cfg);     // 0-100%
    int extPower = get_external_power_status(g_cfg);  // 0=off, 1=on
    int ign      = get_ignition_status(g_cfg);        // 0=off, 1=on

    char msg[160];
    Ql_sprintf(msg,
        "GSM:%d GPS:%s\r\n"
        "Lat:%.6f Lon:%.6f\r\n"
        "Batt:%d%% ExtPwr:%s\r\n"
        "Ign:%s",
        gsmSignal,
        gpsFix ? "Fix" : "NoFix",
        lat,
        lon,
        battery,
        extPower ? "ON" : "OFF",
        ign ? "ON" : "OFF"
    );

    sms_reply(sender, msg);
}




/* === Handle Get The Device Version SMS Command === */ // need clarification
static void handle_version_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    if (!(Ql_stricmp(body, "VERSION") == 0 || Ql_stricmp(body, "V") == 0)) {
        sms_reply(sender, "Failed (invalid format)");
        return;
    }

    char imei[20] = {0};
    get_device_imei(imei, sizeof(imei)); // Reads IMEI from GSM module

    char msg[160];
    
    Ql_sprintf(msg,
        "FW:%s HW:%s\r\nIMEI:%s\r\nBuild:%s %s",
        g_cfg->FW_VERSION,    // e.g., "1.0.0"
        g_cfg->HW_VERSION,    // e.g., "RevA"
        g_cfg->imei,
        __DATE__,      // Compile date
        __TIME__       // Compile time
    );

    sms_reply(sender, msg);
}




/* === Handle Device Reboot SMS Command === */
static void handle_reboot_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    if (!(Ql_stricmp(body, "REBOOT") == 0 || Ql_stricmp(body, "RB") == 0)) {
        sms_reply(sender, "Failed (invalid format)");
        return;
    }

    sms_reply(sender, "Will reboot after 2 seconds.");

    // Small delay to allow SMS to send before reboot
    Ql_Sleep(2000);

    // Reboot the MC65
    Ql_Reset();
}




/* === Handle Set Sleep Mode SMS Command === */
static void handle_sleep_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    if (Ql_strncmp(body, "SET,SLEEP,", 10) != 0) {
        sms_reply(sender, "Failed (format: SET,SLEEP,0/1)");
        return;
    }

    int mode = Ql_atoi(body + 10);
    if (mode != 0 && mode != 1) {
        sms_reply(sender, "Failed (X must be 0 or 1)");
        return;
    }

    g_cfg->sleepMode = mode;
    char mySleepMode[1] = {0};
    mySleepMode[0] = mode;
    saveBytesToFlash("sleepMode.txt",mySleepMode,Ql_strlen(mySleepMode);   // Save to flash so it persists after reboot

    if (mode == 1) {
        enter_low_power_mode();
        sms_reply(sender, "Sleep mode enabled");
    } else {
        exit_low_power_mode();
        sms_reply(sender, "Sleep mode disabled");
    }
}




/* === Handle Help SMS Command === */
static void handle_help_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    if (!(Ql_stricmp(body, "HELP") == 0 || Ql_stricmp(body, "?") == 0)) {
        sms_reply(sender, "Failed (invalid format)");
        return;
    }

    const char* helpText =
        "NCFTrack cmds:\r\n"
        "ADD,<pwd> Set password\r\n"
        "AU,<num> Add user\r\n"
        "NAME,<nm> Set unit name\r\n"
        "SETTINGS Show config\r\n"
        "ST Show status\r\n"
        "V Show version\r\n"
        "RB Reboot\r\n"
        "SET,SLEEP,0/1 Sleep mode\r\n"
        "HELP or ? Show cmds";

    sms_reply(sender, helpText);
}




/* === Handle All Find SMS Command === */ // # need clarification
static void handle_find_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[64];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);
    GNSS_Info myGpsInfo;

    if (Ql_strcmp(cmd, "FIND") == 0) {  // just get the gps location and send sms
        // Immediate location
        parseGPRMC(g_cfg->gpsData,&myGpsInfo);
        int mileage = get_mileage();
        const char* addr = reverse_geocode(myGpsInfo.latitude, myGpsInfo.longitude);

        char msg[160];
        Ql_sprintf(msg, "Lat:%.6f Lon:%.6f\r\nAddr:%s\r\nMileage:%dkm",
                   myGpsInfo.latitude, myGpsInfo.longitude, addr, mileage);
        sms_reply(sender, msg);

    } else if (Ql_strcmp(cmd, "FIND,NEAR") == 0) {
        // Nearest 3 points
        const char* places[3];
        get_nearest_places(places, 3); // this goes online to get the geocoder and report back to the user

        char msg[160];
        Ql_sprintf(msg, "Near:\r\n1.%s\r\n2.%s\r\n3.%s",
                   places[0], places[1], places[2]);
        sms_reply(sender, msg);

    } else if (Ql_strncmp(cmd, "FIND,", 5) == 0) {
        if (Ql_strcmp(cmd+5, "OFF") == 0) { // stops sending sms of location to user
            g_cfg->periodicFindMode = 0;
            char periodicFindMode[1] = {0};
            periodicFindMode[0] = g_cfg->periodicFindMode;
            saveBytesToFlash("periodicFindMode.txt",periodicFindMode,Ql_strlen(periodicFindMode)); 
            sms_reply(sender, "Periodic FIND OFF");
        } else if (Ql_strcmp(cmd+5, "DAILY") == 0) { // sends sms to user on location on daily basis
            g_cfg->periodicFindMode = 1;
            g_cfg->findIntervalMin = 1440;
            char periodicFindMode[1] = {0};
            char findIntervalMin[2] = {0};
            periodicFindMode[0] = g_cfg->periodicFindMode;
            findIntervalMin[0] = g_cfg->findIntervalMin >> 8;
            findIntervalMin[1] = g_cfg->findIntervalMin & 0xff;
            saveBytesToFlash("periodicFindMode.txt",periodicFindMode,Ql_strlen(periodicFindMode)); 
            saveBytesToFlash("findIntervalMin.txt",findIntervalMin,Ql_strlen(findIntervalMin)); 
            sms_reply(sender, "Daily FIND enabled");
        } else if (Ql_strcmp(cmd+5, "WEEKLY") == 0) {  // sends sms to user on location on weekly basis
            g_cfg->periodicFindMode = 1;
            g_cfg->findIntervalMin = 10080;
            char periodicFindMode[1] = {0};
            char findIntervalMin[2] = {0};
            periodicFindMode[0] = g_cfg->periodicFindMode;
            findIntervalMin[0] = g_cfg->findIntervalMin >> 8;
            findIntervalMin[1] = g_cfg->findIntervalMin & 0xff;
            saveBytesToFlash("periodicFindMode.txt",periodicFindMode,Ql_strlen(periodicFindMode)); 
            saveBytesToFlash("findIntervalMin.txt",findIntervalMin,Ql_strlen(findIntervalMin)); 
            sms_reply(sender, "Weekly FIND enabled");
        } else {  // sends sms to user on location based on time set
            int mins = Ql_atoi(cmd+5);
            if (mins >= 1 && mins <= 1440) {
                g_cfg->periodicFindMode = 1;
                g_cfg->findIntervalMin = mins;
                char periodicFindMode[1] = {0};
                char findIntervalMin[2] = {0};
                periodicFindMode[0] = g_cfg->periodicFindMode;
                findIntervalMin[0] = g_cfg->findIntervalMin >> 8;
                findIntervalMin[1] = g_cfg->findIntervalMin & 0xff;
                saveBytesToFlash("periodicFindMode.txt",periodicFindMode,Ql_strlen(periodicFindMode)); 
                saveBytesToFlash("findIntervalMin.txt",findIntervalMin,Ql_strlen(findIntervalMin)); 
                char msg[64];
                Ql_sprintf(msg, "FIND every %d min", mins);
                sms_reply(sender, msg);
            } else {
                sms_reply(sender, "Invalid FIND interval (1-1440)");
            }
        }
    } else {
        sms_reply(sender, "Invalid FIND format");
    }
}


/* === Handle All Trip Report SMS Command === */
static void handle_trip_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[64];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strcmp(cmd, "TRIP,0") == 0) {
        g_cfg->tripReporting = 0;
        char myTripReporting[1] = {0};
        myTripReporting[0] = g_cfg->tripReporting;
        saveBytesToFlash("tripReporting.txt",myTripReporting,Ql_strlen(myTripReporting); 
        sms_reply(sender, "Trip reporting OFF");

    } else if (Ql_strcmp(cmd, "TRIP,1") == 0) {
        g_cfg->tripReporting = 1;
        char myTripReporting[1] = {0};
        myTripReporting[0] = g_cfg->tripReporting;
        saveBytesToFlash("tripReporting.txt",myTripReporting,Ql_strlen(myTripReporting); 
        sms_reply(sender, "Trip reporting ON");

    } else if (Ql_strcmp(cmd, "TRIP,NOW,0") == 0) {
        stop_current_trip();
        sms_reply(sender, "Trip stopped");

    } else if (Ql_strcmp(cmd, "TRIP,NOW,1") == 0) {
        char tripSummary[160];
        get_trip_summary(tripSummary, sizeof(tripSummary));
        sms_reply(sender, tripSummary);

    } else {
        sms_reply(sender, "Invalid TRIP format");
    }
}

/* === Handle Alarm SMS Command === */
static void handle_alarm_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[32];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strcmp(cmd, "ALARM,0") == 0) {
        g_cfg->alarmReporting = 0;
        storage_save(&g_cfg);
        char myAlarmReporting[1] = {0};
        myAlarmReporting[0] = g_cfg->alarmReporting;
        saveBytesToFlash("alarmReporting.txt",myAlarmReporting,Ql_strlen(myAlarmReporting)); 
        sms_reply(sender, "Alarm reporting OFF");

    } else if (Ql_strcmp(cmd, "ALARM,1") == 0) {
        g_cfg->alarmReporting = 1;
        char myAlarmReporting[1] = {0};
        myAlarmReporting[0] = g_cfg->alarmReporting;
        saveBytesToFlash("alarmReporting.txt",myAlarmReporting,Ql_strlen(myAlarmReporting)); 
        sms_reply(sender, "Alarm reporting ON");

    } else {
        sms_reply(sender, "Invalid ALARM format");
    }
}

/* === Handle Listen SMS Command === */
static void handle_listen_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[16];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strcmp(cmd, "LISTEN") != 0) {
        sms_reply(sender, "Invalid LISTEN format");
        return;
    }

    char atCmd[64];
    Ql_sprintf(atCmd, "ATD%s;", sender); // ATD<number>; initiates voice call

    s32 ret = Ql_RIL_SendATCmd(atCmd, Ql_strlen(atCmd), NULL, NULL, 0);
    if (ret == RIL_AT_SUCCESS) {
        sms_reply(sender, "Calling you for live listen...");
    } else {
        sms_reply(sender, "Failed to start call");
    }
}



/* === Handle Speed Settings SMS Command === */
static void handle_speed_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[32];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strcmp(cmd, "SPEED,ON") == 0) {
        g_cfg->speedReporting = 1;
        char mySpeedReporting[1] = {0};
        mySpeedReporting[0] = g_cfg->speedReporting;
        saveBytesToFlash("speedReporting.txt",mySpeedReporting,Ql_strlen(mySpeedReporting)); 
        sms_reply(sender, "Overspeed reporting ON");

    } else if (Ql_strcmp(cmd, "SPEED,OFF") == 0) {
        g_cfg->speedReporting = 0;
        char mySpeedReporting[1] = {0};
        mySpeedReporting[0] = g_cfg->speedReporting;
        saveBytesToFlash("speedReporting.txt",mySpeedReporting,Ql_strlen(mySpeedReporting)); 
        sms_reply(sender, "Overspeed reporting OFF");

    } else if (Ql_strncmp(cmd, "SPEED,", 6) == 0) {
        int limit = Ql_atoi(cmd + 6);
        if (limit >= 30 && limit <= 200) {
            g_cfg->speedReporting = 1;
            char mySpeedReporting[1] = {0};
            mySpeedReporting[0] = g_cfg->speedReporting;
            saveBytesToFlash("speedReporting.txt",mySpeedReporting,Ql_strlen(mySpeedReporting));

            g_cfg->speedLimitKph = limit;
            char mySpeedLimitKph[2] = {0};
            mySpeedLimitKph[0] = g_cfg->speedReporting >> 0;
            mySpeedLimitKph[1] = g_cfg->speedReporting & 0xff;
            saveBytesToFlash("speedLimitKph.txt",mySpeedLimitKph,Ql_strlen(mySpeedLimitKph));

            char msg[64];
            Ql_sprintf(msg, "Speed limit set to %dkm/h", limit);
            sms_reply(sender, msg);
        } else {
            sms_reply(sender, "Invalid speed (30-200 km/h)");
        }

    } else {
        sms_reply(sender, "Invalid SPEED format");
    }
}



/* === Handle Main Power SMS Command === */
static void handle_mainpower_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[32];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strcmp(cmd, "MAINPOWER,ON") == 0 || Ql_strcmp(cmd, "MP,ON") == 0) {
        g_cfg->mainPowerAlarm = 1;
        char mainPowerAlarm[1] = {0};
        mainPowerAlarm[0] = g_cfg->mainPowerAlarm;
        saveBytesToFlash("mainPowerAlarm.txt",mainPowerAlarm,Ql_strlen(mainPowerAlarm));
        sms_reply(sender, "Main power alarm ON");

    } else if (Ql_strcmp(cmd, "MAINPOWER,OFF") == 0 || Ql_strcmp(cmd, "MP,OFF") == 0) {
        g_cfg->mainPowerAlarm = 0;
        char mainPowerAlarm[1] = {0};
        mainPowerAlarm[0] = g_cfg->mainPowerAlarm;
        saveBytesToFlash("mainPowerAlarm.txt",mainPowerAlarm,Ql_strlen(mainPowerAlarm));
        sms_reply(sender, "Main power alarm OFF");

    } else {
        sms_reply(sender, "Invalid MAINPOWER format");
    }
}



/* === Handle Disable SMS Command === */
static void handle_disable_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[32];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strcmp(cmd, "DISABLE,ON") == 0) {
        g_cfg->vehicleDisabled = 1;
        // Our actual GPIO control code to activate/deactivate comes here
        char vehicleDisabled[1] = {0};
        vehicleDisabled[0] = g_cfg->vehicleDisabled;
        saveBytesToFlash("vehicleDisabled.txt",vehicleDisabled,Ql_strlen(vehicleDisabled));
        sms_reply(sender, "Vehicle immobilized");

    } else if (Ql_strcmp(cmd, "DISABLE,OFF") == 0) {
        g_cfg->vehicleDisabled = 0;
        // Our actual GPIO control code to activate/deactivate comes here
        char vehicleDisabled[1] = {0};
        vehicleDisabled[0] = g_cfg->vehicleDisabled;
        saveBytesToFlash("vehicleDisabled.txt",vehicleDisabled,Ql_strlen(vehicleDisabled));
        sms_reply(sender, "Vehicle enabled");

    } else {
        sms_reply(sender, "Invalid DISABLE format");
    }
}




/* === Handle Ignition SMS Command === */
static void handle_ignition_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[32];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strcmp(cmd, "IGNITION,ON") == 0 || Ql_strcmp(cmd, "I,ON") == 0) {
        g_cfg->ignitionReporting = 1;
        char ignitionReporting[1] = {0};
        ignitionReporting[0] = g_cfg->ignitionReporting;
        saveBytesToFlash("ignitionReporting.txt",ignitionReporting,Ql_strlen(ignitionReporting));
        sms_reply(sender, "Ignition reporting ON");

    } else if (Ql_strcmp(cmd, "IGNITION,OFF") == 0 || Ql_strcmp(cmd, "I,OFF") == 0) {
        g_cfg->ignitionReporting = 0;
        char ignitionReporting[1] = {0};
        ignitionReporting[0] = g_cfg->ignitionReporting;
        saveBytesToFlash("ignitionReporting.txt",ignitionReporting,Ql_strlen(ignitionReporting));
        sms_reply(sender, "Ignition reporting OFF");

    } else {
        sms_reply(sender, "Invalid IGNITION format");
    }
}



/* === Handle Set Odo SMS Command === */
static void handle_set_odo_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[64];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strncmp(cmd, "SET,ODO,", 9) == 0) {
        u32 odoValue = Ql_atoi(cmd + 9);
        if (odoValue < 10000000) { // limit to prevent overflow (~10 million km)
            g_cfg->odometerKm = odoValue;
            char odometerKm[2] = {0};
            odometerKm[0] = g_cfg->odometerKm >> 8;
            odometerKm[1] = g_cfg->odometerKm & 0xff;
            saveBytesToFlash("odometerKm.txt",odometerKm,Ql_strlen(odometerKm));
            char msg[64];
            Ql_sprintf(msg, "Odometer set to %lu km", odoValue);
            sms_reply(sender, msg);
        } else {
            sms_reply(sender, "Invalid ODO value");
        }
    } else {
        sms_reply(sender, "Invalid SET,ODO format");
    }
}




/* === Handle Shock SMS Command === */
static void handle_shock_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[32];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strncmp(cmd, "SHOCK,", 6) == 0) {
        int level = Ql_atoi(cmd + 6);
        if (level >= 0 && level <= 5) {
            g_cfg->shockSensitivity = level;
            char shockSensitivity[1] = {0};
            shockSensitivity[0] = g_cfg->shockSensitivity;
            saveBytesToFlash("shockSensitivity.txt",shockSensitivity,Ql_strlen(shockSensitivity));

            char msg[64];
            if (level == 0) {
                Ql_sprintf(msg, "Shock sensor OFF");
            } else {
                Ql_sprintf(msg, "Shock sensitivity set to %d", level);
            }
            sms_reply(sender, msg);
        } else {
            sms_reply(sender, "Invalid SHOCK value (0-5)");
        }
    } else {
        sms_reply(sender, "Invalid SHOCK format");
    }
}



/* === Handle Logger SMS Command === */
static void handle_logger_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[32];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strcmp(cmd, "LOGGER,0") == 0) {
        // Clear stored logs
        storage_clear_logs();
        sms_reply(sender, "All logs cleared");
    } else {
        sms_reply(sender, "Invalid LOGGER format");
    }
}



/* === Handle Heart Beat SMS Command === */
static void handle_heartbeat_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[32];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    if (Ql_strcmp(cmd, "HEARTBEAT") == 0) {
        char msg[64];
        Ql_sprintf(msg, "Heartbeat interval: %u min", g_cfg->heartbeatMin);
        sms_reply(sender, msg);

    } else if (Ql_strncmp(cmd, "HEARTBEAT,", 11) == 0) {
        u32 hbValue = Ql_atoi(cmd + 11);
        if (hbValue >= 1 && hbValue <= 65535) {
            g_cfg->heartbeatMin = hbValue;
            char heartbeatMin[2] = {0};
            heartbeatMin[0] = g_cfg->heartbeatMin >> 8;
            heartbeatMin[1] = g_cfg->heartbeatMin & 0xff;
            saveBytesToFlash("heartbeatMin.txt",heartbeatMin,Ql_strlen(heartbeatMin));

            char msg[64];
            Ql_sprintf(msg, "Heartbeat set to %u min", hbValue);
            sms_reply(sender, msg);
        } else {
            sms_reply(sender, "Invalid heartbeat (1-65535)");
        }
    } else {
        sms_reply(sender, "Invalid HEARTBEAT format");
    }
}




/* === Handle WWW SMS Command === */
static void handle_www_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[8];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_strupr(cmd);

    /* === READ CURRENT SETTINGS === */
    if (Ql_stricmp(body, "WWW") == 0) {
        if (!is_authorized(sender) && g_cfg->allowPublic == 0) {
            sms_reply(sender, "NCFTrack: Access denied (number not registered)");
            return;
        }
        char msg[256];
        Ql_sprintf(msg,
            "IPN:%s;COM:%u;APN:%s,%s,%s;RPT:%u;SLP:%u;RUN:%u;",
            g_cfg->serverHost,
            g_cfg->serverPort,
            g_cfg->apn,
            g_cfg->apnUser,
            g_cfg->apnPass,
            g_cfg->rptSec,
            g_cfg->slpSec,
            g_cfg->runMode
        );
        sms_reply(sender, msg);
        return;
    }

    /* === SET NEW SETTINGS === */
    else if (Ql_strnicmp(body, "WWW:", 4) == 0) {
        if (!is_authorized(sender)) {
            sms_reply(sender, "NCFTrack: Access denied (number not registered)");
            return;
        }

        const char* p = body + 4;
        char host[64] = {0}, apn[64] = {0}, apnUser[32] = {0}, apnPass[32] = {0};
        int port = 0, rpt = 0, slp = 0, run = 0;

        if (Ql_sscanf(p,
            "IPN:%63[^;];COM:%d;APN:%63[^,],%31[^,],%31[^;];RPT:%d;SLP:%d;RUN:%d;",
            host, &port, apn, apnUser, apnPass, &rpt, &slp, &run
        ) < 8) {
            sms_reply(sender, "NCFTrack: Invalid WWW format");
            return;
        }

        if (Ql_strlen(host) == 0 || Ql_strlen(apn) == 0) {
            sms_reply(sender, "NCFTrack: Invalid settings (host/APN empty)");
            return;
        }

        /* === Save config === */
        Ql_strncpy(g_cfg->serverHost, host, sizeof(g_cfg->serverHost)-1);
        g_cfg->serverPort = (u16)port;
        Ql_strncpy(g_cfg->apn, apn, sizeof(g_cfg->apn)-1);
        Ql_strncpy(g_cfg->apnUser, apnUser, sizeof(g_cfg->apnUser)-1);
        Ql_strncpy(g_cfg->apnPass, apnPass, sizeof(g_cfg->apnPass)-1);
        g_cfg->rptSec = (u16)rpt;
        g_cfg->slpSec = (u16)slp;
        g_cfg->runMode = (u8)run;
        
        char* serverHost = Ql_MEM_Alloc(Ql_strlen(g_cfg->serverHost));
        Ql_strncpy(serverHost,g_cfg->serverHost,Ql_strlen(g_cfg->serverHost));
        saveBytesToFlash("serverHost.txt",serverHost,Ql_strlen(serverHost));

        char* apn = Ql_MEM_Alloc(Ql_strlen(g_cfg->apn));
        Ql_strncpy(apn,g_cfg->apn,Ql_strlen(g_cfg->apn));
        saveBytesToFlash("apn.txt",apn,Ql_strlen(apn));

        char* apnUser = Ql_MEM_Alloc(Ql_strlen(g_cfg->apnUser));
        Ql_strncpy(apnUser,g_cfg->apnUser,Ql_strlen(g_cfg->apnUser));
        saveBytesToFlash("apnUser.txt",apnUser,Ql_strlen(apnUser));

        char* apnPass = Ql_MEM_Alloc(Ql_strlen(g_cfg->apnPass));
        Ql_strncpy(apnPass,g_cfg->apnPass,Ql_strlen(g_cfg->apnPass));
        saveBytesToFlash("apnPass.txt",apnPass,Ql_strlen(apnPass));

        char serverPort[2] = {0};
        serverPort[0] = g_cfg->serverPort >> 8;
        serverPort[1] = g_cfg->serverPort & 0xff;
        saveBytesToFlash("serverPort.txt",serverPort,Ql_strlen(serverPort));

        char rptSec[2] = {0};
        rptSec[0] = g_cfg->rptSec >> 8;
        rptSec[1] = g_cfg->rptSec & 0xff;
        saveBytesToFlash("rptSec.txt",rptSec,Ql_strlen(rptSec));

        char slpSec[2] = {0};
        slpSec[0] = g_cfg->slpSec >> 8;
        slpSec[1] = g_cfg->slpSec & 0xff;
        saveBytesToFlash("slpSec.txt",slpSec,Ql_strlen(slpSec));

        char runMode[1] = {0};
        runMode[0] = g_cfg->runMode;
        saveBytesToFlash("runMode.txt",runMode,Ql_strlen(runMode));

        /* === Try to connect if RUN == 1 === */
        if (g_cfg->runMode == 1) {
            g_cfg->mqtt_state =  STATE_NW_QUERY_STATE; // set mqtt to restart
            sms_reply(sender, "NCFTrack: GPRS OK, MQTT auth/connecting");
        } else {
            sms_reply(sender, "NCFTrack: Settings saved, RUN=0 (not connecting)");
        }
    }
}

/* === Handle Device Firmware Remote Update SMS Command === */

void sms_pump(char* smsSender, char* smsContent, DeviceConfig *g_cfg) {
    /* === Check unread SMS (index 1..50 for example) === */
    APP_DEBUG("[SMS] From: %s\r\n", smsSender);
    APP_DEBUG("[SMS] Body: %s\r\n", smsContent);

    /* ==== If Not Authorized React And Go On ==== */
    if (!is_authorized(smsSender)) {
        sms_reply(smsSender, "Access denied (number not registered)");
        Ql_SMS_Delete(index, SMS_DEL_INDEXED_MSG, SIM0);
        continue;
    }

    /* === Convert to Uppercase For Case-Insensitive Compare === */
    char bodyUpper[160];
    Ql_strcpy(bodyUpper, smsContent);
    for (char* p = bodyUpper; *p; ++p) {
        if (*p >= 'a' && *p <= 'z') *p -= 32;
    }

    /* === Check if it starts with ADD, ETC. === */
    if (Ql_strncmp(bodyUpper, "ADD,", 4) == 0) {
        handle_add_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "ADDUSER,", 8) == 0 || Ql_strncmp(bodyUpper, "AU,", 3) == 0) {
        handle_adduser_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "NAME,", 5) == 0 || Ql_strncmp(bodyUpper, "N,UN", 4) == 0) {
        handle_name_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "SET,ALLOWPUBLIC,", 16) == 0 || Ql_strncmp(bodyUpper, "SET,ALLOWPUBLI C,", 17) == 0) {
        handle_allowpublic_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "PASSWORD,", 9) == 0) {
        handle_password_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "GETREPORT,", 10) == 0 || Ql_strncmp(bodyUpper, "GR,", 3) == 0) {
        handle_getreport_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "PASSWORD", 8) == 0) {
        if (Ql_strlen(bodyUpper) == 8) {
            handle_password_read_command(smsSender, smsContent, g_cfg);
        } else if (bodyUpper[8] == ',') {
            handle_password_command(smsSender, smsContent, g_cfg);
        } else {
            sms_reply(smsSender, "Unknown PASSWORD command format");
        }
    } else if (Ql_strncmp(bodyUpper, "LISTUSER", 8) == 0 || Ql_strncmp(bodyUpper, "LU", 2) == 0) {
        handle_listuser_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "DELUSER,", 8) == 0 || Ql_strncmp(bodyUpper, "DU,", 3) == 0) {
        handle_deluser_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "TIME,", 5) == 0) {
        handle_time_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "SMS,", 4) == 0) {
        handle_sms_command(smsSender, smsContent, g_cfg);
    } else if (Ql_stricmp(bodyUpper, "SETTINGS") == 0) {
        handle_settings_command(smsSender, smsContent, g_cfg);
    } else if (Ql_stricmp(bodyUpper, "STATUS") == 0 || Ql_stricmp(bodyUpper, "ST") == 0) {
        handle_status_command(smsSender, smsContent, g_cfg);
    } else if (Ql_stricmp(bodyUpper, "VERSION") == 0 || Ql_stricmp(bodyUpper, "V") == 0) {
        handle_version_command(smsSender, smsContent, g_cfg);
    } else if (Ql_stricmp(bodyUpper, "REBOOT") == 0 || Ql_stricmp(bodyUpper, "RB") == 0) {
        handle_reboot_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "SET,SLEEP,", 10) == 0) {
        handle_sleep_command(smsSender, smsContent, g_cfg);
    } else if (Ql_stricmp(bodyUpper, "HELP") == 0 || Ql_stricmp(bodyUpper, "?") == 0) {
        handle_help_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "FIND", 4) == 0) {
        handle_find_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "TRIP", 4) == 0) {
        handle_trip_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "ALARM", 5) == 0) {
        handle_alarm_command(smsSender, smsContent, g_cfg);
    } else if (Ql_stricmp(bodyUpper, "LISTEN") == 0) {
        handle_listen_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "SPEED", 5) == 0) {
        handle_speed_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "MAINPOWER", 10) == 0 || Ql_strncmp(bodyUpper, "MP", 2) == 0) {
        handle_mainpower_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "DISABLE", 7) == 0) {
        handle_disable_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "IGNITION", 8) == 0 || Ql_strncmp(bodyUpper, "I,", 2) == 0) {
        handle_ignition_command(smsSender, smsContent);
    } else if (Ql_strncmp(bodyUpper, "SET,ODO", 8) == 0) {
        handle_set_odo_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "SHOCK", 5) == 0) {
        handle_shock_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "LOGGER", 6) == 0) {
        handle_logger_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strncmp(bodyUpper, "HEARTBEAT", 9) == 0) {
        handle_heartbeat_command(smsSender, smsContent, g_cfg);
    } else if (Ql_strcmp(bodyUpper, "WWW") == 0) {
        handle_www_command(smsSender, smsContent, g_cfg);
    } else {
        sms_reply(smsSender, "Unknown command");
    }
}



char *my_strtok(char *str, const char *delim) {
    static char *saved = NULL;   // remembers position between calls
    if (str != NULL) {
        saved = str;             // initialize new string
    }
    if (saved == NULL) {
        return NULL;             // no more tokens
    }

    // Skip leading delimiters
    char *token_start = saved;
    while (*token_start && strchr(delim, *token_start)) {
        token_start++;
    }
    if (*token_start == '\0') {
        saved = NULL;            // reached end
        return NULL;
    }

    // Find end of token
    char *token_end = token_start;
    while (*token_end && !strchr(delim, *token_end)) {
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


