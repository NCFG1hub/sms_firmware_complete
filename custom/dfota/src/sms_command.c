#include "sms_command.h"
#include "ql_trace.h"
#include "ql_stdlib.h"
#include "ql_uart.h"
#include "ril_sms.h"
#include "data_types.h"
#include "ril.h"
#include "ril_network.h"
#include "ril_gnss.h"
#include "ql_type.h"
#include "gnss_parser.h"
#include "ql_fs.h";
#include "ql_error.h"
#include "ql_time.h"
#include "ql_gprs.h"




char *my_strtok(char *str, const char *delim);
void extract_key_value(const char *src, const char *key, char *dest, int size, char endDelim);
s32 saveBytesToFlash(char* fileName,char* data,u32 length);

char userPhone[20] = {0};

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
    u32 iResult = RIL_SMS_SendSMS_Text(destNumber, Ql_strlen(destNumber),LIB_SMS_CHARSET_GSM,(u8*)buf,Ql_strlen(buf),&uMsgRef);
    if (iResult != RIL_AT_SUCCESS)
    {
        APP_DEBUG("[SMS] Reply failed (%d) to %s\r\n", message, destNumber);
        return;
    }
    APP_DEBUG("[SMS] Reply sent to %s\r\n", destNumber);
}


static bool is_authorized(const char* number, DeviceConfig *g_cfg) {
    APP_DEBUG("code run here 1\r\n");
    bool myStatus = FALSE;
    
    if (g_cfg->allowPublic != 1) 
        return TRUE;

    APP_DEBUG("code run here 2\r\n");

    if (g_cfg->users == NULL || g_cfg->users[0] == '\0') {
        APP_DEBUG("No users configured\r\n");
        return FALSE;
    }

    char* splitedUsers = my_strtok(g_cfg->users,",");
    char* nextUser;
    APP_DEBUG("code run here 3\r\n");
    /* === Check If Already Registered === */
    for (u8 i = 0; i < g_cfg->userCount; i++) {
        nextUser = my_strtok(splitedUsers,",");
        APP_DEBUG("SEEN user %s\r\n",nextUser);
        if (Ql_strcmp(nextUser, number) == 0) {
            myStatus = TRUE;
            return TRUE;
        }
        splitedUsers = nextUser + Ql_strlen(nextUser) + 1;
    }
    APP_DEBUG("code run here 4\r\n");
    return myStatus;
}

/* === Handle ADD command (Change Device Password) === */
static void handle_add_command(const char* sender, const char* body, DeviceConfig *g_cfg) {

    char savedPassword[25];
    u32 bytesRead = 0;
    bytesRead = readFromFlashString("pwd.txt",savedPassword,250);
    savedPassword[bytesRead] = "\0";

    if(bytesRead <= 0){ // if no password was saved, assume that the password is all zero
         APP_DEBUG("there is no saved password ");
         Ql_strncpy(savedPassword,"000000\0",Ql_strlen("000000\0"));
    }

    char receivedOldPassword[25];
    char receivedNewPassword[25];

    Ql_memset(receivedOldPassword,0,sizeof(receivedOldPassword));
    Ql_memset(receivedNewPassword,0,sizeof(receivedNewPassword));
    
    char* rawSmsData = my_strtok(body,",");
    rawSmsData = rawSmsData + Ql_strlen(rawSmsData) + 1; // advance the pointer to read the old password
    char* readOldPassword = my_strtok(rawSmsData,",");
    Ql_strncpy(receivedOldPassword,readOldPassword,Ql_strlen(readOldPassword)); // save the old password in an array
    receivedOldPassword[Ql_strlen(readOldPassword)] = '\0';

    rawSmsData = readOldPassword + Ql_strlen(readOldPassword) + 1; // advance the pointer to read the new password
    char* readNewPassword = my_strtok(rawSmsData,",");
    Ql_strncpy(receivedNewPassword,readNewPassword,Ql_strlen(readNewPassword)); // save the new password in an array
    receivedNewPassword[Ql_strlen(readNewPassword)] = '\0';

    APP_DEBUG("saved password = %s,  old password = %s, new password = %s\r\n",savedPassword,receivedOldPassword,receivedNewPassword);




    /* === Check it's Alphanumeric Only === */
    for (u32 i = 0; i < Ql_strlen(receivedNewPassword); i++) {
        if (!( (receivedNewPassword[i] >= '0' && receivedNewPassword[i] <= '9') ||
               (receivedNewPassword[i] >= 'A' && receivedNewPassword[i] <= 'Z') ||
               (receivedNewPassword[i] >= 'a' && receivedNewPassword[i] <= 'z') )) {
            sms_reply(sender, "Failed (password must be letters/numbers only)");
            return;
        }
    }

    if(Ql_strcmp(receivedOldPassword,savedPassword) == 0){
            if (saveBytesToFlash("pwd.txt",receivedNewPassword,Ql_strlen(receivedNewPassword)) == 0) {
                Ql_strncpy(g_cfg->password,receivedNewPassword,Ql_strlen(receivedNewPassword));
                char successMsg[100];
                Ql_sprintf(successMsg, "Password changed to %s", receivedNewPassword);
                sms_reply(sender, successMsg);
            } else {
                sms_reply(sender, "Failed to save password");
            }
    }
    else{
        sms_reply(sender, "Invalid password");
    }
}



/* === Handle Add User SMS Command === */
static void handle_adduser_command(const char* sender, const char* body, DeviceConfig *g_cfg) {

    char users[500];
    u32 bytesRead = 0;
    Ql_memset(g_cfg->users,0,sizeof(g_cfg->users));
    bytesRead = readFromFlashString("user.txt",g_cfg->users,200);

    APP_DEBUG("saved users = %s,  total users = %d\r\n",g_cfg->users,g_cfg->userCount);

    char* rawUserData = my_strtok(body,","); 
    rawUserData = rawUserData + Ql_strlen(rawUserData) + 1;

    char* myUserToAdd = my_strtok(rawUserData,","); 

    char formatedUser[25];
    int j = 0; // index for the user
    for(int i=0;i<Ql_strlen(myUserToAdd);++i){ // remove space from the number
        if(myUserToAdd[i] != ' '){
            formatedUser[j] = myUserToAdd[i];
            j = j + 1;
        }
    }
    formatedUser[j] = '\0'; // add end of character null


    u32 len = Ql_strlen(formatedUser);
    if (len < 7 || len > 20) {
        sms_reply(sender, "Failed (invalid phone number length)");
        return;
    }

    /* === Check Digits And '+' Sign At The Start === */
    if (formatedUser[0] != '+' && (formatedUser[0] < '0' || formatedUser[0] > '9')) {
        sms_reply(sender, "Failed (invalid phone number format)");
        return;
    }
    for (u32 i = 1; i < len; i++) {
        if (formatedUser[i] < '0' || formatedUser[i] > '9') {
            sms_reply(sender, "Failed (phone number must be digits only)");
            return;
        }
    }



    char* splitedUsers = my_strtok(g_cfg->users,",");

    /* === Check If Already Registered === */
    for (u8 i = 0; i < g_cfg->userCount; i++) {
        char* nextUser = my_strtok(splitedUsers,",");
        if (Ql_strcmp(nextUser, formatedUser) == 0) {
            sms_reply(sender, "Failed (number already exists)");
            return;
        }
        splitedUsers = nextUser + Ql_strlen(nextUser) + 1;
    }

    /* === Check User Limit === */
    if (g_cfg->userCount >= MAX_USERS) {
        sms_reply(sender, "Failed (user list full)");
        return;
    }

    /* === Add The User If All Is Fine === */
    g_cfg->userCount++;

    char* allNewUser = Ql_MEM_Alloc(Ql_strlen(formatedUser) + Ql_strlen(formatedUser) + 1);
    Ql_strcat(allNewUser,g_cfg->users);
    Ql_strcat(allNewUser,",");
    Ql_strcat(allNewUser,formatedUser);
    
    saveBytesToFlash("user.txt",allNewUser,Ql_strlen(allNewUser));
    char msg[80];
    Ql_sprintf(msg, "User added %s", formatedUser);
    saveBytesToFlash("userCount.txt",g_cfg->userCount,2); // update the total number of user saved
    sms_reply(sender, msg);
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

   if (saveBytesToFlash("name.txt",namePtr,Ql_strlen(namePtr)) == 0) {
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
    
    char* rawTextData = my_strtok(body,","); 
    rawTextData = rawTextData + Ql_strlen(rawTextData) + 1; // move to the next data
    char* allowPublicData = my_strtok(rawTextData,",");

    if (allowPublicData[0] != '0' && allowPublicData[0] != '1') {
        sms_reply(sender, "Failed (value must be 0 or 1)");
        return;
    }

    g_cfg->allowPublic = Ql_atoi(allowPublicData);

    char allowPublic[1];
    allowPublic[0] = g_cfg->allowPublic;

    APP_DEBUG("Allow public received = %d",allowPublic[0]);

    if (saveBytesToFlash("allowPublic.txt",allowPublic,1) == 0) {
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

    if (saveBytesToFlash("pwd.txt",newPass,Ql_strlen(newPass)) == 0) {
        char msg[80];
        Ql_sprintf(msg, "Password changed to %s", newPass);
        Ql_strcpy(g_cfg->password,newPass);
        sms_reply(sender, msg);
    } else {
        sms_reply(sender, "Failed to save password");
    }
}

/* === Handle Get Report SMS Command === */  //############## need clarification

/*assigning users with permission to receive report or not to be done on upgrade*/
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
    

    if (saveBytesToFlash("reportPower.txt",reportPower,Ql_strlen(reportPower)) == 0) {
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
    if (Ql_strcmp(body, "PASSWORD") != 0) {
        sms_reply(sender, "Failed (invalid format)");
        return;
    }

    char msg[60];
    Ql_sprintf(msg, "Password is %s \r\n", g_cfg->password);
    sms_reply(sender, msg);
}

/* === Handle All List User SMS Command === */  //######## need clarification here
static void handle_listuser_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char arg[20] = {0};
    int userLevel = 0;
    if(!is_authorized(sender,g_cfg)){
        sms_reply(sender, "Not authorized");
        return;
    }

    char* rawData = my_strtok(body,",");
    rawData = rawData + Ql_strlen(rawData) + 1;
    char* listType = my_strtok(rawData,",");

    if(!listType){
        sms_reply(sender, "Invalid command");
        return;
    }

    APP_DEBUG(" user list type %s \r\n",listType);

    char userlistMessage[300];

    if(Ql_strcmp(listType,"ALL") == 0 || Ql_strcmp(listType,"All") == 0 || Ql_strcmp(listType,"all") == 0){
            
            Ql_strcat(userlistMessage,"list user User1: admin");

            char* splitedUsers = my_strtok(g_cfg->users,",");

            /* === Check If Already Registered === */
            for (u8 i = 0; i < g_cfg->userCount; i++) {
                char* nextUser = my_strtok(splitedUsers,",");
                char myString[50];
                Ql_sprintf(myString,",User%d:%s",(i+2),nextUser); 
                Ql_strcat(userlistMessage,myString);
                splitedUsers = nextUser + Ql_strlen(nextUser) + 1;
            }

            sms_reply(sender, userlistMessage);
    }
    else{
        if(listType[0] >= '0' || listType[0] <= '9'){
             userLevel = Ql_atoi(listType);
             if(userLevel == 1){
                  Ql_strcat(userlistMessage,"list user User1: admin");
             }

             char* splitedUsers = my_strtok(g_cfg->users,",");

            /* === Check If Already Registered === */
            for (u8 i = 0; i < g_cfg->userCount; i++) {
                if(i == (userLevel - 1)){
                    char* nextUser = my_strtok(splitedUsers,",");
                    char myString[50];
                    Ql_sprintf(myString,",User%d:%s",(i+2),nextUser); 
                    Ql_strcat(userlistMessage,myString);
                    splitedUsers = nextUser + Ql_strlen(nextUser) + 1;
                }
            }

            sms_reply(sender, userlistMessage);
        }
        else{
            sms_reply(sender, "Invalid command");
            return;
        }
    }


    

}



/* === Handle Delete User SMS Command === */ //# need clarification
static void handle_deluser_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    int slot = -1;
    int isPhoneOrLevel = 0;
    
    char* rawData = my_strtok(body,",");
    rawData = rawData + Ql_strlen(rawData) + 1; // skip the head command

    char* rawDeleteDirective = my_strtok(rawData,",");
    if(!rawDeleteDirective){
        sms_reply(sender, "Failed (invalid user slot)");
        return;
    }

    if(Ql_strlen(rawDeleteDirective) >= 4){
          isPhoneOrLevel = 1;
    }
    else{
        isPhoneOrLevel =  0;
        slot = Ql_atoi(rawDeleteDirective);
    }


    char remainingUser[300];

    if(isPhoneOrLevel == 0){
        if (slot < 1 || slot > MAX_USERS) {
            sms_reply(sender, "Failed (invalid user slot)");
            return;
        }

        char* splitedUsers = my_strtok(g_cfg->users,",");

        /* === Check If Already Registered === */
        for (u8 i = 0; i < g_cfg->userCount; i++) {
            char* nextUser = my_strtok(splitedUsers,",");
            if((i+1) != slot){
                Ql_strcat(remainingUser,nextUser);
                Ql_strcat(remainingUser,",");
            }
            splitedUsers = nextUser + Ql_strlen(nextUser) + 1;
        }
    }
    else if(isPhoneOrLevel == 1){
        char* splitedUsers = my_strtok(g_cfg->users,",");

        /* === Check If Already Registered === */
        for (u8 i = 0; i < g_cfg->userCount; i++) {
            char* nextUser = my_strtok(splitedUsers,",");
            if( Ql_strcmp(rawDeleteDirective,nextUser) == 0 ){
                Ql_strcat(remainingUser,nextUser);
                Ql_strcat(remainingUser,",");
            }
            splitedUsers = nextUser + Ql_strlen(nextUser) + 1;
        }
    }


    


    if (saveBytesToFlash("user.txt",remainingUser,Ql_strlen(remainingUser) == 0)) {
        char msg[40];
        Ql_sprintf(msg, "User %d deleted", slot);
        sms_reply(sender, msg);
    } else {
        sms_reply(sender, "Failed to delete user");
    }
}

/* === Handle Time Stamp SMS Command === */ 
static void handle_time_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    ST_Time time;

    char* rawData = my_strtok(body,",");
    rawData = rawData + Ql_strlen(rawData) + 1;
    char* mainData = my_strtok(rawData,",");
    char numBuffer[3];
    int bufIndex = 0;
    for(int i = 0; i < Ql_strlen(mainData); ++i){
        if(mainData[i] >= '0' || mainData[i] >= '9' || mainData[i] == '+' || mainData[i] >= '-'){
            numBuffer[bufIndex] = mainData[i];
            bufIndex = bufIndex + 1;
        }
    }

    int myZone = Ql_atoi(numBuffer);
    time.timezone = myZone;
    Ql_SetLocalTime(&time);



    if (myZone < -12 || myZone > 12) {
        sms_reply(sender, "Failed (offset must be between -12 and +12)");
        return;
    }

    g_cfg->timezoneOffset = myZone;
    char myTimeZoneOffset[2] = {(myZone >> 8),(myZone & 0xff)}; // convert the timezone to bytes and save
    if (saveBytesToFlash("timezoneOffset.txt",myTimeZoneOffset,Ql_strlen(myTimeZoneOffset)) == 0) {
        char msg[50];
        Ql_sprintf(msg, "Timezone offset set to %d", myZone);
        sms_reply(sender, msg);
    } else {
        sms_reply(sender, "Failed to save timezone myZone");
    }
}



static void handle_upgrade_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    
     APP_DEBUG("\r\n<-- running remote upgrade on link %s -->\r\n",g_cfg->upgradeLink);

     ST_GprsConfig apnCfg;
     Ql_memcpy(apnCfg.apnName,	g_cfg->apn, Ql_strlen(g_cfg->apn));
     Ql_memcpy(apnCfg.apnUserId, g_cfg->apnUser, Ql_strlen(g_cfg->apnUser));
     Ql_memcpy(apnCfg.apnPasswd, g_cfg->apnPass, Ql_strlen(g_cfg->apnPass));

     Ql_memset(userPhone,0,sizeof(userPhone));
     Ql_strncpy(userPhone,sender,Ql_strlen(sender));
     userPhone[Ql_strlen(sender)] = '\0';
     g_cfg->isRunUpgrade = 1;
     APP_DEBUG("upgrade started %s \r\n",userPhone);
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
    char msg[160];
    Ql_sprintf(msg,
        "PWD:%s TZ:%d\r\n"
        "SRV:%s:%d\r\n"
        "APN:%s,%s,%s\r\n"
        "RPT:%d SLP:%d\r\n"
        "Public:%d",
        g_cfg->password,
        g_cfg->timezoneOffset,
        g_cfg->serverHost,
        g_cfg->serverPort,
        g_cfg->apn,
        g_cfg->apnUser,
        g_cfg->apnPass,
        g_cfg->rptSec,
        g_cfg->slpSec,
        g_cfg->allowPublic
    );

    sms_reply(sender, msg);
}



/* === Handle Gets The Device Status SMS Command === */ //# need clarification
static void handle_status_command(const char* sender, const char* body, DeviceConfig *g_cfg) {

    int gsmSignal = get_gsm_signal_strength(g_cfg);   // 0-31
    int gpsFix    = 0; //get_gps_fix_status(g_cfg);        // 0 or 1
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

    //char imei[20] = {0};
    //get_device_imei(imei, sizeof(imei)); // Reads IMEI from GSM module

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
    sms_reply(sender, "Will reboot after 2 seconds.");

    // Small delay to allow SMS to send before reboot
    Ql_Sleep(2000);

    // Reboot the MC65
    Ql_Reset(0);
}




/* === Handle Set Sleep Mode SMS Command === */
static void handle_sleep_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    int mode = -1;
    char* rawData = my_strtok(body,",");
    rawData = rawData + Ql_strlen(rawData) + 1; // skip first comma

    char* rawData2 =  my_strtok(rawData,",");
    rawData2 = rawData2 + Ql_strlen(rawData2) + 1; // skip second comma
    char* rawMode = my_strtok(rawData2,",");
    if(rawMode){
        mode = Ql_atoi(rawMode);
    }

    
    if (mode != 0 && mode != 1) {
        sms_reply(sender, "Failed (X must be 0 or 1)");
        return;
    }

    g_cfg->sleepMode = mode;
    char mySleepMode[1] = {0};
    mySleepMode[0] = mode;
    saveBytesToFlash("sleepMode.txt",mySleepMode,Ql_strlen(mySleepMode));   // Save to flash so it persists after reboot

    if (mode == 1) {
       // enter_low_power_mode();
        Ql_SleepEnable();
        sms_reply(sender, "Sleep mode enabled");
    } else {
        //exit_low_power_mode();
        Ql_SleepDisable();
        sms_reply(sender, "Sleep mode disabled");
    }
}




/* === Handle Help SMS Command === */
static void handle_help_command(const char* sender, const char* body, DeviceConfig *g_cfg) {

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
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);
    GNSS_Info myGpsInfo;

    if (Ql_strcmp(cmd, "FIND") == 0) {  // just get the gps location and send sms
        // Immediate location
        //parseGPRMC(g_cfg->gpsData,&myGpsInfo);
        int mileage = g_cfg->odometerKm;
        const char* addr = " ";

        char msg[160];
        Ql_sprintf(msg, "Lat:%.6f Lon:%.6f\r\nAddr:%s\r\nMileage:%dkm",
                   g_cfg->latitude, g_cfg->longitude, addr, mileage);
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
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

    if (Ql_strcmp(cmd, "TRIP,0") == 0) {
        g_cfg->tripReporting = 0;
        char myTripReporting[1] = {0};
        myTripReporting[0] = g_cfg->tripReporting;
        saveBytesToFlash("tripReporting.txt",myTripReporting,Ql_strlen(myTripReporting)); 
        sms_reply(sender, "Trip reporting OFF");

    } else if (Ql_strcmp(cmd, "TRIP,1") == 0) {
        g_cfg->tripReporting = 1;
        char myTripReporting[1] = {0};
        myTripReporting[0] = g_cfg->tripReporting;
        saveBytesToFlash("tripReporting.txt",myTripReporting,Ql_strlen(myTripReporting)); 
        sms_reply(sender, "Trip reporting ON");

    } else if (Ql_strcmp(cmd, "TRIP,NOW,0") == 0) {
        stop_current_trip();
        char tripData[1] = {0};
        tripData[0] = 0;
        saveBytesToFlash("trip.txt",tripData,Ql_strlen(tripData)); 
       // Ql_GPIO_SetLevel(PINNAME_GPIO_0, PINLEVEL_HIGH);   // set GPIO1 = HIGH
        sms_reply(sender, "Trip stopped");

    } else if (Ql_strcmp(cmd, "TRIP,NOW,1") == 0) {
        char tripSummary[160];
        char tripData[1] = {0};
        tripData[0] = 1;
        saveBytesToFlash("trip.txt",tripData,Ql_strlen(tripData)); 
       // Ql_GPIO_SetLevel(PINNAME_GPIO_0, PINLEVEL_LOW);   // set GPIO1 = HIGH
        get_trip_summary(tripSummary, sizeof(tripSummary));
        sms_reply(sender, tripSummary);

    } else {
        sms_reply(sender, "Invalid TRIP format");
    }
}

/* === Handle Alarm SMS Command === */
static void handle_alarm_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

    if (Ql_strcmp(cmd, "ALARM,0") == 0) {
        g_cfg->alarmReporting = 0;
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
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

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
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

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
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

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
    char* rawCommand = my_strtok(body,",");
    rawCommand = rawCommand + Ql_strlen(rawCommand) + 1; // skip the commad head
   
    char* cmd = my_strtok(rawCommand,",");

    if (Ql_strcmp(cmd, "ON") == 0 || Ql_strcmp(cmd, "on") == 0) {
        g_cfg->vehicleDisabled = 1;
        // Our actual GPIO control code to activate/deactivate comes here
        char vehicleDisabled[1] = {0};
        vehicleDisabled[0] = g_cfg->vehicleDisabled;
        saveBytesToFlash("vehicleDisabled.txt",vehicleDisabled,Ql_strlen(vehicleDisabled));
        Ql_GPIO_SetLevel(PINNAME_GPIO_0, PINLEVEL_HIGH); //turn onn the relay
        sms_reply(sender, "Vehicle mobilized");

    } else if (Ql_strcmp(cmd, "OFF") == 0 || Ql_strcmp(cmd, "off") == 0) {
        g_cfg->vehicleDisabled = 0;
        // Our actual GPIO control code to activate/deactivate comes here
        char vehicleDisabled[1] = {0};
        vehicleDisabled[0] = g_cfg->vehicleDisabled;
        saveBytesToFlash("vehicleDisabled.txt",vehicleDisabled,Ql_strlen(vehicleDisabled));
        Ql_GPIO_SetLevel(PINNAME_GPIO_0, PINLEVEL_LOW); // turn off the relay
        sms_reply(sender, "Vehicle immobilized");

    } else {
        sms_reply(sender, "Invalid DISABLE format");
    }
}




/* === Handle Ignition SMS Command === */
static void handle_ignition_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

    if (!is_authorized(sender, g_cfg) && g_cfg->allowPublic == 0) {
        sms_reply(sender, "Access denied (number not registered)");
        return;
    }

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
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

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
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

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
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

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
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

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




static void handle_setImei(const char* sender, const char* body, DeviceConfig *g_cfg) {
    APP_DEBUG("seen command inside setimei function = %s\r\n", body);
    char cmd[250] = {0};
    char recvImei[200] ={0};
    char myPassword[200] = {0};
    Ql_strncpy(cmd, body, Ql_strlen(body));
    cmd[Ql_strlen(body)] = '\0';
    char* rawDataFirst =  my_strtok(cmd,",");
    if(rawDataFirst  == NULL){
         sms_reply(sender, "invalid format");
         return;
    }

    rawDataFirst = rawDataFirst + Ql_strlen(rawDataFirst) + 1;
    char* myImei = my_strtok(rawDataFirst,",");
    Ql_strncpy(recvImei,myImei,Ql_strlen(myImei));
    recvImei[Ql_strlen(myImei)] = '\0';
    if(myImei  == NULL){
         sms_reply(sender, "invalid format");
         return;
    }

    rawDataFirst = myImei + Ql_strlen(myImei) + 1;
    char* factoryPassword = my_strtok(rawDataFirst,";");
    Ql_strncpy(myPassword,factoryPassword,Ql_strlen(factoryPassword));
    myPassword[Ql_strlen(factoryPassword)] = '\0';
    if(factoryPassword  == NULL){
         sms_reply(sender, "invalid format");
         return;
    }
    APP_DEBUG("seen password is %s\r\n", myPassword);
    APP_DEBUG("seen imei is %s\r\n", recvImei);
    
    if(Ql_strcmp(myPassword,"1253633738484") == 0){
        saveBytesToFlash("imei.txt",recvImei,Ql_strlen(recvImei));
        Ql_strncpy(g_cfg->imei,recvImei,Ql_strlen(recvImei)); 
        g_cfg->imei[Ql_strlen(recvImei)] = '\0';
        char msg[200];
        Ql_sprintf(msg, "imei successfully set to %s", recvImei);
        sms_reply(sender, msg);
    }
    else{
        sms_reply(sender, "Not authorized");
    }
    
}



static void handle_setIp(const char* sender, const char* body, DeviceConfig *g_cfg) {
    APP_DEBUG("seen command inside set ip function = %s\r\n", body);
    char cmd[250] = {0};
    char recvIp[200] ={0};
    char myPassword[200] = {0};
    Ql_strncpy(cmd, body, Ql_strlen(body));
    cmd[Ql_strlen(body)] = '\0';
    char* rawDataFirst =  my_strtok(cmd,",");
    if(rawDataFirst  == NULL){
         sms_reply(sender, "invalid format");
         return;
    }

    rawDataFirst = rawDataFirst + Ql_strlen(rawDataFirst) + 1;
    char* myIp = my_strtok(rawDataFirst,",");
    Ql_strncpy(recvIp,myIp,Ql_strlen(myIp));
    recvIp[Ql_strlen(myIp)] = '\0';
    if(myIp  == NULL){
         sms_reply(sender, "invalid format");
         return;
    }

    rawDataFirst = myIp + Ql_strlen(myIp) + 1;
    char* factoryPassword = my_strtok(rawDataFirst,";");
    Ql_strncpy(myPassword,factoryPassword,Ql_strlen(factoryPassword));
    myPassword[Ql_strlen(factoryPassword)] = '\0';
    if(factoryPassword  == NULL){
         sms_reply(sender, "invalid format");
         return;
    }
    APP_DEBUG("seen password is %s\r\n", myPassword);
    APP_DEBUG("seen ip is %s\r\n", recvIp);
    
    if(Ql_strcmp(myPassword,"1253633738484") == 0){
        saveBytesToFlash("ip.txt",recvIp,Ql_strlen(recvIp));
        Ql_memset(g_cfg->ipAddress,0,sizeof(g_cfg->ipAddress));
        Ql_strncpy(g_cfg->ipAddress,recvIp,Ql_strlen(recvIp)); 
        g_cfg->ipAddress[Ql_strlen(recvIp)] = '\0';

        char updateLink[100];
        Ql_sprintf(updateLink,"http://%s:%d/firmware/firmware.bin\0",g_cfg->ipAddress,5000);
        Ql_strncpy(g_cfg->upgradeLink,updateLink,Ql_strlen(updateLink));
        char msg[200];
        Ql_sprintf(msg, "ip successfully set to %s", recvIp);
        sms_reply(sender, msg);
    }
    else{
        sms_reply(sender, "Not authorized");
    }
    
}




/* === Handle WWW SMS Command === */
static void handle_www_command(const char* sender, const char* body, DeviceConfig *g_cfg) {
    APP_DEBUG("my body is ##### %s\r\n", body);
    char cmd[200];
    Ql_strncpy(cmd, body, sizeof(cmd)-1);
    Ql_toupper(cmd);

    APP_DEBUG("ny command is ##### %s\r\n", cmd);
    APP_DEBUG("ny body is ##### %s\r\n", body);

    /* === READ CURRENT SETTINGS === */
    if (Ql_strcmp(body, "WWW") == 0) {
        if (!is_authorized(sender, g_cfg) && g_cfg->allowPublic == 0) {
            sms_reply(sender, "Access denied (number not registered)");
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
    else if (Ql_strncmp(body, "WWW:", 4) == 0) {
        if (!is_authorized(sender, g_cfg)) {
            sms_reply(sender, "Access denied (number not registered)");
            return;
        }

        //const char* p = body + 4;
        char host[64], apn[64], apnUser[32], apnPass[32];
        char temp[128];
        int port = 0, rpt = 0, slp = 0, run = 0;

        
        
        // Extract host
        extract_key_value(body, "IPN:", host, sizeof(host), ';');
        host[sizeof(host)] = '\0';
        

        // Extract port
        extract_key_value(body, "COM:", temp, sizeof(temp), ';');
        port = Ql_atoi(temp);

        // Extract APN block: "web.gp.mtnnigeria.net,web,web"
        extract_key_value(body, "APN:", temp, sizeof(temp), ';');
        char* apnPart = my_strtok(temp,",");
        Ql_strcpy(apn,apnPart);
        apn[Ql_strlen(apnPart)] = '\0';
        apnPart = apnPart + Ql_strlen(apnPart) + 1;
        char* apnUserPart = my_strtok(apnPart,",");
        Ql_strcpy(apnUser,apnUserPart);
        apnUser[Ql_strlen(apnUserPart)] = '\0';
        apnUserPart = apnUserPart + Ql_strlen(apnUserPart) + 1;
        char* appPassPart =  my_strtok(apnUserPart,",");
        Ql_strcpy(apnPass,appPassPart);

        // Extract rpt, slp, run
        extract_key_value(body, "RPT:", temp, sizeof(temp), ';');
        rpt = Ql_atoi(temp);

        extract_key_value(body, "SLP:", temp, sizeof(temp), ';');
        slp = Ql_atoi(temp);

        extract_key_value(body, "RUN:", temp, sizeof(temp), ';');
        run = Ql_atoi(temp);

        APP_DEBUG("my host is %s\r\n", host);
        APP_DEBUG("my apn is %s\r\n", apn);

        if (Ql_strlen(host) == 0 || Ql_strlen(apn) == 0) {
            sms_reply(sender, "Invalid settings (host/APN empty)");
            return;
        }

        

        /* === Save config === */
        Ql_strncpy(g_cfg->serverHost, host, sizeof(g_cfg->serverHost)-1);
        g_cfg->serverHost[Ql_strlen(host)] = '\0';

        g_cfg->serverPort = (u16)port;

        Ql_strncpy(g_cfg->apn, apn, sizeof(g_cfg->apn)-1);
        g_cfg->apn[Ql_strlen(apn)] = '\0';

        Ql_strncpy(g_cfg->apnUser, apnUser, sizeof(g_cfg->apnUser)-1);
        g_cfg->apnUser[Ql_strlen(apnUser)] = '\0';

        Ql_strncpy(g_cfg->apnPass, apnPass, sizeof(g_cfg->apnPass)-1);
        g_cfg->apnPass[Ql_strlen(apnPass)] = '\0';

        g_cfg->rptSec = (u16)rpt;
        g_cfg->slpSec = (u16)slp;
        g_cfg->runMode = (u8)run;

        
        char* serverHost = Ql_MEM_Alloc(Ql_strlen(g_cfg->serverHost));
        Ql_strncpy(serverHost,g_cfg->serverHost,Ql_strlen(g_cfg->serverHost));

        APP_DEBUG("i worked up till here 1 %s\r\n", g_cfg->serverHost);
        saveBytesToFlash("serverHost.txt",g_cfg->serverHost,Ql_strlen(g_cfg->serverHost));

        char* myApn = Ql_MEM_Alloc(Ql_strlen(g_cfg->apn));
        Ql_strncpy(myApn,g_cfg->apn,Ql_strlen(g_cfg->apn));
        APP_DEBUG("i worked up till here 2 %s\r\n", g_cfg->apn);
        saveBytesToFlash("apn.txt",g_cfg->apn,Ql_strlen(g_cfg->apn));

        char* myApnUser = Ql_MEM_Alloc(Ql_strlen(g_cfg->apnUser));
        Ql_strncpy(myApnUser,g_cfg->apnUser,Ql_strlen(g_cfg->apnUser));
        saveBytesToFlash("apnUser.txt",g_cfg->apnUser,Ql_strlen(g_cfg->apnUser));

        char* myApnPass = Ql_MEM_Alloc(Ql_strlen(g_cfg->apnPass));
        Ql_strncpy(myApnPass,g_cfg->apnPass,Ql_strlen(g_cfg->apnPass));
        saveBytesToFlash("apnPass.txt",g_cfg->apnPass,Ql_strlen(g_cfg->apnPass));

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
        
        char myMessage[200] = {0};
        /* === Try to connect if RUN == 1 === */
        if (g_cfg->runMode == 1) {
            g_cfg->mqtt_state =  STATE_NW_QUERY_STATE; // set mqtt to restart
            Ql_sprintf(myMessage,"GPRS OK, MQTT auth/connecting \n imei = %s \n", g_cfg->imei);
            sms_reply(sender, myMessage);
            Ql_Sleep(5000);
            Ql_Reset(0);
        } else {
            
            Ql_sprintf(myMessage,"Settings saved, RUN=0 (not connecting) \n imei = %s \n", g_cfg->imei);
            sms_reply(sender,myMessage);

        }
    }
}

/* === Handle Device Firmware Remote Update SMS Command === */

void sms_pump(char* smsSender, char* smsContent, DeviceConfig *g_cfg) {
    /* === Check unread SMS (index 1..50 for example) === */
    APP_DEBUG("[SMS] From: %s\r\n", smsSender);
    APP_DEBUG("[SMS] Body: %s\r\n", smsContent);

    /* ==== If Not Authorized React And Go On ==== */
    
    if (!is_authorized(smsSender, g_cfg)) {
        sms_reply(smsSender, "Access denied (number not registered)");
        //Ql_SMS_Delete(1, SMS_DEL_INDEXED_MSG, SIM0);
        return;
    }

    /* === Convert to Uppercase For Case-Insensitive Compare === */
    char bodyUpper[1024];
    Ql_memset(bodyUpper,0,sizeof(bodyUpper));
    Ql_strncpy(bodyUpper, smsContent,Ql_strlen(smsContent));
    /*for (char* p = bodyUpper; *p; ++p) {
        if (*p >= 'a' && *p <= 'z') *p -= 32;
    }*/

    APP_DEBUG("command %s\r\n", bodyUpper);

    char* commandHead = my_strtok(bodyUpper,",");

    APP_DEBUG("seen command head %s\r\n", commandHead);

    /* === Check if it starts with ADD, ETC. === */
    if (Ql_strcmp(commandHead, "ADD") == 0) {
        handle_add_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "ADDUSER") == 0 || Ql_strcmp(commandHead, "AU") == 0) {
        handle_adduser_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "NAME") == 0 || Ql_strcmp(commandHead, "N") == 0) {
        handle_name_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "SET") == 0 || Ql_strcmp(commandHead, "Set") == 0 || Ql_strcmp(commandHead, "set") == 0) {
        commandHead =  commandHead + Ql_strlen(commandHead) + 1;
        char* nextCommand = my_strtok(commandHead,",");
        if(Ql_strcmp(nextCommand,"ALLOWPUBLIC") == 0 || Ql_strcmp(nextCommand,"AllowPublic") == 0 || Ql_strcmp(nextCommand,"allowpublic") == 0){
            handle_allowpublic_command(smsSender, smsContent, g_cfg);
        }
        else if(Ql_strcmp(nextCommand,"SLEEP") == 0 || Ql_strcmp(nextCommand,"Sleep") == 0 || Ql_strcmp(nextCommand,"sleep") == 0){
           handle_sleep_command(smsSender, smsContent, g_cfg);
        }
        else if (Ql_strcmp(nextCommand, "ODO") == 0 || Ql_strcmp(nextCommand, "Odo") == 0 || Ql_strcmp(nextCommand, "odo") == 0) {
           handle_set_odo_command(smsSender, smsContent, g_cfg);
        } 
    } 
    else if (Ql_strcmp(commandHead, "PASSWORD") == 0) {
        handle_password_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "GETREPORT") == 0 || Ql_strcmp(commandHead, "GR") == 0) {
        handle_getreport_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "LISTUSER") == 0 || Ql_strcmp(commandHead, "LU") == 0 || Ql_strcmp(commandHead, "ListUser") == 0 ) {
        handle_listuser_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "DELETEUSER") == 0 || Ql_strcmp(commandHead, "DU") == 0 || Ql_strcmp(commandHead, "DeleteUser") == 0) {
        handle_deluser_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "TIME") == 0) {
        handle_time_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "UPGRADE") == 0 || Ql_strcmp(commandHead, "upgrade") == 0 || Ql_strcmp(commandHead, "Upgrade") == 0) {
        handle_upgrade_command(smsSender, smsContent, g_cfg);
        return;
    } 
    else if (Ql_strcmp(commandHead, "SMS") == 0) {
        handle_sms_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "SETTINGS") == 0 || Ql_strcmp(commandHead, "Settings") == 0 || Ql_strcmp(commandHead, "settings") == 0) {
        handle_settings_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "STATUS") == 0 || Ql_strcmp(commandHead, "ST") == 0 || Ql_strcmp(commandHead, "Status") == 0 || Ql_strcmp(commandHead, "status") == 0 || Ql_strcmp(commandHead, "st") == 0 ) {
        handle_status_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "VERSION") == 0 || Ql_strcmp(commandHead, "V") == 0 || Ql_strcmp(commandHead, "Version") == 0 || Ql_strcmp(commandHead, "version") == 0 || Ql_strcmp(commandHead, "v") == 0) {
        handle_version_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "REBOOT") == 0 || Ql_strcmp(commandHead, "RB") == 0 || Ql_strcmp(commandHead, "Reboot") == 0 || Ql_strcmp(commandHead, "reboot") == 0|| Ql_strcmp(commandHead, "rb") == 0) {
        handle_reboot_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "HELP") == 0 || Ql_strcmp(commandHead, "?") == 0) {
        handle_help_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "FIND") == 0) {
        handle_find_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "TRIP") == 0) {
        handle_trip_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "ALARM") == 0) {
        handle_alarm_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "LISTEN") == 0) {
        handle_listen_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "SPEED") == 0) {
        handle_speed_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "MAINPOWER") == 0 || Ql_strcmp(commandHead, "MP") == 0) {
        handle_mainpower_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "DISABLE") == 0 || Ql_strcmp(commandHead, "Disable") == 0 || Ql_strcmp(commandHead, "disable") == 0) {
        handle_disable_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "IGNITION") == 0 || Ql_strcmp(commandHead, "I") == 0) {
        handle_ignition_command(smsSender, smsContent,g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "SHOCK") == 0) {
        handle_shock_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "LOGGER") == 0) {
        handle_logger_command(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "HEARTBEAT") == 0) {
        handle_heartbeat_command(smsSender, smsContent, g_cfg);
    }  
    else if (Ql_strcmp(commandHead, "IMEI") == 0) {
        handle_setImei(smsSender, smsContent, g_cfg);
    } 
    else if (Ql_strcmp(commandHead, "IP") == 0) {
        handle_setIp(smsSender, smsContent, g_cfg);
    } 
    else {
        char bodyUpper2[1024];
        Ql_strncpy(bodyUpper2, smsContent,Ql_strlen(smsContent));
        
        char* commandHead2 =  my_strtok(bodyUpper2,":");
        APP_DEBUG("command %s\r\n", commandHead2);

        if (Ql_strcmp(commandHead2, "WWW") == 0) {
            handle_www_command(smsSender, smsContent, g_cfg);
        }
        else{
            sms_reply(smsSender, "Unknown command");
        }
        
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


void extract_key_value(const char *src, const char *key, char *dest, int size, char endDelim) {
    char *p = Ql_strstr(src, key);
    if (!p) {
        dest[0] = '\0';
        return;
    }
    p += Ql_strlen(key);  // Move pointer after key
    char *q = Ql_strchr(p, endDelim);
    if (!q) q = p + Ql_strlen(p); // If no delimiter, take till end
    int len = (int)(q - p);
    if (len >= size) len = size - 1; // Avoid overflow
    Ql_strncpy(dest, p, len);
    dest[len] = '\0';
}



s32 saveBytesToFlash(char* fileName,char* data,u32 length){
    s32 ret = -1;
    s32 fileHandle = Ql_FS_Open(fileName, QL_FS_CREATE_ALWAYS);
    u32 bytesWritten = 0;
    char dataToSave[length + 1];
    Ql_memset(dataToSave,0,sizeof(dataToSave));
    Ql_memcpy(dataToSave,data,length);
    dataToSave[length] = '\0';
    s32 resp = Ql_FS_Write(fileHandle,dataToSave,(length + 1), &bytesWritten);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("total bytes written %i\r\n", bytesWritten);
    return ret;
}


s32 readFromFlash(char* fileName,char* data,u32 length){
    s32 ret = -1;
    s32 fileHandle = Ql_FS_Open(fileName, QL_FS_READ_ONLY);
    int numberOfBytesRead = -1;
    ret = Ql_FS_Read(fileHandle,data,100,&numberOfBytesRead);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("data read = %s total bytes read = %i file handle = %i\r\n",data,numberOfBytesRead,fileHandle);
    return numberOfBytesRead;
}



s32 readFromFlashString(char* fileName,char* data,u32 length){
    s32 ret = -1;
    s32 fileHandle = Ql_FS_Open(fileName, QL_FS_READ_ONLY);
    int numberOfBytesRead = -1;
    char byteToRead[100];
    ret = Ql_FS_Read(fileHandle,byteToRead,100,&numberOfBytesRead);
    cleanString(byteToRead,data);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("data read = %s total bytes read = %i file handle = %i\r\n",data,numberOfBytesRead,fileHandle);
    return numberOfBytesRead;
}


s32 readAlphabetFromFlash(char* fileName,char* data,u32 length){
    s32 ret = -1;
    s32 fileHandle = Ql_FS_Open(fileName, QL_FS_READ_ONLY);
    int numberOfBytesRead = -1;
    char byteToRead[100];
    ret = Ql_FS_Read(fileHandle,byteToRead,100,&numberOfBytesRead);
    cleanAlpha(byteToRead,data);
    Ql_FS_Close(fileHandle);
    APP_DEBUG("data read = %s total bytes read = %i file handle = %i\r\n",data,numberOfBytesRead,fileHandle);
    return numberOfBytesRead;
}



void cleanAlpha(char *input, char *output) {
    int i, j = 0;
    for (i = 0; input[i] != '\0'; i++) {
        char c = input[i];

        // keep letters, digits, and dot
        if ((c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            (c == '.')) {
            output[j++] = c;
        }
    }
    output[j] = '\0';  // terminate string
}


void cleanString(char *input, char *output) {
    int i, j = 0;
    for (i = 0; input[i] != '\0'; i++) {
        char c = input[i];

        // keep letters, digits, and dot
        if ((c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') ||
            (c == '.') || (c == '+')) {
            output[j++] = c;
        }
    }
    output[j] = '\0';  // terminate string
}