/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of Quectel Co., Ltd. 2019
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ril_gnss.c 
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   The module implements GNSS related APIs.
 *
 * Author:
 * -------
 * -------
 *
 *============================================================================
 *             HISTORY
 *----------------------------------------------------------------------------
 * 
 ****************************************************************************/
#include "ril_gnss.h"
#include "ril.h"
#include "ril_util.h"
#include "ql_stdlib.h"
#include "ql_error.h"
#include "ql_trace.h"

static s32 ATResponse_QGNSSC_Handler(char* line, u32 len, void* userdata);
static s32 ATResponse_GNSSRead_Hdlr(char* line, u32 len, void* userData);
static s32 ATResponse_GNSSReadTS_Hdlr(char* line, u32 len, void* userData);
static CB_GNSSCMD     callback_GNSSCMD = NULL;
static CB_GNSSCMD     callback_GNSSAGPS = NULL;

static s32 ATResponse_QGNSSC_Handler(char* line, u32 len, void* userdata)
{
    char * p1 = NULL;
    char * p2 = NULL;
    char *head = Ql_RIL_FindString(line, len, "+QGNSSC:"); //continue wait
    if(head)
    {
        char strStat[10];
        p1 = Ql_strstr(line, ":");
        p2 = Ql_strstr(p1, "\r\n");
        if(p1 && p2)
        {
            Ql_memset(strStat, 0x0, sizeof(strStat));
            Ql_strncpy(strStat, p1 + 1, p2 - p1 - 1);
            *((u8 *)userdata) = Ql_atoi(strStat);
        }
        return  RIL_ATRSP_CONTINUE;
    }

   head = Ql_RIL_FindLine(line, len, "OK"); // find <CR><LF>OK<CR><LF>, <CR>OK<CR>£¬<LF>OK<LF>
   if(head)
   {
       return  RIL_ATRSP_SUCCESS;
   }

    head = Ql_RIL_FindLine(line, len, "ERROR");// find <CR><LF>ERROR<CR><LF>, <CR>ERROR<CR>£¬<LF>ERROR<LF>
    if(head)
    {  
        return  RIL_ATRSP_FAILED;
    } 

    head = Ql_RIL_FindString(line, len, "+CME ERROR:");//fail
    if(head)
    {
        return  RIL_ATRSP_FAILED;
    }

    return RIL_ATRSP_CONTINUE; //continue wait
}

static s32 ATResponse_GNSSReadTS_Hdlr(char* line, u32 len, void* userData)
{
	char* p1 = NULL;
	char* p2 = NULL;
	char* head = Ql_RIL_FindString(line, len, "+QGNSSTS:"); //continue wait

	if(head)
	{
		char strStat[10];
		p1 = Ql_strstr(line, ":");
	    p2 = Ql_strstr(p1, "\r\n");
		if(p1 && p2)
		{
			Ql_memset(strStat, 0x0, sizeof(strStat));
        	Ql_strncpy(strStat, p1 + 1, p2 - p1 - 1);
			*((u8 *)userData) = Ql_atoi(strStat);
		}
		return  RIL_ATRSP_CONTINUE;
	}

    head = Ql_RIL_FindString(line, len, "+CME ERROR");// find <CR><LF>ERROR<CR><LF>, <CR>ERROR<CR>£¬<LF>ERROR<LF>
	if(head)
	{  
		return  RIL_ATRSP_FAILED;
	}

	head = Ql_RIL_FindLine(line, len, "OK"); // find <CR><LF>OK<CR><LF>, <CR>OK<CR>£¬<LF>OK<LF>
	if(head)
	{
		return  RIL_ATRSP_SUCCESS;
	}

	head = Ql_RIL_FindLine(line, len, "ERROR");// find <CR><LF>ERROR<CR><LF>, <CR>ERROR<CR>£¬<LF>ERROR<LF>
	if(head)
	{
		return  RIL_ATRSP_FAILED;
	}

	return RIL_ATRSP_CONTINUE; //continue wait
}

static s32 ATResponse_GNSSRead_Hdlr(char* line, u32 len, void* userData)
{
	char* p1 = NULL;
	char* p2 = NULL;
	char* head = Ql_RIL_FindString(line, len, "+QGNSSRD:"); //continue wait
	char strTmp[10];

	if(head)
	{
	    Ql_strncat((char*)userData,line,Ql_strlen(line));
		return  RIL_ATRSP_CONTINUE;
	}
    else
    {
        head = Ql_RIL_FindString(line, len, "$GNRMC");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }

        head = Ql_RIL_FindString(line, len, "$GNVTG");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }

        head = Ql_RIL_FindString(line, len, "$GNGGA");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }

        head = Ql_RIL_FindString(line, len, "$GNGSA");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }
        head = Ql_RIL_FindString(line, len, "$GPGSA");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }
        head = Ql_RIL_FindString(line, len, "$GLGSA");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }
        head = Ql_RIL_FindString(line, len, "$BDGSA");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }

        head = Ql_RIL_FindString(line, len, "$GNGSV");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }
        head = Ql_RIL_FindString(line, len, "$GPGSV");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }
        head = Ql_RIL_FindString(line, len, "$GLGSV");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }
        head = Ql_RIL_FindString(line, len, "$BDGSV");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }

        head = Ql_RIL_FindString(line, len, "$GNGLL");
        if(head)
        {
            Ql_strncat((char*)userData,line,Ql_strlen(line));
        }
    }

    head = Ql_RIL_FindString(line, len, "+CME ERROR");// find <CR><LF>ERROR<CR><LF>, <CR>ERROR<CR>£¬<LF>ERROR<LF>
	if(head)
	{  
		return  RIL_ATRSP_FAILED;
	}

	head = Ql_RIL_FindLine(line, len, "OK"); // find <CR><LF>OK<CR><LF>, <CR>OK<CR>£¬<LF>OK<LF>
	if(head)
	{
		return  RIL_ATRSP_SUCCESS;
	}

	head = Ql_RIL_FindLine(line, len, "ERROR");// find <CR><LF>ERROR<CR><LF>, <CR>ERROR<CR>£¬<LF>ERROR<LF>
	if(head)
	{
		return  RIL_ATRSP_FAILED;
	}

	return RIL_ATRSP_CONTINUE; //continue wait
}

void OnURCHandler_GNSSCMD(const char* strURC, void* reserved)
{
	char urcHead[] = "\r\n+QGNSSCMD:\0";
        
 	if ( NULL != callback_GNSSCMD )
 	{
		if( Ql_StrPrefixMatch(strURC, urcHead) )
		{
			callback_GNSSCMD(strURC);
		}
	}
}

void OnURCHandler_GNSSAGPS(const char* strURC, void* reserved)
{
	char urcHead[] = "\r\n+QGAGPS\0";
        
 	if ( NULL != callback_GNSSAGPS )
 	{
		if( Ql_StrPrefixMatch(strURC, urcHead) )
		{
			callback_GNSSAGPS(strURC);
		}
	}
}

/****************************************RIL API************************************************/
s32 RIL_GNSS_Open(u8 op)
{
	char strAT[50] = {"\0"};
    u16  atLength = 0;

	if (op != 0 && op != 1)
	{
		return QL_RET_ERR_INVALID_PARAMETER;
	}

	Ql_memset(strAT, 0x0, sizeof(strAT));

	atLength = Ql_sprintf(strAT, "AT+QGNSSC=%d",op);

	return Ql_RIL_SendATCmd(strAT, atLength, NULL, NULL, 0);
}

s32 RIL_GNSS_SetRefLoc(double lat, double lon)
{
	char strAT[50] = {"\0"};
    u16  atLength = 0;

	Ql_memset(strAT, 0x0, sizeof(strAT));

	atLength = Ql_sprintf(strAT, "AT+QGREFLOC=%f,%f",lat,lon);

	return Ql_RIL_SendATCmd(strAT, atLength, NULL, NULL, 0);
}

s32  RIL_GNSS_GetPowerState(s32 *stat)
{
    s32 retRes = -1;
    s32 nStat = 0;
    char strAT[] = "AT+QGNSSC?\0";

    retRes = Ql_RIL_SendATCmd(strAT, Ql_strlen(strAT), ATResponse_QGNSSC_Handler, &nStat, 0);
    if(RIL_AT_SUCCESS == retRes)
    {
       *stat = nStat; 
    }
    return retRes;
}

s32 RIL_GNSS_Read(u8 *item, u8 *rdBuff)
{
	char strAT[50] = {"\0"};
    u16  atLength = 0;
    s32  ret = -1;
    u8   readBuff[1000];

	Ql_memset(strAT, 0x0, sizeof(strAT));
    Ql_memset(readBuff, 0x0, sizeof(readBuff));

    if(0 == Ql_strncmp(item, "ALL", Ql_strlen(item)))
    {
        atLength = Ql_sprintf(strAT, "AT+QGNSSRD?");
    }
    else
    {
        atLength = Ql_sprintf(strAT, "AT+QGNSSRD=\"NMEA/%s\"", item);
    }

	ret = Ql_RIL_SendATCmd(strAT, atLength, ATResponse_GNSSRead_Hdlr, readBuff, 0);

    if(RIL_ATRSP_SUCCESS == ret)
    {
        Ql_strcpy((char*)rdBuff,readBuff);
    }

    return ret;
}

s32 RIL_GNSS_Read_TimeSync_Status(u8 *status)
{
	char strAT[50] = {"\0"};
    u16  atLength = 0;
    s32  ret = RIL_AT_FAILED;
	u8	 stat;

	Ql_memset(strAT, 0x0, sizeof(strAT));

    atLength = Ql_sprintf(strAT, "AT+QGNSSTS?");

	ret = Ql_RIL_SendATCmd(strAT, atLength, ATResponse_GNSSReadTS_Hdlr, &stat, 0);

    if(RIL_ATRSP_SUCCESS == ret)
    {
        *status = stat;
    }

    return ret;
}

s32 RIL_GNSS_CMD_Send(u8 cmdType, u8 *cmdStr, CB_GNSSCMD cb_GNSSCMD_hdl)
{
	s32 ret = RIL_AT_FAILED;
	char strAT[200]; 

    if (cmdType > 1 || NULL == cmdStr)
    {
        return RIL_AT_INVALID_PARAM;
    }

    callback_GNSSCMD = cb_GNSSCMD_hdl;
    
	Ql_memset( strAT, 0, sizeof(strAT) );
	Ql_sprintf( strAT, "AT+QGNSSCMD=%d,\"%s\"\r\n", cmdType, cmdStr);
	
	ret = Ql_RIL_SendATCmd( strAT, Ql_strlen(strAT), NULL, NULL, 0 ) ;
    
    return ret;
}

s32 RIL_GNSS_AGPS(CB_GNSSCMD cb_GNSSCMD_hdl)
{
	char strAT[50] = {0};
	u16  atLength = 0;
	
	callback_GNSSAGPS = cb_GNSSCMD_hdl;
	Ql_memset(strAT, 0x0, sizeof(strAT));

	atLength = Ql_sprintf(strAT, "AT+QGAGPS");

	return Ql_RIL_SendATCmd(strAT, atLength, NULL, NULL, 0);
}

s32 RIL_GNSS_AGPSAID(void)
{
	char strAT[50] = {0};
	u16  atLength = 0;

	Ql_memset(strAT, 0x0, sizeof(strAT));

	atLength = Ql_sprintf(strAT, "AT+QGAGPSAID");

	return Ql_RIL_SendATCmd(strAT, atLength, NULL, NULL, 0);
}

s32 RIL_GNSS_EPO_Config_APN(u8 context_id,u8 *apnName, u8 *apnUserId, u8 *apnPasswd)
{

	s32  ret = RIL_AT_FAILED;
	char strAT[100] = {0};
	u16  atLength = 0;

	Ql_memset(strAT, 0x0, sizeof(strAT));

	atLength = Ql_sprintf(strAT, "AT+QIFGCNT=%d",context_id);

	ret = Ql_RIL_SendATCmd(strAT, atLength, NULL, NULL, 0);

	if(RIL_ATRSP_SUCCESS != ret)
	{
		return ret;
	}

	Ql_memset(strAT, 0x0, sizeof(strAT));
	atLength = Ql_sprintf(strAT, "AT+QICSGP=1,\"%s\",\"%s\",\"%s\"", apnName, apnUserId, apnPasswd);

	ret = Ql_RIL_SendATCmd(strAT, atLength, NULL, NULL, 0);
	return ret;
}


