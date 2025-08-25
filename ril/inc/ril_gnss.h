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
 *   ril_gnss.h 
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   The module declares GNSS related APIs.
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
#ifndef __RIL_GNSS_H__
#define __RIL_GNSS_H__

#include "ql_type.h"

typedef void (* CB_GNSSCMD)(char *strURC);

/*****************************************************************
* Function:     RIL_GNSS_Open
* 
* Description:
*               Power on/off GNSS.
*
* Parameters:   op:[in]
*                      1: Power on GNSS.
*                      0: Power off GNSS.
* Return:        
*               QL_RET_OK indicates this function successes.
*               QL_RET_ERR_PARAM indicates parameter error.
*****************************************************************/
s32 RIL_GNSS_Open(u8 op);

/******************************************************************************
* Function:     RIL_GNSS_SetRefLoc
*  
* Description:
*               This function sets the reference location for QuecFastFixOnline. 
*
* Parameters:    
*               <lat>:
*                   [in]double, latitude
*               <lon>
*                   [in]double, longitude
* Return:
*                RIL_AT_SUCCESS,send AT successfully.
*                RIL_AT_FAILED, send AT failed.
*                RIL_AT_TIMEOUT,send AT timeout.
*                RIL_AT_BUSY,   sending AT.
*                RIL_AT_INVALID_PARAM, invalid input parameter.
*                RIL_AT_UNINITIALIZED, RIL is not ready, need to wait for MSG_ID_RIL_READY
*                                      and then call Ql_RIL_Initialize to initialize RIL.
******************************************************************************/
s32 RIL_GNSS_SetRefLoc(double lat, double lon);

/******************************************************************************
* Function:     RIL_GNSS_GetPowerState
*  
* Description:
*               This function gets the power state of GNSS. 
*
* Parameters:    
*               <stat>:
*                   [out]pointer of s32, address of s32 variable
* Return:
*                RIL_AT_SUCCESS,send AT successfully.
*                RIL_AT_FAILED, send AT failed.
*                RIL_AT_TIMEOUT,send AT timeout.
*                RIL_AT_BUSY,   sending AT.
*                RIL_AT_INVALID_PARAM, invalid input parameter.
*                RIL_AT_UNINITIALIZED, RIL is not ready, need to wait for MSG_ID_RIL_READY
*                                      and then call Ql_RIL_Initialize to initialize RIL.
******************************************************************************/
s32  RIL_GNSS_GetPowerState(s32 *stat);

/*****************************************************************
* Function:     RIL_GNSS_Read
* 
* Description:
*               Query the navigation information.
*
* Parameters:   item  :  [in] Pointer to the query item
*               rdBuff:  [out] Pointer to the information buffer
* Return:        
*               QL_RET_OK indicates this function successes.
*               QL_RET_ERR_PARAM indicates parameter error.
*****************************************************************/
s32 RIL_GNSS_Read(u8 *item, u8 *rdBuff);

/*****************************************************************
* Function:     RIL_GNSS_CMD_Send
* 
* Description:
*               This function is used to send NMEA to GNSS module.
*
* Parameters:
*                <cmdType>:
*                       [IN] always 0 currently.
*                <cmdStr>:
*                       [IN] this string is an NMEA sentence.
*                <cb_GNSSCMD_hdl>:
*                       [IN] callback function for QGNSSCMD URC handle.
*
* Return:        
*                RIL_AT_SUCCESS,send AT successfully.
*                RIL_AT_FAILED, send AT failed.
*                RIL_AT_TIMEOUT,send AT timeout.
*                RIL_AT_BUSY,   sending AT.
*                RIL_AT_INVALID_PARAM, invalid input parameter.
*                RIL_AT_UNINITIALIZED, RIL is not ready, need to wait for MSG_ID_RIL_READY
*                                      and then call Ql_RIL_Initialize to initialize RIL.               
*****************************************************************/
s32 RIL_GNSS_CMD_Send(u8 cmdType, u8 *cmdStr, CB_GNSSCMD cb_GNSSCMD_hdl);

/*****************************************************************
* Function:     RIL_GNSS_AGPS
* 
* Description:
*               This function is used to download APGS data from server save it 
*               into RAM of modem.
*
*                <cb_GNSSCMD_hdl>:
*               [IN] callback function for QGNSSCMD URC handle.
*
* Return:        
*                RIL_AT_SUCCESS,send AT successfully.
*                RIL_AT_FAILED, send AT failed.
*                RIL_AT_TIMEOUT,send AT timeout.
*                RIL_AT_BUSY,   sending AT.
*                RIL_AT_INVALID_PARAM, invalid input parameter.
*                RIL_AT_UNINITIALIZED, RIL is not ready, need to wait for MSG_ID_RIL_READY
*                                      and then call Ql_RIL_Initialize to initialize RIL.               
*****************************************************************/
	s32 RIL_GNSS_AGPS(CB_GNSSCMD cb_GNSSCMD_hdl);

/*****************************************************************
* Function:     RIL_GNSS_AGPSAID
* 
* Description:
*               This function is used to inject AGPSdata to GNSS module.
*
* Parameters:	NONE
* Return:        
*                RIL_AT_SUCCESS,send AT successfully.
*                RIL_AT_FAILED, send AT failed.
*                RIL_AT_TIMEOUT,send AT timeout.
*                RIL_AT_BUSY,   sending AT.
*                RIL_AT_INVALID_PARAM, invalid input parameter.
*                RIL_AT_UNINITIALIZED, RIL is not ready, need to wait for MSG_ID_RIL_READY
*                                      and then call Ql_RIL_Initialize to initialize RIL.               
*****************************************************************/
s32 RIL_GNSS_AGPSAID(void);

/*****************************************************************
* Function:     RIL_GNSS_Read_TimeSync_Status
* 
* Description:
*               This function is used to read time synchronization status.
*
* Parameters:	
*				<status> : [OUT] point to the result if readed successfully.
* Return:        
*                RIL_AT_SUCCESS,send AT successfully.
*                RIL_AT_FAILED, send AT failed.
*                RIL_AT_TIMEOUT,send AT timeout.
*                RIL_AT_BUSY,   sending AT.
*                RIL_AT_INVALID_PARAM, invalid input parameter.
*                RIL_AT_UNINITIALIZED, RIL is not ready, need to wait for MSG_ID_RIL_READY
*                                      and then call Ql_RIL_Initialize to initialize RIL.               
*****************************************************************/
s32 RIL_GNSS_Read_TimeSync_Status(u8 *status);
#endif	//__RIL_GPS_H__

