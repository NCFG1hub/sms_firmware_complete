


#include "ril_flash.h"
#include "ril.h"
#include "ril_util.h"
#include "ql_stdlib.h"
#include "ql_error.h"
#include "ql_trace.h"
#include "ql_uart.h"




static u32 getStorageSizeResponse(char* line, u32 len, void* userData);
static u32 getAvailableMemoryResponse(char* line, u32 len, void* userData);
static u32 getSavedFilesResponse(char* line, u32 len, void* userData);
static u32 readStorageResponse(char* line, u32 len, void* userData);
static u32 uploadFileToStorageResponse(char* line, u32 len, void* userData);
static u32 downloadFileFromStorageResponse(char* line, u32 len, void* userData);
static u32 deleteFileFromStorageResponse(char* line, u32 len, void* userData);
static u32 openFileFromStorageResponse(char* line, u32 len, void* userData);
static u32 readFileFromStorageResponse(char* line, u32 len, void* userData);
static u32 writeToFileStorageResponse(char* line, u32 len, void* userData);
static u32 closeFileFromStorageResponse(char* line, u32 len, void* userData);
static u32 saveBytesToFlashResponse(char* line, u32 len, void* userData);





static u32 closeFileFromStorageResponse(char* line, u32 len, void* userData)
{
    STORAGE_RESPONSE resp = FAILED;
    if (Ql_RIL_FindLine(line, len, "OK"))
    {
       resp = SUCCESS;
    }

    return resp;
}



static u32 writeToFileStorageResponse(char* line, u32 len, void* userData)
{
    STORAGE_RESPONSE resp = FAILED;
    if (Ql_RIL_FindLine(line, len, "CONNECT"))
    {
       resp = SUCCESS;
    }

    return resp;
}


static u32 saveBytesToFlashResponse(char* line, u32 len, void* userData)
{
    STORAGE_RESPONSE resp = FAILED;
    if (Ql_RIL_FindLine(line, len, "CONNECT"))
    {
       resp = SUCCESS;
    }

    return resp;
}


static u32 readFileFromStorageResponse(char* line, u32 len, void* userData)
{
    STORAGE_RESPONSE resp = FAILED;
    
    if (Ql_RIL_FindLine(line, len, "CONNECT"))
    {
       Ql_strncat((char*)userData,line,Ql_strlen(line));
       resp = SUCCESS;
    }

    return resp;
}


static u32 openFileFromStorageResponse(char* line, u32 len, void* userData)
{
    STORAGE_RESPONSE resp = FAILED;
    if (Ql_RIL_FindLine(line, len, "+QFOPEN:"))
    {
        u32 fileHandlerIndex = 0;
        u32 isFileHandlerSeen = 0;

        u8* myUserData;

        for(u32 i = 0; i < len; ++i){

            if(isFileHandlerSeen == 0){
                if(line[i] == ':'){
                    isFileHandlerSeen = 1;
                }
            }
            else if(isFileHandlerSeen == 1){
                if(line[i] == '\n'){
                    break;
                }
                else{
                    myUserData[fileHandlerIndex] = line[i];
                    fileHandlerIndex++;
                }
            }
        }
        Ql_memcpy((u8*)userData,myUserData,sizeof(myUserData));
        resp = SUCCESS;
    }

    return resp;
}




static u32 deleteFileFromStorageResponse(char* line, u32 len, void* userData)
{
    STORAGE_RESPONSE resp = FAILED;
    if (Ql_RIL_FindLine(line, len, "+QFDEL:"))
    {
       resp = SUCCESS;
    }

    return resp;
}


static u32 downloadFileFromStorageResponse(char* line, u32 len, void* userData)
{
    STORAGE_RESPONSE resp = FAILED;
    if (Ql_RIL_FindLine(line, len, "CONNECT"))
    {
       Ql_strncat((char*)userData,line,Ql_strlen(line));
       resp = SUCCESS;
    }

    return resp;
}




static u32 uploadFileToStorageResponse(char* line, u32 len, void* userData)
{
    STORAGE_RESPONSE resp = FAILED;
    if (Ql_RIL_FindLine(line, len, "CONNECT"))
    {
       resp = SUCCESS;
    }

    return resp;
}



static u32 getSavedFilesResponse(char* line, u32 len, void* userData)
{
    u32 storageSize = 0;
    FLASH_FILES* myUserData;

    if (Ql_RIL_FindLine(line, len, "+QFLST:"))
    {
        u8 isSizeSeen = 0;
        char sizeInChar[10];
        Ql_memset(sizeInChar,0,sizeof(sizeInChar));

        u32 sizeIndex = 0;
        u32 fileNameIndex = 0;

        for(u32 i = 0; i < len; ++i){

            if(isSizeSeen == 0){
                if(line[i] == ':'){
                    isSizeSeen = 1;
                }
            }
            else if(isSizeSeen == 1){
                if(line[i] == ','){
                    isSizeSeen = 2;
                }
                else{
                    myUserData->FILE_NAME[fileNameIndex] = line[i];
                    fileNameIndex++;
                }
            }
            else if(isSizeSeen == 2){
                if(line[i] == '\n'){
                    break;
                }
                else{
                   if(isSizeSeen == 1){
                        sizeInChar[sizeIndex] = line[i];
                        sizeIndex++;
                    }
                }
                
            }
        }

        myUserData->FILE_SIZE = Ql_atoi(sizeInChar);

        Ql_memcpy((FLASH_FILES*)userData,myUserData,sizeof(myUserData));
    }

    return 0;
}



static u32 getAvailableMemoryResponse(char* line, u32 len, void* userData)
{
    u32 storageSize = 0;
    if (Ql_RIL_FindLine(line, len, "+QFLDS:"))
    {
        u8 isSizeSeen = 0;
        char sizeInChar[10];
        Ql_memset(sizeInChar,0,sizeof(sizeInChar));

        u32 sizeIndex = 0;

        for(u32 i = 0; i < len; ++i){

            if(isSizeSeen == 0){
                if(line[i] == ':'){
                    isSizeSeen = 1;
                }
            }
            else if(isSizeSeen == 1){
                if(line[i] == ','){
                    isSizeSeen = 2;
                }
            }
            else if(isSizeSeen == 2){
                if(line[i] == '\n'){
                    break;
                }
                else{
                   if(isSizeSeen == 1){
                        sizeInChar[sizeIndex] = line[i];
                        sizeIndex++;
                    }
                }
                
            }
        }

        storageSize = Ql_atoi(sizeInChar);
    }

    return storageSize;
}
 

static u32 getStorageSizeResponse(char* line, u32 len, void* userData)
{
    u32 storageSize = 0;
    if (Ql_RIL_FindLine(line, len, "+QFLDS:"))
    {
        u8 isSizeSeen = 0;
        char sizeInChar[10];
        Ql_memset(sizeInChar,0,sizeof(sizeInChar));

        u32 sizeIndex = 0;

        for(u32 i = 0; i < len; ++i){

            if(isSizeSeen == 0){
                if(line[i] == ':'){
                    isSizeSeen = 1;
                }
            }
            else{
                if(line[i] == ','){
                    break;
                }
                else{
                   if(isSizeSeen == 1){
                        sizeInChar[sizeIndex] = line[i];
                        sizeIndex++;
                    }
                }
                
            }
        }

        storageSize = Ql_atoi(sizeInChar);
    }

    return storageSize;
}



static u32 readStorageResponse(char* line, u32 len, void* userData) {
     char* savedContent = "";
     Ql_strcpy(savedContent,line);
     Ql_memcpy((char*)userData,savedContent,Ql_strlen(line));
     return 1;
}
    




s32 getStorageSize(){
   s32 ret;
   char strAt[20];
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFLDS=\"UFS\"\r\n");
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),getStorageSizeResponse,NULL,0);
   return ret;
}


s32 getAvailableMemory(){
   s32 ret;
   char strAt[20];
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFLDS=\"UFS\"\r\n");
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),getAvailableMemoryResponse,NULL,0);
   return ret;
}


s32 getSavedFiles(FLASH_FILES* myFiles){
   s32 ret;
   char strAt[20];
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFLST=\"*\"\r\n");
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),getSavedFilesResponse,&myFiles,0);
   return ret;
}



s32 uploadFileToStorage(char *fileName, char *data, u32 dataSize){
   STORAGE_RESPONSE ret;
   char strAt[20]; 
   char ctrlz = 0x1A;  
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFUPL=\"%s\",%i\r\n",fileName,dataSize);
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),uploadFileToStorageResponse,NULL,0);
   if(ret == SUCCESS){
      Ql_Debug_Trace("successfully uploaded\r\n");
   }
   else{
      ret = FAILED;
   }
   return ret;
}



s32 saveBytesToFlash(char *fileName, u8 *data, u32 dataSize){
   STORAGE_RESPONSE ret;
   char strAt[20]; 
   char ctrlz = 0x1A;  
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFWRITE=\"%s\",%i\r\n",fileName,dataSize);
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),saveBytesToFlashResponse,NULL,0);
   if(ret == SUCCESS){
      Ql_UART_Write(UART_PORT1,(u8*)(data), Ql_strlen(data));
      Ql_UART_Write(UART_PORT1, &ctrlz, 1);
   }
   else{
      ret = FAILED;
   }
   return ret;
}


s32 downloadFileFromStorage(char *fileName, char *data, u32 *dataSize){
   STORAGE_RESPONSE ret;
   char strAt[20];
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFDWL=\"%s\"\r\n",fileName);
   char *responseData;
   // handle the data transfer
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),downloadFileFromStorageResponse,&responseData,0);
   return ret;
}


s32 deleteFileFromStorage(char *fileName){
   STORAGE_RESPONSE ret;
   char strAt[20];
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFDEL=\"%s\"\r\n",fileName);
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),deleteFileFromStorageResponse,NULL,0);
   return ret;
}


s32 openFileFromStorage(char *fileName,s32 mode, char *fileHandle){
   /*
     mode = 0 create file if not exist, 
     mode = 1 create a new file and delete old file with same name if it exist
     mode = 2 If the file exists, open it and it only can be read
   */
   STORAGE_RESPONSE ret;
   char strAt[20];
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFOPEN=\"%s\",%i\r\n",fileName,mode); 
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),openFileFromStorageResponse,&fileHandle,0);
   return ret;
}


s32 readFileFromStorage(char *fileHandle, char *data, u32 *dataSize){
   STORAGE_RESPONSE ret;
   char strAt[20];
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFDWL=\"%s\",%i\r\n",fileHandle,dataSize);
   char *responseData;
   // handle the data transfer
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),readFileFromStorageResponse,&responseData,0);
   return ret;
}

s32 writeToFileStorage(char *fileName, char *data, u32 *dataSize){
   STORAGE_RESPONSE ret;
   char strAt[20];
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFWRITE=\"%s\",%i\r\n",fileName,dataSize);
   char *responseData;
   // handle the data transfer
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),writeToFileStorageResponse,&responseData,0);
   if(ret == SUCCESS){
      Ql_UART_Write(UART_PORT1,(u8*)(data), Ql_strlen(data));
   }
   else{
      ret = FAILED;
   }
   return ret;
}


s32 closeFileFromStorage(char *fileHandle){
   STORAGE_RESPONSE ret;
   char strAt[20];
   Ql_memset(strAt,0,sizeof(strAt));
   Ql_sprintf(strAt,"AT+QFCLOSE=\"%s\"\r\n",fileHandle); 
   ret = Ql_RIL_SendATCmd(strAt,Ql_strlen(strAt),closeFileFromStorageResponse,NULL,0);
   return ret;
}


void readFromStorage(char *fileName, char* fileData)
{
    char cmd[100];
    s32 ret;

    Ql_sprintf(cmd, "AT+QFREAD=\"%s\"\r\n",fileName);
    char *responseData;
    ret = Ql_RIL_SendATCmd(cmd, Ql_strlen(cmd), readStorageResponse, responseData, 5000);
    if (ret != RIL_AT_SUCCESS) {
        Ql_Debug_Trace("Failed to read file\r\n");
    }
    else{
        Ql_memcpy(fileData,responseData,Ql_strlen(responseData));
    }

}