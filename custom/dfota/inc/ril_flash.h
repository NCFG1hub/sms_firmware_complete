
#ifndef __RIL_FLASH_H__
#define __RIL_FLASH_H__


#include "ql_type.h"

typedef void (* CB_GNSSCMD)(char *strURC);

{
  SUCCESS,
  FAILED
}STORAGE_RESPONSE;

typedef struct{
    char* FILE_NAME;
    u32 FILE_SIZE;
    u8* fileContent;
} FLASH_FILES;


s32 getStorageSize();
s32 getAvailableMemory();
s32 getSavedFiles(FLASH_FILES* myFiles);
s32 uploadFileToStorage(char *fileName, char *data, u32 dataSize);
s32 downloadFileFromStorage(char *fileName, char *data, u32 *dataSize);
s32 deleteFileFromStorage(char *fileName);
s32 openFileFromStorage(char *fileName,s32 mode, char *fileHandle);
s32 readFileFromStorage(char *fileName, char *data, u32 *dataSize);
s32 closeFileFromStorage(char *fileHandle);
s32 saveBytesToFlash(char *fileName, u8 *data, u32 dataSize);


void readFromStorage(char *fileName, char* fileData);
s32 writeToFileStorage(char *fileName, char *data, u32 *dataSize);

#endif	//__RIL_FLASH_H__