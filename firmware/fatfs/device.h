/*-----------------------------------------------------------------------/
/  Low level disk interface modlue include file   (C)jp7dki, 2022        /
/-----------------------------------------------------------------------*/

#ifndef _DEVICE_DEFINED
#define _DEVICE_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

int Flash_disk_status(void);
int Flash_disk_initialize(void);
int Flash_disk_read(BYTE *buff, LBA_t sector, UINT count);
int Flash_disk_write(BYTE *buff, LBA_t sector, UINT count);

#endif