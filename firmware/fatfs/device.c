#include "diskio.h"
#include "device.h"
#include "hardware/flash.h"
#include "../msc_disks.h"

int Flash_disk_status(void){
    // 常に初期化
    return 0;
}

int Flash_disk_initialize(void){
    // 常に初期化
    return 0;
}

int Flash_disk_read(BYTE *buff, LBA_t sector, UINT count){
    
    DWORD i,j;

/*    if(sector+count > 512){
        return RES_PARERR;
    }*/

    memcpy(buff, &flash_target_contents[sector*SECTOR_SIZE], count*SECTOR_SIZE);

    return RES_OK;
}

int Flash_disk_write(BYTE *buff, LBA_t sector, UINT count){
    return 0;
}
