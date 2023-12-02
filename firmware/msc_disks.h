#ifndef _MSC_DISKS_DEFINED
#define _MSC_DISKS_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// W25Q128JV Block31 のセクタ0の先頭アドレス = 0x1F0000
// W25128JVの書き込み最小単位 = FLASH_PAGE_SIZE(256Bytes)
// FLASH_PAGE_SIZE(256Bytes)はflash.hで定義済
#define FLASH_TARGET_OFFSET 0x1F0000
#define SECTOR_SIZE 4096

enum
{
//	DISK_BLOCK_NUM = 256,		// 8KB is the smallest size that windows allow to mount
//	DISK_BLOCK_SIZE = 512      // Flash sector size is 4096 Bytes
	DISK_BLOCK_NUM = 16,		// 8KB is the smallest size that windows allow to mount
	DISK_BLOCK_SIZE = 4096      // Flash sector size is 4096 Bytes

};

extern uint8_t *flash_target_contents;

//extern uint8_t msc_disk[4][DISK_BLOCK_SIZE];
extern uint8_t msc_disk[DISK_BLOCK_NUM][DISK_BLOCK_SIZE];

void msc_init(void);

#endif