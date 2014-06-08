/*
 * flash.c
 *
 *  Created on: 2013/06/09
 *      Author: ksekimoto
 */

#include <common.h>

flash_info_t flash_info[CONFIG_SYS_MAX_FLASH_BANKS];

unsigned long flash_init(void)
{
    uint32_t flashbase;
    uint32_t j, k;
    flash_info[0].flash_id = FLASH_MAN_FUJ;
    flash_info[0].size = 0x00100000;    /* 1MB */
    flash_info[0].sector_count = 10;
    memset (flash_info[0].protect, 0, 10);
    flashbase = 0x00000000;
    for (j = 0, k = 0; j < 2; j++, k++) {
        flash_info[0].start[k] = flashbase;
        flashbase += 0x00004000;
    }
    for (j = 0; j < 1; j++, k++) {
        flash_info[0].start[k] = flashbase;
        flashbase += 0x00018000;
    }
    for (j = 0; j < 7; j++, k++) {
        flash_info[0].start[k] = flashbase;
        flashbase += 0x00020000;
    }
    return flash_info[0].size;
}

void flash_print_info  (flash_info_t *info)
{
    uint32_t i, j, count, erased;
    unsigned char* p;

    printf ("FM3 Internal Flash\n");
    printf ("  Size: %ld KB in %d Sectors\n", info->size >> 10, info->sector_count);
    printf ("  Sector Start Addresses:");
    for (i = 0; i < info->sector_count; i++) {
        if ((i % 5) == 0) {
            printf ("\n   ");
        }
        if (i < (info->sector_count - 1)) {
            count = info->start[i+1] - info->start[i];
        }
        else {
            count = info->start[0] + info->size - info->start[i];
        }
        p = (unsigned char*)(info->start[i]);
        erased = 1;
        for (j = 0; j < count; j++) {
            if (*p != 0xFF) {
                erased = 0;
                break;
            }
            p++;
        }
        printf (" %08lX%s%s", info->start[i], info->protect[i] ? " RO" : "   ",
            erased ? " E" : "  ");
    }
    printf ("\n");
}

int flash_erase (flash_info_t *info, int s_first, int s_last)
{
    uint32_t ret = 0;
    uint32_t i;
    for (i = s_first; i <= s_last; i++) {
        fm3_flash_erase((uint32_t)info->start[i], (uint32_t)0x1000);
        printf("Erasing sector=%d addr=%08x\n", i, info->start[i]);
    }
    return 0;
}

int write_buff (flash_info_t *info, uchar *src, ulong addr, ulong cnt)
{
    fm3_flash_write((uint32_t)addr, (void *)src, (uint32_t)cnt);
    return 0;
}
