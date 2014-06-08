/*
 * WX_MP3PLCD_F.h
 *
 *  Created on: 2013/06/08
 *      Author: ksekimoto
 */

#ifndef WX_MP3PLCD_F_H_
#define WX_MP3PLCD_F_H_

#define MB9BF618T
//#define MB9BF506N

//#undef DEBUG  /* disable debug messages */

#define CONFIG_SYS_ARMCORTEXM3  /* ARM Cortex-M3 CPU core */
#define CONFIG_SYS_ARMCORTEXM4  /* ARM Cortex-M4 CPU core */

#define CONFIG_SYS_FM3          /* Fujitsu FM3 device */

#define CONFIG_FM3_SERIAL

#define CONFIG_DISPLAY_CPUINFO      1
#define CONFIG_DISPLAY_BOARDINFO    1
#define CONFIG_SYS_BOARD_REV_STR    "Rev 0.1"
#define CONFIG_SYS_PROMPT           "FM3> "     /* monitor prompt */

#define CONFIG_ARCH_CPU_INIT    /* CPU specific initialization */
/* ToDo: clock configuration */
#define CONFIG_SYS_HZ   1000    /* Number of clock ticks in 1 sec */

#define CONFIG_MEM_NVM_BASE         0x00000000  /* internal flash base address */
#define CONFIG_MEM_NVM_LEN          0x00100000  /* internal flash size */
#define CONFIG_MEM_NVM_UBOOT_OFF    0x00000000  /* offset of u-boot on internal flash */

#ifdef MB9BF506N
#define CONFIG_MEM_RAM_BASE         0x1FFF8000
#define CONFIG_MEM_RAM_LEN          (10 * 1024)
#define CONFIG_MEM_RAM_BUF_LEN      (34 * 1024)
#define CONFIG_MEM_MALLOC_LEN       (16 * 1024)
#define CONFIG_MEM_STACK_LEN        (4 * 1024)

#define CONFIG_SYS_RAM_BASE         0x63000000
#define CONFIG_SYS_RAM_SIZE         0x00200000
#define CONFIG_SYS_MALLOC_LEN       0x00004000
#define CONFIG_SYS_MAXARGS          10
#define CONFIG_ENV_SIZE             0x00000400
#define CONFIG_ENV_ADDR
#define CONFIG_SYS_LOAD_ADDR        0x63000000
#endif

#ifdef MB9BF618T
#define CONFIG_MEM_RAM_BASE         0x1FFF0000
#define CONFIG_MEM_RAM_LEN          (74 * 1024)
#define CONFIG_MEM_RAM_BUF_LEN      (34 * 1024)
#define CONFIG_MEM_MALLOC_LEN       (16 * 1024)
#define CONFIG_MEM_STACK_LEN        (4 * 1024)

#define CONFIG_SYS_RAM_BASE         0x60000000
#define CONFIG_SYS_RAM_SIZE         0x00200000
#define CONFIG_SYS_MALLOC_LEN       0x00004000
#define CONFIG_SYS_MAXARGS          10
#define CONFIG_ENV_SIZE             0x00000400
#define CONFIG_ENV_ADDR
#define CONFIG_SYS_LOAD_ADDR        0x60000000
#endif

#define CONFIG_SYS_MEMTEST_START    CONFIG_SYS_RAM_BASE
#define CONFIG_SYS_MEMTEST_END      (CONFIG_SYS_RAM_BASE + CONFIG_SYS_RAM_SIZE)

// network configuration
#define CONFIG_NET_MULTI
#define CONFIG_FM3_ETH
#define CONFIG_FM3_ETH_CH0
#define CONFIG_ETHADDR  00:22:33:44:55:66

#define CONFIG_SYS_CBSIZE       256
#define CONFIG_SYS_PBSIZE       (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_NR_DRAM_BANKS    1

#define CONFIG_SYS_MAX_FLASH_SECT   10
#define CONFIG_SYS_MAX_FLASH_BANKS  1

#define CONFIG_BAUDRATE     115200
#define CONFIG_SYS_BAUDRATE_TABLE   { 9600, 19200, 38400, 57600, 115200 }

#define CONFIG_SYS_FLASH_BASE   0
#define CONFIG_ENV_IS_IN_FLASH
#define CONFIG_ENV_SIZE         (4 * 1024)
#define CONFIG_ENV_ADDR         0x20000
#define CONFIG_INFERNO          1
#define CONFIG_ENV_OVERWRITEE   1

#include <config_cmd_default.h>
#undef CONFIG_CMD_BOOTD
#undef CONFIG_CMD_CONSOLE
#undef CONFIG_CMD_ECHO
#undef CONFIG_CMD_EDITENV
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_IMI
#undef CONFIG_CMD_ITEST
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_MISC
#define CONFIG_CMD_NET
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_SOURCE
#undef CONFIG_CMD_XIMG

#endif /* WX_MP3PLCD_F_H_ */
