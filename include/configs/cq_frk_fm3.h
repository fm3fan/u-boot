/*
 * cq_frk_fm3.h
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef CQ_FRK_FM3_H_
#define CQ_FRK_FM3_H_

#define MB9BF618T
#define WXMP3PLCDFL
//#define MB9BF506N

//#undef DEBUG  /* disable debug messages */

#define CONFIG_SYS_ARMCORTEXM3  /* ARM Cortex-M3 CPU core */
#define CONFIG_SYS_ARMCORTEXM4  /* ARM Cortex-M4 CPU core */
#define CONFIG_ARMCORTEXM3_SYSTICK_CPU

#define CONFIG_SYS_FM3          /* Fujitsu FM3 device */

#define CONFIG_FM3_SERIAL

#define CONFIG_DISPLAY_CPUINFO      1
#define CONFIG_DISPLAY_BOARDINFO    1
#define CONFIG_SYS_BOARD_REV_STR    "Rev 0.1"
#define CONFIG_SYS_PROMPT           "FM3> "     /* monitor prompt */

#define CONFIG_ARCH_CPU_INIT    /* CPU specific initialization */
/* ToDo: clock configuration */
#define CONFIG_SYS_HZ   1000    /* Number of clock ticks in 1 sec */

#ifdef MB9BF506N
#define CONFIG_MEM_NVM_BASE         0x00000000  /* internal flash base address */
#define CONFIG_MEM_NVM_LEN          0x00080000  /* internal flash size */
#define CONFIG_MEM_NVM_UBOOT_OFF    0x00000000  /* offset of u-boot on internal flash */

#define CONFIG_SYS_MAX_FLASH_BANKS  1
#define CONFIG_SYS_MAX_FLASH_SECT   10
#define CONFIG_SYS_FLASH_BASE       0x00000000

#define CONFIG_MEM_RAM_BASE         0x1FFF8000
#define CONFIG_MEM_RAM_LEN          (10 * 1024)
#define CONFIG_MEM_RAM_BUF_LEN      (34 * 1024)
#define CONFIG_MEM_MALLOC_LEN       (16 * 1024)
#define CONFIG_MEM_STACK_LEN        (4 * 1024)
/* external SRAM */
#define CONFIG_NR_DRAM_BANKS        1
#define CONFIG_SYS_RAM_BASE         0x63000000
#define CONFIG_SYS_RAM_SIZE         0x00200000
#define CONFIG_SYS_MALLOC_LEN       CONFIG_MEM_MALLOC_LEN
#define CONFIG_SYS_LOAD_ADDR        CONFIG_SYS_RAM_BASE
/* Env in flash */
#define CONFIG_ENV_IS_IN_FLASH
#define CONFIG_ENV_SIZE             (8 * 1024)
#define CONFIG_ENV_ADDR             0x00020000
#define CONFIG_INFERNO              1
#define CONFIG_ENV_OVERWRITE        1
#endif

#ifdef MB9BF618T
#define CONFIG_MEM_NVM_BASE         0x00000000  /* internal flash base address */
#define CONFIG_MEM_NVM_LEN          0x00100000  /* internal flash size */
#define CONFIG_MEM_NVM_UBOOT_OFF    0x00000000  /* offset of u-boot on internal flash */

#define CONFIG_SYS_MAX_FLASH_BANKS  1
#define CONFIG_SYS_MAX_FLASH_SECT   10
#define CONFIG_SYS_FLASH_BASE       0x00000000

#define CONFIG_MEM_RAM_BASE         0x1FFF0000
#define CONFIG_MEM_RAM_LEN          (40 * 1024)
#define CONFIG_MEM_RAM_BUF_LEN      (68 * 1024)
#define CONFIG_MEM_MALLOC_LEN       (16 * 1024)
#define CONFIG_MEM_STACK_LEN        (4 * 1024)
/* external SRAM */
#define CONFIG_NR_DRAM_BANKS        1
#define CONFIG_SYS_RAM_BASE         0x60000000
#define CONFIG_SYS_RAM_SIZE         0x00200000
#define CONFIG_SYS_MALLOC_LEN       CONFIG_MEM_MALLOC_LEN
#define CONFIG_SYS_LOAD_ADDR        CONFIG_SYS_RAM_BASE
/* Env in flash */
#define CONFIG_ENV_IS_IN_FLASH
#define CONFIG_ENV_SIZE             (8 * 1024)
#define CONFIG_ENV_ADDR             0x00020000
#define CONFIG_INFERNO              1
#define CONFIG_ENV_OVERWRITE        1
#endif

#define CONFIG_SYS_MEMTEST_START    CONFIG_SYS_RAM_BASE
#define CONFIG_SYS_MEMTEST_END      (CONFIG_SYS_RAM_BASE + CONFIG_SYS_RAM_SIZE)

// network configuration
#define CONFIG_NET_MULTI
#define CONFIG_FM3_ETH
#define CONFIG_FM3_ETH_CH0
#define CONFIG_ETHADDR  11:22:33:44:55:66

/* console IO buffer size */
#define CONFIG_SYS_CBSIZE       256
#define CONFIG_SYS_PBSIZE       (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_BAUDRATE     115200
#define CONFIG_SYS_BAUDRATE_TABLE   { 9600, 19200, 38400, 57600, 115200 }

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
#define CONFIG_CMDLINE_EDITING

/*
 * To save memory disable long help
 */
#undef CONFIG_SYS_LONGHELP

/*
 * Max number of command args
 */
#define CONFIG_SYS_MAXARGS      16

/*
 * Auto-boot sequence configuration
 */
#define CONFIG_BOOTDELAY        3
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_HOSTNAME         cq-frk-fm3
#define CONFIG_BOOTARGS         "console=ttyS0,115200 panic=10 earyprintk"
//#define CONFIG_BOOTCOMMAND      "run flashboot"
#define CONFIG_EXTRA_ENV_SETTINGS   \
    "ipaddr=192.168.1.160\0"        \
    "serverip=192.168.1.2\0"

#define CONFIG_SYS_CONSOLE_IS_IN_ENV
//#define CONFIG_EXTRA_ENV_SETTINGS   ""

/*
 * Linux kernel boot parameters configuration
 */
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_CMDLINE_TAG

#endif /* CQ_FRK_FM3_H_ */
