#ifndef __BOARD_H__
#define __BOARD_H__

#include "dram.h"
#include "sunxi_spi.h"
#include "sunxi_usart.h"
#include "sunxi_sdhci.h"

#define USART_DBG usart3_dbg

#define CONFIG_BOOT_SPINAND
#define CONFIG_BOOT_SDCARD
//#define CONFIG_BOOT_MMC

#define CONFIG_FATFS_CACHE_SIZE		 (CONFIG_DTB_LOAD_ADDR - SDRAM_BASE) // in bytes
#define CONFIG_SDMMC_SPEED_TEST_SIZE 1024 // (unit: 512B sectors)

#define CONFIG_CPU_FREQ 1200000000
#define CONFIG_MEM_SIZE 134217728	// 128MiB

// #define CONFIG_ENABLE_CPU_FREQ_DUMP

#define CONFIG_KERNEL_FILENAME  "zImage"
#define CONFIG_OPTEE_FILENAME   "optee.bin"
#define CONFIG_DTB_FILENAME	    "sun8i-duet3d-duetscreen-linux.dtb"
#define CONFIG_CMDLINE_FILENAME "cmdline.txt"

#define CONFIG_DRAM_SCRATCH_BUFFER (SDRAM_BASE)

#define CONFIG_OPTEE_LOAD_ADDR  (SDRAM_BASE + (27 * 1024 * 1024))
#define CONFIG_DTB_LOAD_ADDR	(SDRAM_BASE + (64 * 1024 * 1024))
#define CONFIG_KERNEL_LOAD_ADDR (SDRAM_BASE + (72 * 1024 * 1024))

// 128KB erase sectors, so place them starting from 2nd sector
#define CONFIG_SPINAND_DTB_ADDR	   (128 * 2048)
#define CONFIG_SPINAND_OPTEE_ADDR  (256 * 2048)
#define CONFIG_SPINAND_KERNEL_ADDR (512 * 2048)

extern dram_para_t	 ddr_param;
extern sunxi_usart_t USART_DBG;
extern sunxi_spi_t	 sunxi_spi0;

extern void board_init(void);
extern int	board_sdhci_init(void);

#endif
