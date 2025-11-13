#include "main.h"
#include "fdt.h"
#include "ff.h"
#include "sunxi_gpio.h"
#include "sunxi_sdhci.h"
#include "sunxi_spi.h"
#include "sunxi_clk.h"
#include "sunxi_dma.h"
#include "sdmmc.h"
#include "arm32.h"
#include "reg-ccu.h"
#include "debug.h"
#include "board.h"
#include "barrier.h"

image_info_t image;
extern u32	 _start;
extern u32	 __spl_start;
extern u32	 __spl_end;
extern u32	 __spl_size;
extern u32	 __stack_srv_start;
extern u32	 __stack_srv_end;
extern u32	 __stack_ddr_srv_start;
extern u32	 __stack_ddr_srv_end;

/* Linux zImage Header */
#define LINUX_ZIMAGE_MAGIC 0x016f2818
typedef struct {
	unsigned int code[9];
	unsigned int magic;
	unsigned int start;
	unsigned int end;
} linux_zimage_header_t;

static int boot_image_setup(unsigned char *addr, unsigned int *entry)
{
	linux_zimage_header_t *zimage_header = (linux_zimage_header_t *)addr;

	if (zimage_header->magic == LINUX_ZIMAGE_MAGIC) {
		*entry = ((unsigned int)addr + zimage_header->start);
		return 0;
	}

	error("unsupported kernel image\r\n");

	return -1;
}

#if defined(CONFIG_BOOT_SDCARD) || defined(CONFIG_BOOT_MMC)
#define CHUNK_SIZE 0x20000

static int fatfs_loadimage(char *filename, BYTE *dest)
{
	FIL		 file;
	UINT	 byte_to_read = CHUNK_SIZE;
	UINT	 byte_read;
	UINT	 total_read = 0;
	FRESULT	 fret;
	int		 ret;
	uint32_t UNUSED_DEBUG start, time;

	fret = f_open(&file, filename, FA_OPEN_EXISTING | FA_READ);
	if (fret != FR_OK) {
		error("FATFS: open, filename: [%s]: error %d\r\n", filename, fret);
		ret = -1;
		goto open_fail;
	}

	start = time_ms();

	do {
		byte_read = 0;
		fret	  = f_read(&file, (void *)(dest), byte_to_read, &byte_read);
		dest += byte_to_read;
		total_read += byte_read;
	} while (byte_read >= byte_to_read && fret == FR_OK);

	time = time_ms() - start + 1;

	if (fret != FR_OK) {
		error("FATFS: read: error %d\r\n", fret);
		ret = -1;
		goto read_fail;
	}
	ret = 0;

read_fail:
	fret = f_close(&file);

	debug("FATFS: read in %" PRIu32 "ms at %.2fMB/S\r\n", time, (f32)(total_read / time) / 1024.0f);

open_fail:
	return ret;
}

static int load_sdcard(image_info_t *image)
{
	FATFS	fs;
	FRESULT fret;
	int		ret;
	u32 UNUSED_DEBUG	start;

#if defined(CONFIG_SDMMC_SPEED_TEST_SIZE) && LOG_LEVEL >= LOG_DEBUG
	u32 test_time;
	start = time_ms();
	sdmmc_blk_read(&card0, (u8 *)(SDRAM_BASE), 0, CONFIG_SDMMC_SPEED_TEST_SIZE);
	test_time = time_ms() - start;
	debug("SDMMC: speedtest %uKB in %" PRIu32 "ms at %" PRIu32 "KB/S\r\n", (CONFIG_SDMMC_SPEED_TEST_SIZE * 512) / 1024, test_time,
		  (CONFIG_SDMMC_SPEED_TEST_SIZE * 512) / test_time);
#endif // SDMMC_SPEED_TEST

	start = time_ms();
	/* mount fs */
	fret = f_mount(&fs, "", 1);
	if (fret != FR_OK) {
		error("FATFS: mount error: %d\r\n", fret);
		return -1;
	} else {
		debug("FATFS: mount OK\r\n");
	}

	info("FATFS: read %s addr=%x\r\n", image->of_filename, (unsigned int)image->of_dest);
	ret = fatfs_loadimage(image->of_filename, image->of_dest);
	if (ret)
		return ret;

	info("FATFS: read %s addr=%x\r\n", image->optee_filename, (unsigned int)image->optee_dest);
	ret = fatfs_loadimage(image->optee_filename, image->optee_dest);
	if (ret)
		return ret;

	info("FATFS: read %s addr=%x\r\n", image->filename, (unsigned int)image->dest);
	ret = fatfs_loadimage(image->filename, image->dest);
	if (ret)
		return ret;

	/* apply custom cmdline if present */
	char *cmdline_buf = (char *)CONFIG_DRAM_SCRATCH_BUFFER;
	FIL	  cmdline_file;
	UINT bytes_read;

	fret = f_open(&cmdline_file, "cmdline.txt", FA_OPEN_EXISTING | FA_READ);
	if (fret == FR_OK) {
		memset(cmdline_buf, 0, CMDLINE_MAX_LEN);
		fret = f_read(&cmdline_file, cmdline_buf, CMDLINE_MAX_LEN - 1, &bytes_read);
		f_close(&cmdline_file);

		if (fret == FR_OK && bytes_read > 0) {
			/* strip trailing whitespace/newlines from cmdline */
			while (bytes_read > 0 && 
				   (cmdline_buf[bytes_read - 1] == '\n' || 
				    cmdline_buf[bytes_read - 1] == '\r' || 
				    cmdline_buf[bytes_read - 1] == ' ' ||
				    cmdline_buf[bytes_read - 1] == '\t')) {
				cmdline_buf[bytes_read - 1] = '\0';
				bytes_read--;
			}
			
			/* update bootargs */
			info("FATFS: applying custom cmdline from cmdline.txt\r\n");
			int ret = fixup_chosen_node(image->of_dest, cmdline_buf);
			if (ret < 0) {
				warning("FATFS: failed to set bootargs\r\n");
			} else {
				debug("FATFS: bootargs updated successfully\r\n");
			}
		}
	}

	/* umount fs */
	fret = f_mount(0, "", 0);
	if (fret != FR_OK) {
		error("FATFS: unmount error %d\r\n", fret);
		return -1;
	} else {
		debug("FATFS: unmount OK\r\n");
	}
	debug("FATFS: done in %" PRIu32 "ms\r\n", time_ms() - start);

	return 0;
}

#endif

#ifdef CONFIG_BOOT_SPINAND
int load_spi_nand(sunxi_spi_t *spi, image_info_t *image)
{
	linux_zimage_header_t *hdr;
	unsigned int		   size;
	uint64_t UNUSED_DEBUG	   start, time;

	if (spi_nand_detect(spi) != 0)
		return -1;

	/* get dtb size and read */
	spi_nand_read(spi, image->of_dest, CONFIG_SPINAND_DTB_ADDR, (uint32_t)sizeof(boot_param_header_t));
	if (of_get_magic_number(image->of_dest) != OF_DT_MAGIC) {
		error("SPI-NAND: DTB verification failed\r\n");
		return -1;
	}

	size = of_get_dt_total_size(image->of_dest);
	debug("SPI-NAND: dt blob: Copy from 0x%08x to 0x%08lx size:0x%08x\r\n", CONFIG_SPINAND_DTB_ADDR,
		  (uint32_t)image->of_dest, size);
	start = time_us();
	spi_nand_read(spi, image->of_dest, CONFIG_SPINAND_DTB_ADDR, (uint32_t)size);
	time = time_us() - start;
	info("SPI-NAND: read dt blob of size %u at %.2fMB/S\r\n", size, (f32)(size / time));

	/* get optee and read */
	size = CONFIG_SPINAND_KERNEL_ADDR - CONFIG_SPINAND_OPTEE_ADDR;
	debug("SPI-NAND: optee: Copy from 0x%08x to 0x%08lx size:0x%08x\r\n", CONFIG_SPINAND_OPTEE_ADDR,
		  (uint32_t)image->optee_dest, size);
	start = time_us();
	spi_nand_read(spi, image->optee_dest, CONFIG_SPINAND_OPTEE_ADDR, (uint32_t)size);
	time = time_us() - start;
	info("SPI-NAND: read optee of size %u at %.2fMB/S\r\n", size, (f32)(size / time));

	/* get kernel size and read */
	spi_nand_read(spi, image->dest, CONFIG_SPINAND_KERNEL_ADDR, (uint32_t)sizeof(linux_zimage_header_t));
	hdr = (linux_zimage_header_t *)image->dest;
	if (hdr->magic != LINUX_ZIMAGE_MAGIC) {
		debug("SPI-NAND: zImage verification failed\r\n");
		return -1;
	}
	size = hdr->end - hdr->start;
	debug("SPI-NAND: Image: Copy from 0x%08x to 0x%08lx size:0x%08x\r\n", CONFIG_SPINAND_KERNEL_ADDR,
		  (uint32_t)image->dest, size);
	start = time_us();
	spi_nand_read(spi, image->dest, CONFIG_SPINAND_KERNEL_ADDR, (uint32_t)size);
	time = time_us() - start;
	info("SPI-NAND: read Image of size %u at %.2fMB/S\r\n", size, (f32)(size / time));

	return 0;
}
#endif

// This is basically copied from nboot, although dtb isn't relative to next here
void boot0_jmp_optee(void *optee, void *dtb, void *next)
{
	asm volatile ("mov r2, %0" : : "r" (dtb) : "memory");
	asm volatile ("mov lr, %0" : : "r" (next) : "memory");
	asm volatile ("bx      %0" : : "r" (optee) : "memory");
}

int main(void)
{
	board_init();
	sunxi_clk_init();

	message("\r\n");
	info("AWBoot r%" PRIu32 " starting...\r\n", (u32)BUILD_REVISION);

	sunxi_dram_init();

#ifdef CONFIG_ENABLE_CPU_FREQ_DUMP
	sunxi_clk_dump();
#endif

	memset(&image, 0, sizeof(image_info_t));

	image.of_dest    = (u8 *)CONFIG_DTB_LOAD_ADDR;
	image.optee_dest = (u8 *)CONFIG_OPTEE_LOAD_ADDR;
	image.dest       = (u8 *)CONFIG_KERNEL_LOAD_ADDR;

#if defined(CONFIG_BOOT_SDCARD) || defined(CONFIG_BOOT_MMC)

	strcpy(image.filename, CONFIG_KERNEL_FILENAME);
	strcpy(image.optee_filename, CONFIG_OPTEE_FILENAME);
	strcpy(image.of_filename, CONFIG_DTB_FILENAME);

	if (sunxi_sdhci_init(&sdhci0) != 0) {
		fatal("SMHC: %s controller init failed\r\n", sdhci0.name);
	} else {
		info("SMHC: %s controller v%" PRIx32 " initialized\r\n", sdhci0.name, sdhci0.reg->vers);
	}
	if (sdmmc_init(&card0, &sdhci0) != 0) {
#ifdef CONFIG_BOOT_SPINAND
		warning("SMHC: init failed, trying SPI\r\n");
		goto _spi;
#else
		fatal("SMHC: init failed\r\n");
#endif
	}

#ifdef CONFIG_BOOT_SPINAND
	if (load_sdcard(&image) != 0) {
		warning("SMHC: loading failed, trying SPI\r\n");
	} else {
		goto _boot;
	}
#else
	if (load_sdcard(&image) != 0) {
		fatal("SMHC: card load failed\r\n");
	} else {
		goto _boot;
	}
#endif // CONFIG_SPI_NAND
#endif

#ifdef CONFIG_BOOT_SPINAND
#if defined(CONFIG_BOOT_SDCARD) || defined(CONFIG_BOOT_MMC)
_spi:
#endif
	dma_init();
	dma_test();
	debug("SPI: init\r\n");
	if (sunxi_spi_init(&sunxi_spi0) != 0) {
		fatal("SPI: init failed\r\n");
	}

	if (load_spi_nand(&sunxi_spi0, &image) != 0) {
		fatal("SPI-NAND: loading failed\r\n");
	}

	sunxi_spi_disable(&sunxi_spi0);
	dma_exit();

#endif // CONFIG_SPI_NAND

#if defined(CONFIG_BOOT_SDCARD) || defined(CONFIG_BOOT_MMC)
_boot:
#endif
	/* Fix up memory info */
	info("Fixing memory node in FDT...\r\n");
	fixup_memory_node(image.of_dest, (unsigned int)SDRAM_BASE, (unsigned int)CONFIG_MEM_SIZE);

	/* Boot through OP-TEE if loaded from SPI-NAND */
	info("Initializing OP-TEE...\r\n");

	/* Write the "RAW" signature that OP-TEE checks at offset 0xED
	 * This is required for hardware validation
	 * The values come from the working system's OP-TEE binary from a memory dump
	 */
	volatile uint8_t *optee_sig = (volatile uint8_t *)((uint32_t)image.optee_dest + 0xED);
	optee_sig[0]				= 0x52; // 'R'
	optee_sig[1]				= 0x41; // 'A'
	optee_sig[2]				= 0x57; // 'W'
	optee_sig[3]				= 0x89;
	optee_sig[4]				= 0xE9;

	/* Disable MMU/caches BEFORE jumping to OP-TEE (like nboot does) */
	arm32_mmu_disable();
	arm32_dcache_disable();
	arm32_icache_disable();
	arm32_interrupt_disable();

	/* Create ARM trampoline that boots Linux after OPTEE returns
	 * Pass 0x40000001 (with Thumb bit) to make OPTEE accept it, but
	 * write ARM code at 0x40000000. OPTEE returns in ARM mode.
	 * We cannot make OPTEE return to memory addresses in awboot.
	 */
	volatile uint32_t *trampoline = (volatile uint32_t *)CONFIG_DRAM_SCRATCH_BUFFER;
	int idx = 0;
	
	// Set up Linux boot arguments: r0=0, r1=machine_id, r2=DTB
	uint32_t dtb_addr = (uint32_t)image.of_dest;
	trampoline[idx++] = 0xe3a00000;  // mov r0, #0
	trampoline[idx++] = 0xe3e01000;  // mvn r1, #0  ; r1 = 0xFFFFFFFF (invalid machine ID to force DT)
	trampoline[idx++] = 0xe3002000 | ((dtb_addr >> 0) & 0xFFF) | (((dtb_addr >> 12) & 0xF) << 16);  // movw r2, #low16(dtb)
	trampoline[idx++] = 0xe3402000 | ((dtb_addr >> 16) & 0xFFF) | (((dtb_addr >> 28) & 0xF) << 16); // movt r2, #high16(dtb)
	
	// Jump to kernel entry point
	uint32_t kernel_addr = (uint32_t)image.dest;
	trampoline[idx++] = 0xe3000000 | ((kernel_addr >> 0) & 0xFFF) | (((kernel_addr >> 12) & 0xF) << 16);  // movw r3, #low16(kernel)
	trampoline[idx++] = 0xe3403000 | ((kernel_addr >> 16) & 0xFFF) | (((kernel_addr >> 28) & 0xF) << 16); // movt r3, #high16(kernel)
	trampoline[idx++] = 0xe12fff13;  // bx r3
	
	info("ARM trampoline created at 0x40000000 (will boot Linux at 0x%08lx)\r\n", (unsigned long)image.dest);
	
	/* Jump to OP-TEE for initialization */
	info("Jumping to OP-TEE, will return to 0x40000001...\r\n");
	boot0_jmp_optee(image.optee_dest, image.of_dest, (void *)(CONFIG_DRAM_SCRATCH_BUFFER + 1));

	/* OP-TEE returned, nothing we would expect */
	fatal("OP-TEE returned unexpectedly!\r\n");
	return 0;
}
