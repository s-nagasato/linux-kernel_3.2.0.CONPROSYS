/*
 * Code for MC341.
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
// update 2015.02.03 XDMA_EVENT_INTRO config update
// update 2015.02.04 LAN_INTn, RTC_INTn init set (DBG-SW add)
// update 2015.02.04 UART3, SPI1 init
// update 2015.02.05 uart3 update
// update 2015.02.06 2LAN setup
// update 2015.02.09 spidev test
// 2015.2.11 add usb
// update 2015.02.12 max_speed 6Mbps
// update 2015.02.18 am33xx_cpsw_init
// update 2015.02.18 SPI2.0 // .mode = SPI_MODE_2,
// update 2015.02.25 usb on
// update 2015.02.26 uart5 update
// update 2015.03.02 3G power
// update 2015.03.03 LED update
// update 2015.03.05 gpio1_7
// update 2015.03.05 CM_CLOCKOUT_CTRL
// update 2015.03.05 RTC mcp7940
// update 2015.03.09 nmi set
// update 2015.03.16 RFID add
// update 2015.07.03 option enable dcan (disable uart1)
//                   update Kconfig


//#define MC341LAN2 (1)
#define MC341
#ifndef MC341
/*
*/
#endif

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/i2c/at24.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/wl12xx.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/mfd/tps65217.h>
#include <linux/pwm_backlight.h>
#include <linux/input/ti_tscadc.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/opp.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
// #include <mach/board-am335xevm.h>
#include <mach/board-mc341.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>

#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"
#include "cm33xx.h"

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

/* BBB PHY IDs */
#define BBB_PHY_ID		0x7c0f1
#define BBB_PHY_MASK		0xfffffffe

/* AM335X EVM Phy ID and Debug Registers */
#define AM335X_EVM_PHY_ID		0x4dd074
#define AM335X_EVM_PHY_MASK		0xfffffffe
#define AR8051_PHY_DEBUG_ADDR_REG	0x1d
#define AR8051_PHY_DEBUG_DATA_REG	0x1e
#define AR8051_DEBUG_RGMII_CLK_DLY_REG	0x5
#define AR8051_RGMII_TX_CLK_DLY		BIT(8)

// add 2014.11.25
/* TLK PHY IDs */
// #define TLK110_PHY_ID 0x2000A201
// TLK105 0x2000a211
#define TLK110_PHY_ID   0x2000A201
#define TLK105_PHY_ID   0x2000A211
#define TLK110_PHY_MASK 0xfffffff0

/* TLK110 PHY register offsets */
#define TLK110_COARSEGAIN_REG 0x00A3
#define TLK110_LPFHPF_REG 0x00AC
#define TLK110_SPAREANALOG_REG 0x00B9
#define TLK110_VRCR_REG 0x00D0
#define TLK110_SETFFE_REG 0x0107
#define TLK110_FTSP_REG 0x0154
#define TLK110_ALFATPIDL_REG 0x002A
#define TLK110_PSCOEF21_REG 0x0096
#define TLK110_PSCOEF3_REG 0x0097
#define TLK110_ALFAFACTOR1_REG 0x002C
#define TLK110_ALFAFACTOR2_REG 0x0023
#define TLK110_CFGPS_REG 0x0095
#define TLK110_FTSPTXGAIN_REG 0x0150
#define TLK110_SWSCR3_REG 0x000B
#define TLK110_SCFALLBACK_REG 0x0040
#define TLK110_PHYRCR_REG 0x001F

/* TLK110 register writes values */
#define TLK110_COARSEGAIN_VAL 0x0000
#define TLK110_LPFHPF_VAL 0x8000
#define TLK110_SPANALOG_VAL 0x0000
#define TLK110_VRCR_VAL 0x0008
#define TLK110_SETFFE_VAL 0x0605
#define TLK110_FTSP_VAL 0x0255
#define TLK110_ALFATPIDL_VAL 0x7998
#define TLK110_PSCOEF21_VAL 0x3A20
#define TLK110_PSCOEF3_VAL 0x003F
#define TLK110_ALFACTOR1_VAL 0xFF80
#define TLK110_ALFACTOR2_VAL 0x021C
#define TLK110_CFGPS_VAL 0x0000
#define TLK110_FTSPTXGAIN_VAL 0x6A88
#define TLK110_SWSCR3_VAL 0x0000
#define TLK110_SCFALLBACK_VAL 0xC11D
#define TLK110_PHYRCR_VAL 0x4000

#ifdef CONFIG_TLK110_WORKAROUND
#define mc341_tlk110_phy_init()\
 phy_register_fixup_for_uid(TLK105_PHY_ID, TLK110_PHY_MASK, mc341_tlk110_phy_fixup);
/*
 do { \
 phy_register_fixup_for_uid(TLK105_PHY_ID,\
 TLK110_PHY_MASK,\
 mc341_tlk110_phy_fixup);\
 } while (0);
*/
#else
#define mc341_tlk110_phy_init() do { } while (0);
#endif


#define BEAGLEBONE_LCD_AVDD_EN GPIO_TO_PIN(0, 7)
#define BEAGLEBONE_LCD_BL GPIO_TO_PIN(1, 18)

#define AM33XX_CTRL_REGADDR(reg)				\
		AM33XX_L4_WK_IO_ADDRESS(AM33XX_SCM_BASE + (reg))

// bit 3: 0 - enable, 1 - disable for pull enable
#define AM33XX_PULL_DISA		(1 << 3)
#define AM33XX_PULL_ENBL		(0 << 3)

int selected_pad;
int pad_mux_value;

static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};
static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			= 32,
	.fdd			= 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 0,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 1,
	.invert_frm_clock	= 1,
	.sync_edge		= 0,
	.sync_ctrl		= 1,
	.raster_order		= 0,
};
struct da8xx_lcdc_platform_data  NHD_480272MF_ATXI_pdata = {
	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "NHD-4.3-ATXI#-T-1",
};

#include "common.h"

static struct omap2_hsmmc_info mc341_mmc[] __initdata = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = GPIO_TO_PIN(0, 6),
		.gpio_wp        = GPIO_TO_PIN(3, 18),
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}      /* Terminator */
};


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/*
	 * Setting SYSBOOT[5] should set xdma_event_intr0 pin to mode 3 thereby
	 * allowing clkout1 to be available on xdma_event_intr0.
	 * However, on some boards (like EVM-SK), SYSBOOT[5] isn't properly
	 * latched.
	 * To be extra cautious, setup the pin-mux manually.
	 * If any modules/usecase requries it in different mode, then subsequent
	 * module init call will change the mux accordingly.
	 */
	// update 2015.02.03 XDMA_EVENT_INTRO config update
	// AM33XX_MUX(XDMA_EVENT_INTR0, OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT),
	
//	AM33XX_MUX(XDMA_EVENT_INTR0, OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP), // MC341LAN1
//	{"xdma_event_intr0.spi1_cs1", OMAP_MUX_MODE4 | AM33XX_PIN_OUTPUT},		/* SPI1_CS1 */ // MC341LAN2
#ifndef CONFIG_OMAP_MC341B00
	AM33XX_MUX(XDMA_EVENT_INTR0, OMAP_MUX_MODE4 | AM33XX_PIN_OUTPUT), // MC341LAN2
#endif 
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
// for i2c1
	AM33XX_MUX(UART0_CTSN, OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(UART0_RTSN, OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

struct evm_dev_cfg {
	void (*device_init)(int evm_id, int profile);

/*
* If the device is required on both baseboard & daughter board (ex i2c),
* specify DEV_ON_BASEBOARD
*/
#define DEV_ON_BASEBOARD	0
#define DEV_ON_DGHTR_BRD	1
	u32 device_on;

	u32 profile;	/* Profiles (0-7) in which the module is present */
};

/* AM335X - CPLD Register Offsets */
#define	CPLD_DEVICE_HDR	0x00 /* CPLD Header */
#define	CPLD_DEVICE_ID	0x04 /* CPLD identification */
#define	CPLD_DEVICE_REV	0x0C /* Revision of the CPLD code */
#define	CPLD_CFG_REG	0x10 /* Configuration Register */

static struct i2c_client *cpld_client;
// static u32 am335x_evm_id;
static struct omap_board_config_kernel am335x_evm_config[] __initdata = {
};

/*
* EVM Config held in On-Board eeprom device.
*
* Header Format
*
*  Name			Size	Contents
*			(Bytes)
*-------------------------------------------------------------
*  Header		4	0xAA, 0x55, 0x33, 0xEE
*
*  Board Name		8	Name for board in ASCII.
*				Example "A33515BB" = "AM335x 15x15 Base Board"
*
*  Version		4	Hardware version code for board	in ASCII.
*				"1.0A" = rev.01.0A
*
*  Serial Number	12	Serial number of the board. This is a 12
*				character string which is WWYY4P16nnnn, where
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*
*  Configuration option	32	Codes(TBD) to show the configuration
*				setup on this board.
*
*  Available		32720	Available space for other non-volatile data.
*/
/*
*/
struct am335x_evm_eeprom_config {
	u32	header;
	u8	name[8];
	char	version[4];
	u8	serial[12];
	u8	opt[32];
};
/*
* EVM Config held in daughter board eeprom device.
*
* Header Format
*
*  Name			Size		Contents
*			(Bytes)
*-------------------------------------------------------------
*  Header		4	0xAA, 0x55, 0x33, 0xEE
*
*  Board Name		8	Name for board in ASCII.
*				example "A335GPBD" = "AM335x
*				General Purpose Daughterboard"
*
*  Version		4	Hardware version code for board in
*				in ASCII. "1.0A" = rev.01.0A
*  Serial Number	12	Serial number of the board. This is a 12
*				character string which is: WWYY4P13nnnn, where
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*  Configuration Option	32	Codes to show the configuration
*				setup on this board.
*  CPLD Version	8		CPLD code version for board in ASCII
*				"CPLD1.0A" = rev. 01.0A of the CPLD
*  Available	32700		Available space for other non-volatile
*				codes/data
*/
/*
struct am335x_eeprom_config1 {
	u32	header;
	u8	name[8];
	char	version[4];
	u8	serial[12];
	u8	opt[32];
	u8	cpld_ver[8];
};
*/

static struct am335x_evm_eeprom_config config;
// static struct am335x_eeprom_config1 config1;

struct mc341_eeprom_config {
	u32	header;		// 0xee3355aa
	u8	name[8];	// AM335X03
	u8	clock_mem;	// 0x11 = (0x10 600) + (0x01 128)
					// CPU:0x80 1000, 0x40 800, 0x10 600
					// MEM:0x02 512, 0x00 256, 0x01 128
	u8	display;	//  0x00 CMOS OUT DISABLE LVDS OUT DISABLE
	u8	mac0[6];	// mac address
	u8	mac1[6];	// mac address
	u8	ssp;		// Select Spread Spectrum Clocking 0x00 None
	u8	debport;	// 
	u8	serial[14];	// x(7)9(6)+0x0
};
static struct mc341_eeprom_config config_mc341;

static bool daughter_brd_detected;

// #define EEPROM_MAC_ADDRESS_OFFSET	60 /* 4+8+4+12+32 */
#define EEPROM_MAC_ADDRESS_OFFSET	14 /* 4+8+1+1 */
// #define EEPROM_NO_OF_MAC_ADDR		3
#define EEPROM_NO_OF_MAC_ADDR		2
static char am335x_mac_addr[EEPROM_NO_OF_MAC_ADDR][ETH_ALEN];

#define AM335X_EEPROM_HEADER		0xEE3355AA

static int am33xx_evmid = -EINVAL;

/*
* mc341_set_id - set up board evmid
* @evmid - evm id which needs to be configured
*
* This function is called to configure board evm id.
*/
void mc341_set_id(unsigned int evmid)
{
	am33xx_evmid = evmid;
	return;
}

/*
* mc341_get_id - returns Board Type (EVM/BB/EVM-SK ...)
*
* Note:
*	returns -EINVAL if Board detection hasn't happened yet.
*/
int mc341_get_id(void)
{
	return am33xx_evmid;
}
EXPORT_SYMBOL(mc341_get_id);

// Module pin mux for LCDC
static struct pinmux_config lcdc_pin_mux[] = {
	{NULL, 0},
};

/* Module pin mux for rmii1 */
static struct pinmux_config rmii1_pin_mux[] = {
	{"mii1_crs.rmii1_crs_dv", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"rmii1_refclk.rmii1_refclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};
//MC341LAN2
/* Module pin mux for rmii2 */
static struct pinmux_config rmii2_pin_mux[] = {
	{"gpmc_wait0.rmii2_crs_dv", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_wpn.rmii2_rxerr", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a0.rmii2_txen", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{"gpmc_a4.rmii2_txd1", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{"gpmc_a5.rmii2_txd0", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{"gpmc_a10.rmii2_rxd1", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a11.rmii2_rxd0", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_col.rmii2_refclk", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};


/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_common_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

// static struct pinmux_config mmc0_wp_only_pin_mux[] = {
//	{"ecap0_in_pwm0_out.mmc0_sdwp", OMAP_MUX_MODE5 | AM33XX_PIN_INPUT_PULLUP},
//	{NULL, 0},
// };

static struct pinmux_config mmc0_cd_only_pin_mux[] = {
	{"spi0_cs1.mmc0_sdcd",  OMAP_MUX_MODE5 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};
// pinmux for gpio based key
static struct pinmux_config gpio_keys_pin_mux[] = {
	{"gpmc_ad0.gpio1_0", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},			/* DBG-SW2 */
	{"gpmc_ad1.gpio1_1", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},			/* DBG-SW3 */
	{"gpmc_ad2.gpio1_2", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},			/* DBG-SW4 */
	{"gpmc_ad7.gpmc_ad7", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT}, // RS(Half/full) 2015.01.16
	{"mii1_txd2.gpio0_17", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},/* RTC_INTn */
	{NULL, 0},
};
// pinmux for led device
static struct pinmux_config gpio_led_mux[] = {
	{"gpmc_ad10.gpio0_26", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},			/* LED_ST0n */
	{"gpmc_ad11.gpio0_27", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},			/* LED_ST1n */
        // update 2015.03.02 3G power
	{"gpmc_ad4.gpio1_4",    OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, // 3G power
	{"gpmc_ad5.gpio1_5",    OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT}, // 3G reset
	{"gpmc_oen_ren.gpio2_3", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},		/* F-LED_PWn */
	{NULL, 0},
};

// MC341LAN2
static struct pinmux_config mc341_lan1_model[] = {
	{"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},		/* SHUTDOWN-SWn */
	{"mcasp0_aclkr.gpio3_18", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},		/* PW_RST */
	{"gpmc_ad6.gpio1_6", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},			/* LAN_SPEED_LED */
	{"mii1_txd3.gpio0_16", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},/* LAN_INTn */
	{NULL, 0},
};
static struct pinmux_config mc341_lan2_model[] = {
	{"mii1_txd3.gpio0_16", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},			/* GPIO_INIT_END */
	{"mii1_rxdv.gpio3_4", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},			/* SPI1_UFM-SN */

	{"gpmc_ad3.gpio1_3", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},			/* SHUTDOWN-SWn */
	{"mii1_txclk.gpio3_9", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},			/* PW_RST */
	{"mcasp0_fsr.gpio3_19", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},		/* LAN_SPEED_LED-A */
	{"mcasp0_axr1.gpio3_20", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},		/* LAN_SPEED_LED-B */
	{"mcasp0_aclkr.gpio3_18", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},		/* LAN-A_INTn */
	{"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},		/* LAN-B_INTn */
	{NULL, 0},
};


/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);

}


/*
* @evm_id - evm id which needs to be configured
* @dev_cfg - single evm structure which includes
*				all module inits, pin-mux defines
* @profile - if present, else PROFILE_NONE
* @dghtr_brd_flg - Whether Daughter board is present or not
*/
static void _configure_device(int evm_id, struct evm_dev_cfg *dev_cfg,
	int profile)
{
	int i;

	mc341_set_id(evm_id);

	/*
	* Only General Purpose & Industrial Auto Motro Control
	* EVM has profiles. So check if this evm has profile.
	* If not, ignore the profile comparison
	*/

	/*
	* If the device is on baseboard, directly configure it. Else (device on
	* Daughter board), check if the daughter card is detected.
	*/
	if (profile == PROFILE_NONE) {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->device_on == DEV_ON_BASEBOARD)
				dev_cfg->device_init(evm_id, profile);
			else if (daughter_brd_detected == true)
				dev_cfg->device_init(evm_id, profile);
		}
	} else {
		for (i = 0; dev_cfg->device_init != NULL; dev_cfg++) {
			if (dev_cfg->profile & profile) {
				if (dev_cfg->device_on == DEV_ON_BASEBOARD)
					dev_cfg->device_init(evm_id, profile);
				else if (daughter_brd_detected == true)
					dev_cfg->device_init(evm_id, profile);
			}
		}
	}
}

#define AM335XEVM_WLAN_PMENA_GPIO	GPIO_TO_PIN(1, 30)
#define AM335XEVM_WLAN_IRQ_GPIO		GPIO_TO_PIN(3, 17)
#define AM335XEVM_SK_WLAN_IRQ_GPIO      GPIO_TO_PIN(0, 31)

struct wl12xx_platform_data am335xevm_wlan_data = {
	.irq = OMAP_GPIO_IRQ(AM335XEVM_WLAN_IRQ_GPIO),
#ifdef CONFIG_MACH_AM335XEVM_WILINK8
        .board_ref_clock = WL12XX_REFCLOCK_38,
        .board_tcxo_clock = WL12XX_TCXOCLOCK_26,
#else
	.board_ref_clock = WL12XX_REFCLOCK_38_XTAL, /* 38.4Mhz */
#endif
	.bt_enable_gpio = GPIO_TO_PIN(3, 21),
	.wlan_enable_gpio = GPIO_TO_PIN(1, 16),
};


static struct pinmux_config uart1_wl12xx_pin_mux[] = {
	{"uart1_rxd.uart1_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},		/* UART1_RXD RS485 */
	{"uart1_txd.uart1_txd", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},			/* UART1_TXD RS485 */
//	{"uart1_rtsn.uart1_rtsn", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},	/* UART1_RTSn RS485 */
//	{"uart1_ctsn.uart1_ctsn", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},			/* UART1_CTSn RS485 */
	{"uart1_rtsn.uart1_rtsn", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},	/* UART1_RTSn RS485 */
	{"uart1_ctsn.uart1_ctsn", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},			/* UART1_CTSn RS485 */
// update 2015.03.05 gpio1_7
	{"gpmc_ad7.gpio1_7", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},	/* UART1 mode */
// update 2015.03.09 nmi set
// moved 2015.07.21 
//	{"nmin.nmin", OMAP_MUX_MODE0 },	/* NMI in */
	{NULL, 0},
};
//update 2015.07.03
static struct pinmux_config mc341_dcan_pin_mux[] = {
//	{"uart1_rxd.d_can1_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
	{"uart1_rxd.d_can1_tx", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT_PULLUP},
	{"uart1_txd.d_can1_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
//	{"uart1_ctsn.d_can0_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
	{"uart1_ctsn.d_can0_tx", OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT_PULLUP},
	{"uart1_rtsn.d_can0_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}


static void lcdc_init(int evm_id, int profile)
{
	struct da8xx_lcdc_platform_data *lcdc_pdata;
	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(300000000)) {
		pr_info("Failed configure display PLL, not attempting to"
				"register LCDC\n");
		return;
	}
	switch (evm_id) {
//	case GEN_PURP_EVM:
//		lcdc_pdata = &TFC_S9700RTWV35TR_01B_pdata;
//		break;
	case EVM_SK:
		lcdc_pdata = &NHD_480272MF_ATXI_pdata;
		break;
	default:
		pr_err("LCDC not supported on this evm (%d)\n",evm_id);
		return;
	}

	if (am33xx_register_lcdc(lcdc_pdata))
		pr_info("Failed to register LCDC device\n");

	return;
}

static void rmii1_init(int evm_id, int profile)
{
	setup_pin_mux(rmii1_pin_mux);
// printk(KERN_WARNING "[%s](%d)rmii1_init in!!!",__FILE__,__LINE__); // 12.23
	return;
}



/* SPI 0/1 Platform Data */
/* SPI flash information */
static struct mtd_partition am335x_spi_partitions[] = {
	/* All the partition sizes are listed in terms of erase size */
	{
		.name       = "SPL",
		.offset     = 0,			/* Offset = 0x0 */
		.size       = SZ_128K,
	},
	{
		.name       = "U-Boot",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size       = 4 * SZ_128K,
	},
	{
		.name       = "U-Boot Env",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0xa0000 */
		.size       = 2 * SZ_128K,
	},
	{
		.name       = "Kernel",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0xE0000 */
		.size       = 27 * SZ_128K,
	},
	{
		.name       = "Root File System",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x442000 */
		.size       = 76 * SZ_128K,		/* size ~= 10 MiB */
	},
	{
		.name       = "APL Area",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0xDD0000 */
		.size       = MTDPART_SIZ_FULL,		/* size ~= 20 MiB */
	}
};

static const struct flash_platform_data mc341_spi_flash = {
	// .type      = "w25q64", // org
	.type      = "n25q256a",
	.name      = "spi_flash",
	.parts     = am335x_spi_partitions,
	.nr_parts  = ARRAY_SIZE(am335x_spi_partitions),
};

static void uart1_wl12xx_init(int evm_id, int profile)
{
	setup_pin_mux(uart1_wl12xx_pin_mux);
}

// update 2015.07.03
static void mc341_dcan_init(int evm_id, int profile)
{
	setup_pin_mux(mc341_dcan_pin_mux);
	am33xx_d_can_init(0);
	am33xx_d_can_init(1);
}

#ifdef CONFIG_TI_ST
/* TI-ST for WL12xx BT */

/* Bluetooth Enable PAD for EVM Rev 1.1 and up */
#define AM33XX_CONTROL_PADCONF_MCASP0_AHCLKX_OFFSET		0x09AC

/* Bluetooth Enable PAD for EVM Rev 1.0 */
#define AM33XX_CONTROL_PADCONF_GPMC_CSN2_OFFSET			0x0884

int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}

int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

int plat_kim_chip_enable(struct kim_data_s *kim_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

	/* Configure BT_EN pin so that suspend/resume works correctly on rev 1.1 */
	selected_pad = AM33XX_CONTROL_PADCONF_MCASP0_AHCLKX_OFFSET;
	/* Configure BT_EN pin so that suspend/resume works correctly on rev 1.0 */
	/*selected_pad = AM33XX_CONTROL_PADCONF_GPMC_CSN2_OFFSET;*/

	gpio_direction_output(kim_data->nshutdown, 0);
	msleep(1);
	gpio_direction_output(kim_data->nshutdown, 1);

	/* Enable pullup on the enable pin for keeping BT active during suspend */
	pad_mux_value = readl(AM33XX_CTRL_REGADDR(selected_pad));
	pad_mux_value &= (~AM33XX_PULL_DISA);
	writel(pad_mux_value, AM33XX_CTRL_REGADDR(selected_pad));

	return 0;
}

int plat_kim_chip_disable(struct kim_data_s *kim_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

	gpio_direction_output(kim_data->nshutdown, 0);

	/* Disable pullup on the enable pin to allow BT shut down during suspend */
	pad_mux_value = readl(AM33XX_CTRL_REGADDR(selected_pad));
	pad_mux_value |= AM33XX_PULL_DISA;
	writel(pad_mux_value, AM33XX_CTRL_REGADDR(selected_pad));

	return 0;
}

struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = GPIO_TO_PIN(3, 21),
	.dev_name = "/dev/ttyO1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable,
};

static struct platform_device wl12xx_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static inline void __init am335xevm_init_btwilink(void)
{
	pr_info("am335xevm: bt init\n");

	platform_device_register(&wl12xx_device);
	platform_device_register(&btwilink_device);
}
#endif


static void mmc0_init(int evm_id, int profile)
{
// printk(KERN_WARNING "[%s](%d)mmc0_init in!!!",__FILE__,__LINE__); // 12.23
/* 1231 
	switch (evm_id) {
	case BEAGLE_BONE_A3:
	case BEAGLE_BONE_OLD:
	case EVM_SK:
		setup_pin_mux(mmc0_common_pin_mux);
		setup_pin_mux(mmc0_cd_only_pin_mux);
		break;
	default:
		setup_pin_mux(mmc0_common_pin_mux);
		setup_pin_mux(mmc0_cd_only_pin_mux);
		setup_pin_mux(mmc0_wp_only_pin_mux);
		break;
	}
*/
	setup_pin_mux(mmc0_common_pin_mux);
	setup_pin_mux(mmc0_cd_only_pin_mux);

	omap2_hsmmc_init(mc341_mmc);
	return;
}


// Configure GPIOs for GPIO Keys 
static struct gpio_keys_button mc341_gpio_buttons[] = {
	{
		.code                   = BTN_1,
		.gpio                   = GPIO_TO_PIN(1, 0),
		.desc                   = "SW2",
	},
	{
		.code                   = BTN_2,
		.gpio                   = GPIO_TO_PIN(1, 1),
		.desc                   = "SW3",
		.wakeup                 = 1,
	},
	{
		.code                   = BTN_3,
		.gpio                   = GPIO_TO_PIN(2, 2),
		.desc                   = "SW4",
	},
};

static struct gpio_keys_platform_data mc341_gpio_key_info = {
	.buttons        = mc341_gpio_buttons,
	.nbuttons       = ARRAY_SIZE(mc341_gpio_buttons),
};

static struct platform_device mc341_gpio_keys = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &mc341_gpio_key_info,
	},
};
static void gpio_keys_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(gpio_keys_pin_mux);
	err = platform_device_register(&mc341_gpio_keys);
	if (err)
		pr_err("failed to register gpio key device\n");
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "MC341B-00:LED_ST0n",
		.gpio			= GPIO_TO_PIN(0, 26),
	},
	{
		.name			= "MC341B-00:LED_ST1n",
		.gpio			= GPIO_TO_PIN(0, 27),
	},
/*
	{
		.name			= "MC341B-00:LAN_SPEED_LED",
		.gpio			= GPIO_TO_PIN(1, 6),
	},
*/
	{
		.name			= "MC341B-00:F-LED_PWn",
		.gpio			= GPIO_TO_PIN(2, 3),
	},
	{
		.name			= "MC341B-00:3G_PWn",
		.gpio			= GPIO_TO_PIN(1, 4),
	},
	{
		.name			= "MC341B-00:3G_Reset",
		.gpio			= GPIO_TO_PIN(1, 5),
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};
static void gpio_led_init(int evm_id, int profile)
{
	int err;

	setup_pin_mux(gpio_led_mux);
	err = platform_device_register(&leds_gpio);
	if (err)
		pr_err("failed to register gpio led device\n");
}

/* Module pin mux for SPI0 fash */
static struct pinmux_config spi0_pin_mux[] = {
	{"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},		/* SPI0_SCLK ROM[CLK] */
	{"spi0_d0.spi0_d0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},			/* SPI0_D0 ROM[MISO] */
	{"spi0_d1.spi0_d1", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},			/* SPI0_D1 ROM[MOSI] */
	{"spi0_cs0.spi0_cs0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},		/* SPI0_CS0 ROM[CS] */
	{"mii1_rxclk.gpio3_10", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},			/* ROM_WPn */
	{NULL, 0},
};
static struct spi_board_info mc341_spi0_slave_info[] = {
	{
		.modalias      = "m25p80",
		// .modalias      = "spidev", // for spidev
		// .mode=SPI_MODE_3,	// for spidev
		.platform_data = &mc341_spi_flash,
		.irq           = -1,
		.max_speed_hz  = 24000000,
		.bus_num       = 1,
		.chip_select   = 0,
	},
	// ad
// update 2015.02.12 max_speed 6Mbps
        {
                .modalias       = "spidev",
              //  .max_speed_hz   = 48000000, //48 Mbps
              //  .max_speed_hz   = 24000000, //24 Mbps
                .max_speed_hz   = 6000000, //6 Mbps
                .bus_num        = 2,
                .chip_select    = 0,
/*                .mode = SPI_MODE_3, */
//                .mode = SPI_MODE_1 | SPI_CS_HIGH,
		// update 2015.02.18 SPI2.0
                .mode = SPI_MODE_2,
        },
// update 2015.02.09 spidev test
// update 2015.02.12 max_speed 6Mbps
        { // cpld
                .modalias       = "spidev",
              //  .max_speed_hz   = 48000000, //48 Mbps
              //  .max_speed_hz   = 24000000, //24 Mbps
                .max_speed_hz   = 6000000, //6 Mbps
                .bus_num        = 2,
                .chip_select    = 1,
/*                .mode = SPI_MODE_3, */
//                .mode = SPI_MODE_1 | SPI_CS_HIGH,
//                 .mode = SPI_MODE_2 | SPI_CS_HIGH,
                .mode = SPI_MODE_2,
        },
};
/* pinmux for usb0 drvvbus */
static struct pinmux_config usb0_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};
/* pinmux for usb1 drvvbus */
static struct pinmux_config usb1_pin_mux[] = {
	{"usb1_drvvbus.usb1_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};
static void usb0_init(int evm_id, int profile)
{
	setup_pin_mux(usb0_pin_mux);
	return;
}

static void usb1_init(int evm_id, int profile)
{
	setup_pin_mux(usb1_pin_mux);
	return;
}

// setup spi0
static void spi0_init(int evm_id, int profile)
{
	setup_pin_mux(spi0_pin_mux);
	spi_register_board_info(mc341_spi0_slave_info,
			ARRAY_SIZE(mc341_spi0_slave_info));
	return;
}
// update 2015.02.04 UART3, SPI1 init
static struct pinmux_config spi1_pin_mux[] = {
	{"mcasp0_aclkx.spi1_sclk", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},	/* SPI1_SCLK ROM[CLK] */
	{"mcasp0_fsx.spi1_d0", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},/* SPI1_D0 ROM[MISO] */
	{"mcasp0_axr0.spi1_d1", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},/* SPI1_D1 ROM[MOSI] */
	{"mcasp0_ahclkr.spi1_cs0", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},	/* SPI1_CS0 ROM[CS] */
	{NULL, 0},
};
// update 2015.02.04 UART3, SPI1 init
static struct pinmux_config uart3_pin_mux[] = {
	{"mii1_rxd3.uart3_rxd", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},/* UART3_RXD 3G-CN */
	{"mii1_rxd2.uart3_txd", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},	/* UART3_TXD 3G-CN */
// update 2015.02.05 uart3 update
//	{"uart1_rtsn.uart1_rtsn", OMAP_MUX_MODE6 | AM33XX_PIN_INPUT_PULLUP},/* UART3_RTSn 3G-CN */
//	{"uart1_ctsn.uart1_ctsn", OMAP_MUX_MODE6 | AM33XX_PIN_OUTPUT},	/* UART3_CTSn 3G-CN */
	{"lcd_data11.uart3_rtsn", OMAP_MUX_MODE6 | AM33XX_PIN_OUTPUT},/* UART3_RTSn 3G-CN */
	{"lcd_data10.uart3_ctsn", OMAP_MUX_MODE6 | AM33XX_PIN_INPUT_PULLUP},	/* UART3_CTSn 3G-CN */
	{NULL, 0},
};
// update 2015.02.26 uart5 update
static struct pinmux_config uart5_pin_mux[] = {
	{"uart5_rxd", OMAP_MUX_MODE4 | AM33XX_PIN_INPUT_PULLUP},/* UART5_RXD */
	{"uart5_txd", OMAP_MUX_MODE4 | AM33XX_PIN_OUTPUT},	/* UART5_TXDN */
//	{"uart5_rtsn", OMAP_MUX_MODE6 | AM33XX_PIN_INPUT_PULLUP},/* UART5_RTSn */
//	{"uart5_ctsn", OMAP_MUX_MODE6 | AM33XX_PIN_OUTPUT},	/* UART5_CTSn */
	{"uart5_rtsn", OMAP_MUX_MODE6 | AM33XX_PIN_OUTPUT},/* UART5_RTSn */
	{"uart5_ctsn", OMAP_MUX_MODE6 | AM33XX_PIN_INPUT_PULLUP},	/* UART5_CTSn */
	{NULL, 0},
};

//update 2015.07.22 uart5 rs422/rs485 type update
static struct pinmux_config uart5_rs485_pin_mux[] = {
	{"uart5_rxd", OMAP_MUX_MODE4 | AM33XX_PIN_INPUT_PULLUP},/* UART5_RXD */
	{"uart5_txd", OMAP_MUX_MODE4 | AM33XX_PIN_OUTPUT},	/* UART5_TXDN */
	{"uart5_rtsn", OMAP_MUX_MODE6 | AM33XX_PIN_OUTPUT},/* UART5_RTSn */
	{"uart5_ctsn", OMAP_MUX_MODE6 | AM33XX_PIN_INPUT_PULLUP},	/* UART5_CTSn */ 
	{"gpmc_ad7.gpio1_7", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP}, /* UART5 full/half mode */
	{NULL, 0},
};

// add 2015.07.21
#ifdef CONFIG_MACH_MC34X_ENABLE_NMI_INTERRUPT
static struct pinmux_config nmin_pin_mux[] = {
	{"nmin.nmin", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{NULL,0},
};

static void nmin_init(int evm_id, int profile){
	setup_pin_mux(nmin_pin_mux);
	return ;
}
#endif

/* EVM - Starter Kit */
// koko
static struct evm_dev_cfg evm_sk_dev_cfg[] = {
//	{mmc1_wl12xx_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{mmc0_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
//	{rgmii1_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
//	{rgmii2_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{rmii1_init,	DEV_ON_BASEBOARD, PROFILE_ALL}, // take
	{lcdc_init,     DEV_ON_BASEBOARD, PROFILE_ALL}, // 2014.12.12
// 1129	{enable_ecap2,     DEV_ON_BASEBOARD, PROFILE_ALL},
//	{tsc_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
	{gpio_keys_init,  DEV_ON_BASEBOARD, PROFILE_ALL}, // add 2014.12.01
	{gpio_led_init,  DEV_ON_BASEBOARD, PROFILE_ALL}, // add 2014.12.01
// 1129	{lis331dlh_init, DEV_ON_BASEBOARD, PROFILE_ALL},
// 1129	{mcasp1_init,   DEV_ON_BASEBOARD, PROFILE_ALL},
// update 2015.07.03
#ifdef CONFIG_MACH_MC341B30 // add 2015.07.03 .. see Kconfig
	{mc341_dcan_init, DEV_ON_BASEBOARD, PROFILE_ALL},
#else
	{uart1_wl12xx_init, DEV_ON_BASEBOARD, PROFILE_ALL}, // add 2014.12.01
#endif
//	{wl12xx_init,       DEV_ON_BASEBOARD, PROFILE_ALL},
// 1129	{gpio_ddr_vtt_enb_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
// 1206	{spi0_init,	DEV_ON_DGHTR_BRD, PROFILE_2}, // add 2014.11.24
// 	{spi0_init,	DEV_ON_DGHTR_BRD, PROFILE_ALL}, // add 2014.11.24
//	{spi0_init,	DEV_ON_DGHTR_BRD, PROFILE_0}, // for spidev add 1207
 	{spi0_init,	DEV_ON_BASEBOARD, PROFILE_ALL}, // add 2014.12.23
	{usb0_init,	DEV_ON_BASEBOARD, PROFILE_ALL},// add 2015.02.27
	{usb1_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
#ifdef CONFIG_MACH_MC34X_ENABLE_NMI_INTERRUPT
	{nmin_init,	DEV_ON_BASEBOARD, PROFILE_ALL}, // add 2015.07.21
#endif
	{NULL, 0, 0},
};

// add 2014.11.25
#ifdef CONFIG_TLK110_WORKAROUND
static int mc341_tlk110_phy_fixup(struct phy_device *phydev)
{
 unsigned int val;

// printk(KERN_WARNING "[%s](%d)mc341_tlk110_phy_fixup in!!!",__FILE__,__LINE__);

 /* This is done as a workaround to support TLK110 rev1.0 phy */
 // TLK110_COARSEGAIN_REG 0x00A3
 val = phy_read(phydev, TLK110_COARSEGAIN_REG);
 phy_write(phydev, TLK110_COARSEGAIN_REG, (val | TLK110_COARSEGAIN_VAL));

 // TLK110_LPFHPF_REG 0x00AC
 val = phy_read(phydev, TLK110_LPFHPF_REG);
 phy_write(phydev, TLK110_LPFHPF_REG, (val | TLK110_LPFHPF_VAL));

 // TLK110_SPAREANALOG_REG 0x00B9
 val = phy_read(phydev, TLK110_SPAREANALOG_REG);
 phy_write(phydev, TLK110_SPAREANALOG_REG, (val | TLK110_SPANALOG_VAL));

 // TLK110_VRCR_REG 0x00D0
 val = phy_read(phydev, TLK110_VRCR_REG);
 phy_write(phydev, TLK110_VRCR_REG, (val | TLK110_VRCR_VAL));

 // TLK110_SETFFE_REG 0x0107
 val = phy_read(phydev, TLK110_SETFFE_REG);
 phy_write(phydev, TLK110_SETFFE_REG, (val | TLK110_SETFFE_VAL));

 // TLK110_FTSP_REG 0x0154
 val = phy_read(phydev, TLK110_FTSP_REG);
 phy_write(phydev, TLK110_FTSP_REG, (val | TLK110_FTSP_VAL));

 // TLK110_ALFATPIDL_REG 0x002A
 val = phy_read(phydev, TLK110_ALFATPIDL_REG);
 phy_write(phydev, TLK110_ALFATPIDL_REG, (val | TLK110_ALFATPIDL_VAL));

 // TLK110_PSCOEF21_REG 0x0096
 val = phy_read(phydev, TLK110_PSCOEF21_REG);
 phy_write(phydev, TLK110_PSCOEF21_REG, (val | TLK110_PSCOEF21_VAL));

 // TLK110_PSCOEF3_REG 0x0097
 val = phy_read(phydev, TLK110_PSCOEF3_REG);
 phy_write(phydev, TLK110_PSCOEF3_REG, (val | TLK110_PSCOEF3_VAL));

 // TLK110_ALFAFACTOR1_REG 0x002C
 val = phy_read(phydev, TLK110_ALFAFACTOR1_REG);
 phy_write(phydev, TLK110_ALFAFACTOR1_REG, (val | TLK110_ALFACTOR1_VAL));

 // TLK110_ALFAFACTOR2_REG 0x0023
 val = phy_read(phydev, TLK110_ALFAFACTOR2_REG);
 phy_write(phydev, TLK110_ALFAFACTOR2_REG, (val | TLK110_ALFACTOR2_VAL));

 // TLK110_CFGPS_REG 0x0095
 val = phy_read(phydev, TLK110_CFGPS_REG);
 phy_write(phydev, TLK110_CFGPS_REG, (val | TLK110_CFGPS_VAL));

 // TLK110_FTSPTXGAIN_REG 0x0150
 val = phy_read(phydev, TLK110_FTSPTXGAIN_REG);
 phy_write(phydev, TLK110_FTSPTXGAIN_REG, (val | TLK110_FTSPTXGAIN_VAL));

 // TLK110_SWSCR3_REG 0x000B
 val = phy_read(phydev, TLK110_SWSCR3_REG);
 phy_write(phydev, TLK110_SWSCR3_REG, (val | TLK110_SWSCR3_VAL));

 // TLK110_SCFALLBACK_REG 0x0040
 val = phy_read(phydev, TLK110_SCFALLBACK_REG);
 phy_write(phydev, TLK110_SCFALLBACK_REG, (val | TLK110_SCFALLBACK_VAL));

 // TLK110_PHYRCR_REG 0x001F
 val = phy_read(phydev, TLK110_PHYRCR_REG);
 phy_write(phydev, TLK110_PHYRCR_REG, (val | TLK110_PHYRCR_VAL));

 return 0;
}
#endif


/* EVM - Starter Kit */
static void setup_starterkit(void)
{
// koko
// printk(KERN_WARNING "[%s](%d)setup force!!!", __FILE__,__LINE__ );

	pr_info("The board is a AM335x Starter Kit.\n");

	/* Starter Kit has Micro-SD slot which doesn't have Write Protect pin */
	mc341_mmc[0].gpio_wp = -EINVAL;

// printk(KERN_WARNING "[%s](%d)_configure_device in!!!", __FILE__,__LINE__ );
	_configure_device(EVM_SK, evm_sk_dev_cfg, PROFILE_NONE);

	// update 2015.02.18 am33xx_cpsw_init
	// am33xx_cpsw_init(AM33XX_CPSW_MODE_RMII, "0:00", "0:1e");
	am33xx_cpsw_init(AM33XX_CPSW_MODE_RMII, "0:00", "0:01");

	/* Initialize TLK110 PHY registers for phy version 1.0 */
	mc341_tlk110_phy_init();

	// update 2015.02.04 UART3, SPI1 init
	setup_pin_mux(uart3_pin_mux);
	// update 2015.02.26 uart5 update
#ifdef CONFIG_MACH_MC341B30
	// uart5 RS422/RS485 type
	setup_pin_mux(uart5_rs485_pin_mux);
#else//
	// uart5 RS232C type
	setup_pin_mux(uart5_pin_mux);
#endif

	setup_pin_mux(spi1_pin_mux);
	
	// update 2015.02.06 2LAN setup
#ifdef CONFIG_MACH_MC341B00
	// LAN 1 model
	setup_pin_mux(mc341_lan1_model);
#else
	// LAN 2 model
	// if(MC341LAN2) 
	{
		int ret;
		setup_pin_mux(rmii2_pin_mux);
		setup_pin_mux(mc341_lan2_model);
		// 2015.02.09
			#define GPIO_INIT_END GPIO_TO_PIN(0, 16)
			ret = gpio_request(GPIO_INIT_END, "GPIO_INIT_END");
			if (!ret) {
				gpio_direction_output(GPIO_INIT_END, 1);
			}else{
				printk(KERN_ERR "%s: failed to request GPIO for GPIO_INIT_END port "
                       "gpio control: %d\n", __func__, ret);
			}
	}
#endif

}




/*
static void mc341_setup_daughter_board(struct memory_accessor *m, void *c)
{
//	int ret;

	//
	// Read from the EEPROM to see the presence of daughter board.
	// If present, print the cpld version.
	//

	ret = m->read(m, (char *)&config1, 0, sizeof(config1));
	if (ret == sizeof(config1)) {
		pr_info("Detected a daughter card on AM335x EVM..");
		daughter_brd_detected = true;
	}
	 else {
		pr_info("No daughter card found on AM335x EVM\n");
// koko 
//		daughter_brd_detected = false;
//		return;
printk(KERN_WARNING "[%s](%d)config data force set", __FILE__,__LINE__ );
daughter_brd_detected = true;
config1.header=0xee3355aa;
memcpy( config1.name, "A335X_SK", 8 );
memcpy( config1.version, "1.2B", 4 );
memcpy( config1.serial, "30124P191476", 12 );
memcpy( config1.opt, "SKU#00FFFFFFFFFFFFFFFFFFFFFFFFFF", 32 );
memcpy( config1.cpld_ver, "CPLD", 5 );
	}

	if (!strncmp("CPLD", config1.cpld_ver, 4))
		pr_info("CPLD version: %s\n", config1.cpld_ver);
	else
		pr_err("Unknown CPLD version found\n");

}
*/

static void mc341_setup(struct memory_accessor *mem_acc, void *context)
{
	int ret;
//	char tmp[10];

// koko
printk(KERN_INFO "mc341_setup in!!!" );

	ret = mem_acc->read(mem_acc, (char *)&config_mc341,
		0, sizeof(config_mc341));
	if( ret == sizeof(config_mc341)){
printk(KERN_INFO "config_mc341 data " );
printk(KERN_INFO "config_mc341.header=[%x]", config_mc341.header );
printk(KERN_INFO "config_mc341.name=[%.8s]", config_mc341.name );
printk(KERN_INFO "config_mc341.clock_mem=[%x]", config_mc341.clock_mem );
printk(KERN_INFO "config_mc341.display=[%x]", config_mc341.display );
		// mac addr
		{
			char buf[128],wk[16];
			int i,j;
			memcpy( (char *)&am335x_mac_addr[0][0], (char *)&config_mc341.mac0, 12);
			for(i=0;i<2;i++){
				memset( buf, 0, sizeof(buf) );
				for(j=0;j<6;j++){
					sprintf( wk, "[%2.2x]", am335x_mac_addr[i][j] );
					strcat( buf, wk );
				}
				printk(KERN_INFO "eth%d mac{%s}", i, buf ); // 0102
			}
		}
printk(KERN_INFO "config_mc341.ssp=[%x]", config_mc341.ssp );
printk(KERN_INFO "config_mc341.debport=[%x]", config_mc341.debport );
printk(KERN_INFO "config_mc341.serial=[%.14s]", config_mc341.serial );

	}
	
	
	/* 1st get the MAC address from EEPROM */
/*	
	ret = mem_acc->read(mem_acc, (char *)&am335x_mac_addr,
		EEPROM_MAC_ADDRESS_OFFSET, sizeof(am335x_mac_addr));

printk(KERN_WARNING "[%s](%d) EEPROM_MAC_ADDRESS_OFFSET=%d!!!", __FILE__,__LINE__, EEPROM_MAC_ADDRESS_OFFSET ); // 1223
printk(KERN_WARNING "[%s](%d)ret=%d sizeof(am335x_mac_addr)=%d!!!", __FILE__,__LINE__, ret, sizeof(am335x_mac_addr) ); // 1223
	if (ret != sizeof(am335x_mac_addr)) {
		pr_warning("AM335X: EVM Config read fail: %d\n", ret);
		return;
	}
*/
	
	/* Fillup global mac id */
	am33xx_cpsw_macidfillup(&am335x_mac_addr[0][0],
				&am335x_mac_addr[1][0]);

	/* get board specific data */
/*
	ret = mem_acc->read(mem_acc, (char *)&config, 0, sizeof(config));
printk(KERN_WARNING "[%s](%d)ret=%d sizeof(config)=%d!!!", __FILE__,__LINE__, ret, sizeof(config) ); // 0102
	if (ret != sizeof(config)) {
		pr_err("AM335X EVM config read fail, read %d bytes\n", ret);
		pr_err("This likely means that there either is no/or a failed EEPROM\n");
		goto out;
	}else{
		// koko 
		int i;
		char wk[16];
		char buf[256];
		char *c;

		c=(char *)&config;
		memset( buf, 0, sizeof(buf) );
		for( i=0; i<sizeof(config); i++){
		 sprintf( wk, "[%2.2x]", c[i] );
		 strcat( buf, wk );
		}
		printk(KERN_WARNING "[%s](%d)config read data{%s}", __FILE__,__LINE__, buf );
	}
*/
if(0){
printk(KERN_WARNING "[%s](%d)config data set force", __FILE__,__LINE__ );
config.header=0xee3355aa;
memcpy( config.name, "A335X_SK", 8 );
memcpy( config.version, "1.2B", 4 );
memcpy( config.serial, "30124P191476", 12 );
memcpy( config.opt, "SKU#00FFFFFFFFFFFFFFFFFFFFFFFFFF", 32 );
}

	/*
	if (config.header != AM335X_EEPROM_HEADER) {
		pr_err("AM335X: wrong header 0x%x, expected 0x%x\n",
			config.header, AM335X_EEPROM_HEADER);
		goto out;
	}
	*/

	/*
	if (strncmp("A335", config.name, 4)) {
		pr_err("Board %s\ndoesn't look like an AM335x board\n",
			config.name);
		goto out;
	}
	*/
/*
	snprintf(tmp, sizeof(config.name) + 1, "%s", config.name);
	pr_info("Board name: %s\n", tmp);
	snprintf(tmp, sizeof(config.version) + 1, "%s", config.version);
	pr_info("Board version: %s\n", tmp);
*/
	
//	if (!strncmp("A335X_SK", config.name, 8)) {
		daughter_brd_detected = false;
// printk(KERN_WARNING "[%s](%d)setup_starterkit in!!!", __FILE__,__LINE__ );
		setup_starterkit();
//	} else {
//	}

	return;
/*
out:
*/
	/*
	 * If the EEPROM hasn't been programed or an incorrect header
	 * or board name are read then the hardware details are unknown.
	 * Notify the user and call machine_halt to stop the boot process.
	 */
	pr_err("The error message above indicates that there is an issue with\n"
		   "the EEPROM or the EEPROM contents.  After verifying the EEPROM\n"
		   "contents, if any, refer to the %s function in the\n"
		   "%s file to modify the board\n"
		   "initialization code to match the hardware configuration\n",
		   __func__ , __FILE__);
	machine_halt();
}
/*
static struct at24_platform_data mc341_daughter_board_eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup          = mc341_setup_daughter_board,
	.context        = (void *)NULL,
};
*/
// I2C BUS 2Kbit(256x8bit) EEPROM - BR24L02F-W 
static struct at24_platform_data mc341_baseboard_eeprom_info = {
	.byte_len       = (256*8) / 8,
	.page_size      = 8,
	// .flags          = AT24_FLAG_ADDR16, // 2015.01.16
	.flags          = 0,
	.setup          = mc341_setup,
	.context        = (void *)NULL,
};

/*
*/
static struct regulator_init_data am335x_dummy = {
	.constraints.always_on	= true,
};

/*
*/
static struct regulator_consumer_supply am335x_vdd1_supply[] = {
	REGULATOR_SUPPLY("vdd_mpu", NULL),
};
static struct regulator_init_data am335x_vdd1 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd1_supply),
	.consumer_supplies	= am335x_vdd1_supply,
};
/*
*/
static struct regulator_consumer_supply am335x_vdd2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};
static struct regulator_init_data am335x_vdd2 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd2_supply),
	.consumer_supplies	= am335x_vdd2_supply,
};
/*
*/
static struct tps65910_board am335x_tps65910_info = {
	.tps65910_pmic_init_data[TPS65910_REG_VRTC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VIO]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD1]	= &am335x_vdd1,
	.tps65910_pmic_init_data[TPS65910_REG_VDD2]	= &am335x_vdd2,
	.tps65910_pmic_init_data[TPS65910_REG_VDD3]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VPLL]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDAC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX33]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VMMC]	= &am335x_dummy,
};

/*
* Daughter board Detection.
* Every board has a ID memory (EEPROM) on board. We probe these devices at
* machine init, starting from daughter board and ending with baseboard.
* Assumptions :
*	1. probe for i2c devices are called in the order they are included in
*	   the below struct. Daughter boards eeprom are probed 1st. Baseboard
*	   eeprom probe is called last.
*/
// 0x2d PMIC, 0x50 EEPROM, 0x6f RTC
static struct i2c_board_info __initdata mc341_i2c0_boardinfo[] = {
/*** 2014.12.23
	{
		// Daughter Board EEPROM 
		I2C_BOARD_INFO("24c256", DAUG_BOARD_I2C_ADDR), // 0x51
		.platform_data  = &am335x_daughter_board_eeprom_info,
	},
***/
	{
		/* Baseboard board EEPROM */
		I2C_BOARD_INFO("24c256", BASEBOARD_I2C_ADDR), // 0x50
		.platform_data  = &mc341_baseboard_eeprom_info,
	},
/*** 2014.12.23
	{
		I2C_BOARD_INFO("cpld_reg", 0x35),
	},
	{
		I2C_BOARD_INFO("tlc59108", 0x40),
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x1b),
	},
***/
// 2015.2.11 add 
/****
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1), // 0x2d
		.platform_data  = &am335x_tps65910_info,
	},
****/
// update 2015.03.05 RTC mcp7940
	{
//		I2C_BOARD_INFO("rtc-mcp7940", 0x6f), // RTC
		I2C_BOARD_INFO("mcp7940", 0x6f), // RTC
	},
// update 2015.04.28 RTC rtc-rx8900
	{
//		I2C_BOARD_INFO("rtc-rx8900", 0x32), // RTC
		I2C_BOARD_INFO("rx8900", 0x32), // RTC
	},

};

// update 2015.03.16 RFID add
// 0x54 RFID
static struct i2c_board_info __initdata mc341_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("rfid", 0x54), // RFID
	},
};
// update 2015.03.16 RFID add
static struct pinmux_config i2c1_pin_mux[] = {
//	{"uart0_ctsn.i2c1_sda", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
////	{"uart0_ctsn.i2c1_sda", OMAP_MUX_MODE3 | AM33XX_PIN_INPUT},
//	{"uart0_rtsn.i2c1_scl", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT|AM33XX_PIN_INPUT_PULLUP},
////	{"uart0_rtsn.i2c1_scl", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad8.gpio0_22", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN},
//	{"gpmc_ad9.gpio0_23", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT |AM33XX_INPUT_EN},
	{"gpmc_ad9.gpio0_23", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT },
	{NULL, 0},
};
static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * AM335X beta EVM has USB0 in OTG mode and USB1 in host mode.
	 */
	// .mode           = (MUSB_HOST << 4) | MUSB_OTG,
	.mode           = (MUSB_HOST << 4) | MUSB_HOST,
	.power		= 500,
	.instances	= 1,
};

static int cpld_reg_probe(struct i2c_client *client,
	    const struct i2c_device_id *id)
{
	cpld_client = client;
	return 0;
}

static int __devexit cpld_reg_remove(struct i2c_client *client)
{
	cpld_client = NULL;
	return 0;
}

static const struct i2c_device_id cpld_reg_id[] = {
	{ "cpld_reg", 0 },
	{ }
};

static struct i2c_driver cpld_reg_driver = {
	.driver = {
		.name	= "cpld_reg",
	},
	.probe		= cpld_reg_probe,
	.remove		= cpld_reg_remove,
	.id_table	= cpld_reg_id,
};

static void evm_init_cpld(void)
{
	i2c_add_driver(&cpld_reg_driver);
}

static void __init mc341_i2c_init(void)
{
	int ret=0;
	
	/* Initially assume General Purpose EVM Config */
	// am335x_evm_id = GEN_PURP_EVM;

	evm_init_cpld();

	omap_register_i2c_bus(1, 100, mc341_i2c0_boardinfo,
				ARRAY_SIZE(mc341_i2c0_boardinfo));

// update 2015.03.16 RFID add
	setup_pin_mux(i2c1_pin_mux);
if(0){
         #define GPIO_INIT_I2C1 GPIO_TO_PIN(0, 23)
         ret = gpio_request(GPIO_INIT_I2C1, "GPIO_INIT_I2C1");
         if (!ret) {
            gpio_direction_output(GPIO_INIT_I2C1, 1);
            // gpio_direction_output(GPIO_INIT_I2C1, 0);
         }else{
            printk(KERN_ERR "%s: failed to request GPIO for GPIO_INIT_I2C1 port "
                       "gpio control: %d\n", __func__, ret);
         }
}
if(1){
	omap_register_i2c_bus(2, 100, mc341_i2c1_boardinfo,
				ARRAY_SIZE(mc341_i2c1_boardinfo));
}

	// 2014.12.23
	#define SPI0_ROM_WPN GPIO_TO_PIN(3, 10)
	
	ret = gpio_request(SPI0_ROM_WPN, "SPI0_ROM_WPN");
	if (ret) {
		printk(KERN_ERR "%s: failed to request GPIO for SPI0_ROM_WPN port "
		       "power control: %d\n", __func__, ret);
		return;
	}
	gpio_direction_output(SPI0_ROM_WPN, 1);
}

static struct resource mc341_rtc_resources[] = {
	{
		.start		= AM33XX_RTC_BASE,
		.end		= AM33XX_RTC_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
	{ /* timer irq */
		.start		= AM33XX_IRQ_RTC_TIMER,
		.end		= AM33XX_IRQ_RTC_TIMER,
		.flags		= IORESOURCE_IRQ,
	},
	{ /* alarm irq */
		.start		= AM33XX_IRQ_RTC_ALARM,
		.end		= AM33XX_IRQ_RTC_ALARM,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device mc341_rtc_device = {
	.name           = "omap_rtc",
	.id             = -1,
	.num_resources	= ARRAY_SIZE(mc341_rtc_resources),
	.resource	= mc341_rtc_resources,
};

static int mc341_rtc_init(void)
{
	void __iomem *base;
	struct clk *clk;

	clk = clk_get(NULL, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc : Failed to get RTC clock\n");
		return -1;
	}

	if (clk_enable(clk)) {
		pr_err("rtc: Clock Enable Failed\n");
		return -1;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);

	if (WARN_ON(!base))
		return -ENOMEM;

	/* Unlock the rtc's registers */
	writel(0x83e70b13, base + 0x6c);
	writel(0x95a4f1e0, base + 0x70);

	/*
	 * Enable the 32K OSc
	 * TODO: Need a better way to handle this
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 */
	// writel(0x48, base + 0x54);
	writel(0x40, base + 0x54); // upd 2015.01.06

	iounmap(base);

	return  platform_device_register(&mc341_rtc_device);
}

/* Enable clkout2 */
static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void __init clkout2_enable(void)
{
	struct clk *ck_32;

	ck_32 = clk_get(NULL, "clkout2_ck");
	if (IS_ERR(ck_32)) {
		pr_err("Cannot clk_get ck_32\n");
		return;
	}

	clk_enable(ck_32);

	setup_pin_mux(clkout2_pin_mux);
}

void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{

	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am33xx_emif_base;
}

void __iomem *am33xx_gpio0_base;

void __iomem *am33xx_get_gpio0_base(void)
{
	am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);

	return am33xx_gpio0_base;
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");

}

static void make_spread_spectrum(void);

static void __init mc341_init(void)
{
// printk(KERN_WARNING "[%s](%d)am33xx_cpuidle_init in!!!",__FILE__,__LINE__); // 12.23
	am33xx_cpuidle_init();
// printk(KERN_WARNING "[%s](%d)am33xx_mux_init in!!!",__FILE__,__LINE__); // 12.23
	am33xx_mux_init(board_mux);
// printk(KERN_WARNING "[%s](%d)omap_serial_init in!!!",__FILE__,__LINE__); // 12.23
	omap_serial_init();
// printk(KERN_WARNING "[%s](%d)mc341_rtc_init in!!!",__FILE__,__LINE__); // 12.23
// 2015.03.06 RTC update
#ifdef CONFIG_MACH_MC341_ENABLE_RTC_CLOCK //update 2015.07.03 (See. KConfig)
	mc341_rtc_init();
#endif
// printk(KERN_WARNING "[%s](%d)clkout2_enable in!!!",__FILE__,__LINE__); // 12.23
	clkout2_enable();
// printk(KERN_WARNING "[%s](%d)mc341_i2c_init in!!!",__FILE__,__LINE__); // 12.23
	mc341_i2c_init();
// printk(KERN_WARNING "[%s](%d)omap_sdrc_init in!!!",__FILE__,__LINE__); // 12.23
	omap_sdrc_init(NULL, NULL);
// printk(KERN_WARNING "[%s](%d)usb_musb_init in!!!",__FILE__,__LINE__); // 12.23

	omap_board_config = am335x_evm_config;
	omap_board_config_size = ARRAY_SIZE(am335x_evm_config);
	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
	// 2015.2.11 add usb
	usb_musb_init(&musb_board_data);

// update 2015.03.05 CM_CLOCKOUT_CTRL
{
    void __iomem* io_base;
    unsigned int m;
    io_base = ioremap(AM33XX_CM_BASE + AM33XX_CM_DEVICE_MOD, 0x4);
    if (!io_base) {
        printk(KERN_ERR "ioremap CM_CLKOUT_CTRL failed\n");
        return;
    }
    m=readl(io_base+0x0);
    printk(KERN_INFO "CM_CLKOUT_CTRL[%x]\n", m );
    m &= ~0x3f;
    m |= ( 0x4 <<3 ); // 0x4=DIV5:SYS_CLKOUT2/5
    m |= ( 0x1 ); // 0x1=SEL1:Select L3 Clock
    writel(m, io_base+0x0);
    printk(KERN_INFO "CM_CLKOUT_CTRL[%x]\n", m );
    iounmap(io_base);
}


	make_spread_spectrum();

}

static void __init mc341_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

// CPS-MC341-ADSC1
MACHINE_START(MC341B10, "mc341b10")
	// Maintainer: CONTEC Co.,Ltd.
	.atag_offset	= 0x100,
	.map_io		= mc341_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= mc341_init,
MACHINE_END
//CPS-MC341-DS1
MACHINE_START(MC341B30, "mc341b30")
	// Maintainer: CONTEC Co.,Ltd.
	.atag_offset	= 0x100,
	.map_io		= mc341_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= mc341_init,
MACHINE_END
//CPS-MC342
MACHINE_START(MC342B00, "mc342b00")
	//Maintainer: CONTEC Co.,Ltd.
	.atag_offset	= 0x100,
	.map_io		= mc341_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= mc341_init,
MACHINE_END
MACHINE_START(AM335XEVM, "am335xevm")
	// Maintainer: Texas Instruments
	.atag_offset	= 0x100,
	.map_io		= mc341_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= mc341_init,
MACHINE_END
/*
MACHINE_START(AM335XIAEVM, "am335xiaevm")
	// Maintainer: Texas Instruments
	.atag_offset	= 0x100,
	.map_io		= mc341_map_io,
	.init_irq	= ti81xx_init_irq,
	.init_early	= am33xx_init_early,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= mc341_init,
MACHINE_END
*/
/********************************* Spread Spectrum Clocking ****************************/



/* Descriptor for Spread Spectrum Clocking */
struct ssc_data {
    const  char* name;          /* Name of the clock */
    unsigned int percent;       /* SSC modulation strength in % */
    unsigned int clksel;        /* Offset to CLKSEL register */
    unsigned int deltamstep;    /* Offset to DELTAMSTEP register */
    unsigned int modfreqdiv;    /* Offset to DELTAMSTEP register */
    unsigned int clkmode;       /* Offset to CLKMODE register */
};

/* Descriptor for all AM335x clocks */
static const struct ssc_data mpu_dpll_data = {
        .name       = "MPU",
        .percent    = 2,
        .clksel     = 0x2C,
        .deltamstep = 0x24,
        .modfreqdiv = 0x28,
        .clkmode    = 0x88,
};

static const struct ssc_data ddr_dpll_data = {
        .name       = "DDR",
        .percent    = 2,
        .clksel     = 0x40,
        .deltamstep = 0x38,
        .modfreqdiv = 0x3C,
        .clkmode    = 0x94,
};

static const struct ssc_data lcd_dpll_data = {
        .name       = "LCD",
        .percent    = 2,
        .clksel     = 0x54,
        .deltamstep = 0x4C,
        .modfreqdiv = 0x50,
        .clkmode    = 0x98,
};

static const struct ssc_data core_dpll_data = {
        .name       = "COR",
        .percent    = 2,
        .clksel     = 0x68,
        .deltamstep = 0x60,
        .modfreqdiv = 0x64,
        .clkmode    = 0x90,
};

static const struct ssc_data per_dpll_data = {
        .name       = "PER",
        .percent    = 2,    // watch for V24 jitter!
        .clksel     = 0x9C,
        .deltamstep = 0x74,
        .modfreqdiv = 0x78,
        .clkmode    = 0x8C,
};

/* Function for setup SSC */
static void spread_spectrum_setup(const struct ssc_data* dpll_data)
{
    void __iomem* clock_base;
    struct clk* clock;
    unsigned int f;
    unsigned int fm;
    unsigned int m;
    unsigned int n;
    unsigned int ModFreqDivider;
    unsigned int Exponent;
    unsigned int Mantissa;
    unsigned int delta_m_step;

    clock_base = ioremap(AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD, 0x1000);
    if (!clock_base) {
        printk(KERN_ERR "ioremap spread spectrum clocks failed\n");
        return;
    }

    /* Read PLL dividers m and n */
    m = readl(clock_base + dpll_data->clksel);
    n = m & 0x7F;
    m = (m >> 8) & 0x3FF;
    // printk(KERN_ERR "%s PLL m = %d, n = %d\n", dpll_data->name, m, n);

    /* Calculate Fref */
    clock = clk_get(NULL, "sys_clkin_ck");
    f = clk_get_rate(clock);
    f = f/(1+n);
    // printk(KERN_ERR "%s PLL reference clock is %dHz\n", dpll_data->name, f);

    /* Calculate max. Bandwidth (Modulation Frequency) of PLL */
    fm = f / 70;
    // printk(KERN_ERR "%s PLL Bandwidth is %d\n", dpll_data->name, fm);

    /* Calculate ModFreqDivider */
    ModFreqDivider = f/(4 * fm);
    // printk(KERN_ERR "%s PLL ModFreqDivider is %d\n", dpll_data->name, ModFreqDivider);

    /* Calculate Mantissa/Exponent */
    Exponent = 0;
    Mantissa = ModFreqDivider;
    while ((Mantissa > 127) && (Exponent < 7)) {
        Exponent++;
        Mantissa /= 2;
    }
    if (Mantissa > 127)
        Mantissa = 127;
    // printk(KERN_ERR "%s PLL Mantissa = %d, Exponent = %d\n", dpll_data->name, Mantissa, Exponent);
    ModFreqDivider = Mantissa << Exponent;
    // printk(KERN_ERR "%s PLL revised ModFreqDivider is %d\n", dpll_data->name, ModFreqDivider);

    /* Calculate Modulation steps */
    delta_m_step = (m * dpll_data->percent) << 18;
    delta_m_step /= 100;
    delta_m_step /= ModFreqDivider;
    if (delta_m_step > 0xFFFFF)
        delta_m_step = 0xFFFFF;
    // printk(KERN_ERR "%s PLL Delta_M_Int = %d, Delta_M_Frac = %d\n", dpll_data->name, delta_m_step >> 18, delta_m_step & 0x3FFFF);

    /* Setup Spread Spectrum */
    writel(delta_m_step, clock_base + dpll_data->deltamstep);
    writel((Exponent << 8) | Mantissa, clock_base + dpll_data->modfreqdiv);
    m = readl(clock_base + dpll_data->clkmode);
    m &= ~0xF000;   // clear all SSC flags
    m |=  0x1000;   // enable SSC
    writel(m, clock_base + dpll_data->clkmode);
    printk(KERN_INFO "%s PLL Spread Spectrum enabled with %d percent\n", dpll_data->name, dpll_data->percent);

    iounmap(clock_base); // 2014.12.12
}


static void make_spread_spectrum(void)
{
	if(0){
		printk(KERN_WARNING "[%s](%d)make_spread_spectrum[CPU,MEM] in!!!",__FILE__,__LINE__);
		spread_spectrum_setup(&mpu_dpll_data); // CPU
		spread_spectrum_setup(&ddr_dpll_data); // MEM
	}
//    spread_spectrum_setup(&lcd_dpll_data);
//    spread_spectrum_setup(&core_dpll_data);
//    spread_spectrum_setup(&per_dpll_data);
}


