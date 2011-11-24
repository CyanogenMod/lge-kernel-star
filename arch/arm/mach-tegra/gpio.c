/*
 * arch/arm/mach-tegra/gpio.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include <mach/iomap.h>
#include <mach/pinmux.h>
#include <mach/suspend.h>

#include <mach/nvrm_linux.h>
#include "nvcommon.h"
#include "nvrm_pmu.h"
#include "nvodm_query_discovery.h"
#include "gpio-names.h"

#define GPIO_BANK(x)		((x) >> 5)
#define GPIO_PORT(x)		(((x) >> 3) & 0x3)
#define GPIO_BIT(x)		((x) & 0x7)

#define GPIO_REG(x)		(IO_TO_VIRT(TEGRA_GPIO_BASE) +	\
				 GPIO_BANK(x) * 0x80 +		\
				 GPIO_PORT(x) * 4)

#define GPIO_CNF(x)		(GPIO_REG(x) + 0x00)
#define GPIO_OE(x)		(GPIO_REG(x) + 0x10)
#define GPIO_OUT(x)		(GPIO_REG(x) + 0X20)
#define GPIO_IN(x)		(GPIO_REG(x) + 0x30)
#define GPIO_INT_STA(x)		(GPIO_REG(x) + 0x40)
#define GPIO_INT_ENB(x)		(GPIO_REG(x) + 0x50)
#define GPIO_INT_LVL(x)		(GPIO_REG(x) + 0x60)
#define GPIO_INT_CLR(x)		(GPIO_REG(x) + 0x70)

#define GPIO_MSK_CNF(x)		(GPIO_REG(x) + 0x800)
#define GPIO_MSK_OE(x)		(GPIO_REG(x) + 0x810)
#define GPIO_MSK_OUT(x)		(GPIO_REG(x) + 0X820)
#define GPIO_MSK_INT_STA(x)	(GPIO_REG(x) + 0x840)
#define GPIO_MSK_INT_ENB(x)	(GPIO_REG(x) + 0x850)
#define GPIO_MSK_INT_LVL(x)	(GPIO_REG(x) + 0x860)

#define GPIO_INT_LVL_MASK		0x010101
#define GPIO_INT_LVL_EDGE_RISING	0x000101
#define GPIO_INT_LVL_EDGE_FALLING	0x000100
#define GPIO_INT_LVL_EDGE_BOTH		0x010100
#define GPIO_INT_LVL_LEVEL_HIGH		0x000001
#define GPIO_INT_LVL_LEVEL_LOW		0x000000

//20100724  for gpio setting while sleep [LGE_START]
#define REG_CNF     0
#define REG_OE      1
#define REG_OUT     2
#define REG_INT_ENB 3
#define REG_INT_LVL 4
#define DBG_BUF_SIZE	64

int get_gpio_reg_data(int port, int pin, int gpio, int reg);
//20100724  for gpio setting while sleep [LGE_END]

extern int gpio_get_pinmux_group(int gpio_nr);
int tegra_gpio_io_power_config(int gpio_nr, unsigned int enable);

struct tegra_gpio_bank {
	int bank;
	int irq;
	spinlock_t lvl_lock[4];
#ifdef CONFIG_PM
	u32 cnf[4];
	u32 out[4];
	u32 oe[4];
	u32 int_enb[4];
	u32 int_lvl[4];
#endif
};


static struct tegra_gpio_bank tegra_gpio_banks[] = {
	{.bank = 0, .irq = INT_GPIO1},
	{.bank = 1, .irq = INT_GPIO2},
	{.bank = 2, .irq = INT_GPIO3},
	{.bank = 3, .irq = INT_GPIO4},
	{.bank = 4, .irq = INT_GPIO5},
	{.bank = 5, .irq = INT_GPIO6},
	{.bank = 6, .irq = INT_GPIO7},
};

#if APPLY_SLEEP_GPIO_TABLE
// Group
typedef enum tegra_pin_group_conf {
    ATA = 0x00000000,
    ATB,
    ATC,
    ATD,
    CDEV1,
    CDEV2,
    CSUS,
    DAP1,
    DAP2,
    DAP3,
    DAP4,
    DTA,
    DTB,
    DTC,
    DTD,
    DTE,
    GPU,
    GPV,
    I2CP,
    IRTX,
    IRRX,
    KBCB,
    KBCA,
    PMC,
    PTA,
    RM,
    KBCE,
    KBCF,
    GMA,
    GMC,
    SDIO1,
    OWC,
    GME = 0x00010000,
    SDC,
    SDD,
    SLXA,
    UNUSED_1_4,
    SLXC,
    SLXD,
    SLXK,
    SPDI,
    SPDO,
    SPIA,
    SPIB,
    SPIC,
    SPID,
    SPIE,
    SPIF,
    SPIG,
    SPIH,
    UAA,
    UAB,
    UAC,
    UAD,
    UCA,
    UCB,
    UNUSED_1_24,
    ATE,
    KBCC,
    UNUSED_1_27,
    UNUSED_1_28,
    GMB,
    GMD,
    DDC,
    LD0 = 0x00020000,
    LD1,
    LD2,
    LD3,
    LD4,
    LD5,
    LD6,
    LD7,
    LD8,
    LD9,
    LD10,
    LD11,
    LD12,
    LD13,
    LD14,
    LD15,
    LD16,
    LD17,
    LHP0,
    LHP1,
    LHP2,
    LVP0,
    LVP1,
    HDINT,
    LM0,
    LM1,
    LVS,
    LSC0,
    LSC1,
    LSCK,
    LDC,
    LCSN,
    LSPI = 0x00030000,
    LSDA,
    LSDI,
    LPW0,
    LPW1,
    LPW2,
    LDI,
    LHS,
    LPP,
    KBCD,
    GPU7,
    DTF,
    UDA,
    CRTP,
    SDB,
    UNUSED_3_16,
    UNUSED_3_17,
    UNUSED_3_18,
    UNUSED_3_19,
    UNUSED_3_20,
    UNUSED_3_21,
    UNUSED_3_22,
    UNUSED_3_23,
    UNUSED_3_24,
    UNUSED_3_25,
    UNUSED_3_26,
    UNUSED_3_27,
    UNUSED_3_28,
    UNUSED_3_29,
    UNUSED_3_30,
    UNUSED_3_31,
    TRISTATE_SKIP = 0xffffffff,
} tegra_pin_group_conf_t;

// GPIO, SFIO configure
typedef enum tegra_gpio_sfio_confg {
    SFIO_ENABLE = 0,
    GPIO_ENABLE,
} tegra_gpio_sfio_confg_t;

// Input, output configure
typedef enum tegra_gpio_oe_confg {
    GPIO_INPUT = 0,
    GPIO_OUTPUT,
} tegra_gpio_oe_confg_t;

// out enable configure
typedef enum tegra_gpio_out_confg {
    GPIO_SLEEP_LOW = 0x00000000,
    GPIO_SLEEP_HIGH,
    GPIO_INIT_ONLY_LOW = 0x00010000,
    GPIO_INIT_ONLY_HIGH,
} tegra_gpio_out_confg_t;

struct tegra_init_gpio_info {
    u8 port;
    u8 pin;
	tegra_gpio_sfio_confg_t cnf;
	tegra_gpio_oe_confg_t   oe;
	tegra_gpio_out_confg_t  out;
	tegra_pin_group_conf_t  group;
};

/* tristate group  :  DTE, DTA, dtd, CSUS,(VI power)
			         LCSN, GPV, irtx, LDC, ldi, lhp0~2, lhs, lm0-1, lpp, lpw0-2, LSC0-1, 
                            LSCK, LSDA, LSDI, LSPI, lvp1, lvs, mipi, osc, owc, 
                            UAA, ATA, ATB, ATCFG1, ATE, CDEV1, CDEV2, CRTP, CSI, 
                            DAP1234, dbg, ddc, dsi, dtc, dtf, gma, gmc, gme, GPU7, hdint, hdmi,
                            KBCC, LD0~17, pta, rm, rst, rtc, sdc, sdd, sdio1, slxa, slxc, slxd, slxk, spdi, 
                            spia, spib, spic, spie, spif, spig, spih, tst, tv, uaa, UCA, UCB, UDA, USB, UAD, 
                            xm2c, xm2d, xm2s, (P999BN) kbca  */
/* normal group : ATC, ATD, DTB, GMB, GPU, IRRX, KBCB, KBCD, KBCF, LVP0, PMC, 
                         SPDO, SPID, UAB, UAC,
                        (P999BN) GMD */

#if APPLY_GPIO_INIT
const struct tegra_init_gpio_info tegra_init_gpio_info_array[] = {
    /* Dynamic change */
    { 'w'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW,   LM1},  // WLAN_EN
    { 'z'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW,   LSDI},  // BT_EN

    /* All GPIO output pins should be defined here */
    // CP sleep status (high : +3mA)
    //{ 'h'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW/*GPIO_HIGH*/,  /*TRISTATE_SKIP*/ATD},   // TEST_GPIO2(Sleep status)
    { 't'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTA},   // 8MN_CAM_VCM_EN
    { 'd'-'a',      5, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  DTA},   // VT_CAM_PWDN 
    { 't'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTB},   // FLASH_LED_TOURCH
    { 't'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTB},   // FLASH_LED_INH
    { 'z'-'a' + 2,  1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTE},   // VT_RESET_N 
    { 'z'-'a' + 2,  4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTE},   // FLASH_LED_EN 
    { 'd'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTE},   // 8M_RESET_N
    //{ 'u'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   GPU},   // AP20_UART_SW 
    //{ 'u'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   GPU},   // MDM_UART_SW 
    { 'u'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   GPU},   // VIBE_EN
    //{ 'j'-'a',      6, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   IRRX},  // IPC_MRDY1
    //{ 'r'-'a',      7, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   KBCB},  // MDM_VBUS_EN 
    //{ 's'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  KBCB},  // CHG_EN_SET_N_AP20
    //already { 'r'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   KBCD},  // BL_DCDC_RST_N
    { 'r'-'a',      6, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   KBCD},  // CAM_SUBPM_EN
    //already { 'v'-'a',      7, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   LVP0},  // LCD_RESET_N
    //{ 'k'-'a',      5, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   SPDO},  // HDMI_REG_EN
    { 'x'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   SPID},  // BT_WAKEUP
    //{ 'o'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   UAB},   // IPC_MRDY
    //already { 'v'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   UAC},   // IFX_RESET_1.8V
    //already { 'v'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  UAC},   // IFX_PWRON_1.8V
    //{ 'i'-'a',      7, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  ATC},   // MUIC_SCL ?
    //{ 'k'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  ATC},   // MUIC_SDA ?
    //already { 'k'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  ATC},   // WM_LDO_EN
    { 'g'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   ATC},   // WLAN_WAKEUP 
        
    /* All wakeup pins should be defined here : gpio input enable */
    { 'o'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   UAB},   //  IPC_SRDY2
    { 'z'-'a' + 2,  5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   DTE},   //  NC  +
    { 'a'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   DTE},   // PROXI_OUT(NC) +
    { 'c'-'a',      7, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   GMB},   // BT_HOST_WAKEUP
    { 's'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCB},  // WLAN_HOST_WAKEUP
    { 's'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCB},  // CHG_STATUS_N_AP20
    { 'v'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   UAC},   // MDM_RESET_FLAG +
    { 'v'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   UAC},   // AP_PWR_ON(powerkey)    
    //{ 'w'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIG},  // AUDIO_INT_N
    { 'w'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIH},  // BATT_LOW_INT
    { 'd'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SLXC},  //  HOOK_DET
    //{ 'i'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATB},   // MICROSD_DET_N

    /* tristate group's input pins */
#ifdef CONFIG_MACH_STAR_TMUS
    { 'h'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATD},   // TEST_GPIO1
    { 'h'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATD},   // NC 
    { 'h'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATD},   // NC 
#endif
    { 'u'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   GPU},   // HALL_INT
#ifdef CONFIG_MACH_STAR_TMUS
    { 'u'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   GPU},   // VIBE_PWM
#endif
    { 'u'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   GPU},   // INT_N_MUIC
    { 'u'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   GPU},   // IPC_SRDY1
    { 'r'-'a',      4, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCD},  // COM_INT
#ifdef CONFIG_MACH_STAR_TMUS
    { 'r'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCD},  // BATT_ID
#endif
    { 'q'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCF},  // GYRO_INT_N
    { 'q'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCF},  // NC
    { 'o'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   UAB},   // output검토필요. APTEMP_POWER_OFF_N
    { 'o'-'a',      7, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   UAB},   // SPI2_CLK
    { 'k'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATC},   // THERMAL_IRQ
    { 'g'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATC},   // VOL_KEY_UP
    { 'g'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATC},   // VOL_KEY_DOWN
    { 'g'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATC},   // EARJACK_SENSE
    { 'i'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATC},   // MOTION_INT
    { 'x'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIE},  //  TOUCH_MAKER_ID
    { 'x'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIE},  //  TOUCH_INT
#ifdef CONFIG_MACH_STAR_REV_F
    { 'r'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCA},  // IFX2_AP20 (LGP990)
    { 'r'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCA},  // PROXI_OUT
#endif

    /* group tristate or input */
    { 'j'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   IRTX},  // LCD_MAKER_ID
    //{ 'q'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  KBCC},  // BL_DCDC_SDA
    //{ 'q'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  KBCC},  // BL_DCDC_SOL
    { 'x'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIE},  // TOUCH_MAKER_ID
    { 'x'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIE},  // TOUCH_INT
    { 'w'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIH},  // BATT_LOW_INT
    { 't'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   DTD},   //  VT_PCLK
    { 't'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   CSUS},  //  VT_MCLK
};
#endif

const struct tegra_init_gpio_info tegra_sleep_gpio_info_array[] = {
    /* tristate off */
    { 0xFF,         0, SFIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   PMC},     // PMC
    //{ 0xFF,         0, SFIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   I2CP},    // power i2c 

    /* Dynamic change */
    { 'w'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW,   LM1},  // WLAN_EN
    { 'z'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW,   LSDI},  // BT_EN
#ifdef CONFIG_MACH_STAR_TMUS
    { 's'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW/*GPIO_SLEEP_HIGH*/,  KBCB},  // CHG_EN_SET_N_AP20
#else
    { 's'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_HIGH,    KBCB},  // CHG_EN_SET_N_AP20
    { 'v'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW,     UAC},   // IFX_RESET_1.8V
    { 'v'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_INIT_ONLY_LOW,     UAC},   // IFX_PWRON_1.8V high<BD><C3> 300uA<B9><U+07FB><FD>.
#endif
    
    /* All GPIO output pins should be defined here */
    // CP sleep status (high : +3mA)
#ifdef CONFIG_MACH_STAR_TMUS
    { 'h'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW/*GPIO_HIGH*/,  /*TRISTATE_SKIP*/ATD},   // TEST_GPIO2(Sleep status)
#endif
    { 't'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTA},   // 8MN_CAM_VCM_EN
    { 'd'-'a',      5, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTA},   // VT_CAM_PWDN 
    { 't'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTB},   // FLASH_LED_TOURCH
    { 't'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTB},   // FLASH_LED_INH
    { 'z'-'a' + 2,  1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTE},   // VT_RESET_N 
    { 'z'-'a' + 2,  4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTE},   // FLASH_LED_EN 
    { 'd'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   DTE},   // 8M_RESET_N
    { 'u'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   GPU},   // AP20_UART_SW
    { 'u'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   GPU},   // MDM_UART_SW
    { 'u'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   GPU},   // VIBE_EN
#ifdef CONFIG_MACH_STAR_TMUS
    { 'j'-'a',      6, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   IRRX},  // IPC_MRDY1
#endif
    { 'r'-'a',      7, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   KBCB},  // MDM_VBUS_EN 
    { 'r'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   KBCD},  // BL_DCDC_RST_N
    { 'r'-'a',      6, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   KBCD},  // CAM_SUBPM_EN
    { 'v'-'a',      7, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   LVP0},  // LCD_RESET_N
    { 'k'-'a',      5, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   SPDO},  // HDMI_REG_EN
    { 'x'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   SPID},  // BT_WAKEUP
    { 'o'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   UAB},   // IPC_MRDY
#ifdef CONFIG_MACH_STAR_TMUS
    { 'v'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   UAC},   // IFX_RESET_1.8V
    { 'v'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   UAC},   // IFX_PWRON_1.8V
#endif
    { 'i'-'a',      7, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  ATC},   // MUIC_SCL ?
    { 'k'-'a',      4, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  ATC},   // MUIC_SDA ?
    { 'k'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  ATC},   // WM_LDO_EN
    { 'g'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   ATC},   // WLAN_WAKEUP 
#ifdef CONFIG_MACH_STAR_REV_F
    { 'u'-'a',      3, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   GPU},   // USIF1_SW (LGP90)
    { 'r'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   KBCA},  // IFX1_AP20 (sleep_status) (LGP990)
    { 'j'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  GMD},   // GPS_RESET_N (LGP90) dynamic??
    { 'j'-'a',      2, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_LOW,   GMD},   // GPS_PWR_ON (LGP90) dynamic??
#endif

    /* All wakeup pins should be defined here : gpio input enable */
    { 'o'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   UAB},   //  IPC_SRDY2
    { 'z'-'a' + 2,  5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   DTE},   //  NC  +
    { 'a'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   DTE},   // PROXI_OUT(NC) +
    { 'c'-'a',      7, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   GMB},   // BT_HOST_WAKEUP
    { 's'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCB},  // WLAN_HOST_WAKEUP
    { 's'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCB},  // CHG_STATUS_N_AP20
    { 'v'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   UAC},   // MDM_RESET_FLAG +
    { 'v'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   UAC},   // AP_PWR_ON(powerkey)    
    //{ 'w'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIG},  // AUDIO_INT_N
    { 'w'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIH},  // BATT_LOW_INT
    { 'd'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SLXC},  //  HOOK_DET
    //{ 'i'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATB},   // MICROSD_DET_N

    /* tristate group's input pins */
#ifdef CONFIG_MACH_STAR_TMUS
    { 'h'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATD},   // TEST_GPIO1
    { 'h'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATD},   // NC 
    { 'h'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATD},   // NC 
#endif
    { 'u'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   GPU},   // HALL_INT
#ifdef CONFIG_MACH_STAR_TMUS
    { 'u'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   GPU},   // VIBE_PWM
#endif
    { 'u'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   GPU},   // INT_N_MUIC
    { 'u'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   GPU},   // IPC_SRDY1
    { 'r'-'a',      4, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCD},  // COM_INT
#ifdef CONFIG_MACH_STAR_TMUS
    { 'r'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCD},  // BATT_ID
#endif
    { 'q'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCF},  // GYRO_INT_N
    { 'q'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCF},  // CHG_PGB_N
    { 'o'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   UAB},   // output검토필요. APTEMP_POWER_OFF_N
    { 'o'-'a',      7, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   UAB},   // SPI2_CLK
    { 'k'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATC},   // THERMAL_IRQ
    { 'g'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_HIGH,  ATC},   // VOL_KEY_UP
    { 'g'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_HIGH,  ATC},   // VOL_KEY_DOWN
    { 'g'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATC},   // EARJACK_SENSE
    { 'i'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATC},   // MOTION_INT
    { 'x'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIE},  // TOUCH_MAKER_ID
    { 'x'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIE},  // TOUCH_INT
#ifdef CONFIG_MACH_STAR_REV_F
    { 'r'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCA},  // IFX2_AP20 (LGP990)
#endif
    { 'w'-'a',      2, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   KBCA},  // PROXI_OUT
    /* group tristate or input */
    #if 1
    { 'j'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   IRTX},  // LCD_MAKER_ID
    //{ 'q'-'a',      0, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  KBCC},  // BL_DCDC_SDA
    //{ 'q'-'a',      1, GPIO_ENABLE, GPIO_OUTPUT,    GPIO_SLEEP_HIGH,  KBCC},  // BL_DCDC_SOL
    { 'x'-'a',      5, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIE},  // TOUCH_MAKER_ID
    { 'x'-'a',      6, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIE},  // TOUCH_INT
    { 'w'-'a',      3, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   SPIH},  // BATT_LOW_INT
    { 't'-'a',      0, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   DTD},   //  VT_PCLK
    { 't'-'a',      1, GPIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   CSUS},  //  VT_MCLK
    #endif

    /* voice call ?? */
    //{ 0xFF        0, SFIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   CDEV1},   // AUDIO MCLK
    //{ 0xFF        0, SFIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   CDEV2},   // AUDIO MCLK2
    //{ 0xFF        0, SFIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   ATC},     // GMI_ADV_N, GMI_OE **    
    //{ 0xFF        0, SFIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   DAP1},    // DAP1
    //{ 0xFF        0, SFIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   DAP2},    // DAP2
    //{ 0xFF        0, SFIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   DAP4},    // BT DAP
    //{ 0xFF        0, SFIO_ENABLE, GPIO_INPUT,     GPIO_SLEEP_LOW,   DAP3},    // DAP 
};

static struct tegra_gpio_bank tegra_sleep_gpio_banks[] = {
	{.bank = 0, .irq = INT_GPIO1},
	{.bank = 1, .irq = INT_GPIO2},
	{.bank = 2, .irq = INT_GPIO3},
	{.bank = 3, .irq = INT_GPIO4},
	{.bank = 4, .irq = INT_GPIO5},
	{.bank = 5, .irq = INT_GPIO6},
	{.bank = 6, .irq = INT_GPIO7},
};
#else
//20100724  for gpio setting while sleep [LGE_START]
static struct tegra_gpio_bank tegra_sleep_gpio_banks[] = {
#ifdef CONFIG_MACH_STAR_TMUS	// real
    //  A, B, C, D        
    {.bank = 0, .irq = INT_GPIO1, 
        .cnf        = {0x00000000, 0x00000008, 0x00000000, 0x00000001},
        .out        = {0x00000000, 0x00000008, 0x00000000, 0x00000000}, 
        .oe         = {0x00000000, 0x00000008, 0x00000000, 0x00000001},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}}, 
    //  E, F, G, H
    {.bank = 1, .irq = INT_GPIO2, 
        .cnf        = {0x000000ff, 0x00000000, 0x0000000b, 0x0000000F},
        .out        = {0x0000001f, 0x00000000, 0x00000000, 0x00000004}, 
        .oe         = {0x000000ff, 0x00000000, 0x00000000, 0x00000004},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000000, 0x00000000, 0x00080800, 0x00000000}},   
    //  I, J, K, L
    {.bank = 2, .irq = INT_GPIO3, 
        .cnf        = {0x000000a1, 0x00000045, 0x00000038, 0x00000000},
        .out        = {0x00000080, 0x00000041, 0x00000018, 0x00000000}, 
        .oe         = {0x00000080, 0x00000005, 0x00000038, 0x00000000},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00202000, 0x00000000, 0x00000000, 0x00000000}},   
    //  M, N, O, P
    {.bank = 3, .irq = INT_GPIO4, 
        .cnf        = {0x00000000, 0x00000050, 0x00000021, 0x00000000},
        .out        = {0x00000000, 0x00000050, 0x00000000, 0x00000000}, 
        .oe         = {0x00000000, 0x00000050, 0x00000001, 0x00000000},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000000, 0x00080800, 0x00002020, 0x00000000}},   
    //  Q, R, S, T  
    {.bank = 4, .irq = INT_GPIO5, 
        .cnf        = {0x0000001b, 0x000000FB, 0x00000007, 0x00000011},
        .out        = {0x00000003, 0x00000080, 0x00000002, 0x00000000}, 
        .oe         = {0x0000001b, 0x00000049, 0x00000003, 0x00000011},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000000, 0x00008000, 0x00000000, 0x00000000}},   
    //  U, V, W, X     
    {.bank = 5, .irq = INT_GPIO6, 
        .cnf        = {0x0000007f, 0x00000087, 0x00000000, 0x00000060},
        .out        = {0x00000000, 0x00000002, 0x00000000, 0x00000040}, 
        .oe         = {0x0000001e, 0x00000082, 0x00000000, 0x00000000},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000100, 0x00000000, 0x00000000, 0x00004000}},   
    // Y, Z, AA, AB    
    {.bank = 6, .irq = INT_GPIO7, 
        .cnf        = {0x00000000, 0x00000000, 0x00000000, 0x00000001},
        .out        = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .oe         = {0x00000000, 0x00000000, 0x00000000, 0x00000001},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}},   
};
#else
                    //  PORT 0   ,  PORT 1   ,   PORT2   ,   PORT3        
    //  A, B, C, D        
    {.bank = 0, .irq = INT_GPIO1, 
        .cnf        = {0x00000000, 0x00000008, 0x00000000, 0x00000001},
        .out        = {0x00000000, 0x00000008, 0x00000000, 0x00000000}, 
        .oe         = {0x00000000, 0x00000008, 0x00000000, 0x00000001},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}}, 
    //  E, F, G, H
    {.bank = 1, .irq = INT_GPIO2, 
        .cnf        = {0x000000ff, 0x00000000, 0x0000000b, 0x00000000},
        .out        = {0x0000001f, 0x00000000, 0x00000000, 0x00000000}, 
        .oe         = {0x000000ff, 0x00000000, 0x00000000, 0x00000000},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000000, 0x00000000, 0x00080800, 0x00000000}},   
    //  I, J, K, L
    {.bank = 2, .irq = INT_GPIO3, 
        .cnf        = {0x000000a1, 0x00000005, 0x00000038, 0x00000000},
        .out        = {0x00000080, 0x00000001, 0x00000018, 0x00000000}, 
        .oe         = {0x00000080, 0x00000005, 0x00000038, 0x00000000},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00202000, 0x00000000, 0x00000000, 0x00000000}},   
    //  M, N, O, P
    {.bank = 3, .irq = INT_GPIO4, 
        .cnf        = {0x00000000, 0x00000050, 0x00000021, 0x00000000},
        .out        = {0x00000000, 0x00000050, 0x00000000, 0x00000000}, 
        .oe         = {0x00000000, 0x00000050, 0x00000001, 0x00000000},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000000, 0x00080800, 0x00002020, 0x00000000}},   
    //  Q, R, S, T  
    {.bank = 4, .irq = INT_GPIO5, 
        .cnf        = {0x0000001b, 0x000000FB, 0x00000007, 0x00000011},
        .out        = {0x00000003, 0x00000000, 0x00000002, 0x00000000}, 
        .oe         = {0x0000001b, 0x00000049, 0x00000002, 0x00000011},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000000, 0x00008000, 0x00000000, 0x00000000}},   
    //  U, V, W, X     
    {.bank = 5, .irq = INT_GPIO6, 
        .cnf        = {0x0000003f, 0x00000081, 0x00000000, 0x00000060},
        .out        = {0x00000000, 0x00000001, 0x00000000, 0x00000040}, 
        .oe         = {0x0000001e, 0x00000081, 0x00000000, 0x00000000},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000100, 0x00000000, 0x00000000, 0x00004000}},   
    // Y, Z, AA, AB    
    {.bank = 6, .irq = INT_GPIO7, 
        .cnf        = {0x00000000, 0x00000000, 0x00000000, 0x00000001},
        .out        = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .oe         = {0x00000000, 0x00000000, 0x00000000, 0x00000001},  
        .int_enb    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}, 
        .int_lvl    = {0x00000000, 0x00000000, 0x00000000, 0x00000000}},   
};
#endif 
//20100724  for gpio setting while sleep [LGE_END]
#endif

static int tegra_gpio_compose(int bank, int port, int bit)
{
	return (bank << 5) | ((port & 0x3) << 3) | (bit & 0x7);
}

static void tegra_gpio_mask_write(u32 reg, int gpio, int value)
{
	u32 val;

	val = 0x100 << GPIO_BIT(gpio);
	if (value)
		val |= 1 << GPIO_BIT(gpio);
	__raw_writel(val, reg);
}

static void tegra_set_gpio_tristate(int gpio_nr, tegra_tristate_t ts)
{
	tegra_pingroup_t pg;
	int err;

	pg = gpio_get_pinmux_group(gpio_nr);
	WARN_ON(pg < 0);
	if (pg >= 0) {
		err = tegra_pinmux_set_tristate(pg, ts);
		if (err < 0)
			printk(KERN_ERR "pinmux: can't set pingroup %d tristate"
					" to %d: %d\n", pg, ts, err);
	}
}

void tegra_gpio_enable(int gpio)
{
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 1);
}

void tegra_gpio_disable(int gpio)
{
	tegra_gpio_mask_write(GPIO_MSK_CNF(gpio), gpio, 0);
}

static int tegra_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	WARN_ON(tegra_gpio_io_power_config(offset, 1) != 0);
	tegra_gpio_enable(offset);
	tegra_set_gpio_tristate(offset, TEGRA_TRI_NORMAL);
	return 0;
}

static void tegra_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	tegra_gpio_disable(offset);
	tegra_set_gpio_tristate(offset, TEGRA_TRI_TRISTATE);
	WARN_ON(tegra_gpio_io_power_config(offset, 0) != 0);
}

static void tegra_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	tegra_gpio_mask_write(GPIO_MSK_OUT(offset), offset, value);
}

static int tegra_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	return (__raw_readl(GPIO_IN(offset)) >> GPIO_BIT(offset)) & 0x1;
}

static int tegra_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	tegra_gpio_mask_write(GPIO_MSK_OE(offset), offset, 0);
	return 0;
}

static int tegra_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					int value)
{
	tegra_gpio_set(chip, offset, value);
	tegra_gpio_mask_write(GPIO_MSK_OE(offset), offset, 1);
	return 0;
}



static struct gpio_chip tegra_gpio_chip = {
	.label			= "tegra-gpio",
	.direction_input	= tegra_gpio_direction_input,
	.get			= tegra_gpio_get,
	.direction_output	= tegra_gpio_direction_output,
	.set			= tegra_gpio_set,
	.request		= tegra_gpio_request,
	.free			= tegra_gpio_free,
	.base			= 0,
	.ngpio			= ARCH_NR_GPIOS,
};

static void tegra_gpio_irq_ack(unsigned int irq)
{
	int gpio = irq - INT_GPIO_BASE;

	__raw_writel(1 << GPIO_BIT(gpio), GPIO_INT_CLR(gpio));
}

static void tegra_gpio_irq_mask(unsigned int irq)
{
	int gpio = irq - INT_GPIO_BASE;

	tegra_gpio_mask_write(GPIO_MSK_INT_ENB(gpio), gpio, 0);
}

static void tegra_gpio_irq_unmask(unsigned int irq)
{
	int gpio = irq - INT_GPIO_BASE;

	tegra_gpio_mask_write(GPIO_MSK_INT_ENB(gpio), gpio, 1);
}

// 20110209  disable gpio interrupt during power-off  [START] 

static const unsigned int disable_interrupt_list[] = 
{
    TEGRA_GPIO_PV2,  // power_key
    TEGRA_GPIO_PW3,  // BATT_LOW_INT
  #if defined(CONFIG_MACH_STAR_SKT_REV_D)     
    TEGRA_GPIO_PN5, //  HOOK_DET
  #else
    TEGRA_GPIO_PD3,  //  HOOK_DET
  #endif
    TEGRA_GPIO_PU5,    // HALL_INT
    TEGRA_GPIO_PU0,   // INT_N_MUIC
    TEGRA_GPIO_PR4,  // COM_INT
    TEGRA_GPIO_PQ5,  // GYRO_INT_N
    TEGRA_GPIO_PK2,   // THERMAL_IRQ
    TEGRA_GPIO_PG0,   // VOL_KEY_UP
    TEGRA_GPIO_PG1,   // VOL_KEY_DOWN
    TEGRA_GPIO_PG3,   // EARJACK_SENSE
    TEGRA_GPIO_PI0,   // MOTION_INT
    TEGRA_GPIO_PX6,  //  TOUCH_INT
    TEGRA_GPIO_PR2,  // PROXI_OUT
#if defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)	
    TEGRA_GPIO_PJ6,  // LOWER_TOUCH_INT/ (SU660)
#endif
#if defined(STAR_COUNTRY_KR) && defined(STAR_OPERATOR_SKT)
#if defined(CONFIG_MACH_STAR_SKT_REV_E) || defined(CONFIG_MACH_STAR_SKT_REV_F) 
    TEGRA_GPIO_PV6,  // HomeKey (SU660)
#endif
#endif
    TEGRA_GPIO_PW3,  // BATT_LOW_INT
    TEGRA_GPIO_PV3,	// MDM_RESET_FLAG +
#ifdef CONFIG_MACH_STAR_TMUS
    TEGRA_GPIO_PU6,	// SDRY1
#else
    TEGRA_GPIO_PR1,    // IFX2_AP20 (LGP990)
#endif
    0xFFFF,		      // LAST MARK
};


void tegra_gpio_disable_all_irq(void)
{
	int val=0;	
	int i=0;
	
	while(  TEGRA_GPIO_PBB7 >= disable_interrupt_list[i]  && disable_interrupt_list[i] >= 0 )
	{
		printk("[PowerOff] tegra_gpio_disable_all_irq() :: gpio = %d \n", disable_interrupt_list[i] );
		tegra_gpio_mask_write(GPIO_MSK_INT_ENB(disable_interrupt_list[i]), disable_interrupt_list[i], 0);
		i++;
	}

}
// 20110209  disable gpio interrupt during power-off  [END]

static int tegra_gpio_irq_set_type(unsigned int irq, unsigned int type)
{
	int gpio = irq - INT_GPIO_BASE;
	struct tegra_gpio_bank *bank = get_irq_chip_data(irq);
	int port = GPIO_PORT(gpio);
	int lvl_type;
	int val;
	unsigned long flags;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		lvl_type = GPIO_INT_LVL_EDGE_RISING;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		lvl_type = GPIO_INT_LVL_EDGE_FALLING;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		lvl_type = GPIO_INT_LVL_EDGE_BOTH;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		lvl_type = GPIO_INT_LVL_LEVEL_HIGH;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		lvl_type = GPIO_INT_LVL_LEVEL_LOW;
		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&bank->lvl_lock[port], flags);

	val = __raw_readl(GPIO_INT_LVL(gpio));
	val &= ~(GPIO_INT_LVL_MASK << GPIO_BIT(gpio));
	val |= lvl_type << GPIO_BIT(gpio);
	__raw_writel(val, GPIO_INT_LVL(gpio));

	spin_unlock_irqrestore(&bank->lvl_lock[port], flags);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		__set_irq_handler_unlocked(irq, handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		__set_irq_handler_unlocked(irq, handle_edge_irq);

	return 0;
}

static void tegra_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct tegra_gpio_bank *bank;
	int port;
	int pin;
	int unmasked = 0;

	desc->chip->ack(irq);

	bank = get_irq_data(irq);

	for (port = 0; port < 4; port++) {
		int gpio = tegra_gpio_compose(bank->bank, port, 0);
		unsigned long sta = __raw_readl(GPIO_INT_STA(gpio)) &
			__raw_readl(GPIO_INT_ENB(gpio));
		u32 lvl = __raw_readl(GPIO_INT_LVL(gpio));

		for_each_bit(pin, &sta, 8) {
			__raw_writel(1 << pin, GPIO_INT_CLR(gpio));

			/* if gpio is edge triggered, clear condition
			 * before executing the hander so that we don't
			 * miss edges
			 */
			if (lvl & (0x100 << pin)) {
				unmasked = 1;
				desc->chip->unmask(irq);
			}

			generic_handle_irq(gpio_to_irq(gpio + pin));
		}
	}

	if (!unmasked)
		desc->chip->unmask(irq);

}

#ifdef CONFIG_PM
void tegra_gpio_resume(void)
{
	unsigned long flags;
	int b, p, i;

	local_irq_save(flags);
#if SLEEP_GPIO_LOG
	pr_info("[POWER] <<< Resume GPIO Setting [START] >>>  \n");
#endif
	for (b=0; b<ARRAY_SIZE(tegra_gpio_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		for (p=0; p<ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			__raw_writel(bank->cnf[p], GPIO_CNF(gpio));
			__raw_writel(bank->out[p], GPIO_OUT(gpio));
			__raw_writel(bank->oe[p], GPIO_OE(gpio));
			__raw_writel(bank->int_lvl[p], GPIO_INT_LVL(gpio));
			__raw_writel(bank->int_enb[p], GPIO_INT_ENB(gpio));
#if SLEEP_GPIO_LOG			
			pr_info("%d:%d %02x %02x %02x %02x %06x\n", b, p, __raw_readl(GPIO_CNF(gpio)), 
				__raw_readl(GPIO_OUT(gpio)), __raw_readl(GPIO_OE(gpio)), __raw_readl(GPIO_INT_ENB(gpio)),
				__raw_readl(GPIO_INT_LVL(gpio))	);
#endif			
		}

	}
#if SLEEP_GPIO_LOG
	pr_info("[POWER] <<< Resume GPIO Setting [END] >>>  \n");
#endif
	local_irq_restore(flags);

	for (i=INT_GPIO_BASE; i<(INT_GPIO_BASE+ARCH_NR_GPIOS); i++) {
		struct irq_desc *desc = irq_to_desc(i);
		if (!desc || (desc->status & IRQ_WAKEUP)) continue;
		enable_irq(i);
	}
}

void tegra_gpio_suspend(void)
{
	unsigned long flags;
	int b, p, i;


	for (i=INT_GPIO_BASE; i<(INT_GPIO_BASE+ARCH_NR_GPIOS); i++) {
		struct irq_desc *desc = irq_to_desc(i);
		if (!desc) continue;
		if (desc->status & IRQ_WAKEUP) {
			int gpio = i - INT_GPIO_BASE;
			pr_debug("gpio %d.%d is wakeup\n", gpio/8, gpio&7);
			continue;
                }
		disable_irq(i);
	}

	local_irq_save(flags);
#if SLEEP_GPIO_LOG
	pr_info("[POWER] <<< Suspend GPIO Setting value (before) [START] >>>  \n");
#endif
	for (b=0; b<ARRAY_SIZE(tegra_gpio_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		for (p=0; p<ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			bank->cnf[p] = __raw_readl(GPIO_CNF(gpio));
			bank->out[p] = __raw_readl(GPIO_OUT(gpio));
			bank->oe[p] = __raw_readl(GPIO_OE(gpio));
			bank->int_enb[p] = __raw_readl(GPIO_INT_ENB(gpio));
			bank->int_lvl[p] = __raw_readl(GPIO_INT_LVL(gpio));
#if SLEEP_GPIO_LOG
			pr_info("%d:%d %02x %02x %02x %02x %06x\n", b, p, bank->cnf[p], bank->out[p],
				bank->oe[p], bank->int_enb[p], bank->int_lvl[p]);
#endif				
		}

	}
#if SLEEP_GPIO_LOG
	pr_info("\n[POWER] <<< Suspend GPIO Setting value (before) [END] >>>  \n");
#endif

#if APPLY_SLEEP_GPIO_TABLE
	//20100724  for gpio setting while sleep [LGE_START]
	for (b=0; b<ARRAY_SIZE(tegra_sleep_gpio_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_sleep_gpio_banks[b];

		for (p=0; p<ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			__raw_writel(bank->cnf[p], GPIO_CNF(gpio));
			__raw_writel(bank->oe[p], GPIO_OE(gpio));
            #if 1   // masked bit should be maintained current out status.
            if(bank->out[p] >> 16)
            {
                u32 expected_out = bank->out[p] & 0xFFFF;
                u32 current_out = __raw_readl(GPIO_OUT(gpio));
                current_out  &= (bank->out[p] >> 16);
                expected_out &= ~(bank->out[p] >> 16);
                expected_out |= current_out;
			    __raw_writel(expected_out, GPIO_OUT(gpio));
			}else{
				__raw_writel((bank->out[p] & 0xFFFF), GPIO_OUT(gpio));
			}
            #else
			__raw_writel(bank->out[p], GPIO_OUT(gpio));
            #endif
			__raw_writel(bank->int_lvl[p], GPIO_INT_LVL(gpio));
			__raw_writel(bank->int_enb[p], GPIO_INT_ENB(gpio));
			//pr_info("%d:%d %02x %02x %02x %02x %06x\n", b, p, bank->cnf[p], bank->out[p],
			//	bank->oe[p], bank->int_enb[p], bank->int_lvl[p]);
		}
		
	}
#endif	
	//20100724  for gpio setting while sleep [LGE_END]

#if SLEEP_GPIO_LOG
	pr_info("[POWER] <<< Suspend GPIO Setting value (after) [START] >>>  \n");
	for (b=0; b<ARRAY_SIZE(tegra_gpio_banks); b++) {
		struct tegra_gpio_bank *bank = &tegra_gpio_banks[b];

		for (p=0; p<ARRAY_SIZE(bank->oe); p++) {
			unsigned int gpio = (b<<5) | (p<<3);
			
			pr_info("%d:%d %02x %02x %02x %02x %06x\n", b, p, __raw_readl(GPIO_CNF(gpio)), 
				__raw_readl(GPIO_OUT(gpio)), __raw_readl(GPIO_OE(gpio)), __raw_readl(GPIO_INT_ENB(gpio)),
				__raw_readl(GPIO_INT_LVL(gpio))	);
		}

	}
	pr_info("\n[POWER] <<< Suspend GPIO Setting value (after) [END] >>>  \n");
#endif

	
	local_irq_restore(flags);
}

static int tegra_gpio_wake_enable(unsigned int irq, unsigned int enable)
{
	struct tegra_gpio_bank *bank = get_irq_chip_data(irq);
	return set_irq_wake(bank->irq, enable);
}
#endif

static struct irq_chip tegra_gpio_irq_chip = {
	.name		= "GPIO",
	.ack		= tegra_gpio_irq_ack,
	.mask		= tegra_gpio_irq_mask,
	.unmask		= tegra_gpio_irq_unmask,
	.set_type	= tegra_gpio_irq_set_type,
#ifdef CONFIG_PM
	.set_wake	= tegra_gpio_wake_enable,
#endif
};


/* This lock class tells lockdep that GPIO irqs are in a different
 * category than their parents, so it won't report false recursion.
 */
static struct lock_class_key gpio_lock_class;

static int __init tegra_gpio_init(void)
{
	struct tegra_gpio_bank *bank;
	int i;
	int j;
    int b, p;

#if APPLY_SLEEP_GPIO_TABLE
    extern u32 sleep_pinmux_reg[TRISTATE_REG_NUM + PIN_MUX_CTL_REG_NUM + PULLUPDOWN_REG_NUM];

	for (i = 0; i < 7; i++) {
		for (j = 0; j < 4; j++) {
			tegra_sleep_gpio_banks[i].cnf[j] = 0;
            tegra_sleep_gpio_banks[i].oe[j] = 0;
            tegra_sleep_gpio_banks[i].out[j] = 0;
            tegra_sleep_gpio_banks[i].int_enb[j] = 0;
            tegra_sleep_gpio_banks[i].int_lvl[j] = 0;
		}
	}

    for (i=0; i<ARRAY_SIZE(tegra_sleep_gpio_info_array); i++) {
        // disable tristate group
        if( tegra_sleep_gpio_info_array[i].port == 0xFF ||
            tegra_sleep_gpio_info_array[i].oe == GPIO_OUTPUT)
        {
            u32 tristate_reg_num = tegra_sleep_gpio_info_array[i].group >> 16;
            u32 tristate_reg_bit = tegra_sleep_gpio_info_array[i].group & 0xFFFF;
            sleep_pinmux_reg[tristate_reg_num] &= ~((u32)(0x1 << tristate_reg_bit));
        }

        // It's not GPIO pin, just set tristate.
        if(tegra_sleep_gpio_info_array[i].port != 0xFF){
            b = (tegra_sleep_gpio_info_array[i].port) >> 2;
            p = (tegra_sleep_gpio_info_array[i].port) & 0x3;

            tegra_sleep_gpio_banks[b].cnf[p] |= (tegra_sleep_gpio_info_array[i].cnf)<<(tegra_sleep_gpio_info_array[i].pin);
            tegra_sleep_gpio_banks[b].oe[p] |= (tegra_sleep_gpio_info_array[i].oe) <<(tegra_sleep_gpio_info_array[i].pin);
            tegra_sleep_gpio_banks[b].out[p] |= (tegra_sleep_gpio_info_array[i].out)<<(tegra_sleep_gpio_info_array[i].pin);
        }
    }
#endif

#if APPLY_GPIO_INIT
    for (i=0; i<ARRAY_SIZE(tegra_init_gpio_info_array); i++) {
		unsigned int gpio = (tegra_init_gpio_info_array[i].port<<3);
        u32 current_val;
        u32 expect_bit;
        current_val = __raw_readl(GPIO_CNF(gpio));
		expect_bit = (tegra_init_gpio_info_array[i].cnf)<<(tegra_init_gpio_info_array[i].pin);
        current_val &= ~expect_bit;
		current_val |= expect_bit;
		__raw_writel(current_val, GPIO_CNF(gpio));
        
		current_val = __raw_readl(GPIO_OE(gpio));
		expect_bit = (tegra_init_gpio_info_array[i].oe)<<(tegra_init_gpio_info_array[i].pin);
        current_val &= ~expect_bit;
		current_val |= expect_bit;
		__raw_writel(current_val, GPIO_OE(gpio)); 
        
        current_val = __raw_readl(GPIO_OUT(gpio));
		expect_bit = (tegra_init_gpio_info_array[i].out)<<(tegra_init_gpio_info_array[i].pin);
        current_val &= ~expect_bit;
		current_val |= expect_bit;
		__raw_writel(current_val, GPIO_OUT(gpio));
	}
#endif
	for (i = 0; i < 7; i++) {
		for (j = 0; j < 4; j++) {
			int gpio = tegra_gpio_compose(i, j, 0);
			__raw_writel(0x00, GPIO_INT_ENB(gpio));
		}
	}

	gpiochip_add(&tegra_gpio_chip);

	for (i = INT_GPIO_BASE; i < (INT_GPIO_BASE + ARCH_NR_GPIOS); i++) {
		bank = &tegra_gpio_banks[GPIO_BANK(irq_to_gpio(i))];

		lockdep_set_class(&irq_desc[i].lock, &gpio_lock_class);
		set_irq_chip_data(i, bank);
		set_irq_chip(i, &tegra_gpio_irq_chip);
		set_irq_handler(i, handle_simple_irq);
		set_irq_flags(i, IRQF_VALID);
	}

	for (i = 0; i < ARRAY_SIZE(tegra_gpio_banks); i++) {
		bank = &tegra_gpio_banks[i];

		set_irq_chained_handler(bank->irq, tegra_gpio_irq_handler);
		set_irq_data(bank->irq, bank);

		for (j = 0; j < 4; j++)
			spin_lock_init(&bank->lvl_lock[j]);
	}

	return 0;
}

postcore_initcall(tegra_gpio_init);

#ifdef	CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

//20100724  for gpio setting while sleep [LGE_START]
#include <asm/uaccess.h>
#include <linux/io.h>
//20100724  for gpio setting while sleep [LGE_END]

static int dbg_gpio_show(struct seq_file *s, void *unused)
{
	int i;
	int j;

	//20100724  for gpio setting while sleep [LGE_START]
	if ( gpio_dbgfs_mode == NORMAL_MODE )								 
	{								
		seq_printf(s, "ctrl:port CNF OE OUT IN INT_STA INT_ENB INT_LVL (hex) \n");
	}
	else
	{
		seq_printf(s, "ctrl:port CNF OE OUT INT_ENB INT_LVL (hex) \n");
	}
	//20100724  for gpio setting while sleep [LGE_END]
	
	for (i = 0; i < 7; i++) {
		for (j = 0; j < 4; j++) {
			int gpio = tegra_gpio_compose(i, j, 0);
			//20100724  for gpio setting while sleep [LGE_START]
			if ( gpio_dbgfs_mode == NORMAL_MODE )								 
			{								
			seq_printf(s, "%d:%d %02x %02x %02x %02x %02x %02x %06x\n",
			       i, j,
			       __raw_readl(GPIO_CNF(gpio)),
			       __raw_readl(GPIO_OE(gpio)),
			       __raw_readl(GPIO_OUT(gpio)),
			       __raw_readl(GPIO_IN(gpio)),
			       __raw_readl(GPIO_INT_STA(gpio)),
			       __raw_readl(GPIO_INT_ENB(gpio)),
			       __raw_readl(GPIO_INT_LVL(gpio)));
		}
			else
			{
				seq_printf(s, "%d:%d %02x %02x %02x %02x %06x \n",
				       i, j,
				       tegra_sleep_gpio_banks[i].cnf[j],
				       tegra_sleep_gpio_banks[i].oe[j],
				       tegra_sleep_gpio_banks[i].out[j],
				       tegra_sleep_gpio_banks[i].int_enb[j],
				       tegra_sleep_gpio_banks[i].int_lvl[j]);
			}
			//20100724  for gpio setting while sleep [LGE_START]
		}
	}
	return 0;
}

static int dbg_gpio_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_gpio_show, &inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_gpio_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


//20100724  for gpio setting while sleep [LGE_START]
typedef struct  {
    int port_num;
    
}dbg_port_info;


static int dbg_open_port(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
            return 0;
}
static ssize_t dbg_get_port(struct file *file, char __user *userbuf,
                                size_t count, loff_t *ppos)
{
        dbg_port_info *port_info = file->private_data;
        char buf[64];
        int ret, port, pin, gpio;
        
        port = (int)port_info->port_num / 4;                            
        pin = port_info->port_num % 4;                                  
        gpio = tegra_gpio_compose(port, pin, 0);
	
        ret= snprintf(buf, sizeof(buf) - 1, "[CTRL=%d ,Port=%d] CNF=0x%x, OE=0x%x, OUT=0x%x \n",
            (port_info->port_num/4), port_info->port_num%4, 
            get_gpio_reg_data(port, pin, gpio, REG_CNF),
            get_gpio_reg_data(port, pin, gpio, REG_OE),
            get_gpio_reg_data(port, pin, gpio, REG_OUT)); 

        return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}

static ssize_t dbg_set_port(struct file *file, const char __user *ubuf,
                                size_t count, loff_t *ppos)
{
        dbg_port_info *port_info = file->private_data;
        char buf[16];
        long result;
		int len;
        memset(buf, 0, sizeof(buf));

        if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
            return -EFAULT;

		len = strlen(buf);
		buf[len-1]=NULL;
		if (strcmp(buf,"AA")==0)
		{
			result = 26;
		}
		else if (strcmp(buf,"BB")==0)
		{
			result = 27;
		}
		else 
		{
			result = (int)buf[0]-65;
		}
				  
        port_info->port_num = result;
        return count;
}
static const struct file_operations fops_port = {
    .read = dbg_get_port,
    .write = dbg_set_port,
    .open = dbg_open_port,
};

int gpio_dbgfs_mode=0;   // 0=normal, 1=sleep

static int dbg_open_mode(struct inode *inode, struct file *file)
{
        file->private_data = inode->i_private;
            return 0;
}
static ssize_t dbg_get_mode(struct file *file, char __user *userbuf,
                                size_t count, loff_t *ppos)
{
        char buf[64];
        int ret;
        
        ret= snprintf(buf, sizeof(buf) - 1, "Selected mode is %d (0=normal, 1=sleep) \n", gpio_dbgfs_mode); 

        return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}
static ssize_t dbg_set_mode(struct file *file, const char __user *ubuf,
                                size_t count, loff_t *ppos)
{
        char buf[16];
        long result=0;
        int len;
        memset(buf, 0, sizeof(buf));

        if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
            return -EFAULT;

        len = strlen(buf);
        buf[len-1]=NULL;
        if (strcmp(buf,"1")==0 || strcmp(buf,"sleep")==0 || strcmp(buf,"SLEEP")==0){
            result = 1;
        }
                  
        gpio_dbgfs_mode = result;
        return count;
}
static const struct file_operations fops_mode = {
    .read = dbg_get_mode,
    .write = dbg_set_mode,
    .open = dbg_open_mode,
};

int get_gpio_reg_data(int port, int pin, int gpio, int reg)
{
	int data=0;
	
    if ( gpio_dbgfs_mode == NORMAL_MODE )                                
    {                               
        switch( reg )
        {
            case REG_CNF : 
                data = __raw_readl(GPIO_CNF(gpio));
                break;
            case REG_OE : 
                data = __raw_readl(GPIO_OE(gpio));
                break;
            case REG_OUT : 
                data = __raw_readl(GPIO_OUT(gpio));
                break;
            case REG_INT_ENB : 
                data = __raw_readl(GPIO_INT_ENB(gpio));
                break;
            case REG_INT_LVL : 
                data = __raw_readl(GPIO_INT_LVL(gpio));
                break;
        }
    }                                                               
    else                                                            
    {
        switch( reg )
        {
            case REG_CNF : 
                data = tegra_sleep_gpio_banks[port].cnf[pin];
                break;
            case REG_OE : 
                data = tegra_sleep_gpio_banks[port].oe[pin];
                break;
            case REG_OUT : 
                data = tegra_sleep_gpio_banks[port].out[pin];
                break;
            case REG_INT_ENB : 
                data = tegra_sleep_gpio_banks[port].int_enb[pin];
                break;
            case REG_INT_LVL : 
                data = tegra_sleep_gpio_banks[port].int_lvl[pin];
                break;
        }
    }

	return data;
}

int dbg_get_process( struct file *file, char *buf, int reg )
{
    dbg_port_info *port_info = file->private_data;
    int port, pin, gpio, data=0;  
    
    port = (int)port_info->port_num / 4;                            
    pin = port_info->port_num % 4;                                  
    gpio = tegra_gpio_compose(port, pin, 0);

	data = get_gpio_reg_data(port, pin, gpio, reg);
	
    return snprintf(buf, DBG_BUF_SIZE-1, "Controller %d / Port %d : value=%02x\n"   
       ,port+1, pin, data);         
}

int dbg_set_process( struct file *file, const char __user *ubuf, int count, int reg )
{
    dbg_port_info *port_info = file->private_data;                  
    char buf[16];                                                   
    long result, ret;                                               
    int port, pin, gpio;    
    
    memset(buf, 0, sizeof(buf));                                    
    if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))      
        return -EFAULT;     
    
    ret = strict_strtol(buf, 16, &result);                          
    port = (int)port_info->port_num / 4;                            
    pin = port_info->port_num % 4;                                  
    gpio = tegra_gpio_compose(port, pin, 0);
    
    if ( gpio_dbgfs_mode == NORMAL_MODE )                            
    {   
        switch( reg )
        {
            case REG_CNF : 
                __raw_writel(result, GPIO_CNF(gpio));
                break;
            case REG_OE : 
                __raw_writel(result, GPIO_OE(gpio));
                break;
            case REG_OUT : 
                __raw_writel(result, GPIO_OUT(gpio));
                break;
            case REG_INT_ENB : 
                __raw_writel(result, GPIO_INT_ENB(gpio));
                break;
            case REG_INT_LVL : 
                __raw_writel(result, GPIO_INT_LVL(gpio));
                break;
        }
    }                                                               
    else                                                            
    {   
        switch( reg )
        {
            case REG_CNF : 
                tegra_sleep_gpio_banks[port].cnf[pin] = result;
                break;
            case REG_OE : 
                tegra_sleep_gpio_banks[port].oe[pin] = result;
                break;
            case REG_OUT : 
                tegra_sleep_gpio_banks[port].out[pin] = result;
                break;
            case REG_INT_ENB : 
                tegra_sleep_gpio_banks[port].int_enb[pin] = result;
                break;
            case REG_INT_LVL : 
                tegra_sleep_gpio_banks[port].int_lvl[pin] = result;
                break;
        }
    }                                                               
    return count;                                                   
}

#define DBG_GPIO_FOS(_reg)  \
static int dbg_open_ ## _reg(struct inode *inode, struct file *file)    \
{                                                                       \
        file->private_data = inode->i_private;                          \
        return 0;                                                       \
}                                                                       \
static ssize_t dbg_get_ ## _reg(struct file *file, char __user *userbuf,    \
                                size_t count, loff_t *ppos)             \
{                                                                       \
        char buf[DBG_BUF_SIZE];                                         \
        int ret;                                                        \
        ret = dbg_get_process(file, &buf[0], REG_ ## _reg);             \
        return simple_read_from_buffer(userbuf, count, ppos, buf, ret); \
}                                                                       \
static ssize_t dbg_set_ ## _reg(struct file *file, const char __user *ubuf, \
                                size_t count, loff_t *ppos)             \
{                                                                       \
        return dbg_set_process( file, ubuf, count, REG_ ## _reg);       \
}                                                                       \
static const struct file_operations fops_ ## _reg = {                   \
    .read = dbg_get_ ## _reg,                                           \
    .write = dbg_set_ ## _reg,                                          \
    .open = dbg_open_ ## _reg,                                          \
};

DBG_GPIO_FOS(CNF);
DBG_GPIO_FOS(OE);
DBG_GPIO_FOS(OUT);
DBG_GPIO_FOS(INT_ENB);
DBG_GPIO_FOS(INT_LVL);

void create_additional_debugfs(void)
{
    dbg_port_info *buff;
    buff =(dbg_port_info *) kmalloc(sizeof(dbg_port_info),GFP_KERNEL);
    buff->port_num = 0;

    debugfs_create_file("port",0666, NULL, buff, &fops_port);
    debugfs_create_file("mode",0666, NULL, buff, &fops_mode);
    
    debugfs_create_file("CNF",0666, NULL, buff, &fops_CNF);
    debugfs_create_file("OE",0666, NULL, buff, &fops_OE);
    debugfs_create_file("OUT",0666, NULL, buff, &fops_OUT);
    debugfs_create_file("INT_ENB",0666, NULL, buff, &fops_INT_ENB);
    debugfs_create_file("INT_LVL",0666, NULL, buff, &fops_INT_LVL);
}

//20100724  for gpio setting while sleep [LGE_END]

static int __init tegra_gpio_debuginit(void)
{
	(void) debugfs_create_file("tegra_gpio", S_IRUGO,
					NULL, NULL, &debug_fops);
	
	//20100724  for gpio setting while sleep [LGE_START]
	create_additional_debugfs();
	//20100724  for gpio setting while sleep [LGE_END]
	
	return 0;
}
late_initcall(tegra_gpio_debuginit);
#endif

static char *tegra_gpio_rail_names[] = {
	[TEGRA_VDDIO_BB]    = "vddio bb",
	[TEGRA_VDDIO_LCD]   = "vddio lcd",
	[TEGRA_VDDIO_VI]    = "vddio vi",
	[TEGRA_VDDIO_UART]  = "vddio uart",
	[TEGRA_VDDIO_DDR]   = "vddio ddr",
	[TEGRA_VDDIO_NAND]  = "vddio nand",
	[TEGRA_VDDIO_SYS]   = "vddio sys",
	[TEGRA_VDDIO_AUDIO] = "vddio audio",
	[TEGRA_VDDIO_SD]    = "vddio sd",
};

int tegra_gpio_io_power_config(int gpio_nr, unsigned int enable)
{
	struct regulator *regulator = NULL;
	tegra_pingroup_t pg;
	const char*  railName;
	int vddio_id;


	pg = gpio_get_pinmux_group(gpio_nr);
	if (pg < 0)
		return pg;
	vddio_id = tegra_pinmux_get_vddio(pg);
	if(vddio_id < 0)
		return vddio_id;

	railName = tegra_gpio_rail_names[vddio_id];

	regulator = regulator_get(NULL, railName);
	if (IS_ERR_OR_NULL(regulator)) {
		printk(KERN_ERR "Unable to get regulator for %s, vddid 0x%x\n",
		       railName, vddio_id);
		return PTR_ERR(regulator);
	}

	if (enable)
		regulator_enable(regulator);
	else
		regulator_disable(regulator);

	return 0;
}
