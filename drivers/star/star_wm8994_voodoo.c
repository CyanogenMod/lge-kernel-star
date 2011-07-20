#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"

#include "star_wm8994_voodoo.h"
#include "wm8994.h"

static star_wm8994_device *g_wm8994;

/* ASoC code compatibilty */
int codec = 0;

/* Voodoo Sound Samsung compatibilty */

bool bypass_write_hook = false;
short unsigned int debug_log_level = LOG_OFF;
unsigned short hp_level[2] = { CONFIG_SND_VOODOO_HP_LEVEL,
			       CONFIG_SND_VOODOO_HP_LEVEL };

// global active or kill switch
bool enable = false;

/*
 * Some functions borrowed from LG's star_wm8994.c
 */

static NvBool
WriteWolfsonRegister(star_wm8994_device * wm8994, NvU32 RegIndex, NvU32 Data)
{
	int i;
	NvOdmI2cStatus I2cTransStatus = NvOdmI2cStatus_Timeout;
	NvU8 pTxBuffer[4];
	NvOdmI2cTransactionInfo TransactionInfo;

	for (i = 0;
	     i < WM8994_I2C_RETRY_COUNT
	     && I2cTransStatus != NvOdmI2cStatus_Success; i++) {
		pTxBuffer[0] = (NvU8) ((RegIndex >> 8) & 0xFF);
		pTxBuffer[1] = (NvU8) (RegIndex & 0xFF);
		pTxBuffer[2] = (NvU8) ((Data >> 8) & 0xFF);
		pTxBuffer[3] = (NvU8) ((Data) & 0xFF);

		TransactionInfo.Address = wm8994->i2c_address;
		TransactionInfo.Buf = pTxBuffer;
		TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
		TransactionInfo.NumBytes = 4;

		I2cTransStatus =
		    NvOdmI2cTransaction(wm8994->h_gen2_i2c, &TransactionInfo, 1,
					400, WM8994_I2C_TIMEOUT);
	}

	if (I2cTransStatus == NvOdmI2cStatus_Success) {
		return NV_TRUE;
	}
	printk("[star wm8994 driver] i2c transaction error\n");
	return NV_FALSE;
}

static NvBool ReadWolfsonRegister(star_wm8994_device * wm8994, NvU32 RegIndex,
				  NvU32 * Data)
{
	int i;
	NvU8 *pReadBuffer;
	NvOdmI2cStatus status = NvOdmI2cStatus_Timeout;
	NvOdmI2cTransactionInfo *pTransactionInfo;

	pReadBuffer = NvOdmOsAlloc(2);
	if (!pReadBuffer) {
		return NV_FALSE;
	}

	pTransactionInfo = NvOdmOsAlloc(sizeof(NvOdmI2cTransactionInfo) * 2);
	if (!pTransactionInfo) {
		NvOdmOsFree(pReadBuffer);
		return NV_FALSE;
	}

	for (i = 0;
	     i < WM8994_I2C_RETRY_COUNT && status != NvOdmI2cStatus_Success;
	     i++) {
		pReadBuffer[0] = (NvU8) ((RegIndex >> 8) & 0xFF);
		pReadBuffer[1] = (NvU8) (RegIndex & 0xFF);

		pTransactionInfo[0].Address = wm8994->i2c_address;
		pTransactionInfo[0].Buf = pReadBuffer;
		pTransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
		pTransactionInfo[0].NumBytes = 2;

		pTransactionInfo[1].Address = (wm8994->i2c_address | 0x1);
		pTransactionInfo[1].Buf = pReadBuffer;
		pTransactionInfo[1].Flags = 0;
		pTransactionInfo[1].NumBytes = 2;

		status =
		    NvOdmI2cTransaction(wm8994->h_gen2_i2c, pTransactionInfo, 2,
					400, WM8994_I2C_TIMEOUT);
	}

	if (status != NvOdmI2cStatus_Success) {
		printk("NvOdmWM8994I2cRead Failed: %d\n", status);
		NvOdmOsFree(pReadBuffer);
		NvOdmOsFree(pTransactionInfo);
		return NV_FALSE;
	}

	*Data = (NvU32) ((pReadBuffer[0] << 8) | pReadBuffer[1]);

	NvOdmOsFree(pReadBuffer);
	NvOdmOsFree(pTransactionInfo);
	return NV_TRUE;
}

/* Custom Code */

unsigned int wm8994_read(int codec, unsigned int reg)
{
	NvU32 r_data = 0;
	ReadWolfsonRegister(g_wm8994, reg, &r_data);
	return r_data;
}

int wm8994_write(int codec, unsigned int reg, unsigned int value)
{
	if (!enable)
		return 0;

	if (debug_log(LOG_VERBOSE))
		printk("Voodoo sound: internal wm8994_write 0x%03X 0x%04X\n",
		       reg, value);

	WriteWolfsonRegister(g_wm8994, reg, value);
	return 0;
}

bool debug_log(short unsigned int level)
{
	if (debug_log_level >= level)
		return true;

	return false;
}

int hpvol(int channel)
{
	int vol;

	vol = hp_level[channel];

	if (vol > MAX_HP_AMP_LEVEL)
		return MAX_HP_AMP_LEVEL;

	return vol;
}


void write_hpvol(unsigned short l, unsigned short r)
{
	unsigned short val;

	// we don't need the Volume Update flag when sending the first volume
	val = (WM8994_HPOUT1L_MUTE_N | l);
	val |= WM8994_HPOUT1L_ZC;
	wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

	// this time we write the right volume plus the Volume Update flag.
	// This way, both volume are set at the same time
	val = (WM8994_HPOUT1_VU | WM8994_HPOUT1R_MUTE_N | r);
	val |= WM8994_HPOUT1L_ZC;
	wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
}

void update_hpvol(bool with_fade)
{
	unsigned short val;
	unsigned short i;
	short steps;
	int hp_level_old[2];
	unsigned short hp_level_registers[2] = { WM8994_LEFT_OUTPUT_VOLUME,
						 WM8994_RIGHT_OUTPUT_VOLUME };

	if (!with_fade) {
		bypass_write_hook = true;
		write_hpvol(hpvol(0), hpvol(1));
		bypass_write_hook = false;
		return;
	}

	// read previous levels
	for (i = 0; i < 2; i++) {
		val = wm8994_read(codec, hp_level_registers[i]);
		val &= ~(WM8994_HPOUT1_VU_MASK);
		val &= ~(WM8994_HPOUT1L_ZC_MASK);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK);
		hp_level_old[i] = val;

		if (hp_level_old[i] < 0)
			hp_level_old[i] = 0;

		if (debug_log(LOG_INFOS))
			printk("Voodoo sound: previous hp_level[%hu]: %d\n",
			       i, hp_level_old[i]);
	}

	// calculate number of steps for volume fade
	steps = hp_level[0] - hp_level_old[0];
	if (debug_log(LOG_INFOS))
		printk("Voodoo sound: volume change steps: %hd "
		       "start: %hu, end: %hu\n",
		       steps,
		       hp_level_old[0],
		       hp_level[0]);

	while (steps != 0) {
		if (hp_level[0] < hp_level_old[0])
			steps++;
		else
			steps--;

		if (debug_log(LOG_INFOS))
			printk("Voodoo sound: volume: %hu\n",
			       (hpvol(0) - steps));

		bypass_write_hook = true;
		write_hpvol(hpvol(0) - steps, hpvol(1) - steps);
		bypass_write_hook = false;

		if (steps != 0)
			udelay(1000);
	}

}

unsigned int voodoo_hook_wm8994_write(int codec,
				      unsigned int reg, unsigned int value)
{
	// kill switch
	if (!enable)
		return value;

	if (reg == WM8994_LEFT_OUTPUT_VOLUME)
		value =
		    (WM8994_HPOUT1_VU |
		     WM8994_HPOUT1L_MUTE_N |
		     hpvol(0));

	if (reg == WM8994_RIGHT_OUTPUT_VOLUME)
		value =
		    (WM8994_HPOUT1_VU |
		     WM8994_HPOUT1R_MUTE_N |
		     hpvol(1));

	return value;
}


void update_enable()
{
	if (enable)
		voodoo_sound_misc_register();
	else
		voodoo_sound_misc_deregister();
}

/*
 * misc control functions
 */

#define DECLARE_BOOL_SHOW(name) 					       \
static ssize_t name##_show(struct device *dev,				       \
struct device_attribute *attr, char *buf)				       \
{									       \
	return sprintf(buf,"%u\n",(name ? 1 : 0));			       \
}

#ifndef MODULE
DECLARE_BOOL_SHOW(enable);
static ssize_t enable_store(struct device *dev,
			    struct device_attribute *attr, const char *buf,
			    size_t size)
{
	unsigned short state;
	bool bool_state;
	if (sscanf(buf, "%hu", &state) == 1) {
		bool_state = state == 0 ? false : true;
		if (state != enable) {
			enable = bool_state;
			update_enable();
		}
	}
	return size;
}
#endif

static ssize_t debug_log_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	return sprintf(buf, "%u\n", debug_log_level);
}

static ssize_t debug_log_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	sscanf(buf, "%hu", &debug_log_level);
	return size;
}

static ssize_t headphone_amplifier_level_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	// output median of left and right headphone amplifier volumes
	return sprintf(buf, "%u\n", (hp_level[0] + hp_level[1]) / 2);
}

static ssize_t headphone_amplifier_level_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t size)
{
	unsigned short vol;
	if (sscanf(buf, "%hu", &vol) == 1) {

		// hard limit to MAX_HP_AMP_LEVEL because 63 introduces distortions
		if (vol > MAX_HP_AMP_LEVEL)
			vol = MAX_HP_AMP_LEVEL;

		// left and right are set to the same volumes by this control
		hp_level[0] = hp_level[1] = vol;

		update_hpvol(true);
	}
	return size;
}

static ssize_t headphone_amplifier_level_max_show(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	return sprintf(buf, "%u\n", MAX_HP_AMP_LEVEL);
}

static ssize_t show_wm8994_register_dump(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	// modified version of register_dump from wm8994_aries.c
	// r = wm8994 register
	int r;

	for (r = 0; r <= 0x6; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x15, wm8994_read(codec, 0x15));

	for (r = 0x18; r <= 0x3C; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x4C, wm8994_read(codec, 0x4C));

	for (r = 0x51; r <= 0x5C; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x60, wm8994_read(codec, 0x60));
	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x101, wm8994_read(codec, 0x101));
	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x110, wm8994_read(codec, 0x110));
	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x111, wm8994_read(codec, 0x111));

	for (r = 0x200; r <= 0x212; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x220; r <= 0x224; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x240; r <= 0x244; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x300; r <= 0x317; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x400; r <= 0x411; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x420; r <= 0x423; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x440; r <= 0x444; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x450; r <= 0x454; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x480; r <= 0x493; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x4A0; r <= 0x4B3; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x500; r <= 0x503; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x510, wm8994_read(codec, 0x510));
	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x520, wm8994_read(codec, 0x520));
	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x521, wm8994_read(codec, 0x521));

	for (r = 0x540; r <= 0x544; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x580; r <= 0x593; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	for (r = 0x600; r <= 0x614; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x620, wm8994_read(codec, 0x620));
	sprintf(buf, "%s0x%X 0x%X\n", buf, 0x621, wm8994_read(codec, 0x621));

	for (r = 0x700; r <= 0x70A; r++)
		sprintf(buf, "%s0x%X 0x%X\n", buf, r, wm8994_read(codec, r));

	return sprintf(buf, "%s", buf);
}

static ssize_t store_wm8994_write(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	short unsigned int reg = 0;
	short unsigned int val = 0;
	int unsigned bytes_read = 0;

	while (sscanf(buf, "%hx %hx%n", &reg, &val, &bytes_read) == 2) {
		buf += bytes_read;
		printk("Voodoo sound: read from sysfs: %X, %X\n", reg, val);

		bypass_write_hook = true;
		wm8994_write(codec, reg, val);
		bypass_write_hook = false;
	}
	return size;
}


static ssize_t voodoo_sound_version(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", VOODOO_SOUND_VERSION);
}

static DEVICE_ATTR(debug_log, S_IRUGO | S_IWUGO,
		   debug_log_show,
		   debug_log_store);

static DEVICE_ATTR(headphone_amplifier_level, S_IRUGO | S_IWUGO,
		   headphone_amplifier_level_show,
		   headphone_amplifier_level_store);

static DEVICE_ATTR(headphone_amplifier_level_max, S_IRUGO,
		   headphone_amplifier_level_max_show,
		   NULL);

static DEVICE_ATTR(wm8994_register_dump, S_IRUGO,
		   show_wm8994_register_dump,
		   NULL);

static DEVICE_ATTR(wm8994_write, S_IWUSR,
		   NULL,
		   store_wm8994_write);

#ifdef MODULE
static DEVICE_ATTR(module, 0,
		   NULL,
		   NULL);
#endif

static DEVICE_ATTR(version, S_IRUGO,
		   voodoo_sound_version,
		   NULL);

#ifndef MODULE
static DEVICE_ATTR(enable, S_IRUGO | S_IWUGO,
		   enable_show,
		   enable_store);
#endif

static struct attribute *voodoo_sound_attributes[] = {
	&dev_attr_debug_log.attr,
	&dev_attr_headphone_amplifier_level.attr,
	&dev_attr_headphone_amplifier_level_max.attr,
	&dev_attr_wm8994_register_dump.attr,
	&dev_attr_wm8994_write.attr,
#ifdef MODULE
	&dev_attr_module.attr,
#endif
	&dev_attr_version.attr,
	NULL
};

#ifndef MODULE
static struct attribute *voodoo_sound_control_attributes[] = {
	&dev_attr_enable.attr,
	NULL
};
#endif

static struct attribute_group voodoo_sound_group = {
	.attrs = voodoo_sound_attributes,
};

#ifndef MODULE
static struct attribute_group voodoo_sound_control_group = {
	.attrs = voodoo_sound_control_attributes,
};
#endif

static struct miscdevice voodoo_sound_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "voodoo_sound",
};

#ifndef MODULE
static struct miscdevice voodoo_sound_control_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "voodoo_sound_control",
};
#endif

/**
 * All the device spefic initializations happen here. 
 */

void voodoo_sound_misc_register() {
	misc_register(&voodoo_sound_device);
	if (sysfs_create_group(&voodoo_sound_device.this_device->kobj,
			       &voodoo_sound_group) < 0) {
		printk("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for (%s)!\n",
		       voodoo_sound_device.name);
	}
}

void voodoo_sound_misc_deregister(void) {
	sysfs_remove_group(&voodoo_sound_device.this_device->kobj,
			   &voodoo_sound_group);
	misc_deregister(&voodoo_sound_device);
}

int voodoo_sound_init(void)
{
	NvS32 err = 0;

	const NvOdmPeripheralConnectivity *pcon = NULL;

	printk("Voodoo sound: initializing driver v%d\n", VOODOO_SOUND_VERSION);

	pcon =
	    NvOdmPeripheralGetGuid(NV_ODM_GUID
				   ('w', 'o', 'l', 'f', '8', '9', '9', '4'));
	if (pcon == NULL) {
		return err;
	}
	g_wm8994 = kzalloc(sizeof(*g_wm8994), GFP_KERNEL);
	g_wm8994->i2c_address = pcon->AddressList[2].Address;
	g_wm8994->h_gen2_i2c =
	    NvOdmI2cPinMuxOpen(NvOdmIoModule_I2c, 1, NvOdmI2cPinMap_Config2);

	if (g_wm8994->h_gen2_i2c == NULL) {
		return err;
	}

	voodoo_sound_misc_register();

#ifndef MODULE
	misc_register(&voodoo_sound_control_device);
	if (sysfs_create_group(&voodoo_sound_control_device.this_device->kobj,
			       &voodoo_sound_control_group) < 0) {
		printk("%s sysfs_create_group fail\n", __FUNCTION__);
		pr_err("Failed to create sysfs group for device (%s)!\n",
		       voodoo_sound_control_device.name);
	}
#endif
	enable = true;

	return 0;
}

void voodoo_sound_exit(void)
{
	printk("Voodoo sound: removing driver v%d\n", VOODOO_SOUND_VERSION);
	voodoo_sound_misc_deregister();
}

#ifndef MODULE
module_init(voodoo_sound_init);
module_exit(voodoo_sound_exit);
#endif
