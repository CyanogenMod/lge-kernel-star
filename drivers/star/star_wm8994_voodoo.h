#include "nvcommon.h"
#include "nvodm_services.h"
#include "nvodm_query.h"
#include "nvodm_query_discovery.h"

#define VOODOO_SOUND_VERSION 1

typedef struct star_wm8994_device_data {
	NvOdmServicesI2cHandle h_gen2_i2c;
	NvU32 i2c_address;
} star_wm8994_device;

#define CONFIG_SND_VOODOO_HP_LEVEL 48
#define MAX_HP_AMP_LEVEL 54

enum debug_log { LOG_OFF, LOG_INFOS, LOG_VERBOSE };
bool debug_log(short unsigned int level);

#define WM8994_I2C_RETRY_COUNT 5
#define WM8994_I2C_TIMEOUT 20

int voodoo_sound_init(void);
void voodoo_sound_exit(void);
unsigned int voodoo_hook_wm8994_write(int codec,
				      unsigned int reg, unsigned int value);
void update_enable(void);
void voodoo_sound_misc_register(void);
void voodoo_sound_misc_deregister(void);
