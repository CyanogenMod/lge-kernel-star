#define MOD_TAG "LGE_SENSOR_VERIFY"
#define LG_SENSOR_DEBUG 0 
#define NV_DEBUG 0

#define GYRO_WHID 'a' 
#define KXTF9_WHID 'b'
#define COMPASS_WHID 'c'

#define SENSOR_OK   (sensor_status) 0x0000
#define SENSOR_ERROR    (sensor_status) 0x0001
typedef u32 sensor_status;

enum sensor_match_id {
	SENSOR_GYRO_ID		= 1,
	SENSOR_KXTF9_ID		= 2,
	SENSOR_COMPASS_ID	= 4,
	SENSOR_ALL_ID		= 7,
};

