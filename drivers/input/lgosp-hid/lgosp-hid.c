#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include <linux/input.h>
#include <linux/kd.h>

#include <asm/uaccess.h>

/************************************************************/
/*					general definition						*/
#define OSP_HID_MODULE_PATH 	"/system/etc/lgosp/lgosp-hid.ko"
#define OSP_HID_DEV_NAME 		"lgosp-hid"
#define OSP_HID_DEV_PATH 		"/dev/lgosp-hid"
#define OSP_HID_DEV_MAJOR		255
#define OSP_HID_DEV_MINOR		1
#define	DMSG					printk

//#define OSP_HID_KEYBOARD_SUPPORT
//#define OSP_HID_TOUCH_SUPPORT

#ifdef OSP_HID_TOUCH_SUPPORT
#define OSP_SCREEN_QVGA
//#define OSP_SCREEN_HVGA

#if defined(OSP_SCREEN_QVGA)
#define	TOUCH_AXIS_X		480
#define	TOUCH_AXIS_Y		800
#elif defined(OSP_SCREEN_HVGA)
#define TOUCH_AXIS_X		320
#define TOUCH_AXIS_Y		480
#endif
#endif /* OSP_HID_TOUCH_SUPPORT */
/************************************************************/

static struct input_dev *osp_hid_dev;

struct HID_COMMAND
{
	/* general event */
	unsigned int type;
	unsigned int code;
	int value;
};

#ifdef OSP_HID_KEYBOARD_SUPPORT
#define NR_SCANCODES 256

static const unsigned char hid_keyboard[NR_SCANCODES] = {
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_A, KEY_B,
	KEY_C, KEY_D, KEY_E, KEY_F, KEY_G, KEY_H, KEY_I, KEY_J, KEY_K, KEY_L,
	KEY_M, KEY_N, KEY_O, KEY_P, KEY_Q, KEY_R, KEY_S, KEY_T, KEY_U, KEY_V,
	KEY_W, KEY_X, KEY_Y, KEY_Z, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6,
	KEY_7, KEY_8, KEY_9, KEY_0, KEY_ENTER, KEY_ESC, KEY_BACKSPACE,
	KEY_TAB, KEY_SPACE, KEY_MINUS, KEY_EQUAL, KEY_LEFTBRACE,
	KEY_RIGHTBRACE, KEY_BACKSLASH, KEY_BACKSLASH, KEY_SEMICOLON,
	KEY_APOSTROPHE, KEY_GRAVE, KEY_COMMA, KEY_DOT, KEY_SLASH,
	KEY_CAPSLOCK, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6,
	KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_F11, KEY_F12, KEY_SYSRQ,
	KEY_SCROLLLOCK, KEY_PAUSE, KEY_INSERT, KEY_HOME, KEY_PAGEUP,
	KEY_DELETE, KEY_END, KEY_PAGEDOWN, KEY_RIGHT, KEY_LEFT, KEY_DOWN,
	KEY_UP, KEY_NUMLOCK, KEY_KPSLASH, KEY_KPASTERISK, KEY_KPMINUS,
	KEY_KPPLUS, KEY_KPENTER, KEY_KP1, KEY_KP2, KEY_KP3, KEY_KP4, KEY_KP5,
	KEY_KP6, KEY_KP7, KEY_KP8, KEY_KP9, KEY_KP0, KEY_KPDOT, KEY_102ND,
	KEY_COMPOSE, KEY_POWER, KEY_KPEQUAL, KEY_F13, KEY_F14, KEY_F15,
	KEY_F16, KEY_F17, KEY_F18, KEY_F19, KEY_F20, KEY_F21, KEY_F22,
	KEY_F23, KEY_F24, KEY_OPEN, KEY_HELP, KEY_PROPS, KEY_FRONT, KEY_STOP,
	KEY_AGAIN, KEY_UNDO, KEY_CUT, KEY_COPY, KEY_PASTE, KEY_FIND, KEY_MUTE,
	KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_KPCOMMA, KEY_RESERVED, KEY_RO, KEY_KATAKANAHIRAGANA , KEY_YEN,
	KEY_HENKAN, KEY_MUHENKAN, KEY_KPJPCOMMA, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_HANGEUL, KEY_HANJA, KEY_KATAKANA, KEY_HIRAGANA,
	KEY_ZENKAKUHANKAKU, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
	KEY_RESERVED, KEY_RESERVED, KEY_LEFTCTRL, KEY_LEFTSHIFT, KEY_LEFTALT,
	KEY_LEFTMETA, KEY_RIGHTCTRL, KEY_RIGHTSHIFT, KEY_RIGHTALT,
	KEY_RIGHTMETA, KEY_PLAYPAUSE, KEY_STOPCD, KEY_PREVIOUSSONG,
	KEY_NEXTSONG, KEY_EJECTCD, KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_MUTE,
	KEY_WWW, KEY_BACK, KEY_FORWARD, KEY_STOP, KEY_FIND, KEY_SCROLLUP,
	KEY_SCROLLDOWN, KEY_EDIT, KEY_SLEEP, KEY_SCREENLOCK, KEY_REFRESH,
	KEY_CALC, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED
};
#endif /* OSP_HID_KEYBOARD_SUPPORT */

/************************************************************/
static struct HID_COMMAND hid_cmd;

/************************************************************/
/*					file operations								*/

int osp_hid_open(struct inode* inode, struct file* filp)
{
	DMSG("[osp_hid_open]\n");
	return 0;
}

int osp_hid_release(struct inode* inode, struct file* filp)
{
	DMSG("[osp_hid_release]\n");
	return 0;
}

ssize_t osp_hid_read(struct file* filp, char* buf, size_t count, loff_t* f_pos)
{
	DMSG("[osp_hid_read] %d\n",count);
	return count;
}

ssize_t osp_hid_write(struct file* filp, const char* buf, size_t count, loff_t* f_pos)
{
	if(count >= sizeof(hid_cmd))
	{
		unsigned long result;

		result = copy_from_user(&hid_cmd, buf, sizeof(hid_cmd));

		//DMSG("[osp_hid_write] type:%x, code:%x, value:%x\n", hid_cmd.type, hid_cmd.code, hid_cmd.value);

		//input_event(osp_hid_dev, hid_cmd.type, hid_cmd.code, hid_cmd.value);
		switch (hid_cmd.type)
		{
		case EV_SYN:
			DMSG("[osp_hid_write] EV_SYN\n");
			input_sync(osp_hid_dev);
			break;

		case EV_KEY:
			DMSG("[osp_hid_write] EV_KEY, code:0x%x, value:0x%x\n", hid_cmd.code, hid_cmd.value);
			input_event(osp_hid_dev, hid_cmd.type, hid_cmd.code, hid_cmd.value);
			input_sync(osp_hid_dev);
			break;

		case EV_REL:
			DMSG("[osp_hid_write] EV_REL, code:0x%x, value:0x%x\n", hid_cmd.code, hid_cmd.value);
			input_event(osp_hid_dev, hid_cmd.type, hid_cmd.code, hid_cmd.value);
			break;

		case EV_ABS:
			DMSG("[osp_hid_write] EV_ABS, code:0x%x, value:0x%x\n", hid_cmd.code, hid_cmd.value);
			input_event(osp_hid_dev, hid_cmd.type, hid_cmd.code, hid_cmd.value);
			break;

		default:
			break;
		}
	}
	else
	{
		DMSG("[input_write] UNKNOWN event\n");
	}

	return count;
}

struct file_operations osp_hid_fops =
{
	.owner 	= 	THIS_MODULE,
	.read 	= 	osp_hid_read,
	.write	=	osp_hid_write,
	.open	=	osp_hid_open,
	.release	=	osp_hid_release,
};


/************************************************************/
/*					dummy operations	for hid					*/

static int dummyt_open(struct input_dev *dev)
{
    return(0);
}

static void dummy_close(struct input_dev *dev)
{
}

static int dummy_event(struct input_dev *dev, unsigned int type, 
                           unsigned int code, int value)
{
    return(0);
}

static int register_hid(struct input_dev *input)
{
#ifdef OSP_HID_KEYBOARD_SUPPORT
	int i;
#endif
	int error;

	/* register as an input device */
	input->event = dummy_event;
	input->open = dummyt_open;
	input->close = dummy_close;
	input->name = OSP_HID_DEV_NAME;
	input->uniq = "HID Device";
	input->id.bustype = BUS_USB;
	input->id.vendor = 0;
	input->id.product = 0;
	input->id.version = 0;
	//input->dev = NULL;

	input->evbit[BIT_WORD(EV_KEY)] |= BIT_MASK(EV_KEY);
	
#ifdef OSP_HID_TOUCH_SUPPORT
	/* touch */
	input->evbit[BIT_WORD(EV_ABS)] |= BIT_MASK(EV_ABS);
	input->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input, ABS_X, 0, TOUCH_AXIS_X, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, TOUCH_AXIS_Y, 0, 0);
#endif /* OSP_HID_TOUCH_SUPPORT */

#ifdef OSP_HID_KEYBOARD_SUPPORT
	/* keyboard */
	for(i=0;i<NR_SCANCODES;i++)
	{
		set_bit(hid_keyboard[i],input->keybit);
	}
#endif /* OSP_HID_KEYBOARD_SUPPORT */

	/* some keys */
	set_bit(KEY_UP, input->keybit);
	set_bit(KEY_LEFT, input->keybit);
	set_bit(KEY_RIGHT, input->keybit);
	set_bit(KEY_DOWN, input->keybit);
	set_bit(KEY_INSERT, input->keybit);
	set_bit(KEY_DELETE, input->keybit);
	set_bit(KEY_ENTER, input->keybit);
	
	/* buttons */
	set_bit(KEY_EMAIL, input->keybit);	/* @ key */
	set_bit(KEY_SEARCH, input->keybit);
	set_bit(KEY_CAMERA, input->keybit);
	set_bit(KEY_BACK, input->keybit);
	set_bit(KEY_MENU, input->keybit);
	set_bit(KEY_SEND, input->keybit);
	set_bit(KEY_HOME, input->keybit);
	set_bit(KEY_END, input->keybit);
	set_bit(KEY_POWER, input->keybit);
	set_bit(KEY_VOLUMEDOWN, input->keybit);
	set_bit(KEY_VOLUMEUP, input->keybit);

	error = input_register_device(input);
	if (error)
	{
		DMSG("Failed to register device (error:%d)\n", error);
		return error;
	}
	
	DMSG("HID Registered\n");

	return 0;
}


static int unregister_hid(struct input_dev *input)
{
	if (input != NULL)
	{
		input_unregister_device(input);

		DMSG("HID Unregistered\n");
	}
	
	return 0;
}

/************************************************************/

static int osp_hid_init(void)
{
	int result = -1;

	DMSG("[osp_hid_init]\n");

	result = register_chrdev(OSP_HID_DEV_MAJOR, OSP_HID_DEV_NAME, &osp_hid_fops);
	if(result < 0)
	{
		DMSG("Failed to register as a character device(result:%d)\n", result);
		return result;
	}

	osp_hid_dev = input_allocate_device();
	if (osp_hid_dev == NULL) 
	{
		DMSG("Failed to allocate input device\n");
	}
	else
	{
		register_hid(osp_hid_dev);
	}

	return 0;
}

static void osp_hid_exit(void)
{
	unregister_chrdev(OSP_HID_DEV_MAJOR, OSP_HID_DEV_NAME);

	if (osp_hid_dev != NULL)
	{
		unregister_hid(osp_hid_dev);
		input_free_device(osp_hid_dev);
	}

	DMSG("[osp_hid_exit]\n");
}

module_init(osp_hid_init);
module_exit(osp_hid_exit);

MODULE_LICENSE("GPL");

