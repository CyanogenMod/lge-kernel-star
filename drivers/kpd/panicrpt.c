/***
 * Description : panic report on LCD
 * Author : xwolf (xwolf@lge.com)
            pyocool.cho (pyocool.cho@lge.com)
 */

#include <linux/syscalls.h>

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/reboot.h>

#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <linux/vt_kern.h>
//2011.07.27 pyocool.cho@lge.com "volume up/down key" start
//#include <linux/gpio.h>
//2011.07.27 pyocool.cho@lge.com "volume up/down key" end
#include <linux/input.h>

#include <asm/uaccess.h>
//2011.07.27 pyocool.cho@lge.com "volume up/down key" start
#include "../../arch/arm/mach-tegra/include/mach/gpio.h"
#include "../../arch/arm/mach-tegra/lge/star/include/mach-tegra/gpio-names.h"
//2011.07.27 pyocool.cho@lge.com "volume up/down key" end 

/***
 * features and defines
 */
#define PANICRPT_USEIOCTRL
#define PANICRPT_USESYSFILE
//#define PANICRPT_USEMALLOC

#define PANICRPT_DISPLAYTHROUGHCONSOLE

#define PANICRPT_IOCTL_APPPANIC  0x1018

#define PANICRPT_COLUMNS         60
#define PANICRPT_ROWS            50
#define PANICRPT_ERRBUFSIZE      (PANICRPT_COLUMNS * PANICRPT_ROWS)

/***
 * typedefines
 */
enum
{
    EPANICRPT_KERNEL,
    EPANICRPT_MODEM,
    EPANICRPT_APPL,

    EPANICRPT_MAX
};

/***
 * external functions
 */
extern void fbcon_putcs_panicerr (struct vc_data *vc,
                                  const unsigned short *s,
			                      int count, int ypos, int xpos);
extern void fbcon_update_panicerr (struct vc_data *vc);
#if 0	//2011.07.27 pyocool.cho@lge.com "kernel panic"
extern int  get_lge_massversion (void);
#endif

/***
 * local functions
 */
static int panicrpt_waitinput (void);
static int panicrpt_displayerr (int epanicpos, char *pcerrmsg);
static int panicrpt_displaystring (char *pcerrmsg, int rows, int bupdate);
#if defined (PANICRPT_DISPLAYTHROUGHCONSOLE)
static int panicrpt_chartoword (char *pcerrmsg, unsigned short *pwerrmsg, int length);
#endif /* PANICRPT_DISPLAYTHROUGHCONSOLE */

/***
 * local valuables
 */
/* ipanicrpt_notwork : when it is '1', it means error-report is disabled */
static int ipanicrpt_notwork = 1;
static int ipanicrpt_inpanic = 0;

#if !defined (PANICRPT_USEMALLOC)
static char           acpanicrpt_errbuff[PANICRPT_ERRBUFSIZE];
static unsigned short awpanicrpt_cvrtbuff[PANICRPT_ERRBUFSIZE];
#endif /* !PANICRPT_USEMALLOC */
/***
 * implemetation
 */

/***
 * panicrpt_geterrbuff
 * return a error-buffer for displaying
 * return : when it reurns '> 0', pperrbuf is valid
 * NOTICE : never release the error-buffer
 */
int panicrpt_geterrbuff (char** pperrbuf)
{
    if (ipanicrpt_notwork)
    {
        return 0;
    }
#if defined (PANICRPT_USEMALLOC)
    *pperrbuf = vmalloc (PANICRPT_ERRBUFSIZE);
    if (*pperrbuf == (char*)0) {
        return 0;
    }
#else
    *pperrbuf = acpanicrpt_errbuff;
#endif /* PANICRPT_USEMALLOC */
    memset ((char*)*pperrbuf, 0, PANICRPT_ERRBUFSIZE);
    return PANICRPT_ERRBUFSIZE;
}

/***
 * panicrpt_goinpanic
 * display info of panic to LCD
 * return value : when it reurns '1'(int), panicrpt is doing something
 */
int panicrpt_goinpanic (int epanicpos, char *pbuf)
{
    	if (!ipanicrpt_notwork) {
        	if (ipanicrpt_inpanic == 0) {
            		ipanicrpt_inpanic = 1;
            		panicrpt_displayerr (epanicpos, pbuf);
//2011.07.27 pyocool.cho@lge.com "volume up/down key" start
                        //while(1) { } ;
            		preempt_enable ();
            		raw_local_irq_enable ();

			panicrpt_waitinput ();
            	//	machine_emergency_restart ();
//2011.07.27 pyocool.cho@lge.com "volume up/down key" end
        	}
   	}	
    	return ipanicrpt_notwork;
}

/***
 * panicrpt_ispanic
 * when the system is in panic, return 1
 * return value : when it reurns '1'(int), panicrpt is activating now
 */
int panicrpt_ispanic (void)
{
    return ipanicrpt_inpanic;
}

EXPORT_SYMBOL (panicrpt_geterrbuff);
EXPORT_SYMBOL (panicrpt_goinpanic);
EXPORT_SYMBOL (panicrpt_ispanic);

static int panicrpt_waitinput (void)
{
	unsigned int up = 0;
	unsigned int down = 0;

#if defined(CONFIG_MACH_STAR_SU660)
	up = TEGRA_GPIO_PG0;
	down = TEGRA_GPIO_PG1;
#elif defined(CONFIG_MACH_STAR_P990)
	up = TEGRA_GPIO_PG1;
	down = TEGRA_GPIO_PG0;
#elif defined(CONFIG_MACH_STAR_P999)
	up = TEGRA_GPIO_PG1;
	down = TEGRA_GPIO_PG0;
#elif defined(CONFIG_MACH_BSSQ)
	up = TEGRA_GPIO_PO7; // need to change
	down = TEGRA_GPIO_PO4; // need to change
#endif	
    	do {
			if(gpio_get_value(down) == 0) {
				return KEY_VOLUMEDOWN;
			}
			else if(gpio_get_value(up) == 0) {
				return KEY_VOLUMEUP;
			}
        	mdelay (100);
    	} while (1);
    	return 1;
}
//2011.07.27 pyocool.cho@lge.com "volume up/down key" end

static int panicrpt_displayerr (int epanicpos, char *pcerrmsg)
{
    panicrpt_displaystring ((char*)0, 0, 0);
    switch (epanicpos) {
        case EPANICRPT_KERNEL :
            panicrpt_displaystring (" =====KERNEL CRASH=====", 1, 0);
            break;
        case EPANICRPT_MODEM  :
            panicrpt_displaystring (" =====MODEM CRASH=====", 1, 0);
            break;
        case EPANICRPT_APPL   :
            panicrpt_displaystring (" =====APPLICATION CRASH=====", 1, 0);
            break;
        default               : break;
    }
    panicrpt_displaystring ("reboot when press VolUp and VolDown", 1, 0);
    panicrpt_displaystring ("======================================", 1, 0);
    if (pcerrmsg) {
        panicrpt_displaystring (pcerrmsg, PANICRPT_ROWS, 0);
    }

    panicrpt_displaystring ((char*)0, 0, 1);
    return 1;
}

static int panicrpt_displaystring (char *pcerrmsg, int rows, int bupdate)
{
#if defined (PANICRPT_DISPLAYTHROUGHCONSOLE)
    static int dprows = 0;

    if (pcerrmsg == (char*)0) {
        dprows = 0;
    } else {
        unsigned short *pwerrmsg;
        struct vc_data *vc;

        vc       = vc_cons[0].d;
#if defined (PANICRPT_USEMALLOC)
        pwerrmsg = vmalloc ((PANICRPT_COLUMNS + 1) * 2);
#else
        pwerrmsg = awpanicrpt_cvrtbuff;
#endif /* PANICRPT_USEMALLOC */
        if (pwerrmsg == (unsigned short*)0) {
	        printk(KERN_EMERG "panicrpt_displaystring pwerrmsg is NULL\n");
            return 0;
        }
        do {
            int linelength, index;

            linelength = strlen (pcerrmsg);
            memset (pwerrmsg, 0, (PANICRPT_COLUMNS + 1) * 2);
            if (linelength > 0) {
                index = 0;
                do {
                    if (*(pcerrmsg + index) == 0x0A) {
                        *(pcerrmsg + index++) = 0;
                        break;
                    }
                    index++;
                } while (*(pcerrmsg + index) && (index < PANICRPT_COLUMNS));
                linelength = index;
            }
            if (linelength > 0) {
                panicrpt_chartoword (pcerrmsg, pwerrmsg, linelength);
                pcerrmsg += linelength;
            }
            fbcon_putcs_panicerr (vc, pwerrmsg, PANICRPT_COLUMNS, dprows++, 0);
        } while (--rows);
#if defined (PANICRPT_USEMALLOC)
        vfree (pwerrmsg);
#endif /* PANICRPT_USEMALLOC */
    }
    if (bupdate) {
        struct vc_data *vc;

        vc       = vc_cons[0].d;
        fbcon_update_panicerr (vc);
    }
#endif /* PANICRPT_DISPLAYTHROUGHCONSOLE */
    return 1;
}

#if defined (PANICRPT_DISPLAYTHROUGHCONSOLE)
static int panicrpt_chartoword (char *pcerrmsg, unsigned short *pwerrmsg, int length)
{
    char *prepwerrmsg;

    prepwerrmsg = (char*)pwerrmsg;
    while (length--) {
        *prepwerrmsg++ = *pcerrmsg++;
        *prepwerrmsg++ = 0;
    }
    return 1;
}
#endif /* PANICRPT_DISPLAYTHROUGHCONSOLE */

#if defined (PANICRPT_USESYSFILE)
static ssize_t panicrpt_showfatalmode (struct device *dev,
                                       struct device_attribute *attr,
                                       char *buf)
{
    sprintf (buf, "%d\n", ipanicrpt_notwork);
    return (ssize_t)(strlen (buf) + 1);
}

static ssize_t panicrpt_storefatalmode (struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t size)
{
    ipanicrpt_notwork = !!((int)simple_strtoul (buf, NULL, 10));
    return size;
}

static DEVICE_ATTR(fatalmode, 0666, panicrpt_showfatalmode, panicrpt_storefatalmode);

static struct attribute *panicrpt_attributes[] = {
    &dev_attr_fatalmode.attr,
    NULL
};

static const struct attribute_group panicrpt_group = {
    .attrs = panicrpt_attributes,
};
#endif /* PANICRPT_USESYSFILE */

#if defined (PANICRPT_USEIOCTRL)
static long panicrpt_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
    long retval;

    retval = 0;
    switch(cmd) {
        case PANICRPT_IOCTL_APPPANIC :
           if (!ipanicrpt_notwork) {
               char *perrbuf;
               int  errbufsize;

               errbufsize = panicrpt_geterrbuff (&perrbuf);
               if (errbufsize > 0) {
                   if (copy_from_user (perrbuf, (void __user *)arg, errbufsize)) {
                       retval = -EFAULT;
                   } else {
                       panicrpt_goinpanic (EPANICRPT_APPL, perrbuf);
                   }
               }
           }
           break;

        default                      : break;
    }
    return retval;
}

static int panicrpt_dummy (struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations panicrpt_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = panicrpt_ioctl,
    .open           = panicrpt_dummy,
    .release        = panicrpt_dummy,
};
#endif /* PANICRPT_USEIOCTRL */

static struct miscdevice panicrpt_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "panicrpt",
#if defined (PANICRPT_USEIOCTRL)
    .fops  = &panicrpt_fops,
#endif /* PANICRPT_USEIOCTRL */
};

/***
 * register to misc driver
 */
static int panicrpt_registerdriver(void)
{
#if defined (PANICRPT_USESYSFILE) || defined (PANICRPT_USEIOCTRL)
    int error;

    /*
     * dont register the driver at mass version
     */
#if 0		//2011.07.27 pyocool.cho@lge.com "kernel panic"
    if (get_lge_massversion ()) {
        return -1;
    }
#endif
    /*
     * ipanicrpt_notwork is set when checking mass version
     */
    ipanicrpt_notwork = 0;

    error = misc_register (&panicrpt_device);
    if (error == 0) {
#if defined (PANICRPT_USESYSFILE)
        if ((error = sysfs_create_group
                         (&panicrpt_device.this_device->kobj, &panicrpt_group))) {
            error = -ENOMEM;
        }
        else
#endif /* PANICRPT_USESYSFILE */
        {
            return 0;
        }
    }
    misc_deregister (&panicrpt_device);
    return error;
#else
    return -1;
#endif /* PANICRPT_USESYSFILE || PANICRPT_USEIOCTRL */
}

static int __init panicrpt_init (void)
{
    return panicrpt_registerdriver ();
}

module_init (panicrpt_init);

#if defined (MODULE)
static int __exit panicrpt_exit (void)
{
    return misc_deregister (&panicrpt_device);
}

module_exit (panicrpt_exit);
#endif /* MODULE */

MODULE_DESCRIPTION ("panic report handler for kernel/modem/application");
MODULE_AUTHOR ("xwolf xwolf@lge.com");
MODULE_LICENSE ("GPL");
