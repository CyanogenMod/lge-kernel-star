/**
	@brief		 star hw test
 
	@author		 cs77.ha@lge.com
	@date		 2010-06-03
 
	@version	 V1.00		 2010.06.03		 Changsu Ha	 Create
*/


#if defined(CONFIG_STAR_PMIC)

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/debugfs.h>


#include "nvodm_query_discovery.h"
#include "nvrm_pmu.h"
#include "mach/nvrm_linux.h"

//#include <linux/delay.h>

#define MAX_REFCOUNT_LOOP   10
#define MAX_PMIC_LDO_NUM      20

static NvU32 powerRefCount[32];

extern NvRmDeviceHandle s_hRmGlobal;

struct pmic_struct {
        char *name;
        int ldo_number;
};

struct pmic_struct star_pmic[MAX_PMIC_LDO_NUM] = {
    { "LDO01", 0 },
    { "LDO02", 1 },
    { "LDO03", 2 },
    { "LDO04", 3 },
    { "LDO05", 4 },
    { "LDO06", 5 },
    { "LDO07", 6 },
    { "LDO08", 7 },
    { "LDO09", 8 },
    { "LDO10", 9 },
    { "LDO11", 10 },
    { "LDO12", 11 },
    { "LDO13", 12 },
    { "LDO14", 13 },
    { "LDO15", 14 },
    { "LDO16", 15 },
    { "LDO17", 16 },
    { "LDO18", 17 },
    { "LDO19", 18 },
    { "LDO20", 19 },
};

static int pmic_set(void *data, u64 val)
{
    u32 settling_time;
    const NvOdmPeripheralConnectivity *con = NULL;
    int ldo_number;
    int j;
    NvU32 millivolts;
    NvU32 loop_count = MAX_REFCOUNT_LOOP;
    NvRmPmuVddRailCapabilities rail;
    
    struct pmic_struct *pmic_info  = data;

    ldo_number = pmic_info->ldo_number;

    printk("PMIC(debugfs) : WARNING!! reference counter will be broken\n");

    con = NvOdmPeripheralGetGuid(NV_ODM_GUID('a','l','l','p','o','w','e','r'));
    if (con == NULL){
        printk("PMIC(debugfs) : query ERROR\n");
        return 0;
    }

    if(ldo_number == 2){
        printk("PMIC(debugfs) : skip LDO2 test, always on!!!\n");
        return 0;
    }


    NvRmPmuGetVoltage(s_hRmGlobal, 
                con->AddressList[ldo_number].Address, &millivolts);
    if(val){
        if(millivolts){
            printk("PMIC(debugfs) : LDO%d already turned on\n"
                    , ldo_number+1);
            return 0;
        }
        NvRmPmuGetCapabilities(s_hRmGlobal,
                con->AddressList[ldo_number].Address, &rail);
        for(j=MAX_REFCOUNT_LOOP;j>powerRefCount[ldo_number];j--){
            NvRmPmuSetVoltage(s_hRmGlobal,
                    con->AddressList[ldo_number].Address,
                    rail.requestMilliVolts, &settling_time);
        }
        printk("PMIC(debugfs) : LDO%d turn on\n", 
                ldo_number+1);
        //udelay(settling_time);
    }else{
        if(!millivolts){
            printk("PMIC(debugfs) : LDO%d already turned off\n", 
                    ldo_number+1);
            return 0;
        }
        
        while(millivolts && loop_count--){
            NvRmPmuSetVoltage(s_hRmGlobal,
                    con->AddressList[ldo_number].Address, NVODM_VOLTAGE_OFF,
                    &settling_time);
            NvRmPmuGetVoltage(s_hRmGlobal,
                con->AddressList[ldo_number].Address, &millivolts);
            if(!millivolts){
                powerRefCount[ldo_number]=loop_count;
                printk("PMIC(debugfs) : LDO%d turn off\n", 
                        ldo_number+1);
            }
        }
        //udelay(settling_time);
    }
    return 0;
}

static int pmic_get(void *data, u64 *val)
{
    const NvOdmPeripheralConnectivity *con = NULL;
    int ldo_number;
    NvU32 millivolts;

    struct pmic_struct *pmic_info  = data;

    ldo_number = pmic_info->ldo_number;

    con = NvOdmPeripheralGetGuid(NV_ODM_GUID('a','l','l','p','o','w','e','r'));
    if (con == NULL){
        *val = 9999;
        printk("PMIC(debugfs) : query ERROR\n");
        return 0;
    }
        
    NvRmPmuGetVoltage(s_hRmGlobal, 
                con->AddressList[ldo_number].Address, &millivolts);

    if(millivolts){
        *val = 1;
        printk("PMIC(debugfs) : LDO%d (on)\n", 
                        ldo_number+1);
    }else{
        *val = 0;
        printk("PMIC(debugfs) : LDO%d (off)\n", 
                        ldo_number+1);
    }

    return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pmic_fops, 
                        pmic_get, pmic_set, "%llu\n");


static int star_gpio_set(void *data, u64 val)
{
    int value = (int)val;
    
    printk("STAR_GPIO(debugfs) : star_gpio_set gpio_nub=%d, config=%s, value=%d\n", 
            value/100, ((value/10)%10)? "in":"out", value%10);
    return 0;
}

static int star_gpio_get(void *data, u64 *val)
{
    printk("STAR_GPIO(debugfs) : star_gpio_get\n");
    return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(star_gpio_fops, 
                        star_gpio_get, star_gpio_set, "%llu\n");


static int __init star_pmic_init(void)
{
    struct dentry *dent_root, *dent_sub;
    unsigned i;

    dent_root = debugfs_create_dir("star", 0);
    
    if (IS_ERR(dent_root))
        return PTR_ERR(dent_root);

    dent_sub = debugfs_create_dir("pmic", dent_root);
    
    if (IS_ERR(dent_sub))
        return PTR_ERR(dent_sub);

    for (i = 0; i < MAX_PMIC_LDO_NUM; i++) {
		debugfs_create_file(star_pmic[i].name, 0644, dent_sub,
                            &star_pmic[i], &pmic_fops);
    }

    debugfs_create_file("gpio", 0644, dent_root,
                            NULL, &star_gpio_fops);

    return 0;
}

late_initcall(star_pmic_init);

#endif  /* CONFIG_STAR_PMIC */

