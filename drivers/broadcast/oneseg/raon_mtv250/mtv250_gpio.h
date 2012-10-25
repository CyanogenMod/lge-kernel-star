#ifndef __MTV818_GPIO_H__
#define __MTV818_GPIO_H__


#ifdef __cplusplus 
extern "C"{ 
#endif  

#if defined(CONFIG_KS1001) || defined(CONFIG_KS1103)
#include <mach/gpio.h>
#include <mach/gpio-names.h>

#define MTV_1_2V_EN					TEGRA_GPIO_PF2
#define MTV_1_8V_EN					TEGRA_GPIO_PE4
#define MTV_PWR_EN					TEGRA_GPIO_PF3
#define RAONTV_IRQ_INT				TEGRA_GPIO_PO6

static inline int mtv250_configure_gpio(void)
{
	//2011-05-02 taew00k.kang local gpio setting
	gpio_request(MTV_1_2V_EN, "mtv250 1.2V EN");
	tegra_gpio_enable(MTV_1_2V_EN);
	gpio_direction_output(MTV_1_2V_EN, 0);

	//2011-07-22 taew00k.kang local gpio setting
	gpio_request(MTV_1_8V_EN, "mtv250 1.8V EN");
	tegra_gpio_enable(MTV_1_8V_EN);
	gpio_direction_output(MTV_1_8V_EN, 1);// init value always high value


	gpio_request(MTV_PWR_EN, "mtv250 EN");
	tegra_gpio_enable(MTV_PWR_EN);
	gpio_direction_output(MTV_PWR_EN, 0);	

	gpio_request(RAONTV_IRQ_INT, "MTV250 INT");
	tegra_gpio_enable(RAONTV_IRQ_INT);	
	gpio_direction_input(RAONTV_IRQ_INT);

	return 0;
}

#elif defined(CONFIG_MACH_LGE_P940)
#include <mach/gpio.h>

#if defined(CONFIG_MACH_LGE_P940_EVB)
#define MTV_PWR_EN					(42)
#else //after CONFIG_MACH_LGE_P940_HW_A, changed gpio pin map
#define MTV_PWR_EN					(46)
#endif
#define RAONTV_IRQ_INT				(44)

static inline int mtv250_configure_gpio(void)
{
	if(gpio_request(MTV_PWR_EN, "MTV_PWR_EN"))		
		DMBMSG("MTV_PWR_EN Port request error!!!\n");
	
	gpio_direction_output(MTV_PWR_EN, 0); // power down
	
	
	if(gpio_request(RAONTV_IRQ_INT, "RAONTV_IRQ_INT"))		
		DMBMSG("RAONTV_IRQ_INT Port request error!!!\n");

	gpio_direction_input(RAONTV_IRQ_INT);  
}

#else
	#error "Code not present"
#endif

#ifdef __cplusplus 
} 
#endif 

#endif /* __MTV818_GPIO_H__*/
