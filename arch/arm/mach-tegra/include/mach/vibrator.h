#ifndef __PWM_VIBRATOR_H
#define __PWM_VIBRATOR_H

#ifdef __KERNEL__

struct pwm_vib_platform_data {
	int     max_timeout;
	u8      active_low;
	int     initial_vibrate;
	int		pwm_id;
	int		duty_ns;
	int		period_ns;
	int		enable;
	int 	(*power)(char* reg_id, int on);	
};

#endif /* __KERNEL__ */

#endif
