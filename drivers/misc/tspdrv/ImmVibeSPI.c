/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

//#include <mach-tegra/gpio-names.h>
#include "../gpio-names.h"

extern int device_power_control(char* reg_id, bool on);

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS   1
#define VIB_DEBUG       1


#define NV_PWM_PERIOD_22_4HZ_NS     22400  // 22.4 kHz     
#define NV_PWM_NS                   1000*1000*1000
#define NV_PWM_PERIOD_22_4HZ_TO_NS  (NV_PWM_NS/NV_PWM_PERIOD_22_4HZ_NS) 

#define NV_PWM_DUTY_50_TO_NS        (NV_PWM_PERIOD_22_4HZ_TO_NS >> 1) // 50%

static struct pwm_device* nv_vib_pwm = NULL;

static bool g_bAmpEnabled = false;

#if defined(CONFIG_LU6500)
static int nv_vibe_gpio_en =  TEGRA_GPIO_PF3;
#else
static int nv_vibe_gpio_en =  TEGRA_GPIO_PU4;
#endif

static void nv_vibrator_gpio_enable (int enable)
{
	if (enable)
		gpio_set_value(nv_vibe_gpio_en, 1);
	else 	
		gpio_set_value(nv_vibe_gpio_en, 0);
}

static void nv_vibrator_LDO_enable(int val)
{
	if (val == 1){        
		device_power_control("vcc_motor_3v0", 1);
	}else{
		device_power_control("vcc_motor_3v0", 0);
	}
}

static void vib_enable(int on )
{
	nv_vibrator_gpio_enable(on);
	nv_vibrator_LDO_enable(on);
}

static void vib_generatePWM(int on)
{
    if(nv_vib_pwm == NULL){
        DbgOut(( "[ImmVibeSPI] vib_generatePWM nv_vib_pwm =  %d \n", nv_vib_pwm ));
        return;
    }
    
	if(on) {
        pwm_config(nv_vib_pwm, NV_PWM_DUTY_50_TO_NS, NV_PWM_PERIOD_22_4HZ_TO_NS);
        pwm_enable(nv_vib_pwm);
	}
	else {
        pwm_disable(nv_vib_pwm);
	}
}

////////////////////////

/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpDisable start[%d]\n", g_bAmpEnabled ));
    if (g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable.\n"));

        g_bAmpEnabled = false;

		vib_enable(false);
		vib_generatePWM(false);
    }

    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    if(nv_vib_pwm == NULL){
        nv_vibrator_gpio_enable(0);
        DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_AmpEnable nv_vib_pwm =  %d \n", nv_vib_pwm ));
        return VIBE_E_FAIL;
    }

    if (!g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpEnable.\n"));

        g_bAmpEnabled = true;

		vib_generatePWM(true);
		vib_enable(true);
    }
    else
    {
        DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpEnable [%d]\n", g_bAmpEnabled ));
    }

    return VIBE_S_SUCCESS;
}


/* Called at initialization time to set PWM freq, disable amp, etc... */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize( void)
{
	int ret = 0;

   	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Initialize\n" ));
    
//	g_bAmpEnabled = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */
    
	ret = gpio_request(nv_vibe_gpio_en, "Nv Vibrator Enable");
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to request GPIO_%d for vibrator\n", __func__, nv_vibe_gpio_en);
	}

	tegra_gpio_enable(nv_vibe_gpio_en);

	gpio_direction_output(nv_vibe_gpio_en, 0);
	
#if defined(CONFIG_LU6500)
	nv_vib_pwm	=	pwm_request(0, "vibrator"); 	
#else
	nv_vib_pwm	=	pwm_request(3, "vibrator"); 	
#endif
	if (IS_ERR(nv_vib_pwm)) {
        DbgOut(( "[ImmVibeSPI] : unable to request PWM for vibrator\n" ));
        nv_vib_pwm = NULL;
		goto	err_pwm;
	}
	else{
        DbgOut(( "[ImmVibeSPI] : got pwm for vibrator\n" ));
	}

//   	ImmVibeSPI_ForceOut_AmpDisable( 0 );	
    gpio_set_value(nv_vibe_gpio_en, 0);
    pwm_disable(nv_vib_pwm);

    return VIBE_S_SUCCESS;

err_pwm:
    return VIBE_E_FAIL;
}


/* Called at termination time to set PWM freq, disable amp, etc... */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate( void )
{
   	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Terminate\n" ));

	ImmVibeSPI_ForceOut_AmpDisable(0);

    return VIBE_S_SUCCESS;
}

bool bInTestMode = 0; /* 20110125 jiwon.seo@lge.com for ELT vibrator */

/*
** Called by the real-time loop to set PWM duty cycle
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    VibeUInt32 nTmp;
    VibeInt8 nForce;

    if(nv_vib_pwm == NULL){
        DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nv_vib_pwm =  %d \n", nv_vib_pwm ));
        return VIBE_E_FAIL;
    }


    switch (nOutputSignalBitDepth)
    {
        case 8:
            /* pForceOutputBuffer is expected to contain 1 byte */
            if (nBufferSizeInBytes != 1){
                DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes ));
                return VIBE_E_FAIL;
            }
            nForce = pForceOutputBuffer[0];
            break;
        case 16:
            /* pForceOutputBuffer is expected to contain 2 byte */
            if (nBufferSizeInBytes != 2){
                DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes ));
                return VIBE_E_FAIL;
            }
            /* Map 16-bit value to 8-bit */
            nForce = ((VibeInt16*)pForceOutputBuffer)[0] >> 8;
            break;
        default:
            /* Unexpected bit depth */
            DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes ));
            return VIBE_E_FAIL;
    }

    DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nForce =  %d \n", nForce ));

    /* Check the Force value with Max and Min force value */
    if (nForce > 127) nForce = 127;
    if (nForce < -127) nForce = -127;

    if (nForce == 0)
    {        
        nv_vibrator_gpio_enable(0);
        pwm_disable(nv_vib_pwm);        
    }    
    else
    {
        nv_vibrator_gpio_enable(1);

        
        nTmp = (((nForce+128) * (NV_PWM_PERIOD_22_4HZ_TO_NS)) >> 8); // cal duty ns 
            
        pwm_config(nv_vib_pwm, nTmp, NV_PWM_PERIOD_22_4HZ_TO_NS);
        pwm_enable(nv_vib_pwm);
    }        

    return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
    /* This function is not called for ERM device */
	return VIBE_S_SUCCESS;
}

/*** Called to get the device name (device name must be returned as ANSI char)*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName( VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    return VIBE_S_SUCCESS;
}

