/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2009 Immersion Corporation. All Rights Reserved. 
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

#ifdef 	IMMVIBESPIAPI
#undef 	IMMVIBESPIAPI
#endif
#define 	IMMVIBESPIAPI static

#include 	"nvcommon.h"
#include 	"nvodm_services.h"
#include 	"nvodm_query.h"
#include 	"nvodm_query_discovery.h"

/* This SPI supports only one actuator. */
#define 	NUM_ACTUATORS 	1
NvU32	g_requestedPeriod = 23000; //23KHz freq
static bool g_bAmpEnabled = false;

typedef struct star_vib_device_data
{
	NvOdmServicesGpioHandle 	h_vib_gpio;
	NvOdmGpioPinHandle  		h_vib_gpio_pin;
	NvOdmServicesPmuHandle  	h_vib_pmu;
	NvOdmServicesPwmHandle 		hOdmPwm;
	NvU32   					vdd_id;
   	NvU32   					en_pin;
   	NvU32   					en_port;
} star_vib_device;

static 	star_vib_device 		*g_vib;

static int 	vib_set_power_rail( NvU32 vdd_id, NvBool is_enable )
{
	NvOdmServicesPmuHandle 			h_pmu = NvOdmServicesPmuOpen( );
	NvOdmServicesPmuVddRailCapabilities 	vddrailcap;
	NvU32 							settletime;

	if ( h_pmu ) {
    	NvOdmServicesPmuGetCapabilities( h_pmu, vdd_id, &vddrailcap );
    	if ( is_enable ) {
    		DbgOut(( "[ImmVibeSPI] vibrator PMU enable\n" ));
    		NvOdmServicesPmuSetVoltage( h_pmu, vdd_id, vddrailcap.requestMilliVolts, &settletime );
    	} else {
    		DbgOut(( "[ImmVibeSPI] vibrator PMU do not enable\n" ));
    		NvOdmServicesPmuSetVoltage(h_pmu, vdd_id, NVODM_VOLTAGE_OFF, &settletime);
    	}

    	if ( settletime )
    		NvOdmOsWaitUS( settletime );

    	NvOdmServicesPmuClose( h_pmu );
    	DbgOut(( "[ImmVibeSPI] vibrator voltage =  %d or %d \n", vddrailcap.requestMilliVolts, vddrailcap.MinMilliVolts ));
    	return 0;
	}

	return -1;

}

static int vib_init( )
{
	const 	NvOdmPeripheralConnectivity *pcon;
	int 		err;
	NvBool 	found_gpio = NV_FALSE;
	int 		loop;

	g_vib = kzalloc( sizeof(*g_vib), GFP_KERNEL );
	if ( g_vib == NULL ) {
		err = -1;
		printk( "[%s] fail vib\n", __func__ );
		return err;
	}

 	pcon = ( NvOdmPeripheralConnectivity* ) NvOdmPeripheralGetGuid( NV_ODM_GUID('v','i','b','r','a','t','o','r') );
	for ( loop=0; loop< pcon->NumAddress; loop++ ) {
		switch ( pcon->AddressList[loop].Interface ) {
			case NvOdmIoModule_Gpio:
				g_vib->en_port = pcon->AddressList[loop].Instance;
	        	g_vib->en_pin = pcon->AddressList[loop].Address;
	        	found_gpio = NV_TRUE;
	        	break;

	    	case NvOdmIoModule_Vdd:
            	g_vib->vdd_id = pcon->AddressList[loop].Address;
            	DbgOut(( "VIB POWER %d\n", g_vib->vdd_id ));
				if ( vib_set_power_rail( g_vib->vdd_id,  NV_TRUE) != 0 )
					return -ENOSYS;
		        break;

			default:
		        break;
		} // end of switch

	} // end of for

	DbgOut(( "[ImmVibeSPI][%s] : vibrator Int Port = %c, Int Pin = %d\n", __func__, (g_vib->en_port+'a'), g_vib->en_pin ));

	g_vib->h_vib_gpio = NvOdmGpioOpen( );
	if( ! g_vib->h_vib_gpio ) {
		printk( "[ImmVibeSPI][%s] : Failed to open gpio\n", __func__ );
		err = - ENOSYS;
		return err;
	}

#ifdef MACH_STAR_REV_A
	g_vib->en_port = 'u' - 'a';
	g_vib->en_pin = 5;
#endif

#if defined ( CONFIG_MACH_STAR )
	g_vib->en_port = 'u' - 'a';
	g_vib->en_pin = 4;
#endif

	g_vib->h_vib_gpio_pin = NvOdmGpioAcquirePinHandle( g_vib->h_vib_gpio, g_vib->en_port, g_vib->en_pin );
	if ( ! g_vib->h_vib_gpio_pin ) {
    	printk( "[ImmVibeSPI][%s] : Failed to acquire the pin handle\n", __func__ );
    	err = -ENOSYS;
    	return err;
	}

	g_vib->hOdmPwm = NvOdmPwmOpen( );
	if ( ! g_vib->hOdmPwm ) {
		printk( "[ImmVibeSPI][%s] : Failed to acquire the PWM handle\n", __func__ );
		err = - ENOSYS;
		return err;
	}

	return 0;
}

#if 0
static void vib_close( )
{

	NvU32	RequestedPeriod,ReturnedPeriod;
	NvU32 	DutyCycle;

	if ( g_vib != NULL ) {

		NvOdmServicesPmuClose( g_vib->h_vib_pmu );
		g_vib->h_vib_pmu = NULL;
		g_vib->vdd_id = 0;

		NvOdmGpioReleasePinHandle( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin );
		NvOdmGpioClose( g_vib->h_vib_gpio );

		if ( g_vib->hOdmPwm!= NULL ) {
			RequestedPeriod = 90000; // 49000;	
			DutyCycle = 0x00320000; // 0x00630000;
			NvOdmPwmConfig( g_vib->hOdmPwm, NvOdmPwmOutputId_PWM3, NvOdmPwmMode_Disable, DutyCycle, &RequestedPeriod, &ReturnedPeriod );
		}
				
		kfree( g_vib );
		g_vib = NULL;

	}

}
#endif

//20101218  vib disable on reboot
void vib_enable( NvBool on )
{
  	/*
	if ( vib_set_power_rail( g_vib->vdd_id, on ) != 0 ) {
		printk( "[ImmVibeSPI][%s] : Failed to vib_set_power_rail\n", __func__ );
	} 
	*/

	NvOdmGpioConfig( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, NvOdmGpioPinMode_Output );
	NvOdmGpioSetState( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, on );
}
//20100111  vib disable on reboot 2nd [START]
void vib_enable_reboot( NvBool on )
{
  	
	if ( vib_set_power_rail( g_vib->vdd_id, on ) != 0 ) {
		printk( "[ImmVibeSPI][%s] : Failed to vib_set_power_rail\n", __func__ );
	}
	

	NvOdmGpioConfig( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, NvOdmGpioPinMode_Output );
	NvOdmGpioSetState( g_vib->h_vib_gpio, g_vib->h_vib_gpio_pin, on );
}
//20100111  vib disable on reboot 2nd [END]

// mode [1] : disable
// mode [2] : enable
static void vib_generatePWM( NvOdmPwmMode mode )
{
	NvU32	ReturnedPeriod;
	NvU32	DutyCycle;
	
	DbgOut(( "[ImmVibeSPI] : vib_generatePWM start.. mode[%d]\n", mode ));	

	if ( g_vib->hOdmPwm!= NULL ) {
		DutyCycle = 0x00320000; // 50% duty
		NvOdmPwmConfig( g_vib->hOdmPwm, NvOdmPwmOutputId_PWM0, mode, DutyCycle, &g_requestedPeriod, &ReturnedPeriod );
	}
	else	{
		printk( "[ImmVibeSPI] : Failed to vib_generatePWM.\n");
	}
	DbgOut(( "[ImmVibeSPI] : vib_generatePWM end\n" ));	
}


/* Called to disable amp (disable output force) */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable( VibeUInt8 nActuatorIndex )
{
	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpDisable[%d]\n", g_bAmpEnabled ));

    if ( g_bAmpEnabled ) {
        g_bAmpEnabled = false;

		// Disable GPIO(enable pin)
		vib_enable( NV_FALSE );

		// Disable PWM CLK
		vib_generatePWM( NvOdmPwmMode_Disable );
	}

	return VIBE_S_SUCCESS;
}

/* Called to enable amp (enable output force) */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable( VibeUInt8 nActuatorIndex )
{
	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_AmpEnabled[%d]\n", g_bAmpEnabled ));

	if ( ! g_bAmpEnabled ) {
        g_bAmpEnabled = true;

		// Generate PWM CLK
		vib_generatePWM(NvOdmPwmMode_Enable);

		// Enable GPIO(enable pin)
		vib_enable(NV_TRUE);
	}

   	return VIBE_S_SUCCESS;
}

/* Called at initialization time to set PWM freq, disable amp, etc... */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize( void )
{
	int status = 0;

   	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Initialize\n" ));

   	status = vib_init( );
   	DbgOut(( "[ImmVibeSPI] vib_init status =  %d \n", status ));

   	g_bAmpEnabled = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */

   	/* 
    	** Disable amp.
    	** If multiple actuators are supported, please make sure to call ImmVibeSPI_ForceOut_AmpDisable
   	** for each actuator (provide the actuator index as input argument).
    	*/
   	ImmVibeSPI_ForceOut_AmpDisable( 0 );

   	return VIBE_S_SUCCESS;
}

/* Called at termination time to set PWM freq, disable amp, etc... */
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate( void )
{
   	DbgOut(( "[ImmVibeSPI] : ImmVibeSPI_ForceOut_Terminate\n" ));

   	/* 
    	** Disable amp.
    	** If multiple actuators are supported, please make sure to call ImmVibeSPI_ForceOut_AmpDisable
    	** for each actuator (provide the actuator index as input argument).
    	*/
   	ImmVibeSPI_ForceOut_AmpDisable(0);

   	return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Set( VibeUInt8 nActuatorIndex, VibeInt8 nForce )
{
	NvU32	DutyCycle;
	NvU32	ReturnedPeriod;
	
//	DbgOut(( "[ImmVibeSPI] ImmVibeSPI_ForceOut_Set nForce =  %d \n", nForce ));

   	if ( nForce == 0 ) {
		DutyCycle = 0x00320000; // 50% duty
   	} else {
		DutyCycle = ((100*nForce/256)+50) << 16;
//		printk( "[ImmVibeSPI] ImmVibeSPI_ForceOut_Set DutyCycle =  %x \n", DutyCycle );
   	}

	NvOdmPwmConfig( g_vib->hOdmPwm, NvOdmPwmOutputId_PWM0, NvOdmPwmMode_Enable, DutyCycle, &g_requestedPeriod, &ReturnedPeriod );

   	return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples( VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer )
{
    VibeStatus status = VIBE_E_FAIL;

    /* nOutputSignalBitDepth should always be 8 */

    if (1 == nBufferSizeInBytes)
    {
        status = ImmVibeSPI_ForceOut_Set(nActuatorIndex, pForceOutputBuffer[0]);
    }
    else
    {
        /* Send 'nBufferSizeInBytes' bytes of data to HW */
        /* Will get here only if configured to handle Piezo actuators */
		printk( "[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d \n", nBufferSizeInBytes );
    }

    return status;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency( VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue )
{

   	/* This function is not called for ERM device */
	return VIBE_S_SUCCESS;

}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName( VibeUInt8 nActuatorIndex, char *szDevName, int nSize )
{

//#error Please review the code between the #if and #endif

#if 0   /* The following code is provided as a sample. Please modify as required. */
    	if ((!szDevName) || (nSize < 1)) 
		return VIBE_E_FAIL;

    	DbgOut((KERN_DEBUG "ImmVibeSPI_Device_GetName.\n"));

    	strncpy(szDevName, "Generic Linux Device", nSize-1);
    	szDevName[nSize - 1] = '\0';    /* make sure the string is NULL terminated */
#endif

    	return VIBE_S_SUCCESS;

}
