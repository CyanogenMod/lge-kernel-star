/*
 * Copyright (c) 2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "pcf50626_batterycharger.h"
#include "pcf50626_adc.h"
#include "pcf50626_i2c.h"
#include "pcf50626_reg.h"


/* Get battery Voltage */
NvBool 
Pcf50626BatteryChargerGetVoltage(NvOdmPmuDeviceHandle hDevice, NvU32 *res)
{
    NvU32 volt = 0;
    ///TODO: check on HW to see the relation between adc output and the voltage. for now, assume they are the same.
    if(! Pcf50626AdcVBatSenseRead(hDevice,&volt))
        return NV_FALSE;

    *res = volt;
    return NV_TRUE; 
}



/* check OnKey Level */
NvBool
Pcf50626BatteryChargerOnKeyStatus(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_OOCS_ADDR, &data))
        return NV_FALSE
        ;
    data = (data >> PCF50626_OOCS_ONKEY_SHIFT) & PCF50626_OOCS_ONKEY_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 

    return NV_TRUE;
}

/* check OnKey Level */
NvBool
Pcf50626BatteryChargerRec1Status(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_OOCS_ADDR, &data))
        return NV_FALSE;
    data = (data >> PCF50626_OOCS_REC1_SHIFT) & PCF50626_OOCS_REC1_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 

    return NV_TRUE;
}

/* check battery status */
NvBool
Pcf50626BatteryChargerBattStatus(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_OOCS_ADDR, &data))
        return NV_FALSE;
    data = (data >> PCF50626_OOCS_BATOK_SHIFT) & PCF50626_OOCS_BATOK_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 

    return NV_TRUE;
}

/* check main charger status */
NvBool
Pcf50626BatteryChargerMainChgPresent(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_OOCS_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_OOCS_MCHGOK_SHIFT) & PCF50626_OOCS_MCHGOK_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}


/* check USB charger status */
NvBool
Pcf50626BatteryChargerUsbChgPresent(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_OOCS_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_OOCS_UCHGOK_SHIFT) & PCF50626_OOCS_UCHGOK_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}


/* check temparature status */
NvBool
Pcf50626BatteryChargerTempStatus(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_OOCS_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_OOCS_TEMPOK_SHIFT) & PCF50626_OOCS_TEMPOK_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}

/* check CBC batt_ful status */
NvBool
Pcf50626BatteryChargerCBCBattFul(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_CBCS1_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_CBCS1_BATTFUL_SHIFT) & PCF50626_CBCS1_BATTFUL_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}


/* check CBC thermal limit activation status */
NvBool
Pcf50626BatteryChargerCBCTlimStatus(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_CBCS1_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_CBCS1_TLIMIT_SHIFT) & PCF50626_CBCS1_TLIMIT_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}

/* check CBC batt_ful status */
NvBool
Pcf50626BatteryChargerCBCWdExpired(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_CBCS1_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_CBCS1_WDEXP_SHIFT) & PCF50626_CBCS1_WDEXP_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}


/* check CBC charger current status */
NvBool
Pcf50626BatteryChargerCBCChgCurStatus(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_CBCS1_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_CBCS1_ILIMIT_SHIFT) & PCF50626_CBCS1_ILIMIT_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}

/* check CBC charger voltage status */
NvBool
Pcf50626BatteryChargerCBCChgVoltStatus(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_CBCS1_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_CBCS1_VLIMIT_SHIFT) & PCF50626_CBCS1_VLIMIT_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}

/* check CBC charger resume status */
NvBool
Pcf50626BatteryChargerCBCChgResStatus(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_CBCS1_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_CBCS1_RESSTAT_SHIFT) & PCF50626_CBCS1_RESSTAT_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}


/* check USB suspend status */
NvBool
Pcf50626BatteryChargerCBCUsbSuspStat(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_CBCS2_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_CBCS2_USBSUSPSTAT_SHIFT) & PCF50626_CBCS2_USBSUSPSTAT_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}

/* check charger over-voltage protection status */
NvBool
Pcf50626BatteryChargerCBCChgOvpStat(NvOdmPmuDeviceHandle hDevice, NvBool *status)
{
    NvU8 data = 0;
    
    if(! Pcf50626I2cRead8(hDevice, PCF50626_CBCS2_ADDR, &data))
        return NV_FALSE;

    data = (data >> PCF50626_CBCS2_CHGOVP_SHIFT) & PCF50626_CBCS2_CHGOVP_MASK;
    *status = (data == 0 ? NV_FALSE : NV_TRUE ); 
    return NV_TRUE;
}



