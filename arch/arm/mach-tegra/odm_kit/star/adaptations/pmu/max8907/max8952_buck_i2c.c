#include <linux/kernel.h>

#include "nvodm_pmu.h"
#include "nvodm_services.h"
#include "max8952_buck_i2c.h"
#include "max8952_buck_reg.h"

// Function declaration
NvBool Max8952I2cWrite8(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
    NvU8 Data)
{
    NvU8 WriteBuffer[2];
    NvOdmI2cTransactionInfo TransactionInfo;
    NvOdmI2cStatus status = NvOdmI2cStatus_Success;    
    Max8907PrivData *hPmu = (Max8907PrivData*)hDevice->pPrivate;
    NvU32 i;
    
    for (i = 0; i < MAX8952_I2C_RETRY_CNT; i++)
    {
        WriteBuffer[0] = Addr & 0xFF;   // PMU offset
        WriteBuffer[1] = Data & 0xFF;   // written data
    
        TransactionInfo.Address = MAX8952_SLAVE_ADDR;
        TransactionInfo.Buf = &WriteBuffer[0];
        TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo.NumBytes = 2;
    
#if defined(CONFIG_MACH_STAR)
        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo, 1,
                                     MAX8952_I2C_SPEED_KHZ, MAX8952_I2C_TIMEOUT);
#else
        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo, 1,
                                     MAX8952_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
#endif

        if (status == NvOdmI2cStatus_Success)
            return NV_TRUE;
        printk("[MAX8952] I2C WARNING : Max8952I2cWrite8 status=%d\n", status);
    }

    printk("[MAX8952] I2C ERROR : Max8952I2cWrite8 status=%d\n", status);
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("NvOdmPmuI2cWrite8 Failed: Timeout\n")); 
            break;
         case NvOdmI2cStatus_SlaveNotFound:
         default:
            NVODMPMU_PRINTF(("NvOdmPmuI2cWrite8 Failed: SlaveNotFound\n"));
            break;             
    }
    BUG();
    return NV_FALSE;
}

NvBool Max8952I2cRead8(
    NvOdmPmuDeviceHandle hDevice,
    NvU8 Addr,
    NvU8 *Data)
{
    NvU8 ReadBuffer = 0;
    NvOdmI2cStatus  status = NvOdmI2cStatus_Success;    
    Max8907PrivData *hPmu = (Max8907PrivData*)hDevice->pPrivate;
    NvOdmI2cTransactionInfo TransactionInfo[2];
    NvU32 i;

    for (i = 0; i < MAX8952_I2C_RETRY_CNT; i++)
    {
        // Write the PMU offset
        ReadBuffer = Addr;
    
        TransactionInfo[0].Address = MAX8952_SLAVE_ADDR;
        TransactionInfo[0].Buf = &ReadBuffer;
        TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
        TransactionInfo[0].NumBytes = 1;
    
        TransactionInfo[1].Address = (MAX8952_SLAVE_ADDR | 0x1);;
        TransactionInfo[1].Buf = &ReadBuffer;
        TransactionInfo[1].Flags = 0;
        TransactionInfo[1].NumBytes = 1;
    
        // Read data from PMU at the specified offset
#if defined(CONFIG_MACH_STAR)
        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo[0], 2, 
                                     MAX8952_I2C_SPEED_KHZ, MAX8952_I2C_TIMEOUT);
#else
        status = NvOdmI2cTransaction(hPmu->hOdmI2C, &TransactionInfo[0], 2, 
                                     MAX8952_I2C_SPEED_KHZ, NV_WAIT_INFINITE);
#endif

        if (status == NvOdmI2cStatus_Success){
            *Data = ReadBuffer;
            return NV_TRUE;
        }
        printk("[MAX8952] I2C WARNING : Max8952I2cRead8 status=%d\n", status);
    }

    printk("[MAX8952] I2C ERROR : Max8952I2cRead8 status=%d\n", status);
    switch (status)
    {
        case NvOdmI2cStatus_Timeout:
            NVODMPMU_PRINTF(("NvOdmPmuI2cRead8 Failed: Timeout\n")); 
            break;
         case NvOdmI2cStatus_SlaveNotFound:
         default:
            NVODMPMU_PRINTF(("NvOdmPmuI2cRead8 Failed: SlaveNotFound\n"));
            break;             
    }
    BUG();
    return NV_FALSE;
}

