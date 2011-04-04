#include <linux/broadcast/broadcast_lg2102_includes.h>
#ifdef LGD_I2C_GPIO_CTRL_ENABLE


#define LGD_I2C_REGISTER			BIO_EEPROM_REG
#define LGD_I2C_CLK_MASK			BIO_EEPROM_CLK_M
#define LGD_I2C_DATA_MASK			BIO_EEPROM_DATA_M
#define LGD_I2C_OUTPUT				0xffffffff
#define LGD_I2C_INPUT				0x00000000

/************************************************************************/
/* I2C_SDA_CONFIGREG_INPUT()                                            */
/* 아래의 메크로에 SDA port를 입력으로 설정하는 코드 입력               */
/************************************************************************/
#define I2C_SDA_CONFIGREG_INPUT()		BIO_TRISTATE(LGD_I2C_REGISTER, LGD_I2C_DATA_MASK, LGD_I2C_INPUT)

/************************************************************************/
/* I2C_SDA_CONFIGREG_OUTPUT()                                           */
/* 아래의 메크로에 SDA port를 출력으로 설정하는 코드 입력               */
/************************************************************************/
#define I2C_SDA_CONFIGREG_OUTPUT()		BIO_TRISTATE(LGD_I2C_REGISTER, LGD_I2C_DATA_MASK, LGD_I2C_OUTPUT)

/************************************************************************/
/* I2C_SCL_CONFIGREG_INPUT()                                            */
/* 아래의 메크로에 SCL port를 입력으로 설정하는 코드 입력               */
/************************************************************************/
#define I2C_SCL_CONFIGREG_INPUT()		BIO_TRISTATE(LGD_I2C_REGISTER, LGD_I2C_CLK_MASK, LGD_I2C_INPUT)

/************************************************************************/
/* I2C_SCL_CONFIGREG_OUTPUT()                                           */
/* 아래의 메크로에 SCL port를 출력으로 설정하는 코드 입력               */
/************************************************************************/
#define I2C_SCL_CONFIGREG_OUTPUT()		BIO_TRISTATE(LGD_I2C_REGISTER, LGD_I2C_CLK_MASK, LGD_I2C_OUTPUT)

/************************************************************************/
/* I2C_SCL_WRITE(X) 													*/
/* SCL X값 출력 														*/
/************************************************************************/
#define LGD_I2C_CLOCK_WRITE_CTRL(X)		BIO_OUT(LGD_I2C_REGISTER, LGD_I2C_CLK_MASK, X)

/************************************************************************/
/* I2C_SDA_WRITE(X) 													*/
/* SDA X값 출력 														*/
/************************************************************************/
#define LGD_I2C_DATA_WRITE_CTRL(X)		BIO_OUT(LGD_I2C_REGISTER, LGD_I2C_DATA_MASK, X)

/************************************************************************/
/* I2C_SDA_READ()														*/
/* SDA 입력 															*/
/************************************************************************/
#define LGD_I2C_DATA_READ_CTRL()		BIO_INM(LGD_I2C_REGISTER, LGD_I2C_DATA_MASK)


LGD_UINT8 I2C_SDA_READ(void)
{
	LGD_UINT8 ucBit;
	ucBit = LGD_I2C_DATA_READ_CTRL() ? 1 : 0;
	return ucBit;
}

void I2C_SDA_WRITE(LGD_UINT8 ucBit)
{
	LGD_UINT32 ulBitData;
	ulBitData = (ucBit == I2C_BIT_HIGH) ? LGD_I2C_DATA_MASK : 0;
	LGD_I2C_DATA_WRITE_CTRL(ulBitData);
}

void I2C_SCL_WRITE(LGD_UINT8 ucBit)
{
	LGD_UINT32 ulBitData;
	ulBitData = (ucBit == I2C_BIT_HIGH) ? LGD_I2C_CLK_MASK : 0;
	LGD_I2C_CLOCK_WRITE_CTRL(ulBitData);
}

void CLOCK_WAIT(LGD_UINT8 ucDelay)
{
//	clk_busy_wait(ucDelay);
}

void LGD_GPIO_I2C_READ_ACK(void)
{
	I2C_SDA_WRITE(I2C_BIT_LOW);
	I2C_SDA_CONFIGREG_OUTPUT();
	I2C_SCL_WRITE(I2C_BIT_HIGH);
	CLOCK_WAIT(1);
	I2C_SCL_WRITE(I2C_BIT_LOW);
	CLOCK_WAIT(1);
}

LGD_I2C_ACK LGD_GPIO_I2C_ACK(void)
{
	LGD_I2C_ACK AckBit;
	I2C_SDA_CONFIGREG_INPUT();
	I2C_SCL_WRITE(I2C_BIT_HIGH);
	AckBit = (LGD_I2C_ACK)I2C_SDA_READ();

	I2C_SCL_WRITE(I2C_BIT_LOW);
	I2C_SDA_CONFIGREG_OUTPUT();
	return AckBit;
}

void LGD_GPIO_I2C_INIT(void)
{
	I2C_SCL_CONFIGREG_OUTPUT();
	I2C_SDA_CONFIGREG_OUTPUT();
	CLOCK_WAIT(0);
	I2C_SCL_WRITE(I2C_BIT_HIGH);
	I2C_SDA_WRITE(I2C_BIT_HIGH);
	CLOCK_WAIT(1);
}

void LGD_GPIO_I2C_START_COMMAND(void)
{
	I2C_SDA_WRITE(I2C_BIT_HIGH);
	I2C_SCL_WRITE(I2C_BIT_HIGH);
	I2C_SDA_WRITE(I2C_BIT_LOW);
	CLOCK_WAIT(1);
	I2C_SCL_WRITE(I2C_BIT_LOW);
}

void LGD_GPIO_I2C_STOP_COMMAND(void)
{ 
	I2C_SDA_WRITE(I2C_BIT_LOW);
	I2C_SCL_WRITE(I2C_BIT_HIGH);
	CLOCK_WAIT(0);
	I2C_SDA_WRITE(I2C_BIT_HIGH);
	CLOCK_WAIT(1);
}

LGD_I2C_ACK LGD_GPIO_WRITE_BYTE_IO(LGD_UINT8 ucData)
{
	LGD_UINT8 	nLoop;
	LGD_UINT32 	ulBitValue;

	for(nLoop = 0; nLoop < I2C_DATA_BIT_MAX; nLoop++)
	{
		ulBitValue = (ucData & 0x80) ? I2C_BIT_HIGH : I2C_BIT_LOW;
		I2C_SDA_WRITE(ulBitValue);
		I2C_SCL_WRITE(I2C_BIT_HIGH);
		CLOCK_WAIT(1);
		I2C_SCL_WRITE(I2C_BIT_LOW);
		CLOCK_WAIT(1);
		ucData <<= 1;
	}

    return LGD_GPIO_I2C_ACK();
}

LGD_UINT8 LGD_GPIO_READ_BYTE_IO(void)
{
	LGD_UINT8 ucData = 0;
	LGD_UINT8 nLoop;

	for(nLoop = 0; nLoop < I2C_DATA_BIT_MAX; nLoop++)
	{
		CLOCK_WAIT(1);
		I2C_SCL_WRITE(I2C_BIT_HIGH);
		CLOCK_WAIT(1);

		ucData <<= 1;
		if(I2C_SDA_READ())
			ucData |= 1;

		I2C_SCL_WRITE(I2C_BIT_LOW);
	}
	return ucData;
}

LGD_I2C_ACK LGD_GPIO_READ_BYTES(LGD_UINT8* pBuff, LGD_UINT16 uiSize)
{
	LGD_UINT16  nLoop;
	LGD_I2C_ACK AckStatus;
	AckStatus = I2C_ACK_SUCCESS;
	
	for(nLoop = 0; nLoop < uiSize; nLoop++) {
		I2C_SDA_CONFIGREG_INPUT();
		pBuff[nLoop] = LGD_GPIO_READ_BYTE_IO();
		LGD_GPIO_I2C_READ_ACK();
		CLOCK_WAIT(2);
	}
	return AckStatus;
}

LGD_I2C_ACK LGD_GPIO_WRITE_BYTES(LGD_UINT8* pBuff, LGD_UINT16 uiSize)
{
	LGD_INT16   nLoop;
	LGD_I2C_ACK AckStatus;
	AckStatus = I2C_ACK_SUCCESS;

	for(nLoop = 0; nLoop < uiSize; nLoop++){
		AckStatus =  LGD_GPIO_WRITE_BYTE_IO(pBuff[nLoop]);
		if(AckStatus == I2C_ACK_SUCCESS) continue;
		else return AckStatus;
	}

	return AckStatus;
}

LGD_I2C_ACK LGD_GPIO_CTRL_WRITE(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT8* pBuff, LGD_UINT16 uiSize)
{
	LGD_UINT16	ucCnt;
	LGD_UINT8	abyBuff[2];
	LGD_I2C_ACK AckStatus;
	
	AckStatus = I2C_ACK_SUCCESS;
	ucCnt = 0;
	
	abyBuff[ucCnt++] = (LGD_UINT8)((uiAddr >> 8) & 0xff);
	abyBuff[ucCnt++] = (LGD_UINT8)uiAddr;

	// I2C Start Bit Control
	LGD_GPIO_I2C_START_COMMAND();

	// I2C write command write(0x80)
	AckStatus = LGD_GPIO_WRITE_BYTE_IO(LGD_I2C_ADDRMASK_W(ucI2CID));
	if(AckStatus != I2C_ACK_SUCCESS)
	{
		LGD_GPIO_I2C_STOP_COMMAND();
		return I2C_ACK_ERROR;
	}

	// I2C address write
	AckStatus = LGD_GPIO_WRITE_BYTES(abyBuff, ucCnt);
	if(AckStatus != I2C_ACK_SUCCESS)
	{
		LGD_GPIO_I2C_STOP_COMMAND();
		return I2C_ACK_ERROR;
	}

	// I2C data write
	AckStatus = LGD_GPIO_WRITE_BYTES(pBuff, uiSize);
	if(AckStatus != I2C_ACK_SUCCESS)
	{
		LGD_GPIO_I2C_STOP_COMMAND();
		return I2C_ACK_ERROR;
	}

	// I2C Stop Bit Control
	LGD_GPIO_I2C_STOP_COMMAND();

	return AckStatus;
}

LGD_I2C_ACK LGD_GPIO_CTRL_READ(LGD_UINT8 ucI2CID, LGD_UINT16 uiAddr, LGD_UINT8* pBuff, LGD_UINT16 uiSize)
{
	LGD_UINT16	ucCnt;
	LGD_UINT8	abyBuff[2];
	LGD_I2C_ACK AckStatus;

	AckStatus = I2C_ACK_SUCCESS;
	ucCnt = 0;
	
	abyBuff[ucCnt++] = (LGD_UINT8)((uiAddr >> 8) & 0xff);
	abyBuff[ucCnt++] = (LGD_UINT8)uiAddr;

	// I2C Start Bit Control
	LGD_GPIO_I2C_START_COMMAND();

	// I2C command write(0x80)
	AckStatus = LGD_GPIO_WRITE_BYTE_IO(LGD_I2C_ADDRMASK_W(ucI2CID));
	if(AckStatus != I2C_ACK_SUCCESS)
	{
		LGD_GPIO_I2C_STOP_COMMAND();
		return I2C_ACK_ERROR;
	}

	// I2C address write
	AckStatus = LGD_GPIO_WRITE_BYTES(abyBuff, ucCnt);
	if(AckStatus != I2C_ACK_SUCCESS)
	{
		LGD_GPIO_I2C_STOP_COMMAND();
		return I2C_ACK_ERROR;
	}

	// I2C Stop Bit Control
	LGD_GPIO_I2C_STOP_COMMAND();
	CLOCK_WAIT(5);

	// I2C Start Bit Control
	LGD_GPIO_I2C_START_COMMAND();

	// I2C read command write(0x81)
	AckStatus = LGD_GPIO_WRITE_BYTE_IO(LGD_I2C_ADDRMASK_R(ucI2CID));
	if(AckStatus != I2C_ACK_SUCCESS)
	{
		LGD_GPIO_I2C_STOP_COMMAND();
		return I2C_ACK_ERROR;
	}

	// I2C data receive
	AckStatus = LGD_GPIO_READ_BYTES(pBuff, uiSize);
	if(AckStatus != I2C_ACK_SUCCESS)
	{
		LGD_GPIO_I2C_STOP_COMMAND();
		return AckStatus;
	}

	// I2C Stop Bit Control
	LGD_GPIO_I2C_STOP_COMMAND();
	return AckStatus;
}

#endif
