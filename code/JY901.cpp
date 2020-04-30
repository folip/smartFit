#include "JY901.h"
#include "string.h"

CJY901 ::CJY901 ()
{
	ucDevAddr =0x50;
}
void CJY901::StartIIC()
{
	ucDevAddr = 0x50;
	Wire.begin();
}
void CJY901::StartIIC(unsigned char ucAddr)
{
	ucDevAddr = ucAddr;
	Wire.begin();
}
void CJY901 ::CopeSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) 
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;
	}
}
void CJY901::readRegisters(unsigned char deviceAddr,unsigned char addressToRead, unsigned char bytesToRead, char * dest)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(deviceAddr, bytesToRead); //Ask for bytes, once done, bus is released by default

  while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();    
}
void CJY901::writeRegister(unsigned char deviceAddr,unsigned char addressToWrite,unsigned char bytesToRead, char *dataToWrite)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(addressToWrite);
  for(int i = 0 ; i < bytesToRead ; i++)
  Wire.write(dataToWrite[i]);
  Wire.endTransmission(); //Stop transmitting
}

short CJY901::ReadWord(unsigned char ucAddr)
{
	short sResult;
	readRegisters(ucDevAddr, ucAddr, 2, (char *)&sResult);
	return sResult;
}
void CJY901::WriteWord(unsigned char ucAddr,short sData)
{	
	writeRegister(ucDevAddr, ucAddr, 2, (char *)&sData);
}
void CJY901::ReadData(unsigned char ucAddr,unsigned char ucLength,char chrData[])
{
	readRegisters(ucDevAddr, ucAddr, ucLength, chrData);
}

void CJY901::GetAcc()
{
	readRegisters(ucDevAddr, AX, 6, (char *)&stcAcc);
}
void CJY901::GetGyro()
{
	readRegisters(ucDevAddr, GX, 6, (char *)&stcGyro);
}

void CJY901::GetAngle()
{
	readRegisters(ucDevAddr, Roll, 6, (char *)&stcAngle);
}
