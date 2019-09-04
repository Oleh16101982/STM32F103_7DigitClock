
#include "stm32f10x_conf.h"
#include "bcdhex.h"


uint32_t BCD16Great5(uint32_t inpBCD)
{
	uint32_t retVal = 0x00000000;
	retVal = inpBCD;
	if ((retVal & 0x0000000F) > 0x00000004) {retVal += 0x00000003;}
	if ((retVal & 0x000000F0) > 0x00000040) {retVal += 0x00000030;}
	if ((retVal & 0x00000F00) > 0x00000400) {retVal += 0x00000300;}
	if ((retVal & 0x0000F000) > 0x00004000) {retVal += 0x00003000;}
	if ((retVal & 0x000F0000) > 0x00040000) {retVal += 0x00030000;}	
	return retVal;
}

void BCD16Great5_ref(uint32_t* retVal)
{
	if ((*retVal & 0x0000000F) > 0x00000004) {*retVal += 0x00000003;}
	if ((*retVal & 0x000000F0) > 0x00000040) {*retVal += 0x00000030;}
	if ((*retVal & 0x00000F00) > 0x00000400) {*retVal += 0x00000300;}
	if ((*retVal & 0x0000F000) > 0x00004000) {*retVal += 0x00003000;}
	if ((*retVal & 0x000F0000) > 0x00040000) {*retVal += 0x00030000;}	
}

uint32_t HEX16_to_BCD(uint16_t inputHex)
{
	uint32_t retVal = 0x00000000;
												if ((inputHex & 0x8000) != 0) {retVal = 0x00000001; }
	retVal = retVal << 1;	if ((inputHex & 0x4000) != 0) {retVal |= 0x00000001 ; }
	retVal = retVal << 1;	if ((inputHex & 0x2000) != 0)	{retVal |= 0x00000001 ; }
	retVal = retVal << 1;	if ((inputHex & 0x1000) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1;	if ((inputHex & 0x0800) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1;	if ((inputHex & 0x0400) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1;	if ((inputHex & 0x0200) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1; if ((inputHex & 0x0100) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal);	
	retVal = retVal << 1;	if ((inputHex & 0x0080) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1;	if ((inputHex & 0x0040) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1;	if ((inputHex & 0x0020) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1;	if ((inputHex & 0x0010) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1;	if ((inputHex & 0x0008) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1;	if ((inputHex & 0x0004) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1;	if ((inputHex & 0x0002) != 0)	{retVal |= 0x00000001 ; }BCD16Great5_ref(&retVal); 
	retVal = retVal << 1;	if ((inputHex & 0x0001) != 0)	{retVal |= 0x00000001 ; }
	return retVal;
}


uint16_t BCD8Great5(uint16_t inpBCD)
{
	uint16_t retVal = 0x0000;
	retVal = inpBCD;
	if ((retVal & 0x000F) > 0x0004) {retVal += 0x0003;}
	if ((retVal & 0x00F0) > 0x0040) {retVal += 0x0030;}
	if ((retVal & 0x0F00) > 0x0400) {retVal += 0x0300;}
	if ((retVal & 0xF000) > 0x4000) {retVal += 0x3000;}
	return retVal;
}

void BCD8Great5_ref(uint16_t* retVal)
{
	if ((*retVal & 0x000F) > 0x0004) {*retVal += 0x0003;}
	if ((*retVal & 0x00F0) > 0x0040) {*retVal += 0x0030;}
	if ((*retVal & 0x0F00) > 0x0400) {*retVal += 0x0300;}
	if ((*retVal & 0xF000) > 0x4000) {*retVal += 0x3000;}	
}

uint16_t HEX8_to_BCD(uint8_t inputHex)
{
	uint16_t retVal = 0x0000;
												if ((inputHex & 0x80) != 0) {retVal = 0x0001;}
	retVal = retVal << 1;	if ((inputHex & 0x40) != 0)	{retVal |= 0x0001;} 
	retVal = retVal << 1;	if ((inputHex & 0x20) != 0)	{retVal |= 0x0001 ;} BCD8Great5_ref(&retVal);
	retVal = retVal << 1;	if ((inputHex & 0x10) != 0)	{retVal |= 0x0001 ;} BCD8Great5_ref(&retVal);
	retVal = retVal << 1;	if ((inputHex & 0x08) != 0)	{retVal |= 0x0001 ;} BCD8Great5_ref(&retVal);
	retVal = retVal << 1;	if ((inputHex & 0x04) != 0)	{retVal |= 0x0001 ;} BCD8Great5_ref(&retVal);
	retVal = retVal << 1;	if ((inputHex & 0x02) != 0)	{retVal |= 0x0001 ;} BCD8Great5_ref(&retVal);
	retVal = retVal << 1;	if ((inputHex & 0x01) != 0)	{retVal |= 0x0001 ;}

	return retVal;
}
