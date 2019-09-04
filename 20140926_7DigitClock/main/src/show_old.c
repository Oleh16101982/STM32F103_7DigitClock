
#include "stm32f10x_conf.h"
#include "show.h"
#include "main.h"

extern uint8_t arrPWM[0x40];

extern uint8_t arrDigit[10];	 
	 
extern uint8_t arrAnodeValue[4];
extern uint8_t arrSegmentValue[8];
extern uint8_t arrSeparatorValue[2];

extern uint16_t Anode0_Val;
extern uint16_t Anode1_Val ;
extern uint16_t Anode2_Val;
extern uint16_t Anode3_Val;

extern uint16_t SegmentA_Val;
extern uint16_t SegmentB_Val;
extern uint16_t SegmentC_Val;
extern uint16_t SegmentD_Val;
extern uint16_t SegmentE_Val;
extern uint16_t SegmentF_Val;
extern uint16_t SegmentG_Val;
extern uint16_t SegmentDP_Val;

extern uint8_t 	FlagRTC_Irq;
extern uint16_t MinSeparator1_Val;
extern uint16_t MaxSeparator1_Val;
extern uint16_t MinSeparator2_Val;
extern uint16_t MaxSeparator2_Val;
extern uint16_t Separator1_Val;
extern uint16_t Separator2_Val;

void ShowSeparator(void)
{
// Separator 1	
// fade in	
	if (FlagRTC_Irq & 0x02)	{if (arrSeparatorValue[0] < MaxSeparator1_Val)	{++arrSeparatorValue[0];}}
// fade out	
	if (FlagRTC_Irq & 0x04)	{if (arrSeparatorValue[0] > MinSeparator1_Val) {--arrSeparatorValue[0];}}
// Separator 2	
// fade in	
	if (FlagRTC_Irq & 0x08)	{if (arrSeparatorValue[1] < MaxSeparator2_Val)	{++arrSeparatorValue[1];}}
// fade out	
	if (FlagRTC_Irq & 0x10)	{if (arrSeparatorValue[1] > MinSeparator2_Val) {--arrSeparatorValue[1];}}
	
	Separator1_Val = arrPWM[arrSeparatorValue[0]];
	Separator2_Val = arrPWM[arrSeparatorValue[1]];	
	SetCCR(CCR_SEPARATOR);
}
