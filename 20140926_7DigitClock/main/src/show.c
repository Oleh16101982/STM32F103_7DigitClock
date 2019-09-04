
#include "stm32f10x_conf.h"
#include "stm32f10x_tim.h"
#include "show.h"
#include "main.h"
	 
volatile uint8_t arrAnodeValue[MAXANODE] = {0x3f, 0x3f, 0x3f, 0x3f};
volatile uint8_t arrAnodeValueCurr[MAXANODE];
volatile uint8_t arrAnodeValuePrev[MAXANODE];

volatile uint8_t arrSegmentValue[MAXSEGMENT] = {0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f};
volatile uint8_t arrSegmentValueCurr[MAXSEGMENT];
// volatile uint8_t arrSegmentValuePrev[MAXSEGMENT];

volatile uint8_t arrSysTickSegmentValueCurr[MAXANODE][MAXSEGMENT] = {
																																	0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f,
																																	0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f,
																																	0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f,
																																	0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f
																																};

volatile uint8_t arr7DigitSegmentValueCurr[MAXANODE] = {0x00, 0x00, 0x00, 0x00};
volatile uint8_t arr7DigitSegmentValuePrev[MAXANODE] = {0x00, 0x00, 0x00, 0x00};

volatile uint8_t arrSeparatorValue[MAXSEPARATOR] = {0x3f, 0x3f};
volatile uint8_t arrSeparatorValueCurr[MAXSEPARATOR];
volatile uint8_t arrSeparatorValuePrev[MAXSEPARATOR];

volatile uint8_t arrSeparatorCurr[MAXSEPARATOR];
volatile uint8_t arrSeparatorPrev[MAXSEPARATOR];


volatile uint8_t currAnode = 0;

volatile uint8_t ind_dp = DP_NO_IND;
/*
volatile uint16_t Ind7DigitSegmentValue;
volatile uint16_t Ind7DigitSegmentValuePrev;
volatile uint8_t IndAnodeValue;
volatile uint8_t IndAnodeValuePrev;
volatile uint8_t IndSeparatorValue;
volatile uint8_t IndSeparatorValuePrev;
*/
extern TIM_OCInitTypeDef TIM_OCConfig;
extern uint8_t WorkMode;

extern uint16_t bcd_hour_minutes;
extern uint16_t bcd_minutes_second;
extern uint16_t bcd_day_month;
extern uint16_t bcd_year;
extern uint16_t bcd_alarm[2];
extern uint16_t bcd_temperature[2];

extern uint8_t SeparatorMode;
extern uint8_t currSysTick;

void SetAnodeValue(uint8_t numberAnode)
{
  TIM_OCConfig.TIM_Pulse = arrPWM[arrAnodeValueCurr[0]];
  TIM_OC1Init(TIM1, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = arrPWM[arrAnodeValueCurr[1]];
  TIM_OC2Init(TIM1, &TIM_OCConfig);

  TIM_OCConfig.TIM_Pulse = arrPWM[arrAnodeValueCurr[2]];
  TIM_OC3Init(TIM1, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = arrPWM[arrAnodeValueCurr[3]];
  TIM_OC4Init(TIM1, &TIM_OCConfig);	
}

void defAnodeValue(uint8_t numberAnode)
{
	arrAnodeValuePrev[numberAnode] = arrAnodeValueCurr[numberAnode];
	arrAnodeValueCurr[0] =  arrAnodeValueCurr[1] = arrAnodeValueCurr[2] = arrAnodeValueCurr[3] = 0;
	arrAnodeValueCurr[numberAnode] = arrAnodeValue[numberAnode];
	SetAnodeValue(numberAnode);
}

void def7DigitSegmentValueCurr(void)
{
	uint8_t i;
	for (i = 0 ; i < MAXANODE; i++) {arr7DigitSegmentValuePrev[i] = arr7DigitSegmentValueCurr[i];}
	switch (WorkMode)
	{
		case MODE_HOUR_MINUTES :
		{
			for(i = 0; i < MAXANODE; i++) { arr7DigitSegmentValueCurr[i] = arrDigit[(uint8_t)((bcd_hour_minutes >> (4 * i)) & 0x000F)] ;}
/*			
			arr7DigitSegmentValueCurr[0] = arrDigit[(uint8_t)((bcd_hour_minutes & 0x000F) >> 0x00)];
			arr7DigitSegmentValueCurr[1] = arrDigit[(uint8_t)((bcd_hour_minutes & 0x00F0) >> 0x04)];
			arr7DigitSegmentValueCurr[2] = arrDigit[(uint8_t)((bcd_hour_minutes & 0x0F00) >> 0x08)];
			arr7DigitSegmentValueCurr[3] = arrDigit[(uint8_t)((bcd_hour_minutes & 0xF000) >> 0x0C)];
*/			
			break;
		}
		case MODE_MINUTES_SECOND :
		{
			for(i = 0; i < MAXANODE; i++) { arr7DigitSegmentValueCurr[i] = arrDigit[(uint8_t)((bcd_minutes_second >> (4 * i)) & 0x000F)] ;}
/*			
			arr7DigitSegmentValueCurr[0] = arrDigit[(uint8_t)((bcd_minutes_second & 0x000F) >> 0x00)];
			arr7DigitSegmentValueCurr[1] = arrDigit[(uint8_t)((bcd_minutes_second & 0x00F0) >> 0x04)];
			arr7DigitSegmentValueCurr[2] = arrDigit[(uint8_t)((bcd_minutes_second & 0x0F00) >> 0x08)];
			arr7DigitSegmentValueCurr[3] = arrDigit[(uint8_t)((bcd_minutes_second & 0xF000) >> 0x0C)];
*/			
			break;
		}
		case MODE_DAY_MONTH :
		{
			for(i = 0; i < MAXANODE; i++) { arr7DigitSegmentValueCurr[i] = arrDigit[(uint8_t)((bcd_day_month >> (4 * i)) & 0x000F)] ;}
/*			
			arr7DigitSegmentValueCurr[0] = arrDigit[(uint8_t)((bcd_day_month & 0x000F) >> 0x00)];
			arr7DigitSegmentValueCurr[1] = arrDigit[(uint8_t)((bcd_day_month & 0x00F0) >> 0x04)];
			arr7DigitSegmentValueCurr[2] = arrDigit[(uint8_t)((bcd_day_month & 0x0F00) >> 0x08)];
			arr7DigitSegmentValueCurr[3] = arrDigit[(uint8_t)((bcd_day_month & 0xF000) >> 0x0C)];
*/			
			break;
		}			
		case MODE_YEAR :
		{
			for(i = 0; i < MAXANODE; i++) { arr7DigitSegmentValueCurr[i] = arrDigit[(uint8_t)((bcd_year >> (4 * i)) & 0x000F)] ;}
/*
			arr7DigitSegmentValueCurr[0] = arrDigit[(uint8_t)((bcd_year & 0x000F) >> 0x00)];
			arr7DigitSegmentValueCurr[1] = arrDigit[(uint8_t)((bcd_year & 0x00F0) >> 0x04)];
			arr7DigitSegmentValueCurr[2] = arrDigit[(uint8_t)((bcd_year & 0x0F00) >> 0x08)];
			arr7DigitSegmentValueCurr[3] = arrDigit[(uint8_t)((bcd_year & 0xF000) >> 0x0C)];
*/			
			break;
		}			
		case MODE_ALARM1 :
		{
			for(i = 0; i < MAXANODE; i++) { arr7DigitSegmentValueCurr[i] = arrDigit[(uint8_t)((bcd_alarm[0] >> (4 * i)) & 0x000F)] ;}
/*			
			arr7DigitSegmentValueCurr[0] = arrDigit[(uint8_t)((bcd_alarm[0] & 0x000F) >> 0x00)];
			arr7DigitSegmentValueCurr[1] = arrDigit[(uint8_t)((bcd_alarm[0] & 0x00F0) >> 0x04)];
			arr7DigitSegmentValueCurr[2] = arrDigit[(uint8_t)((bcd_alarm[0] & 0x0F00) >> 0x08)];
			arr7DigitSegmentValueCurr[3] = arrDigit[(uint8_t)((bcd_alarm[0] & 0xF000) >> 0x0C)];
*/			
			break;
		}			
		case MODE_ALARM2 :
		{
			for(i = 0; i < MAXANODE; i++) { arr7DigitSegmentValueCurr[i] = arrDigit[(uint8_t)((bcd_alarm[1] >> (4 * i)) & 0x000F)] ;}
/*			
			arr7DigitSegmentValueCurr[0] = arrDigit[(uint8_t)((bcd_alarm[1] & 0x000F) >> 0x00)];
			arr7DigitSegmentValueCurr[1] = arrDigit[(uint8_t)((bcd_alarm[1] & 0x00F0) >> 0x04)];
			arr7DigitSegmentValueCurr[2] = arrDigit[(uint8_t)((bcd_alarm[1] & 0x0F00) >> 0x08)];
			arr7DigitSegmentValueCurr[3] = arrDigit[(uint8_t)((bcd_alarm[1] & 0xF000) >> 0x0C)];
*/			
			break;
		}			
		case MODE_TEMPERATURE1 :
		{
			for(i = 0; i < MAXANODE; i++) { arr7DigitSegmentValueCurr[i] = arrDigit[(uint8_t)((bcd_temperature[0] >> (4 * i)) & 0x000F)] ;}
/*			
			arr7DigitSegmentValueCurr[0] = arrDigit[(uint8_t)((bcd_temperature[0] & 0x000F) >> 0x00)];
			arr7DigitSegmentValueCurr[1] = arrDigit[(uint8_t)((bcd_temperature[0] & 0x00F0) >> 0x04)];
			arr7DigitSegmentValueCurr[2] = arrDigit[(uint8_t)((bcd_temperature[0] & 0x0F00) >> 0x08)];
			arr7DigitSegmentValueCurr[3] = arrDigit[(uint8_t)((bcd_temperature[0] & 0xF000) >> 0x0C)];
*/			
			break;
		}			
		case MODE_TEMPERATURE2 :
		{
			for(i = 0; i < MAXANODE; i++) { arr7DigitSegmentValueCurr[i] = arrDigit[(uint8_t)((bcd_temperature[1] >> (4 * i)) & 0x000F)] ;}
/*			
			arr7DigitSegmentValueCurr[0] = arrDigit[(uint8_t)((bcd_temperature[1] & 0x000F) >> 0x00)];
			arr7DigitSegmentValueCurr[1] = arrDigit[(uint8_t)((bcd_temperature[1] & 0x00F0) >> 0x04)];
			arr7DigitSegmentValueCurr[2] = arrDigit[(uint8_t)((bcd_temperature[1] & 0x0F00) >> 0x08)];
			arr7DigitSegmentValueCurr[3] = arrDigit[(uint8_t)((bcd_temperature[1] & 0xF000) >> 0x0C)];
*/			
			break;
		}			
		case MODE_MIX1 :
		{
			
			
			break;
		}			
		case MODE_MIX2 :
		{
			
			
			break;
		}		
	}
	
}

void SetSegmentValue(void)
{
  TIM_OCConfig.TIM_Pulse = arrPWM[arrSysTickSegmentValueCurr[currAnode][0]];
  TIM_OC1Init(TIM2, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = arrPWM[arrSysTickSegmentValueCurr[currAnode][1]];
  TIM_OC2Init(TIM2, &TIM_OCConfig);

  TIM_OCConfig.TIM_Pulse = arrPWM[arrSysTickSegmentValueCurr[currAnode][2]];
  TIM_OC3Init(TIM2, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = arrPWM[arrSysTickSegmentValueCurr[currAnode][3]];
  TIM_OC4Init(TIM2, &TIM_OCConfig);

  TIM_OCConfig.TIM_Pulse = arrPWM[arrSysTickSegmentValueCurr[currAnode][4]];
  TIM_OC1Init(TIM3, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = arrPWM[arrSysTickSegmentValueCurr[currAnode][5]];
  TIM_OC2Init(TIM3, &TIM_OCConfig);

  TIM_OCConfig.TIM_Pulse = arrPWM[arrSysTickSegmentValueCurr[currAnode][6]];
  TIM_OC3Init(TIM3, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = arrPWM[arrSysTickSegmentValueCurr[currAnode][7]];
  TIM_OC4Init(TIM3, &TIM_OCConfig);
}

void defSegmentValue(uint8_t numberAnode)
{
	uint8_t SegmentValue = 0;
	uint8_t i;
	SegmentValue = arrDigit[arr7DigitSegmentValueCurr[numberAnode]];
	if (ind_dp) {SegmentValue |= 0xF0;} else { SegmentValue &= 0x7F;}
	for (i = 0; i < MAXSEGMENT ; i++) {arrSegmentValueCurr[i] = arrSegmentValue[i] & (SegmentValue & (0x01 << i));}
/*	
	arrSegmentValueCurr[0] = arrSegmentValue[0] & (SegmentValue & (0x01 << 0));
	arrSegmentValueCurr[1] = arrSegmentValue[1] & (SegmentValue & (0x01 << 1));
	arrSegmentValueCurr[2] = arrSegmentValue[2] & (SegmentValue & (0x01 << 2));
	arrSegmentValueCurr[3] = arrSegmentValue[3] & (SegmentValue & (0x01 << 3));
	arrSegmentValueCurr[4] = arrSegmentValue[4] & (SegmentValue & (0x01 << 4));
	arrSegmentValueCurr[5] = arrSegmentValue[5] & (SegmentValue & (0x01 << 5));
	arrSegmentValueCurr[6] = arrSegmentValue[6] & (SegmentValue & (0x01 << 6));
	arrSegmentValueCurr[7] = arrSegmentValue[7] & (SegmentValue & (0x01 << 7));
*/	
	SetSegmentValue();
}

void defSysTickSegmentValue(void)
{
	uint8_t i, j;
// arr7DigitSegmentValuePrev[i]  arr7DigitSegmentValueCurr[i]	
	for (i = 0; i < MAXANODE; i++)
	{
		for (j = 0; j < MAXSEGMENT; j++)
		{
			if (((arr7DigitSegmentValuePrev[i] & 0x01) << j) < ((arr7DigitSegmentValueCurr[i] & 0x01) << j))
			{
				if (currSysTick <= arrSegmentValue[j])
				{
					arrSysTickSegmentValueCurr[i][j] = arrPWM[currSysTick];
				}
				else
				{
					arrSysTickSegmentValueCurr[i][j] = arrPWM[arrSegmentValue[j]];
				}
			}
			if (((arr7DigitSegmentValuePrev[i] & 0x01) << j) > ((arr7DigitSegmentValueCurr[i] & 0x01) << j))
			{
				if (MAXSTEPSPWM - 1 - currSysTick >= arrSegmentValue[j])
				{
					arrSysTickSegmentValueCurr[i][j] = arrPWM[arrSegmentValue[j]];
				}
				else
				{
					arrSysTickSegmentValueCurr[i][j] = arrPWM[MAXSTEPSPWM - 1 - currSysTick];
				}
			}
			if (((arr7DigitSegmentValuePrev[i] & 0x01) << j) == ((arr7DigitSegmentValueCurr[i] & 0x01) << j))
			{
				arrSysTickSegmentValueCurr[i][j] = arrPWM[arrSysTickSegmentValueCurr[i][j]];
			}
		}
	}
	SetSegmentValue();
}

void ChangeSeparatorMode(void)
{
		switch (SeparatorMode)
	{
		case MODE_SEPARATOR_BLINK_ON :
		{
			arrSeparatorCurr[0] = arrSeparatorCurr[1] = SET_SEPARATOR_ON;
			break;
		}
		case MODE_SEPARATOR_BLINK_OFF :	
		{
			arrSeparatorCurr[0] = SET_SEPARATOR_ON;
			arrSeparatorCurr[1] = SET_SEPARATOR_OFF;			
			break;
		}
		case MODE_SEPARATOR_NO_BLINK_ON :
		{
			arrSeparatorCurr[0] = SET_SEPARATOR_ON;
			arrSeparatorCurr[1] = SET_SEPARATOR_OFF;				
			break;
		}			
		case MODE_SEPARATOR_NO_BLINK_OFF :
		{
			arrSeparatorCurr[0] = SET_SEPARATOR_OFF;
			arrSeparatorCurr[1] = SET_SEPARATOR_ON;			
			break;
		}
		case MODE_SEPARATOR_NO_BLINK_ALL_ON :
		{
			arrSeparatorCurr[0] = SET_SEPARATOR_ON;
			arrSeparatorCurr[1] = SET_SEPARATOR_ON;			
			break;
		}			
		case MODE_SEPARATOR_NO_BLINK_ALL_OFF :	
		{
			arrSeparatorCurr[0] = SET_SEPARATOR_OFF;
			arrSeparatorCurr[1] = SET_SEPARATOR_OFF;				
			break;
		}			
	}
	
}

void SetSeparatorValue(void)
{
  TIM_OCConfig.TIM_Pulse = arrSeparatorValueCurr[0];
  TIM_OC1Init(TIM15, &TIM_OCConfig);

	TIM_OCConfig.TIM_Pulse = arrSeparatorValueCurr[1];
  TIM_OC2Init(TIM15, &TIM_OCConfig);	
}


void SetSeparatorValueBlink(void)
{
	uint8_t i;
	for (i = 0 ; i < MAXSEPARATOR; i++)
	{
		if (arrSeparatorPrev[i] < arrSeparatorCurr[i])
		{
			if (currSysTick <= arrSeparatorValue[i])
			{
				arrSeparatorValueCurr[i] = arrPWM[currSysTick];
			}
			else
			{
				arrSeparatorValueCurr[i] = arrPWM[arrSeparatorValue[i]];
			}
		}
		if (arrSeparatorPrev[i] > arrSeparatorCurr[i])
		{
				if ((MAXSTEPSPWM - 1 - currSysTick) >= arrSeparatorValue[i])
			{
				arrSeparatorValueCurr[i] = arrPWM[arrSeparatorValue[i]];
			}
			else
			{
				arrSeparatorValueCurr[i] = arrPWM[MAXSTEPSPWM - 1 - currSysTick];
			}
		}
		if (arrSeparatorPrev[i] == arrSeparatorCurr[i])
		{
			arrSeparatorValueCurr[i] = arrPWM[arrSeparatorPrev[i]];
		}			
	}
	SetSeparatorValue();
}

void SetSeparatorValueNoBlink(void)
{
	uint8_t i;
	for (i = 0 ; i < MAXSEPARATOR; i++)
	{
		arrSeparatorValueCurr[i] = arrPWM[arrSeparatorValue[i]] * arrSeparatorCurr[i];
	}
}

void defSetSeparatorValue(void)
{
		if ((SeparatorMode == MODE_SEPARATOR_BLINK_ON) || (SeparatorMode == MODE_SEPARATOR_BLINK_OFF))
		{
			SetSeparatorValueBlink();
		}
		else
		{
			SetSeparatorValueNoBlink();
		}
}

void DefineSeparartorValue(void)
{
	arrSeparatorPrev[0] = arrSeparatorCurr[0];
	arrSeparatorPrev[0] = arrSeparatorCurr[1];	
	switch (SeparatorMode)
	{
		case MODE_SEPARATOR_BLINK_ON :
		{
			arrSeparatorCurr[0] = (~arrSeparatorCurr[0]) & 0x01;
			arrSeparatorCurr[1] = (~arrSeparatorCurr[1]) & 0x01;
			break;
		}
		case MODE_SEPARATOR_BLINK_OFF :	
		{
			arrSeparatorCurr[0] = (~arrSeparatorCurr[0]) & 0x01;
			arrSeparatorCurr[1] = (~arrSeparatorCurr[1]) & 0x01;
			break;
		}
		case MODE_SEPARATOR_NO_BLINK_ON :
		{
			arrSeparatorCurr[0] = 0x01;
			arrSeparatorCurr[1] = 0x00;
			break;
		}			
		case MODE_SEPARATOR_NO_BLINK_OFF :
		{
			arrSeparatorCurr[0] = 0x00;
			arrSeparatorCurr[1] = 0x01;			
			break;
		}		
		case MODE_SEPARATOR_NO_BLINK_ALL_ON :
		{
			arrSeparatorCurr[0] = 0x01;
			arrSeparatorCurr[1] = 0x01;			
			break;
		}			
		case MODE_SEPARATOR_NO_BLINK_ALL_OFF :	
		{
			arrSeparatorCurr[0] = 0x00;
			arrSeparatorCurr[1] = 0x00;			
			break;
		}	
		
	}	
}



