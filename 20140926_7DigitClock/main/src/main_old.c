
#include "stm32f10x_conf.h"
#include "stm32f10x_it.h"
#include "main.h"
#include "clock.h"
#include "bcdhex.h"
#include "rtc.h"

// 0,3,4,6,9,11,14,16,18,21,23,26,29,32,35,38,41,45,48,52,55,59,63,66,70,74,78,82,84,86,91,95,99,104,108,113,115,117,122,126,131,136,141,146,151,156,161,166,172,177,182,187,193,198,204,210,215,221,227,232,238,244,250,255
// const static uint8_t arrPWM[0x40] = {0,3,4,6,9,11,14,16,18,21,23,26,29,32,35,38,41,45,48,52,55,59,63,66,70,74,78,82,84,86,91,95,99,104,108,113,115,117,122,126,131,136,141,146,151,156,161,166,172,177,182,187,193,198,204,210,215,221,227,232,238,244,250,255};
// another table
// 
// const static uint8_t arrPWM[0x40] = {0,1,2,3,4,5,6,6,6,7,7,8,8,9,9,10,11,12,12,13,14,15,16,17,19,20,21,23,24,26,28,30,32,34,36,39,42,44,48,51,54,58,62,66,71,76,81,87,93,100,106,114,122,130,139, 149,159,170,182,195,208,223,238,255};
uint8_t arrPWM[0x40] = {0,1,2,3,4,5,6,6,6,7,7,8,8,9,9,10,11,12,12,13,14,15,16,17,19,20,21,23,24,26,28,30,32,34,36,39,42,44,48,51,54,58,62,66,71,76,81,87,93,100,106,114,122,130,139, 149,159,170,182,195,208,223,238,255};

	//const static uint8_t arrDigit[10] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0xef, 0x6f };	 
// uint8_t arrDigit[10] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0xff, 0x6f };	 
	 uint8_t arrDigit[10] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };	 
// uint8_t arrDigit[10] = {0xc0, 0xf9, 0x5b, 0xd0, 0x99, 0x92, 0x82, 0xf8, 0x10, 0x90 };	 
	 
 volatile uint8_t arrAnodeValue[4] = {0x3f, 0x3f, 0x3f, 0x3f};
// volatile uint8_t arrAnodeValue[4] = {0x3f, 0x02, 0x3f, 0x3f};
 volatile uint8_t arrSegmentValue[8] = {0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f};
// volatile uint8_t arrSegmentValue[8] = {0x0f, 0x1f, 0x2f, 0x3c, 0x3f, 0x3f, 0x3f, 0x3f};
volatile uint8_t arrSeparatorValue[2] = {0x3f, 0x3f};

volatile uint8_t ind_dp = 0x00;

volatile uint8_t currAnode = 0;
volatile uint8_t currSegment = 0;
volatile uint8_t currNumber	= 0;
volatile uint16_t currMode = MODE_HOUR_MINUTES;
volatile uint8_t WorkMode; //  = WORK_MODE_NORMAL;
//bcd format
uint16_t IndValue = 0x2468;
uint16_t IndValuenext = 0x2469;

uint32_t cntSysTickConfig = 2000;
// Timer 1
volatile uint16_t Anode0_Val = 0;
volatile uint16_t Anode1_Val = 0;
volatile uint16_t Anode2_Val = 0;
volatile uint16_t Anode3_Val = 0;
// Timer 2
volatile uint16_t SegmentA_Val = 0;
volatile uint16_t SegmentB_Val = 0;
volatile uint16_t SegmentC_Val = 0;
volatile uint16_t SegmentD_Val = 0;
// Timer 3
volatile uint16_t SegmentE_Val = 0;
volatile uint16_t SegmentF_Val = 0;
volatile uint16_t SegmentG_Val = 0;
volatile uint16_t SegmentDP_Val = 0;
// Timer 15
// FlagRTC_Irq bit mask
//	bit0  0 - no fade 1 - fade in/out
//	bit1	fade in separator 1
//	bit2	fade out separator 1
//	bit3	fade in separator 2
//	bit4	fade out separator 2
volatile uint8_t 	FlagRTC_Irq = 0x0c;

volatile uint16_t MinSeparator1_Val = 0x00;
volatile uint16_t MaxSeparator1_Val = 0x3f;
volatile uint16_t MinSeparator2_Val = 0x00;
volatile uint16_t MaxSeparator2_Val = 0x3f;
volatile uint16_t Separator1_Val;
volatile uint16_t Separator2_Val;

volatile uint16_t currmode = MODE_HOUR_MINUTES;

uint16_t PrescalerValue = 0;

RTC_t rtc;
RTC_t_BCD rtcBCD;
RTC_t rtcnext;
RTC_t_BCD rtcBCDnext;



GPIO_InitTypeDef   GPIO_InitStructure;
EXTI_InitTypeDef   EXTI_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;
	
TIM_TimeBaseInitTypeDef TIM_BaseConfig;
TIM_OCInitTypeDef TIM_OCConfig;
	
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

void SetCCRAnode(void)
{
  TIM_OCConfig.TIM_Pulse = Anode0_Val;
  TIM_OC1Init(TIM1, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = Anode1_Val;
  TIM_OC2Init(TIM1, &TIM_OCConfig);

  TIM_OCConfig.TIM_Pulse = Anode2_Val;
  TIM_OC3Init(TIM1, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = Anode3_Val;
  TIM_OC4Init(TIM1, &TIM_OCConfig);
}

void SetCCRSegmentTIM2(void)
{
  TIM_OCConfig.TIM_Pulse = SegmentA_Val;
  TIM_OC1Init(TIM2, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = SegmentB_Val;
  TIM_OC2Init(TIM2, &TIM_OCConfig);

  TIM_OCConfig.TIM_Pulse = SegmentC_Val;
  TIM_OC3Init(TIM2, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = SegmentD_Val;
  TIM_OC4Init(TIM2, &TIM_OCConfig);	
	
}

void SetCCRSegmentTIM3(void)
{
  TIM_OCConfig.TIM_Pulse = SegmentE_Val;
  TIM_OC1Init(TIM3, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = SegmentF_Val;
  TIM_OC2Init(TIM3, &TIM_OCConfig);

  TIM_OCConfig.TIM_Pulse = SegmentG_Val;
  TIM_OC3Init(TIM3, &TIM_OCConfig);
	
  TIM_OCConfig.TIM_Pulse = SegmentDP_Val;
  TIM_OC4Init(TIM3, &TIM_OCConfig);	
}

void SetCCRSeparator(void)
{
//	GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction)( 1 - GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_14)));	
//	GPIO_WriteBit(TEST_PORT, TEST_PIN_2, (BitAction)( 1 - GPIO_ReadOutputDataBit(TEST_PORT,TEST_PIN_2)));
  TIM_OCConfig.TIM_Pulse = Separator1_Val;
//  TIM_OCConfig.TIM_Pulse = 0x7f;	
  TIM_OC1Init(TIM15, &TIM_OCConfig);

	TIM_OCConfig.TIM_Pulse = Separator2_Val;
//  TIM_OCConfig.TIM_Pulse = 0x0f;		
  TIM_OC2Init(TIM15, &TIM_OCConfig);
	
}


void SetCCR(uint8_t what)
{
	if ((what & CCR_SEPARATOR) != 0x00) {SetCCRSeparator();}
	if ((what & CCR_SEGMENT_TIM3) != 0x00) {SetCCRSegmentTIM3();}
	if ((what & CCR_SEGMENT_TIM2) != 0x00) {SetCCRSegmentTIM2();}
	if ((what & CCR_ANODE) != 0x00) {SetCCRAnode();}
	
}

void RCC_PeriphClockConfig(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP  | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM6, ENABLE);	
}
void GPIO_Config(void)
{
// 	GPIO_Pin_6 | GPIO_Pin_7 for TEST output

	GPIO_InitStructure.GPIO_Pin = TEST_PIN_1 | TEST_PIN_2 | TEST_PIN_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_Init(TEST_PORT, &GPIO_InitStructure);		
	
// 	GPIO_Pin_2							for Beeper output
	
	GPIO_InitStructure.GPIO_Pin = BEEPER_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_Init(BEEPER_PORT, &GPIO_InitStructure);		

// GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 for button pull-up	
	
	GPIO_InitStructure.GPIO_Pin = BUTTON_UP | BUTTON_DOWN | BUTTON_MODE | BUTTON_ENTER;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
	GPIO_Init(BUTTON_PORT, &GPIO_InitStructure);	
	
// GPIO_Pin_13 for IR input 
	
	GPIO_InitStructure.GPIO_Pin = REMOTE_CONTROL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
	GPIO_Init(REMOTE_CONTROL_PORT, &GPIO_InitStructure);	
	
// GPIO_Pin_12 for ds18b20	
	GPIO_InitStructure.GPIO_Pin = TEMP_SENSOR_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	GPIO_Init(TEMP_SENSOR_PORT, &GPIO_InitStructure);	
	
	
// GPIO_Pin_12 for Motion Detect	
	GPIO_InitStructure.GPIO_Pin = MOTION_DETECT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
	GPIO_Init(MOTION_DETECT_PORT, &GPIO_InitStructure);	
	
// GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 		for TIM2 (seg ABCD)	
// GPIO_Pin_6 | GPIO_Pin_7															for TIM3	(seg EF)
// GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11	for TIM1	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);	


// GPIO_Pin_0 | GPIO_Pin_1	for TIM3	(seg GDp)

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
// GPIO_Pin_14 | GPIO_Pin_15	for TIM15	(separatorr1 & 2)	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
}


void EXTI_Config(void)
{
// settings Motion Detect	
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);

// settings IR Sensor	
	  GPIO_EXTILineConfig(REMOTE_CONTROL_PORT_SOURCE, REMOTE_CONTROL_PIN_SOURCE);

// for buttons
	GPIO_EXTILineConfig(BUTTON_PORT_SOURCE, BUTTON_UP_SOURCE);
	GPIO_EXTILineConfig(BUTTON_PORT_SOURCE, BUTTON_DOWN_SOURCE);
	GPIO_EXTILineConfig(BUTTON_PORT_SOURCE, BUTTON_MODE_SOURCE);
	GPIO_EXTILineConfig(BUTTON_PORT_SOURCE, BUTTON_ENTER_SOURCE);
	
    EXTI_DeInit();
    EXTI_InitStructure.EXTI_Line = EXTI_Line12 | REMOTE_CONTROL_PIN_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
	
    EXTI_InitStructure.EXTI_Line = BUTTON_UP_EXTI_LINE | BUTTON_DOWN_EXTI_LINE | BUTTON_MODE_EXTI_LINE | BUTTON_ENTER_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn , 3);	

		NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn , 4);	
	
}

void TIM1_Config(void)
{
  /* -----------------------------------------------------------------------
    TIM1 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM1CLK frequency is set to SystemCoreClock (Hz), to get TIM1 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM1CLK / TIM1 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM1 is running at 36 KHz: TIM1 Frequency = TIM1 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100 = 50%
    TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100 = 37.5%
    TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR)* 100 = 25%
    TIM1 Channel4 duty cycle = (TIM1_CCR4/ TIM1_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */
  /* Compute the prescaler value */
//  PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	PrescalerValue = TIMER_PRESCALER - 1;

  /* Time base configuration */
  TIM_BaseConfig.TIM_Period = TIMER_PERIOD;
  TIM_BaseConfig.TIM_Prescaler = PrescalerValue;
  TIM_BaseConfig.TIM_ClockDivision = 0;
  TIM_BaseConfig.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM1, &TIM_BaseConfig);	
	
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCConfig.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
//TIM1->BDTR = TIM_BDTR_MOE;	
  TIM_OCConfig.TIM_Pulse =  Anode0_Val;
  TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM1, &TIM_OCConfig);

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = Anode1_Val;

  TIM_OC2Init(TIM1, &TIM_OCConfig);

  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = Anode2_Val;

  TIM_OC3Init(TIM1, &TIM_OCConfig);

  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = Anode3_Val;

  TIM_OC4Init(TIM1, &TIM_OCConfig);

  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);	

	TIM_CtrlPWMOutputs(TIM1, ENABLE);	

  /* Automatic Output enable, Break, dead time and lock configuration*/
/*	
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  TIM_BDTRInitStructure.TIM_DeadTime = 5;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
*/

	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  /* TIM1 enable counter */
//  TIM_Cmd(TIM1, ENABLE);	
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
// Enable the RTC Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);		
	
}


void TIM2_Config(void)
{
  /* -----------------------------------------------------------------------
    TIM2 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM2CLK frequency is set to SystemCoreClock (Hz), to get TIM2 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM2CLK / TIM2 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM2 is running at 36 KHz: TIM2 Frequency = TIM2 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM2 Channel1 duty cycle = (TIM2_CCR1/ TIM2_ARR)* 100 = 50%
    TIM2 Channel2 duty cycle = (TIM2_CCR2/ TIM2_ARR)* 100 = 37.5%
    TIM2 Channel3 duty cycle = (TIM2_CCR3/ TIM2_ARR)* 100 = 25%
    TIM2 Channel4 duty cycle = (TIM2_CCR4/ TIM2_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */
  /* Compute the prescaler value */
//  PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	PrescalerValue = TIMER_PRESCALER - 1;

  /* Time base configuration */
  TIM_BaseConfig.TIM_Period = TIMER_PERIOD;
  TIM_BaseConfig.TIM_Prescaler = PrescalerValue;
  TIM_BaseConfig.TIM_ClockDivision = 0;
  TIM_BaseConfig.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_BaseConfig);	
	
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCConfig.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
// TIM2->BDTR = TIM_BDTR_MOE;	
  TIM_OCConfig.TIM_Pulse = SegmentA_Val;
  TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCConfig);

  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = SegmentB_Val;

  TIM_OC2Init(TIM2, &TIM_OCConfig);

  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = SegmentC_Val;

  TIM_OC3Init(TIM2, &TIM_OCConfig);

  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = SegmentD_Val;

  TIM_OC4Init(TIM2, &TIM_OCConfig);

  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM2, ENABLE);	

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
  /* TIM2 enable counter */
//  TIM_Cmd(TIM2, ENABLE);	
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
// Enable the RTC Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);			
	
}

void TIM3_Config(void)
{
  /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */
  /* Compute the prescaler value */
//  PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	PrescalerValue = TIMER_PRESCALER - 1;

  /* Time base configuration */
  TIM_BaseConfig.TIM_Period = TIMER_PERIOD;
  TIM_BaseConfig.TIM_Prescaler = PrescalerValue;
  TIM_BaseConfig.TIM_ClockDivision = 0;
  TIM_BaseConfig.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_BaseConfig);	
	
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCConfig.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = SegmentE_Val;
  TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCConfig);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
// TIM3->BDTR = TIM_BDTR_MOE;	
  TIM_OCConfig.TIM_Pulse = SegmentF_Val;

  TIM_OC2Init(TIM3, &TIM_OCConfig);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = SegmentG_Val;

  TIM_OC3Init(TIM3, &TIM_OCConfig);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = SegmentDP_Val;

  TIM_OC4Init(TIM3, &TIM_OCConfig);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);	

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

  /* TIM3 enable counter */
//  TIM_Cmd(TIM3, ENABLE);	
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
// Enable the RTC Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);		
	
}


void TIM15_Config(void)
{
  /* -----------------------------------------------------------------------
    TIM15 Configuration: generate 2 PWM signals with 2 different duty cycles:
    The TIM15CLK frequency is set to SystemCoreClock (Hz), to get TIM15 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM15CLK / TIM15 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM15 is running at 36 KHz: TIM15 Frequency = TIM15 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM15 Channel1 duty cycle = (TIM15_CCR1/ TIM15_ARR)* 100 = 50%
    TIM15 Channel2 duty cycle = (TIM15_CCR2/ TIM15_ARR)* 100 = 37.5%
  ----------------------------------------------------------------------- */
  /* Compute the prescaler value ??????????????? */
  //  PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15 , ENABLE);	
	GPIO_PinRemapConfig(GPIO_Remap_TIM15,ENABLE);
//	PrescalerValue = TIMER_PRESCALER - 1;

  /* Time base configuration ????????????? */
  TIM_BaseConfig.TIM_Period = 0xFF;
//  TIM_BaseConfig.TIM_Prescaler = 367 - 1;
  TIM_BaseConfig.TIM_Prescaler = 183 - 1;
  TIM_BaseConfig.TIM_ClockDivision = 0;	

  TIM_BaseConfig.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM15, &TIM_BaseConfig);	
	
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCConfig.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
	TIM15->BDTR = TIM_BDTR_MOE;
  TIM_OCConfig.TIM_Pulse = Separator1_Val;
  TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM15, &TIM_OCConfig);

  TIM_OC1PreloadConfig(TIM15, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = Separator2_Val;

  TIM_OC2Init(TIM15, &TIM_OCConfig);

  TIM_OC2PreloadConfig(TIM15, TIM_OCPreload_Enable);


  TIM_ARRPreloadConfig(TIM15, ENABLE);	

  /* TIM15 enable counter */
	
	TIM_ITConfig(TIM15, TIM_IT_Update, ENABLE);
	
  TIM_Cmd(TIM15, ENABLE);	
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
// Enable the RTC Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM15_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);		
	
}

void TIM6_Config(void)
{
  /* -----------------------------------------------------------------------
    TIM6 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM6CLK frequency is set to SystemCoreClock (Hz), to get TIM6 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM6CLK / TIM6 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM6 is running at 36 KHz: TIM6 Frequency = TIM6 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM6 Channel1 duty cycle = (TIM6_CCR1/ TIM6_ARR)* 100 = 50%
    TIM6 Channel2 duty cycle = (TIM6_CCR2/ TIM6_ARR)* 100 = 37.5%
  ----------------------------------------------------------------------- */
  /* Compute the prescaler value */
//  PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	PrescalerValue = 200 - 1;

  /* Time base configuration */
  TIM_BaseConfig.TIM_Period = 1000;
  TIM_BaseConfig.TIM_Prescaler = PrescalerValue;
  TIM_BaseConfig.TIM_ClockDivision = 0;
  TIM_BaseConfig.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM6, &TIM_BaseConfig);	
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
  /* TIM6 enable counter */
/////  TIM_Cmd(TIM6, ENABLE);	
	
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
// Enable the RTC Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
}

void SysTickConfig(void)
{
	SysTick_Config(SystemCoreClock / cntSysTickConfig);
//	SysTick_Config(SystemCoreClock / 10);
}

void RTC_Config(void)
{
	PWR_BackupAccessCmd(ENABLE);
	
  if(BKP_ReadBackupRegister(BKP_DR1) != 0x5a5a)
  {
    /*Enables the clock to Backup and power interface peripherals    */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP | RCC_APB1Periph_PWR,ENABLE);
		BKP_DeInit();
    /*Enable 32.768 kHz external oscillator */
    RCC_LSEConfig(RCC_LSE_ON);
  
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {} // Wait till LSE is ready

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    /* RTC Enabled */
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForLastTask();
    /*Wait for RTC registers synchronisation */
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
		RTC_SetPrescaler(32767); // Set RTC prescaler: set RTC period to 1sec. RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) 
		RTC_WaitForLastTask();   // Wait until last write operation on RTC registers has finished 
			
    BKP_WriteBackupRegister(BKP_DR1, 0x5a5a);
			
  }
  else
  {
    /* PWR and BKP clocks selection */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
//    for(WaitForOscSource=0;WaitForOscSource<5000;WaitForOscSource++);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
  }
	

//////// Old RTC Init	
/*	
  BKP_DeInit(); // Reset Backup Domain

	RCC_LSEConfig(RCC_LSE_ON);   // Enable LSE
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {} // Wait till LSE is ready
	
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // Select LSE as RTC Clock Source 
  RCC_RTCCLKCmd(ENABLE);   // Enable RTC Clock
  RTC_WaitForSynchro();   // Wait for RTC registers synchronization
  RTC_WaitForLastTask();   // Wait until last write operation on RTC registers has finished 
  RTC_SetPrescaler(32767); // Set RTC prescaler: set RTC period to 1sec. RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) 
  RTC_WaitForLastTask();   // Wait until last write operation on RTC registers has finished 
*/
//////// End Old RTC Init
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
// Enable the RTC Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
	RTC_ITConfig(RTC_IT_SEC, ENABLE); // Enable the RTC Second
	RTC_WaitForLastTask();
	RTC_ITConfig(RTC_IT_ALR, ENABLE);
  RTC_WaitForLastTask();	  // Wait until last write operation on RTC registers has finished 		
}




void StartTimers(void)
{
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM15, ENABLE);		
}

void defAnodeValue(void)
{
	Anode0_Val = 0x00;Anode1_Val = 0x00;Anode2_Val = 0x00;Anode3_Val = 0x00;	
	
	switch (currAnode)
	{
		case 0 :	{Anode0_Val = arrPWM[arrAnodeValue[0]];break;}
		case 1 :	{Anode1_Val = arrPWM[arrAnodeValue[1]];break;}
		case 2 :	{Anode2_Val = arrPWM[arrAnodeValue[2]];break;}
		case 3 :	{Anode3_Val = arrPWM[arrAnodeValue[3]];break;}
	}
//		GPIO_WriteBit(TEST_PORT, TEST_PIN_1, (BitAction)( 1 - GPIO_ReadOutputDataBit(TEST_PORT,TEST_PIN_1)));	
	SetCCR(CCR_ANODE);
}


void defSegmentTIM2Value(void)
{
	uint8_t indsegment;
	switch (currAnode)
	{
		case 0 :
		{
			indsegment = arrDigit[(uint8_t)(IndValue & 0x000f)];
			if ((indsegment & 0x01) != 0x00) {SegmentA_Val = arrPWM[arrSegmentValue[0]];} else {SegmentA_Val = 0;}
			if ((indsegment & 0x02) != 0x00) {SegmentB_Val = arrPWM[arrSegmentValue[1]];} else {SegmentB_Val = 0;}
			if ((indsegment & 0x04) != 0x00) {SegmentC_Val = arrPWM[arrSegmentValue[2]];} else {SegmentC_Val = 0;}
			if ((indsegment & 0x08) != 0x00) {SegmentD_Val = arrPWM[arrSegmentValue[3]];} else {SegmentD_Val = 0;}
			break;
		}
		case 1 :
		{
			indsegment = arrDigit[(uint8_t)((IndValue >> 4 )& 0x000f)];
			if ((indsegment & 0x01) != 0x00) {SegmentA_Val = arrPWM[arrSegmentValue[0]];} else {SegmentA_Val = 0;}
			if ((indsegment & 0x02) != 0x00) {SegmentB_Val = arrPWM[arrSegmentValue[1]];} else {SegmentB_Val = 0;}
			if ((indsegment & 0x04) != 0x00) {SegmentC_Val = arrPWM[arrSegmentValue[2]];} else {SegmentC_Val = 0;}
			if ((indsegment & 0x08) != 0x00) {SegmentD_Val = arrPWM[arrSegmentValue[3]];} else {SegmentD_Val = 0;}
			break;
		}
		case 2 :
		{
			indsegment = arrDigit[(uint8_t)((IndValue >> 8 )& 0x000f)];
			if ((indsegment & 0x01) != 0x00) {SegmentA_Val = arrPWM[arrSegmentValue[0]];} else {SegmentA_Val = 0;}
			if ((indsegment & 0x02) != 0x00) {SegmentB_Val = arrPWM[arrSegmentValue[1]];} else {SegmentB_Val = 0;}
			if ((indsegment & 0x04) != 0x00) {SegmentC_Val = arrPWM[arrSegmentValue[2]];} else {SegmentC_Val = 0;}
			if ((indsegment & 0x08) != 0x00) {SegmentD_Val = arrPWM[arrSegmentValue[3]];} else {SegmentD_Val = 0;}
			break;
		}
		case 3 :
		{
			indsegment = arrDigit[(uint8_t)((IndValue >> 12 )& 0x000f)];
			if ((indsegment & 0x01) != 0x00) {SegmentA_Val = arrPWM[arrSegmentValue[0]];} else {SegmentA_Val = 0;}
			if ((indsegment & 0x02) != 0x00) {SegmentB_Val = arrPWM[arrSegmentValue[1]];} else {SegmentB_Val = 0;}
			if ((indsegment & 0x04) != 0x00) {SegmentC_Val = arrPWM[arrSegmentValue[2]];} else {SegmentC_Val = 0;}
			if ((indsegment & 0x08) != 0x00) {SegmentD_Val = arrPWM[arrSegmentValue[3]];} else {SegmentD_Val = 0;}
			break;
		}
		
	}
	SetCCR(CCR_SEGMENT_TIM2);
}

void defSegmentTIM3Value(uint8_t ind_dp)
{
	uint8_t indsegment;
	switch (currAnode)
	{
		case 0 :
		{
			indsegment = arrDigit[(uint8_t)(IndValue & 0x000f)];
			if ((indsegment & 0x10) != 0x00) {SegmentE_Val = arrPWM[arrSegmentValue[4]];} else {SegmentE_Val = 0;}
			if ((indsegment & 0x20) != 0x00) {SegmentF_Val = arrPWM[arrSegmentValue[5]];} else {SegmentF_Val = 0;}
			if ((indsegment & 0x40) != 0x00) {SegmentG_Val = arrPWM[arrSegmentValue[6]];} else {SegmentG_Val = 0;}
			if (ind_dp != 0x00) {SegmentDP_Val = arrPWM[arrSegmentValue[7]];} else {SegmentDP_Val = 0;}
			break;
		}
		case 1 :
		{
			indsegment = arrDigit[(uint8_t)((IndValue >> 4 )& 0x000f)];
			if ((indsegment & 0x10) != 0x00) {SegmentE_Val = arrPWM[arrSegmentValue[4]];} else {SegmentE_Val = 0;}
			if ((indsegment & 0x20) != 0x00) {SegmentF_Val = arrPWM[arrSegmentValue[5]];} else {SegmentF_Val = 0;}
			if ((indsegment & 0x40) != 0x00) {SegmentG_Val = arrPWM[arrSegmentValue[6]];} else {SegmentG_Val = 0;}
			if (ind_dp != 0x00) {SegmentDP_Val = arrPWM[arrSegmentValue[7]];} else {SegmentDP_Val = 0;}
			break;
		}
		case 2 :
		{
			indsegment = arrDigit[(uint8_t)((IndValue >> 8 )& 0x000f)];
			if ((indsegment & 0x10) != 0x00) {SegmentE_Val = arrPWM[arrSegmentValue[4]];} else {SegmentE_Val = 0;}
			if ((indsegment & 0x20) != 0x00) {SegmentF_Val = arrPWM[arrSegmentValue[5]];} else {SegmentF_Val = 0;}
			if ((indsegment & 0x40) != 0x00) {SegmentG_Val = arrPWM[arrSegmentValue[6]];} else {SegmentG_Val = 0;}
			if (ind_dp != 0x00) {SegmentDP_Val = arrPWM[arrSegmentValue[7]];} else {SegmentDP_Val = 0;}
			break;
		}
		case 3 :
		{
			indsegment = arrDigit[(uint8_t)((IndValue >> 12 )& 0x000f)];
			if ((indsegment & 0x10) != 0x00) {SegmentE_Val = arrPWM[arrSegmentValue[4]];} else {SegmentE_Val = 0;}
			if ((indsegment & 0x20) != 0x00) {SegmentF_Val = arrPWM[arrSegmentValue[5]];} else {SegmentF_Val = 0;}
			if ((indsegment & 0x40) != 0x00) {SegmentG_Val = arrPWM[arrSegmentValue[6]];} else {SegmentG_Val = 0;}
			if (ind_dp != 0x00) {SegmentDP_Val = arrPWM[arrSegmentValue[7]];} else {SegmentDP_Val = 0;}
			break;
		}		
	}	
	SetCCR(CCR_SEGMENT_TIM3);
}

void defSegmentValue(uint8_t ind_dp)
{
	defSegmentTIM2Value();
	defSegmentTIM3Value(ind_dp);	
}



void defIndValuenext(void)
{
	
}

void defIndValue(void)
{
	
	rtc_gettime(&rtc);
	rtc_gettimeBCD(&rtc, &rtcBCD);
	defIndValuenext();

	
	switch (currmode)
	{
		case MODE_HOUR_MINUTES :
		{
			IndValue  = rtcBCD.hour << 8;
			IndValue = IndValue | rtcBCD.min;	
			break;
		}
		case MODE_MINUTES_SECOND :
		{
			IndValue  = rtcBCD.min << 8;
			IndValue = IndValue | rtcBCD.sec;
			break;
		}
		case MODE_DAY_MONTH :
		{
			
			break;
		}
		case MODE_YEAR :
		{
			
			break;
		}
		case MODE_ALARM1 :
		{
			
			break;
		}
		case MODE_ALARM2 :
		{
			
			break;
		}
		case MODE_TEMPERATURE1 :
		{
			
			break;
		}
		case MODE_TEMPERATURE2 :
		{
			
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
		default :
		{
			
			break;
		}
	}	
	
	
	
}

int main(void)
{
	arrSeparatorValue[0] = MinSeparator1_Val;
	arrSeparatorValue[1] = MinSeparator2_Val;
	Separator1_Val = arrPWM[arrSeparatorValue[0]];
	Separator2_Val = arrPWM[arrSeparatorValue[1]];
	RCC_PeriphClockConfig();
	GPIO_Config();
//	GPIO_SetBits(GPIOB, TEST_PIN_3);
//	GPIO_ResetBits(GPIOB, TEST_PIN_1);
	
	EXTI_Config();	
	TIM1_Config();
	TIM2_Config();
	TIM3_Config();
	TIM15_Config();
	TIM6_Config();

	SysTickConfig();

// GPIO_SetBits(GPIOB, GPIO_Pin_14 );		
//	GPIO_SetBits(GPIOB, TEST_PIN_1 | TEST_PIN_3);	
//	GPIO_ResetBits(GPIOB, TEST_PIN_2);

  // Allow access to BKP Domain 
	
////  if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
////  {
		
    RTC_Config();
////    BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
////  }
////  else
////  {
////    RTC_WaitForSynchro();
////    RTC_ITConfig(RTC_IT_SEC, ENABLE);
////    RTC_WaitForLastTask();
////  }
	
	StartTimers();
	currAnode = 0;
	defAnodeValue();
	defSegmentValue(ind_dp);
//	SetCCR(CCR_ANODE | CCR_SEGMENT_TIM2 | CCR_SEGMENT_TIM3 | CCR_SEPARATOR);	
	
//	GPIO_SetBits(TEST_PORT, TEST_PIN_1 | TEST_PIN_2 | TEST_PIN_3);	
	
GPIO_ResetBits(TEST_PORT, TEST_PIN_1);
GPIO_ResetBits(TEST_PORT, TEST_PIN_2);
	
	while (1)
	{
		if (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_UP))
		{
//			GPIO_SetBits(TEST_PORT, TEST_PIN_1);	
		}
		else
		{
//			GPIO_ResetBits(TEST_PORT, TEST_PIN_1);	
		}
		
		if (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_DOWN))
		{
//			GPIO_SetBits(TEST_PORT, TEST_PIN_2);	
		}
		else
		{
//			GPIO_ResetBits(TEST_PORT, TEST_PIN_2);	
		}

		if (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_ENTER))
		{
//			GPIO_SetBits(TEST_PORT, TEST_PIN_3);	
		}
		else
		{
//			GPIO_ResetBits(TEST_PORT, TEST_PIN_3);	
		}
		if (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_MODE))
		{
//			GPIO_SetBits(TEST_PORT, TEST_PIN_1);	
		}
		else
		{
//			GPIO_ResetBits(TEST_PORT, TEST_PIN_1);	
		}		
		
	}
}	
