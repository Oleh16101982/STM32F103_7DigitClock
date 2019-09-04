
#include "stm32f10x_conf.h"
#include "main.h"
#include "show.h"

// Backup registers
// DR1	- flag of CONFIGURATION_RESET (0x5A5A)
// DR2	- bcd alarm 1
// DR3	- bcd alarm 2
// DR4	-
// DR5	-	
// DR6	-
// DR7	-
// DR8	-
// DR9	-
// DR10	-
uint16_t BKPDataReg[BKP_DR_NUMBER] =  {BKP_DR1, BKP_DR2, BKP_DR3, BKP_DR4, BKP_DR5, BKP_DR6, BKP_DR7, BKP_DR8, BKP_DR9, BKP_DR10};

uint16_t PrescalerValue;
uint32_t cntSysTickConfig = 1000;
volatile uint8_t currSysTick = 0x00;

volatile uint16_t bcd_hour_minutes;
volatile uint16_t bcd_minutes_second;
volatile uint16_t bcd_day_month;
volatile uint16_t bcd_year;
volatile uint16_t bcd_alarm[2];
volatile uint16_t bcd_temperature[2];

volatile uint8_t WorkMode;
volatile uint8_t SeparatorMode = MODE_SEPARATOR_BLINK_ON;

GPIO_InitTypeDef   GPIO_InitStructure;
EXTI_InitTypeDef   EXTI_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;
	
TIM_TimeBaseInitTypeDef TIM_BaseConfig;
TIM_OCInitTypeDef TIM_OCConfig;
	
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

void RCC_PeriphClockConfig(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP  | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM6, ENABLE);	
}

void GPIO_Config(void)
{
// 	GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 for TEST output

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
  TIM_OCConfig.TIM_Pulse =  0x00; // Anode0_Val;
  TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM1, &TIM_OCConfig);

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = 0x00; // Anode1_Val;

  TIM_OC2Init(TIM1, &TIM_OCConfig);

  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = 0x00; // Anode2_Val;

  TIM_OC3Init(TIM1, &TIM_OCConfig);

  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = 0x00; // Anode3_Val;

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
  TIM_OCConfig.TIM_Pulse = 0x00; // SegmentA_Val;
  TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCConfig);

  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = 0x00; // SegmentB_Val;

  TIM_OC2Init(TIM2, &TIM_OCConfig);

  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = 0x00; // SegmentC_Val;

  TIM_OC3Init(TIM2, &TIM_OCConfig);

  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = 0x00; // SegmentD_Val;

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
  TIM_OCConfig.TIM_Pulse = 0x00; // SegmentE_Val;
  TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCConfig);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
// TIM3->BDTR = TIM_BDTR_MOE;	
  TIM_OCConfig.TIM_Pulse = 0x00; // SegmentF_Val;

  TIM_OC2Init(TIM3, &TIM_OCConfig);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = 0x00; // SegmentG_Val;

  TIM_OC3Init(TIM3, &TIM_OCConfig);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = 0x00; // SegmentDP_Val;

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
  TIM_OCConfig.TIM_Pulse = 0x00; // Separator1_Val;
  TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM15, &TIM_OCConfig);

  TIM_OC1PreloadConfig(TIM15, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCConfig.TIM_Pulse = 0x00; // Separator2_Val;

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
	
  if(BKP_ReadBackupRegister(BKP_DR1) != CONFIGURATION_RESET)
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
		WorkMode = WORK_MODE_SETUP & MODE_HOUR_MINUTES;	
  }
  else
  {
    /* Check if the Power On Reset flag is set */
    if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
    {      
    }
    /* Check if the Pin Reset flag is set */
    else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
    {      
    }
    
    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
		WorkMode = MODE_HOUR_MINUTES;
  }
	RTC_ITConfig(RTC_IT_SEC, ENABLE); // Enable the RTC Second
	RTC_WaitForLastTask();
	RTC_ITConfig(RTC_IT_ALR, ENABLE);
  RTC_WaitForLastTask();	  // Wait until last write operation on RTC registers has finished 	

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
// Enable the RTC Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
	
}

void StartTimers(void)
{
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM15, ENABLE);		
}

int main(void)
{
////	arrSeparatorValue[0] = MinSeparator1_Val;
////	arrSeparatorValue[1] = MinSeparator2_Val;
////	Separator1_Val = arrPWM[arrSeparatorValue[0]];
////	Separator2_Val = arrPWM[arrSeparatorValue[1]];
	ChangeSeparatorMode();
	RCC_PeriphClockConfig();
	GPIO_Config();
	EXTI_Config();	
	PrescalerValue = TIMER_PRESCALER - 1;
	TIM1_Config();
	TIM2_Config();
	TIM3_Config();
	TIM15_Config();
	TIM6_Config();	
  RTC_Config();
	StartTimers();
	SysTickConfig();
	
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
