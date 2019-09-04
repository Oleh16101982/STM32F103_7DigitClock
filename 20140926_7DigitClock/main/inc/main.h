#include "stm32f10x_conf.h"

#define TEST_PORT			GPIOB
#define TEST_PIN_1		GPIO_Pin_5
#define TEST_PIN_2		GPIO_Pin_6
#define TEST_PIN_3		GPIO_Pin_7

#define BEEPER_PORT		GPIOB
#define BEEPER_PIN		GPIO_Pin_2

#define BUTTON_PORT		GPIOB
#define BUTTON_UP			GPIO_Pin_8
#define BUTTON_DOWN		GPIO_Pin_9
#define BUTTON_MODE		GPIO_Pin_10
#define BUTTON_ENTER	GPIO_Pin_11

#define BUTTON_PORT_SOURCE		GPIO_PortSourceGPIOB
#define BUTTON_UP_SOURCE			GPIO_PinSource8
#define BUTTON_DOWN_SOURCE		GPIO_PinSource9
#define BUTTON_MODE_SOURCE		GPIO_PinSource10
#define BUTTON_ENTER_SOURCE		GPIO_PinSource11

#define BUTTON_UP_EXTI_LINE			EXTI_Line8
#define BUTTON_DOWN_EXTI_LINE		EXTI_Line9
#define BUTTON_MODE_EXTI_LINE		EXTI_Line10
#define BUTTON_ENTER_EXTI_LINE	EXTI_Line11


#define REMOTE_CONTROL_PORT	GPIOB
#define REMOTE_CONTROL_PIN	GPIO_Pin_13

#define REMOTE_CONTROL_PORT_SOURCE		GPIO_PortSourceGPIOB
#define REMOTE_CONTROL_PIN_SOURCE			GPIO_PinSource13
#define REMOTE_CONTROL_PIN_EXTI_LINE	EXTI_Line13


#define TEMP_SENSOR_PORT	GPIOB
#define TEMP_SENSOR_PIN		GPIO_Pin_12


#define MOTION_DETECT_PORT	GPIOA
#define MOTION_DETECT_PIN		GPIO_Pin_12

#define TIMER_PRESCALER 235
// #define TIMER_PRESCALER 1000
#define TIMER_PERIOD 0xFF

#define DP_IND 0x01
#define DP_NO_IND 0x00

#define CCR_ANODE 0x01
#define CCR_SEGMENT_TIM2 0x02
#define CCR_SEGMENT_TIM3 0x04
#define CCR_SEPARATOR 0x08

// define Modes for indicator
#define MODE_HOUR_MINUTES 0x0001
#define MODE_MINUTES_SECOND 0x0002
#define MODE_DAY_MONTH 0x0003
#define MODE_YEAR 0x0004
#define MODE_ALARM1 0x0005
#define MODE_ALARM2 0x0006
#define MODE_TEMPERATURE1 0x0007
#define MODE_TEMPERATURE2 0x0008
#define MODE_MIX1 0x00009 // hour_minutes - day_month - temperature1 - temperature2
#define MODE_MIX2 0x0000A // hour_minutes - temperature1 - temperature2

// define modes normal, setup, etc.
#define WORK_MODE_NORMAL 0x00
#define WORK_MODE_SETUP 0x01

#define CONFIGURATION_RESET = 0x5a


void SetCCR(uint8_t what);
void defAnodeValue(void);
void defSegmentTIM2Value(void);
void defSegmentTIM3Value(uint8_t ind_dp);
void defIndValue(void);

