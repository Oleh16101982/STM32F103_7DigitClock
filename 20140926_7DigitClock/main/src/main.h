#define BKP_DR_NUMBER 10   

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

#define SYSTICK_CNT_MAX 0xFF

#define DP_IND 0x01
#define DP_NO_IND 0x00

#define CCR_ANODE 0x01
#define CCR_SEGMENT_TIM2 0x02
#define CCR_SEGMENT_TIM3 0x04
#define CCR_SEPARATOR 0x08

// define Modes for indicator
#define MODE_HOUR_MINUTES 0x01
#define MODE_MINUTES_SECOND 0x02
#define MODE_DAY_MONTH 0x03
#define MODE_YEAR 0x04
#define MODE_ALARM1 0x05
#define MODE_ALARM2 0x06
#define MODE_TEMPERATURE1 0x07
#define MODE_TEMPERATURE2 0x08
#define MODE_MIX1 0x09 // hour_minutes - day_month - temperature1 - temperature2
#define MODE_MIX2 0x0A // hour_minutes - temperature1 - temperature2

#define SET_SEPARATOR_OFF 0x00
#define SET_SEPARATOR_ON 0x01

#define MODE_SEPARATOR_BLINK_ON 					0x00 // blink 1 - on 2 on
#define MODE_SEPARATOR_BLINK_OFF 					0x01 // blink 1 - on 2 off
#define MODE_SEPARATOR_NO_BLINK_ON 				0x02 // no blink 1 - on 2 off
#define MODE_SEPARATOR_NO_BLINK_OFF 			0x03 // no blink 1 - off 2 on
#define MODE_SEPARATOR_NO_BLINK_ALL_ON 			0x04 // no blink 1 - on 2 on
#define MODE_SEPARATOR_NO_BLINK_ALL_OFF 	0x05 // no blink 1 - off 2 off



// define modes normal, setup, etc.
#define WORK_MODE_NORMAL 0x00
#define WORK_MODE_SETUP 0x80

#define CONFIGURATION_RESET 0x5A5A

void SysTickConfig(void);

