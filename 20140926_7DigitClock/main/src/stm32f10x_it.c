/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f10x.h"
#include "show.h"
#include "RtcBcdValue.h"

#include "stm32f10x_it.h"
// #include "stm32f10x_usart.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define PRESSED_BUTTON_NONE		0x00
#define PRESSED_BUTTON_UP			0x01
#define PRESSED_BUTTON_DOWN		0x02
#define PRESSED_BUTTON_MODE		0x04
#define PRESSED_BUTTON_ENTER	0x08

#define LONG_PRESSED_BUTTON_UP		0x10
#define LONG_PRESSED_BUTTON_DOWN	0x20
#define LONG_PRESSED_BUTTON_MODE	0x40
#define LONG_PRESSED_BUTTON_ENTER	0x80


extern uint8_t WorkMode;
extern uint8_t currAnode;
extern uint8_t currSysTick;
uint8_t pressed_buttons = 0x00;
uint32_t prevRTCcounter = 0x00;



/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
//	GPIO_WriteBit(TEST_PORT, TEST_PIN_1, (BitAction)( 1 - GPIO_ReadOutputDataBit(TEST_PORT,TEST_PIN_1)));
	defSetSeparatorValue();
	defSysTickSegmentValue();
	++currSysTick;
	if (currSysTick > SYSTICK_CNT_MAX) {currSysTick = 0x00;}
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/


void EXTI0_IRQHandler(void)
{

}

void EXTI9_5_IRQHandler(void)
{
	
// BUTTON_UP			GPIO_Pin_8
// BUTTON_DOWN		GPIO_Pin_9
	
	if(EXTI_GetITStatus(BUTTON_UP_EXTI_LINE) != RESET)
	{
		if (pressed_buttons == PRESSED_BUTTON_NONE || pressed_buttons == PRESSED_BUTTON_UP)
		{
			if (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_UP))
			{
				pressed_buttons |= PRESSED_BUTTON_UP;
				GPIO_SetBits(TEST_PORT, TEST_PIN_1);	
			}
			else
			{
				pressed_buttons &= ~PRESSED_BUTTON_UP;
				GPIO_ResetBits(TEST_PORT, TEST_PIN_1);	
			}		
		}
		
		EXTI_ClearITPendingBit(BUTTON_UP_EXTI_LINE);
	}
	
	if(EXTI_GetITStatus(BUTTON_DOWN_EXTI_LINE) != RESET)
	{
		if (pressed_buttons == PRESSED_BUTTON_NONE || pressed_buttons == PRESSED_BUTTON_DOWN)
		{
			if (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_DOWN))
			{
				pressed_buttons |= PRESSED_BUTTON_DOWN;
				GPIO_SetBits(TEST_PORT, TEST_PIN_2);	
			}
			else
			{
				pressed_buttons &= ~PRESSED_BUTTON_DOWN;
				GPIO_ResetBits(TEST_PORT, TEST_PIN_2);	
			}		
		}
		EXTI_ClearITPendingBit(BUTTON_DOWN_EXTI_LINE);
	}

}

void EXTI15_10_IRQHandler(void)
{
// BUTTON_MODE		GPIO_Pin_10
// BUTTON_ENTER	GPIO_Pin_11	
	
	
	if(EXTI_GetITStatus(BUTTON_MODE_EXTI_LINE) != RESET)
	{
		if (pressed_buttons == PRESSED_BUTTON_NONE || pressed_buttons == PRESSED_BUTTON_MODE)
		{
			if (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_MODE))
			{
				pressed_buttons |= PRESSED_BUTTON_MODE;
				prevRTCcounter = RTC_GetCounter();
			}		
			else
			{
				pressed_buttons &= ~PRESSED_BUTTON_MODE;
				if ((RTC_GetCounter() - prevRTCcounter) > 2)
				{

				}
				else
				{
///					++mode;
///					if (mode == MODE_DAY_MONTH) {mode = MODE_HOUR_MINUTES;}
				}
			}
		}
		EXTI_ClearITPendingBit(BUTTON_MODE_EXTI_LINE);
	}
	
	if(EXTI_GetITStatus(BUTTON_ENTER_EXTI_LINE) != RESET)
	{
		if (pressed_buttons == PRESSED_BUTTON_NONE || pressed_buttons == PRESSED_BUTTON_ENTER)
		{
			if (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_ENTER))
			{
				pressed_buttons |= PRESSED_BUTTON_ENTER;
			}		
			else
			{
				pressed_buttons &= ~PRESSED_BUTTON_ENTER;
			}
		}		
		EXTI_ClearITPendingBit(BUTTON_ENTER_EXTI_LINE);
	}	
	
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
	
	if(EXTI_GetITStatus(REMOTE_CONTROL_PIN_EXTI_LINE) != RESET)
	{
		
		EXTI_ClearITPendingBit(REMOTE_CONTROL_PIN_EXTI_LINE);
	}

}

void USART2_IRQHandler(void)
{
		
}


void RTC_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_SEC) != RESET)
  {	
		DefineRtcBcdValue();
		def7DigitSegmentValueCurr();
		DefineSeparartorValue();
		SysTickConfig();
    RTC_ClearITPendingBit(RTC_IT_SEC);
		RTC_WaitForLastTask();
  }
  if(RTC_GetITStatus(RTC_IT_ALR) != RESET)
  {
	
//		GPIO_WriteBit(TEST_PORT, TEST_PIN_2, (BitAction)( 1 - GPIO_ReadOutputDataBit(TEST_PORT,TEST_PIN_2)));
    RTC_ClearITPendingBit(RTC_IT_ALR);
		RTC_WaitForLastTask();
  }	
}

void TIM1_UP_TIM16_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
// GPIO_WriteBit(TEST_PORT, TEST_PIN_3, (BitAction)( 1 - GPIO_ReadOutputDataBit(TEST_PORT,TEST_PIN_3)));		
		defAnodeValue(currAnode);
		defSegmentValue(currAnode);
		++currAnode;
		if (currAnode == MAXANODE) {currAnode = 0;}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM15, TIM_IT_Update) != RESET)
	{				
		
		TIM_ClearITPendingBit(TIM15, TIM_IT_Update);
	}
}

void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{

		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}
}

void ADC1_IRQHandler(void)
{

}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
