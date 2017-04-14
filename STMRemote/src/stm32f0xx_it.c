/**
  ******************************************************************************
  * @file    STM32F072B_Ex02_Linear_DISCO\src\stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    04-April-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "tsl_user.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern __IO uint32_t Gv_SystickCounter;
extern __IO uint32_t Gv_EOA;

extern __IO uint8_t Touch_Sensor_Position;
extern __IO uint16_t Gyro_Y;
extern __IO uint16_t Gyro_Z;
__IO uint8_t User_Button_State;

extern void PollGyro(void);
extern void USART1WriteChar(char);
extern void USART1WriteString(char*);

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
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
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
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
  * @brief  This function handles SysTick Handler, which is configured to go off every 1 ms.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  // Global variables used by the SystickDelay() routine (in main.c)
  if (Gv_SystickCounter != 0)
  {
    Gv_SystickCounter--;
  }
	
  // TSL timing for ECS, DTO, ...
  TSL_tim_ProcessIT();
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles external interrupt requests.
  * @param  None
  * @retval None
  */
void EXTI0_1_IRQHandler(void)
{
	// Clear the interrupt
	EXTI->PR |= EXTI_PR_PR0;
	
	// Do something else
	User_Button_State = !User_Button_State;
}

/**
  * @brief  This function handles the TIM3 interrupt.
  * @param  None
  * @retval None
	*/
void TIM3_IRQHandler(void)
{
	// Clear the interrupt
	TIM3->SR &= ~TIM_SR_UIF;
	
	PollGyro();
}

/**
  * @brief  This function handles the TIM2 interrupt.
  * @param  None
  * @retval None
	*/
void TIM2_IRQHandler(void)
{
	// Clear the interrupt
	TIM2->SR &= ~TIM_SR_UIF;
	
	// Write system status
	{
		// TODO: Remove offset 48
		USART1WriteChar('t');
		USART1WriteChar(Touch_Sensor_Position + 48);
		USART1WriteChar(',');
		
		USART1WriteChar('b');
		USART1WriteChar(User_Button_State + 48);
		USART1WriteChar(',');
		
		USART1WriteChar('y');
		USART1WriteChar((Gyro_Y >> 8));
		USART1WriteChar((Gyro_Y & 0xFF));
		USART1WriteChar(',');
		
		USART1WriteChar('z');
		USART1WriteChar((Gyro_Z >> 8));
		USART1WriteChar((Gyro_Z & 0xFF));
		USART1WriteString("\r\n");
	}
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
  * @brief  This function handles Touch Sensing Controller interrupt requests.
  * @param  None
  * @retval None
  */
void TS_IRQHandler(void)
{
#if TSLPRM_TSC_IODEF > 0 // Default = Input Floating
  // Set IO default in Output PP Low to discharge all capacitors
  TSC->CR &= (uint32_t)(~(1 << 4));
#endif
  TSC->ICR |= 0x03; // Clear EOAF and MCEF flags
  Gv_EOA = 1; // To inform the main loop routine of the End Of Acquisition
}

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
