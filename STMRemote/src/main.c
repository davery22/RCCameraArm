/**
  ******************************************************************************
  * @file    STM32F072B_Ex02_Linear_DISCO\src\main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    04-April-2014
  * @brief   Basic example on how to use the TouchSensing Driver on
  *          STM32F072B Discovery board.
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
#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "tsl_user.h"

/* Private typedefs ----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private functions prototype -----------------------------------------------*/

void Init_Std_GPIOs(void);
void ProcessSensors(void);
void SystickDelay(__IO uint32_t nTime);

/* Global variables ----------------------------------------------------------*/

__IO uint32_t Gv_SystickCounter;


/**
  * @brief  Main routine.
  * @param  None
  * @retval None
  */
int main(void)
{
  
  //============================================================================
  // Init system clock
  //============================================================================
  // At this stage the microcontroller clock setting is already configured, 
  // this is done through SystemInit() function which is called from startup
  // file (startup_stm32f072.s) before to branch to application main.
  // To reconfigure the default setting of SystemInit() function, refer to
  // system_stm32f0xx.c file

  //============================================================================
  // Init Standard GPIOs used by the Application (not Touch-Sensing)
  //============================================================================  

  Init_Std_GPIOs();

  //============================================================================
  // Init STMTouch driver
  //============================================================================  

  TSL_user_Init();
	
	//============================================================================
  // Init Other Stuff
  //============================================================================  

	{
		// Enable the SYSCFG clock
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
		
		// Use SYSCFG to route PA0 to EXTI0
		SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;
		
		// Enable external interrupt for EXTI0, rising edge trigger
		EXTI->IMR = 0x1; //EXTI_IMR_MR0_Msk;
		EXTI->RTSR = 0x1; //EXTI_RTSR_TR0_Msk;

		// Enable the interrupt with NVIC, set priority 1
		NVIC_EnableIRQ(EXTI0_1_IRQn);
		NVIC_SetPriority(EXTI0_1_IRQn, 1);
		
		// Set the user button (PA0) to input, low-speed, pull-down
		GPIOA->MODER &= ~(0x3);
		GPIOA->OSPEEDR &= ~(0x3);
		GPIOA->PUPDR = 0x1;
	}

  //============================================================================
  // Main loop
  //============================================================================

  for (;;)
  {
    // Execute STMTouch Driver state machine
    if (TSL_user_Action() == TSL_STATUS_OK)
    {
      ProcessSensors(); // Execute sensors related tasks
    }
    else
    {
      // Execute other tasks...
    }
  }

}


/**
  * @brief  Initializes GPIOs not related to Touch-Sensing.
  * @param  None
  * @retval None
  */
void Init_Std_GPIOs(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  // Enable GPIOs clocks
  RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN |
                  RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOFEN);

  // Configure all unused IOs in Analog to reduce current consumption.
  // Excepted PA13 (SWDAT) and PA14 (SWCLK) used for debugging the application.
  GPIOA->MODER |= 0xC3FFFFFF;
  GPIOA->PUPDR = 0;
  GPIOB->MODER = 0xFFFFFFFF;
  GPIOB->PUPDR = 0;
  GPIOC->MODER = 0xFFFFFFFF;
  GPIOC->PUPDR = 0;
  GPIOD->MODER = 0xFFFFFFFF;
  GPIOD->PUPDR = 0;
  GPIOF->MODER = 0xFFFFFFFF;
  GPIOF->PUPDR = 0;

  // LEDs (PC6, PC7, PC8, PC9)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  LED3_OFF;
  LED4_OFF;
  LED5_OFF;
  LED6_OFF;
}


/**
  * @brief  Manage the activity on sensors when touched/released (example)
  * @param  None
  * @retval None
  */
void ProcessSensors(void)
{
  // Due to LED6 shared between the ECS activity and Linear position
  if (!Gv_ProcessSensor && MyLinRots[0].p_Data->StateId == TSL_STATEID_RELEASE)
  {
    return;
  }
 
  LED3_OFF;
  LED4_OFF;
  LED5_OFF;
  LED6_OFF;

  if (MyLinRots[0].p_Data->Position <= 112)
  {
    LED3_ON;
  }

  if (MyLinRots[0].p_Data->Position <= 80)
  {
    LED5_ON;
  }

  if (MyLinRots[0].p_Data->Position <= 48)
  {
    LED6_ON;
  }

  if (MyLinRots[0].p_Data->Position <= 16)
  {
    LED4_ON;
  }
}


/**
  * @brief  Executed when a sensor is in Error state
  * @param  None
  * @retval None
  */
void MyLinRots_ErrorStateProcess(void)
{
  // Add here your own processing when a sensor is in Error state
  TSL_linrot_SetStateOff();
  LED4_OFF;
  LED5_OFF;
  LED6_OFF;
  for (;;)
  {
    LED3_TOGGLE; // RED
    SystickDelay(200);
  }
}


/**
  * @brief  Executed when a sensor is in Off state
  * @param  None
  * @retval None
  */
void MyLinRots_OffStateProcess(void)
{
  // Add here your own processing when a sensor is in Off state
}


//-------------------
// CallBack functions
//-------------------

/**
  * @brief  Executed at each timer interruption (option must be enabled)
  * @param  None
  * @retval None
  */
void TSL_CallBack_TimerTick(void)
{
}


#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  LED4_OFF;
  LED5_OFF;
  LED6_OFF;
  for (;;)
  {
    LED3_TOGGLE; // RED
    SystickDelay(100);
  }
}
#endif


/**
  * @brief  Add a delay using the Systick
  * @param  nTime Delay in milliseconds.
  * @retval None
  */
void SystickDelay(__IO uint32_t nTime)
{
  Gv_SystickCounter = nTime;
  while (Gv_SystickCounter != 0)
  {
    // The Gv_SystickCounter variable is decremented every ms by the Systick interrupt routine  
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
