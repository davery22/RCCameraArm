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
#include "tsl_user.h"

/* Private typedefs ----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private functions prototype -----------------------------------------------*/

void Init_Std_GPIOs(void);
void User_Button_Init(void);
void USART1_user_Init(void);
void I2C2_user_Init(void);
	void InitializeGyro(void);
void TIM2_user_Init(void);
void TIM3_user_Init(void);
void ProcessSensors(void);
void SystickDelay(__IO uint32_t nTime);

void USART1WriteString(char *str);
void USART1WriteChar(char ch);

/* Global variables ----------------------------------------------------------*/

__IO uint32_t Gv_SystickCounter;
__IO uint8_t Touch_Sensor_Position;
__IO uint16_t Gyro_Y;
__IO uint16_t Gyro_Z;

uint16_t i2c_wait_count;


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
  // Init User Button with EXTI interrupt
  //============================================================================  

	User_Button_Init();
	
	//============================================================================
  // Init USART
  //============================================================================  

	USART1_user_Init();
	
	//============================================================================
  // Init I2C2
  //============================================================================  

	I2C2_user_Init();
	
	//============================================================================
  // Init TIM2 and TIM3
  //============================================================================  

	TIM2_user_Init(); // Used to write to USART periodically
	TIM3_user_Init(); // Used to poll the gyro sensor periodically

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
  * @brief  Initializes the User Button, including EXTI interrupt (uses PA0)
  * @param  None
  * @retval None
  */
void User_Button_Init(void)
{
	// Enable the SYSCFG clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	
	// Use SYSCFG to route PA0 to EXTI0
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PA;
	
	// Enable external interrupt for EXTI0, rising edge trigger
	EXTI->IMR = EXTI_IMR_MR0;
	EXTI->RTSR = EXTI_RTSR_TR0;

	// Enable the interrupt with NVIC, set priority 1
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_SetPriority(EXTI0_1_IRQn, 1);
	
	// Set the user button (PA0) to input, low-speed, pull-down
	GPIOA->MODER &= ~(0x3);
	GPIOA->OSPEEDR &= ~(0x3);
	GPIOA->PUPDR = 0x1;
}


/**
  * @brief  Initializes USART1 (uses PA9,PA10)
  * @param  None
  * @retval None
  */
void USART1_user_Init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
	// Set PA9/10 to alternate function mode
	GPIOA->MODER |= 0x280000; GPIOA->MODER &= ~0x140000;
	
	// Select USART_1 RX/TX function for PA9/10
	GPIOA->AFR[1] &= ~(0xFF0);
	GPIOA->AFR[1] |= 0x110;
	
	/** USART_1 setup **/
	{
		// Set the baud rate to 115200
		USART1->BRR = SystemCoreClock/115200;
		
		// Enable Transmitter and Receiver
		USART1->CR1 |= 0xC;
		
		// Enable RXNE interrupt in USART1 and NVIC
		//USART1->CR1 |= 0x20;
		//NVIC_EnableIRQ(USART1_IRQn);
		//NVIC_SetPriority(USART1_IRQn, 1);
		
		// LAST, enable the USART itself
		USART1->CR1 |= 0x1;
	}
}


/**
  * @brief  Initializes I2C2
  * @param  None
  * @retval None
  */
void I2C2_user_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Set PB11/13 to alternate function, open-drain, AF = I2C2_SDA/I2C2_SCL respectively
	GPIOB->MODER |= 0x08800000; GPIOB->MODER &= ~(0x04400000);
	GPIOB->OTYPER |= 0x2800;
	GPIOB->AFR[1] |= 0x00501000; GPIOB->AFR[1] &= ~(0x00A0E000);
	
	// Set PB14 and PC0 to output, push-pull, initial high
	GPIOB->MODER |= 0x10000000; GPIOB->MODER &= ~(0x20000000);
	GPIOB->OTYPER &= ~(0x4000);
	GPIOB->ODR |= 0x4000; // Controls SD0 - lsb of gyro slave address (high => 0x6B)
	
	GPIOC->MODER |= 0x1; GPIOC->MODER &= ~(0x2);
	GPIOC->OTYPER &= ~(0x1);
	GPIOC->ODR |= 0x1;
	
	// I2C2 peripheral initial setup
	{
		// Setup peripheral to use 100kHz standard mode
		// (PRESC = 1, SCLDEL = 0x4, SDADEL = 0x2, SCLH = 0xF, SCLL = 0x13)
		I2C2->TIMINGR |= 0x10420F13; I2C2->TIMINGR &= 0x1F420F13;
		
		// Enable I2C2
		I2C2->CR1 |= 0x1;
	}
	
	// Initialize the Gyro Sensor via I2C2
	InitializeGyro();
}


/**
  * @brief  Initializes the Gyro Sensor via I2C2.
  * @param  None
  * @retval None
  */
void InitializeGyro(void)
{
	// Write configuration to CR1
	{
		// Initial I2C write (SADD[7:1] = 0x6B [gyro addr], NBYTES = 3, RD_WRN = 0 [write], START = 1)
		I2C2->CR2 |= (0x6B << 1); I2C2->CR2 &= ~(0x28);
		I2C2->CR2 |= 0x00020000; I2C2->CR2 &= 0xFF02FFFF;
		I2C2->CR2 &= ~0x0400;
		I2C2->CR2 |= 0x2000;
		
		while (!((I2C2->ISR & I2C_ISR_TXIS) ^ (I2C2->ISR & I2C_ISR_NACKF))) {}
		
		// Write the address of the gyro's CR1 register
		I2C2->TXDR = 0x20;
			
		while (!((I2C2->ISR & I2C_ISR_TXIS) ^ (I2C2->ISR & I2C_ISR_NACKF))) {}
		
		// Write the configuration bit pattern
		I2C2->TXDR = 0x0B;
		
		while (!(I2C2->ISR & I2C_ISR_TC)) {}
			
		// End the transfer (set STOP bit)
		I2C2->CR2 |= 0x4000;
	}
}


/**
  * @brief  Initializes TIM2 to fire an interrupt at a certain interval.
  * @param  None
  * @retval None
  */
void TIM2_user_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	// Enable interrupt on TIM2 update
	TIM2->DIER |= 0x1;
	
	// Divide TIM2 clock to get 1kHz
	TIM2->PSC = SystemCoreClock/1000 - 1;
	
	// Set TIM2 reset at count=10; together with previous steps, triggers interrupt at 1Hz
	TIM2->ARR = 10;
	
	// Enable TIM2
	TIM2->CR1 |= 0x1;
	
	// Enable TIM2's interrupt in the NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 0);
}


/**
  * @brief  Initializes TIM3 to fire an interrupt at a certain interval.
  * @param  None
  * @retval None
  */
void TIM3_user_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// Enable interrupt on TIM3 update
	TIM3->DIER |= 0x1;
	
	// Divide TIM3 clock to get 1kHz
	TIM3->PSC = SystemCoreClock/1000 - 1;
	
	// Set TIM3 reset at count=100; together with previous steps, triggers interrupt at 10Hz
	TIM3->ARR = 10;
	
	// Enable TIM3
	TIM3->CR1 |= 0x1;
	
	// Enable TIM3's interrupt in the NVIC
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 0);
}


/**
  * @brief  Manage the activity on sensors when touched/released (example)
  * @param  None
  * @retval None
  */
void ProcessSensors(void)
{
	uint8_t pos = MyLinRots[0].p_Data->Position;
	
  // Due to LED6 shared between the ECS activity and Linear position
  if (!Gv_ProcessSensor && MyLinRots[0].p_Data->StateId == TSL_STATEID_RELEASE)
  {
    return;
  }
	
	Touch_Sensor_Position = pos;
 
  LED3_OFF;
  LED4_OFF;
  LED5_OFF;
  LED6_OFF;
	
  if (pos <= 112)
  {
    LED3_ON;
  }

  if (pos <= 80)
  {
    LED5_ON;
  }

  if (pos <= 48)
  {
    LED6_ON;
  }

  if (pos <= 16)
  {
    LED4_ON;
  }
}


/** Retrieves Y(pitch) and Z(yaw) status from the gyro.
*/
void PollGyro() {
	// Read status from Y,Z registers
	{	
		// Write register address to I2C
		I2C2->CR2 |= (0x6B << 1); I2C2->CR2 &= ~(0x28);
		I2C2->CR2 |= 0x00010000; I2C2->CR2 &= 0xFF01FFFF;
		I2C2->CR2 &= ~0x0400;
		I2C2->CR2 |= 0x2000;
		
		while (!((I2C2->ISR & I2C_ISR_TXIS) ^ (I2C2->ISR & I2C_ISR_NACKF)) && (++i2c_wait_count)) {}
    if (!i2c_wait_count) { I2C2->CR2 |= 0x4000; i2c_wait_count = 0; return; } i2c_wait_count = 0;
		
		// Write the address of the gyro's Y-low register
		I2C2->TXDR = 0xAA;
		
		while (!(I2C2->ISR & I2C_ISR_TC) && (++i2c_wait_count)) {}
    if (!i2c_wait_count) { I2C2->CR2 |= 0x4000; i2c_wait_count = 0; return; } i2c_wait_count = 0;
		
		// RESTART, Read from the I2C
		I2C2->CR2 |= (0x6B << 1); I2C2->CR2 &= ~(0x28);
		I2C2->CR2 |= 0x00040000; I2C2->CR2 &= 0xFF04FFFF;
		I2C2->CR2 |= 0x0400;
		I2C2->CR2 |= 0x2000;
			
		while (!((I2C2->ISR & I2C_ISR_RXNE) ^ (I2C2->ISR & I2C_ISR_NACKF)) && (++i2c_wait_count)) {}
    if (!i2c_wait_count) { I2C2->CR2 |= 0x4000; i2c_wait_count = 0; return; } i2c_wait_count = 0;
			
		Gyro_Y = I2C2->RXDR;
			
		while (!((I2C2->ISR & I2C_ISR_RXNE) ^ (I2C2->ISR & I2C_ISR_NACKF)) && (++i2c_wait_count)) {}
    if (!i2c_wait_count) { I2C2->CR2 |= 0x4000; i2c_wait_count = 0; return; } i2c_wait_count = 0;
			
		Gyro_Y = Gyro_Y | ((0xFFFF & I2C2->RXDR) << 8);
			
		while (!((I2C2->ISR & I2C_ISR_RXNE) ^ (I2C2->ISR & I2C_ISR_NACKF)) && (++i2c_wait_count)) {}
    if (!i2c_wait_count) { I2C2->CR2 |= 0x4000; i2c_wait_count = 0; return; } i2c_wait_count = 0;
			
		Gyro_Z = I2C2->RXDR;
			
		while (!((I2C2->ISR & I2C_ISR_RXNE) ^ (I2C2->ISR & I2C_ISR_NACKF)) && (++i2c_wait_count)) {}
    if (!i2c_wait_count) { I2C2->CR2 |= 0x4000; i2c_wait_count = 0; return; } i2c_wait_count = 0;
			
		Gyro_Z = Gyro_Z | ((0xFFFF & I2C2->RXDR) << 8);
		
		while (!(I2C2->ISR & I2C_ISR_TC) && (++i2c_wait_count)) {}
    if (!i2c_wait_count) { I2C2->CR2 |= 0x4000; i2c_wait_count = 0; return; } i2c_wait_count = 0;
			
		// End the transfer (set STOP bit)
		I2C2->CR2 |= 0x4000;
	}
}


/** Write a string over USART1
*/
void USART1WriteString(char *str) {
	while (*str) {
		USART1WriteChar(*str); str++;
	}
}

/** Write a single character over USART1
*/
void USART1WriteChar(char ch) {
	// Wait for transmit register to be empty
	while ((USART1->ISR & 0x80) != 0x80) {}
		
	USART1->TDR = ch;
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
		__WFI();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
