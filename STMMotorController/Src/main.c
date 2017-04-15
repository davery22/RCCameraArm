/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "motor.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

void LED_init(void);
void button_init(void);
void User_Button_Init(void);
void USART1_user_Init(void);
void USART1WriteChar(char);


/* Main Program Code -----------------------------------------------*/
int main(void)
{
	HAL_Init();  // Reset of all peripherals, Initializes the Flash interface and the Systick. 
	SystemClock_Config();  // Configure the system clock 

	//Init peripherals, blinks PC9 LED in loop as heartbeat.
	LED_init();                             // Initialize LED's
	User_Button_Init();
	USART1_user_Init();
	//button_Init();                          // Initialize button  
	//motor_init();                           // Initialize motor code 
	
	while(1) {
			//__WFI();
			GPIOC->ODR ^= GPIO_ODR_9;           // Toggle LED
			HAL_Delay(128);                     // Delay 1/8th second
	}
}


void LED_init(void)
{
	// Initialize PC8 and PC9 for LED's
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;                  // Set PC8 & PC9 to outputs
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                    // Set to push-pull output type
	GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
											(GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));   // Set to low speed
	GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
										(GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));             // Set to no pull-up/down
	GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);                                   // Shut off LED's
}


void button_init(void)
{
	// Initialize PA0 for button input
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
	GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}


/**
  * @brief  Initializes the User Button, including EXTI interrupt (uses PA0)
  * @param  None
  * @retval None
  */
void User_Button_Init(void)
{
	// Enable the SYSCFG clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
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
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
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
		
		// Enable Receiver and Transmitter, respectively
		USART1->CR1 |= 0x4; USART1->CR1 |= 0x8;
		
		// Enable RXNE interrupt in USART1 and NVIC
		USART1->CR1 |= 0x20;
		NVIC_EnableIRQ(USART1_IRQn);
		NVIC_SetPriority(USART1_IRQn, 1);
		
		// LAST, enable the USART itself
		USART1->CR1 |= 0x1;
	}
}


/** Write a single character over USART1
*/
void USART1WriteChar(char ch) {
	// Wait for transmit register to be empty
	while ((USART1->ISR & 0x80) != 0x80) {}
		
	USART1->TDR = ch;
}


/* Called by SysTick Interrupt
* Performs button debouncing, changes target speed on button rising edge
*/
void HAL_SYSTICK_Callback(void)
{
	static uint32_t debouncer = 0;
	
	debouncer = (debouncer << 1);
	if(GPIOA->IDR & (1 << 0)) {
			debouncer |= 0x1;
	}

	if(debouncer == 0x7FFFFFFF) {
	switch(target_rpm) {
			case 80:
					target_rpm = 50;
					break;
			case 50:
					target_rpm = 81;
					break;
			case 0:
					target_rpm = 80;
					break;
			default:
					target_rpm = 0;
					break;
			}
	}
}


/** System Clock Configuration */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

		/**Initializes the CPU, AHB and APB busses clocks 
		*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

		/**Initializes the CPU, AHB and APB busses clocks 
		*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
															|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}

		/**Configure the Systick interrupt time 
		*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

		/**Configure the Systick 
		*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while(1) 
	{
	}
	/* USER CODE END Error_Handler */ 
}

