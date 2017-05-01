// Includes
#include "main.h"
#include "stm32f0xx_hal.h"
#include "motor.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

void LED_Init(void);
void User_Button_Init(void);
void USART1_user_Init(void);

void USART1_WriteChar(uint8_t);
void ReadGyroData(uint8_t);
uint8_t InGyroBounds(uint8_t);
void AdjustTargetRPM(void);

int32_t Gyro_Y;
int32_t Gyro_Z;

int32_t Gyro_Y_Velocity;
int32_t Gyro_Z_Velocity;

/*
 * PINS IN USE FOR MAIN APPLICATION:
 *
 * PC6  -  OUTPUT -  Red LED
 * PC7  -  OUTPUT -  Blue LED
 * PC8  -  OUTPUT -  Orange LED
 * PC9  -  OUTPUT -  Green LED
 *
 * PA0  -  OUTPUT -  User Button
 *
 * PA9  -  INPUT  -  USART1 Rx
 * PA10 -  OUTPUT -  USART1 Tx
 * 
 */

 #define DEBUG (0)

/* Main Program Code -----------------------------------------------*/
int main(void) {
    HAL_Init();  // Reset of all peripherals, Initializes the Flash interface and the Systick. 
    SystemClock_Config();  // Configure the system clock 

    //Init peripherals, blinks PC9 LED in loop as heartbeat.
    LED_Init();
    #if (DEBUG)
			User_Button_Init();
		#else
			USART1_user_Init();
		#endif
    Motor_Init();
    
    while(1) {
        GPIOC->ODR ^= GPIO_ODR_9;           // Toggle LED
        HAL_Delay(128);                     // Delay 1/8th second
    }
}


void LED_Init(void) {
    // Set up clock
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
    // Initalize LED pins  - PC6,PC7,PC8,PC9
		GPIOC->MODER |= 0x55000; GPIOC->MODER &= ~0xAA000;
		GPIOC->OTYPER &= ~0x3C0;
		GPIOC->OSPEEDR &= ~0xFF000;
		GPIOC->PUPDR &= ~0xFF000;

    GPIOC->ODR &= ~0x3C0;		// Shut them all off initially
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
	
	// Select USART1 RX/TX function for PA9/10
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
void USART1WriteChar(uint8_t ch) {
	// Wait for transmit register to be empty
	while ((USART1->ISR & 0x80) != 0x80) {}
		
	USART1->TDR = ch;
}



/* Gyro Reading over USART1 -----------------------------------------------*/
// States
#define READ_IDLE_1	(0)
#define READ_GYRO_Y_HI (1)
#define READ_GYRO_Y_LO (2)
#define READ_BREAK (3)
#define READ_IDLE_2 (4)
#define READ_GYRO_Z_HI (5)
#define READ_GYRO_Z_LO (6)
#define READ_FINISH (7)

// Header bytes
//#define START (0x7F)
//#define END   (0x80) // Hamming distance of 8 from START, for some security
										 // Note that these values are unobtainable for (signed!) gyro readings, due to the divide by 260

// Debug macros
#define TOGGLE_RED \
	GPIOC->ODR ^= 0x40; \

#define TOGGLE_BLUE \
	GPIOC->ODR ^= 0x80; \

#define TOGGLE_ORANGE \
	GPIOC->ODR ^= 0x100; \

#define TOGGLE_GREEN \
	GPIOC->ODR ^= 0x200; \

// Helper function - gyro bounds check
//uint8_t InGyroBounds(uint8_t ch) { return !(0x7F <= ch && ch <= 0x81); }

/** Reads the gyro data from USART1
 *
 *  Protocol is four bytes:
 *
 *  START			(0x7F)
 *  GYRO_Y 		(Range: [0x00,0x7E] u [0x82,0xFF] - Divide original 2-byte value by decimal 260) - decimal range is [-126,126]
 *  GYRO_Z 		(Range: [0x00,0x7E] u [0x82,0xFF] - Divide original 2-byte value by decimal 260)
 *  END       (0x80)
 *
 *  Any time START is encountered a new read sequence is begun
 *  Readings are accepted if   both GYRO_Y and GYRO_Z are in range   and   END is received directly after GYRO_Z
 *  
 */
void ReadGyroData(uint8_t ch)
{
	static char state;
	static int16_t yavg = -89;	// Average resting y gyro value
	static int16_t zavg = 218;	// Average resting z gyro value
	static int16_t temp_y;			// New y value
	static int16_t temp_z;			// New z value
	
	TOGGLE_ORANGE
	
	// UART communication protocol
	switch(state)
	{
		case READ_IDLE_1:
			if (ch == 'y') state++;
			break;
		case READ_GYRO_Y_HI:
			temp_y = ch;
			state++;
			break;
		case READ_GYRO_Y_LO:
			temp_y = ((temp_y) << 8) + ch;
			state++;
			break;
		case READ_BREAK:
			if (ch == ',') state++; else state = READ_IDLE_1; TOGGLE_RED
			break;
		case READ_IDLE_2:
			if (ch == 'z') state++; else state = READ_IDLE_1;
			break;
		case READ_GYRO_Z_HI:
			temp_z = ch;
			state++;
			break;
		case READ_GYRO_Z_LO:
			temp_z = ((temp_z) << 8) + ch;
			state++;
			break;
		case READ_FINISH:
			if (ch == '\n'){
				// Subtract out noise, add exponential decay to gyro data
				Gyro_Y += temp_y - yavg - (Gyro_Y>>8);
				Gyro_Z += temp_z - zavg - (Gyro_Z>>8);
				AdjustTargetRPM(); 
				TOGGLE_BLUE
			}
			
			state = READ_IDLE_1;
			break;
		default:
			state = READ_IDLE_1;
			break;
	}
}


// No idea what these should really be - depends on size of values out of the gyro and usart rx rate
#define GYRO_BOUND (10000)
#define RPM_SCALAR (6)

/** Adjusts the target RPM based on the gyro reading
*/

/*int32_t prev_Y = 0;
int32_t prev_Z = 0;*/
int32_t max_rpm = 80;

void AdjustTargetRPM(void)
{
	// Get velocity pulse in possible range for motors
	Gyro_Y_Velocity = (Gyro_Y>>12);
	Gyro_Z_Velocity = (Gyro_Z>>12);
	
	{ // Motor speed debugging output
	USART1WriteChar((unsigned char)(Gyro_Y_Velocity));
	USART1WriteChar('\t');
	USART1WriteChar((unsigned char)(Gyro_Z_Velocity));
	USART1WriteChar('\n');
	}
	
	y_target_rpm = Gyro_Y_Velocity;// >> RPM_SCALAR;
	if (y_target_rpm > 80) y_target_rpm = max_rpm; else if (y_target_rpm < -max_rpm) y_target_rpm = -max_rpm;
	z_target_rpm = Gyro_Z_Velocity;// >> RPM_SCALAR;
	if (z_target_rpm > 80) z_target_rpm = max_rpm; else if (z_target_rpm < -max_rpm) z_target_rpm = -max_rpm;
	
	// Invert targets (to get the right direction for motors)
	y_target_rpm = -y_target_rpm;
	z_target_rpm = -z_target_rpm;
	
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

