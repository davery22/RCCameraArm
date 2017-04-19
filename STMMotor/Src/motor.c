// Includes
#include "motor.h"
#include "stm32f0xx_hal.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 * --------------------------------------------------------------------------------------------------------------
 */
volatile int16_t y_error_integral;
volatile int16_t z_error_integral;

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variables for STMStudio Debug Viewing (no real purpose to be global otherwise)
 * -------------------------------------------------------------------------------------------------------------
 */
#define ARM_DOWN (5500)
#define ARM_UP (0)
#define ARM_LEFT (5500)
#define ARM_RIGHT (0)

volatile uint8_t y_duty_cycle;    // Output PWM duty cycle
volatile int16_t y_target_rpm;    // Desired speed target
volatile int16_t y_motor_speed;   // Measured motor speed
volatile int16_t y_pos = ARM_DOWN;
volatile int16_t y_error;         // Speed error signal

volatile uint8_t z_duty_cycle;    // Output PWM duty cycle
volatile int16_t z_target_rpm;    // Desired speed target
volatile int16_t z_motor_speed;   // Measured motor speed
volatile int16_t z_pos = ARM_LEFT;
volatile int16_t z_error;         // Speed error signal

volatile uint8_t Kp;              // Proportional gain
volatile uint8_t Ki;              // Integral gain

/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
 
// Sets up the entire motor drive system
void motor_init(void)
{
	Kp = 27;     // Set default proportional gain
	Ki = 6;     // Set default integral gain
	
	pwm_init();
	encoder_init();
}


// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void)
{
	RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN);
	
	// Motor Y Init
	{
		// Set up a pin for H-bridge PWM output (TIMER 16 CH1) - use PB8 -> AF2
		GPIOB->MODER |= 0x20000; GPIOB->MODER &= ~0x10000;
		GPIOB->AFR[1] |= 0x2; GPIOB->AFR[1] &= ~0xD;
			
		// Set up a few GPIO output pins for direction control - use PA4,PA5
		GPIOA->MODER |= 0x500; GPIOA->MODER &= ~0xA00;
		GPIOA->OTYPER &= ~0x30;
		GPIOA->OSPEEDR &= ~0xF00;
		GPIOA->PUPDR &= ~0xF00;

		// Set up PWM timer
		RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
		TIM16->CR1 = 0;                                 // Clear control register

		// Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
		TIM16->CCMR1 = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
		TIM16->CCER = TIM_CCER_CC1E;                    // Enable capture-compare channel 1
		TIM16->PSC = SystemCoreClock/1000000 - 1;       // Run timer on 1Mhz
		TIM16->ARR = 50;                                // PWM at 20kHz
		TIM16->CCR1 = 0;                                // Start PWM at 0% duty cycle
		TIM16->BDTR |= TIM_BDTR_MOE;

		TIM16->CR1 |= TIM_CR1_CEN;                      // Enable timer
	}

	// Motor Z Init
	{
		// Set up a pin for H-bridge PWM output (TIMER 17 CH1) - use PB9 -> AF2
		GPIOB->MODER |= 0x80000; GPIOB->MODER &= ~0x40000;
		GPIOB->AFR[1] |= 0x20; GPIOB->AFR[1] &= ~0xD0;
		
		// Set up a few GPIO output pins for direction control - use PA8,PA9
		GPIOA->MODER |= 0x50000; GPIOA->MODER &= ~0xA0000;
		GPIOA->OTYPER &= ~0x300;
		GPIOA->OSPEEDR &= ~0xF0000;
		GPIOA->PUPDR &= ~0xF0000;

		// Set up PWM timer
		RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
		TIM17->CR1 = 0;                                 // Clear control register

		// Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
		TIM17->CCMR1 = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
		TIM17->CCER = TIM_CCER_CC1E;                    // Enable capture-compare channel 1
		TIM17->PSC = SystemCoreClock/1000000 - 1;       // Run timer on 1Mhz
		TIM17->ARR = 50;                                // PWM at 20kHz
		TIM17->CCR1 = 0;                                // Start PWM at 0% duty cycle
		TIM17->BDTR |= TIM_BDTR_MOE;

		TIM17->CR1 |= TIM_CR1_CEN;                      // Enable timer
	}
}


// Sets up encoder interface to read motor speed
void encoder_init(void)
{    
	RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN);
	
	// Motor Y Init
	{
		// Set up encoder input (5V TOLERANT!) pins (TIMER 3 CH1 and CH2) - use PB4,PB5 -> AF1
		GPIOB->MODER |= 0xA00; GPIOB->MODER &= ~0x500;
		GPIOB->AFR[0] |= 0x110000; GPIOB->AFR[0] &= ~0xEE0000;

		// Set up encoder interface (TIM3 encoder input mode)
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

		//Clear control registers
		TIM3->CCMR1 = 0;
		TIM3->CCER = 0;
		TIM3->SMCR = 0;
		TIM3->CR1 = 0;

		TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
		TIM3->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
		TIM3->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
		TIM3->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation

		TIM3->CR1 |= TIM_CR1_CEN;                               // Enable timer
	}

	// Motor Z Init
	{
		// Set up encoder input (5V TOLERANT!) pins (TIMER 2 CH1 and CH2) - use PA15,PB3 -> AF2
		GPIOA->MODER |= 0x80000000; GPIOA->MODER &= ~0x40000000;
		GPIOA->AFR[1] |= 0x20000000; GPIOA->AFR[1] &= ~0xD0000000;

		GPIOB->MODER |= 0x80; GPIOB->MODER &= ~0x40;
		GPIOB->AFR[0] |= 0x2000; GPIOB->AFR[0] &= ~0xD000;

		// Set up encoder interface (TIM3 encoder input mode)
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

		//Clear control registers
		TIM2->CCMR1 = 0;
		TIM2->CCER = 0;
		TIM2->SMCR = 0;
		TIM2->CR1 = 0;

		TIM2->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
		TIM2->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
		TIM2->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
		TIM2->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation

		TIM2->CR1 |= TIM_CR1_CEN;                               // Enable timer
	}
    
	// Configure a second timer (TIM6) to fire an ISR on update event
	// Used to periodically check and update speed variable
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Choose sampling rate to give 2:1 ration between encoder value and target speed
	TIM6->PSC = SystemCoreClock/1000000 - 1;
	TIM6->ARR = 37500; 
	
	TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
	TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

	NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
	NVIC_SetPriority(TIM6_DAC_IRQn,2);
}






#define MOTOR_Y (0)
#define MOTOR_Z (1)

#define MOTOR_Y_FORWARD() \
	GPIOA->ODR |= 0x20; \
	GPIOA->ODR &= ~0x10; \

#define MOTOR_Y_REVERSE() \
	GPIOA->ODR |= 0x10; \
	GPIOA->ODR &= ~0x20; \

#define MOTOR_Z_FORWARD() \
	GPIOA->ODR |= 0x200; \
	GPIOA->ODR &= ~0x100; \

#define MOTOR_Z_REVERSE() \
	GPIOA->ODR |= 0x100; \
	GPIOA->ODR &= ~0x200; \

// Set the duty cycle of the PWM, accepts (-100,100)
void pwm_setDutyCycle(int8_t duty, uint8_t motor)
{
	// Validate
	if (duty < -100 || duty > 100)
	{
		return;
	}

	// Adjust motor direction
	if (duty < 0)
	{
		duty = -duty;
		if (motor == MOTOR_Y) { MOTOR_Y_REVERSE(); /*} else {*/ MOTOR_Z_REVERSE(); }
	}
	else if (duty > 0)
	{
		if (motor == MOTOR_Y) { MOTOR_Y_FORWARD(); /*} else {*/ MOTOR_Z_FORWARD(); }
	}

	// Adjust motor duty cycle
	if (motor == MOTOR_Y)
	{
		TIM16->CCR1 = ((uint32_t)duty*TIM16->ARR)/100;  // Use linear transform to produce CCR1 value
	/*}
	else
	{*/
		TIM17->CCR1 = ((uint32_t)duty*TIM17->ARR)/100;  // Use linear transform to produce CCR1 value
	}
}


/** Adjusts the target_rpm if it is driving the position out of bounds
*/
void bound_pos(uint8_t motor)
{
	if (motor == MOTOR_Y)
	{
		if (y_pos < ARM_UP)
		{
			y_pos = ARM_UP; y_target_rpm = 0;
		}
		else if (y_pos > ARM_DOWN)
		{
			y_pos = ARM_DOWN; y_target_rpm = 0;
		}
	}
	else
	{
		if (z_pos < ARM_RIGHT)
		{
			z_pos = ARM_RIGHT; z_target_rpm = 0;
		}
		else if (y_pos > ARM_LEFT)
		{
			z_pos = ARM_LEFT; z_target_rpm = 0;
		}
	}
}


// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void)
{
	y_motor_speed = (TIM3->CNT - 0x7FFF);
	y_pos += y_motor_speed/2;
	bound_pos(MOTOR_Y);
	
	TIM3->CNT = 0x7FFF; // Reset back to center point

	z_motor_speed = (TIM2->CNT - 0x7FFF);
	z_pos += z_motor_speed/2;
	bound_pos(MOTOR_Z);
	
	TIM2->CNT = 0x7FFF; // Reset back to center point
	
	// Call the PI update function
	PI_update();

	// Acknowledge the interrupt
	TIM6->SR &= ~TIM_SR_UIF;
}


void PI_update(void)
{
	// Motor Y PI control
	{
		// calculate error signal and write to "error" variable
		y_error = y_target_rpm - y_motor_speed/2;
			
		// Calculate integral portion of PI controller, write to "error_integral" variable
		y_error_integral += Ki * y_error;
			
		// Clamp the value of the integral to a limited positive range
		if (y_error_integral < -3200) y_error_integral = -3200; else if (y_error_integral > 3200) y_error_integral = 3200;
			
		// Calculate proportional portion, add integral and write to "output" variable
		int16_t y_error_proportional = Kp * y_error;
			
		// Calculate output and divide the output into the proper range for output adjustment
		int16_t y_output = (y_error_integral + y_error_proportional) >> 5;
			 
		// Clamp the output value between -100 and 100
		if (y_output < -100) y_output = -100; else if (y_output > 100) y_output = 100;
			
		pwm_setDutyCycle(y_output, MOTOR_Y);
		y_duty_cycle = y_output;
	}

	// Motor Z PI control
	/*{
		// calculate error signal and write to "error" variable
		z_error = z_target_rpm - z_motor_speed/2;
		
		// Calculate integral portion of PI controller, write to "error_integral" variable
		z_error_integral += Ki * z_error;
		
		// Clamp the value of the integral to a limited positive range
		if (z_error_integral < -3200) z_error_integral = -3200; else if (z_error_integral > 3200) z_error_integral = 3200;
		
		// Calculate proportional portion, add integral and write to "output" variable
		int16_t z_error_proportional = Kp * z_error;
		
		// Calculate output and divide the output into the proper range for output adjustment
		int16_t z_output = (z_error_integral + z_error_proportional) >> 5;
		 
		// Clamp the output value between -100 and 100
		if (z_output < -100) z_output = -100; else if (z_output > 100) z_output = 100;
		
		pwm_setDutyCycle(z_output, MOTOR_Z);
		z_duty_cycle = z_output;
	}*/
}
