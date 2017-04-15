
#include "motor.h"
#include "stm32f0xx_hal.h"

/* -------------------------------------------------------------------------------------------------------------
*  Global Variable and Type Declarations
*  -------------------------------------------------------------------------------------------------------------
*/
volatile int16_t error_integral;    // Integrated error signal


/* -------------------------------------------------------------------------------------------------------------
*  Global Variables for STMStudio Debug Viewing (no real purpose to be global otherwise)
* -------------------------------------------------------------------------------------------------------------
*/
volatile uint8_t duty_cycle;    // Output PWM duty cycle
volatile int16_t target_rpm;		// Desired speed target
volatile int16_t motor_speed;   // Measured motor speed
volatile int8_t adc_value;      // ADC measured motor current
volatile int16_t error;         // Speed error signal
volatile uint8_t Kp;            // Proportional gain
volatile uint8_t Ki;            // Integral gain

uint8_t ENCODER_SAMPLING_FREQ;

/* -------------------------------------------------------------------------------------------------------------
*  Motor Control and Initialization Functions
* -------------------------------------------------------------------------------------------------------------
*/

// Sets up the entire motor drive system
void motor_init(void)
{
	Kp = 1;     // Set default proportional gain
	Ki = 1;     // Set default integral gain

	pwm_init();
	encoder_init();
}

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void)
{
	RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN);

	// Set up a pin for H-bridge PWM output (TIMER 16 CH1) : PB8 -> AF2
	GPIOB->MODER |= 0x20000; GPIOB->MODER &= ~0x10000;
	GPIOB->AFR[1] |= 0x2; GPIOB->AFR[1] &= ~0xD;

	// Set up a few GPIO output pins for direction control : PA4,PA5 (output, push-pull, low-speed, no pupdr)
	GPIOA->MODER |= 0x500; GPIOA->MODER &= ~0xA00;
	GPIOA->OTYPER &= ~0x30;
	GPIOA->OSPEEDR &= ~0xF00;
	GPIOA->PUPDR &= ~0xF00;

	// Initialize one direction pin to high, the other low
	GPIOA->ODR |= 0x10; GPIOA->ODR &= ~0x20;

	/* Hint: These pins are processor outputs, inputs to the H-bridge
	 *       they can be ordinary 3.3v pins.
	 *       If you hook up the motor and the encoder reports you are 
	 *       running in reverse, either swap the direction pins or the
	 *       encoder pins. (we'll only be using forward speed in this lab)
	 */

	// Set up PWM timer
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	TIM16->CR1 = 0;                         // Clear control register

	// Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
	TIM16->CCMR1 = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
	TIM16->CCER = TIM_CCER_CC1E;           			// Enable capture-compare channel 1
	TIM16->PSC = SystemCoreClock/10000000 - 1;  // Run timer on 1Mhz
	TIM16->ARR = 50;                        		// PWM at 20kHz
	TIM16->CCR1 = 0;                        		// Start PWM at 0% duty cycle
	TIM16->BDTR |= TIM_BDTR_MOE;  							// Set master output enable (only for timers that have complementary outputs)

	TIM16->CR1 |= TIM_CR1_CEN;              		// Enable timer
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty)
{
	if(duty <= 100) {
			TIM16->CCR1 = ((uint32_t)duty*TIM16->ARR)/100;  // Use linear transform to produce CCR1 value
	}
}

// Sets up encoder interface to read motor speed
void encoder_init(void)
{	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// Set up encoder input pins (TIMER 3 CH1 and CH2) : PB4,PB5 -> AF1,AF1 (CH1,CH2)
	GPIOB->MODER |= 0xA00; GPIOB->MODER &= ~0x500;
	GPIOB->AFR[0] |= 0x00110000; GPIOB->AFR[0] &= ~0x00EE0000;

	/* Hint: MAKE SURE THAT YOU USE 5V TOLERANT PINS FOR THE ENCODER INPUTS!
	 *       You'll fry the processor otherwise, read the lab to find out why!
	 */

	// Set up encoder interface (TIM3 encoder input mode)
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CCMR1 = 0;    //Clear control registers
	TIM3->CCER = 0;
	TIM3->SMCR = 0;
	TIM3->CR1 = 0;

	TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
	TIM3->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
	TIM3->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
	TIM3->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
	// (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
	//  just another option, the mid-bias is a bit simpler to understand though.)
	TIM3->CR1 |= TIM_CR1_CEN;                               // Enable timer


	// Configure a second timer (TIM6) to fire an ISR on update event
	// Used to periodically check and update speed variable
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Select PSC and ARR values that give an appropriate interrupt rate

	/* Hint: See section in lab on sampling rate! 
	 *       Recommend choosing a sample rate that gives 2:1 ratio between encoder value 
	 *       and target speed. (Example: 200 RPM = 400 Encoder count for interrupt period)
	 *       This is so your system will match the lab solution
	 */ 
	 
	// 60/640000*en_count = period
	
	TIM6->PSC = SystemCoreClock/1000000 - 1; // Configured to 1 MHz
	TIM6->ARR = 15000; 											// Interrupt every 15000 us (for en_count=160)
	
	ENCODER_SAMPLING_FREQ = (SystemCoreClock/(TIM6->PSC + 1)) / TIM6->ARR;

	TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
	TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

	NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
	NVIC_SetPriority(TIM6_DAC_IRQn,2);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void)
{
	/* Calculate the motor speed in raw encoder counts
	 * Note the motor speed is signed! Motor can be run in reverse.
	 * Speed is measured by how far the counter moved from center point
	 */
	motor_speed = (TIM3->CNT - 0x7FFF);
	TIM3->CNT = 0x7FFF; // Reset back to center point
	
	// Call the PI update function
	PI_update();

	TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

void PI_update(void)
{
	/* Run PI control loop
	 *
	 * Make sure to use the indicated variable names. This allows STMStudio to monitor
	 * the condition of the system!
	 *
	 * target_rpm -> target motor speed in RPM
	 * motor_speed -> raw motor speed in encoder counts
	 * error -> error signal (difference between measured speed and target)
	 * error_integral -> integrated error signal
	 * Kp -> Proportional Gain
	 * Ki -> Integral Gain
	 * output -> raw output signal from PI controller
	 * duty_cycle -> used to report the duty cycle of the system 
	 * adc_value -> raw ADC counts to report current
	 *
	 */
	
	// calculate error signal and write to "error" variable
	// 64 * 50 = 3200 encoder counts per motor shaft rotation
	error = 64 * target_rpm - (motor_speed * ENCODER_SAMPLING_FREQ * 60 / 50);
	
	/* Hint: Remember that your calculated motor speed may not be directly in RPM!
	 *       You will need to convert the target or encoder speeds to the same units.
	 *       I recommend converting to whatever units result in larger values, gives
	 *       more resolution.
	 */
	
	
	// Calculate integral portion of PI controller, write to "error_integral" variable
	error_integral += Ki*error;
	
	// Clamp the value of the integral to a limited positive range
	if (error_integral < 0) error_integral = 0; else if (error_integral > 3200) error_integral = 3200;
	
	/* Hint: The value clamp is needed to prevent excessive "windup" in the integral.
	 *       You'll read more about this for the post-lab. The exact value is arbitrary
	 *       but affects the PI tuning.
	 *       Recommend that you clamp between 0 and 3200 (what is used in the lab solution)
	 */
	
	// Calculate proportional portion, add integral and write to "output" variable
	int16_t error_proportional = Kp*error;
	
	int16_t output = error_integral + error_proportional;
	
	/* Because the calculated values for the PI controller are significantly larger than 
	 * the allowable range for duty cycle, you'll need to divide the result down into 
	 * an appropriate range. (Maximum integral clamp / X = 100% duty cycle)
	 * 
	 * Hint: If you chose 3200 for the integral clamp you should divide by 32 (right shift by 5 bits), 
	 *       this will give you an output of 100 at maximum integral "windup".
	 *
	 * This division also turns the above calculations into pseudo fixed-point. This is because
	 * the lowest 5 bits act as if they were below the decimal point until the division where they
	 * were truncated off to result in an integer value. 
	 *
	 * Technically most of this is arbitrary, in a real system you would want to use a fixed-point
	 * math library. The main difference that these values make is the difference in the gain values
	 * required for tuning.
	 */

	// Divide the output into the proper range for output adjustment
	output >>= 5;
	 
	// Clamp the output value between 0 and 100 
	if (output < 0) output = 0; else if (output > 100) output = 100;
	
	pwm_setDutyCycle(output);
	duty_cycle = output;            // For debug viewing
}
