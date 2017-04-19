
#ifndef MOTOR_H_
#define MOTOR_H_

#include "stm32f0xx_hal.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
extern volatile int16_t y_error_integral;    // Integrated error signal
extern volatile int16_t z_error_integral;    // Integrated error signal

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variables for Debug Viewing (no real purpose to be global otherwise)
 * -------------------------------------------------------------------------------------------------------------
 */
extern volatile uint8_t y_duty_cycle;    // Output PWM duty cycle
extern volatile int16_t y_target_rpm;    // Desired speed target
extern volatile int16_t y_motor_speed;   // Measured motor speed
extern volatile int16_t y_error;         // Speed error signal
extern volatile int16_t y_pos;

extern volatile uint8_t z_duty_cycle;    // Output PWM duty cycle
extern volatile int16_t z_target_rpm;    // Desired speed target
extern volatile int16_t z_motor_speed;   // Measured motor speed
extern volatile int16_t z_error;         // Speed error signal
extern volatile int16_t z_pos;

extern volatile uint8_t Kp;            // Proportional gain
extern volatile uint8_t Ki;            // Integral gain


/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the entire motor drive system
void motor_init(void);

// Set the duty cycle of the PWM, accepts (-100,100)
void pwm_setDutyCycle(int8_t duty, uint8_t motor);

// PI control code is called within a timer interrupt
void PI_update(void);


/* -------------------------------------------------------------------------------------------------------------
 *  Internal-Use Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void);

// Sets up encoder interface to read motor speed
void encoder_init(void);

// Sets up ADC to measure motor current
void ADC_init(void);

#endif /* MOTOR_H_ */
