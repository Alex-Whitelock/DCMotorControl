
#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f0xx.h"
#include "delay.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
volatile int16_t error_integral;    // Integrated error signal


/* -------------------------------------------------------------------------------------------------------------
 *  Global Variables for Debug Viewing (no real purpose to be global otherwise)
 * -------------------------------------------------------------------------------------------------------------
 */
volatile uint8_t duty_cycle;    // Output PWM duty cycle
volatile int16_t target_rpm;    // Desired speed target
volatile int16_t motor_speed;   // Measured motor speed
volatile int8_t adc_value;      // ADC measured motor current
volatile int16_t error;         // Speed error signal
static int16_t max_rpm = 200;



volatile int8_t overshot;
volatile int8_t dir;
volatile int16_t motor_ticks;
volatile int16_t halved_ticks; //This is a havled ticks value to know when to decrease the value of the encoder ticks.
            // Integral gain


/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

void set_initial_target_rpm(int16_t);

// Sets up the entire motor drive system
void motor_init(void);

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty);

// PI control code is called within a timer interrupt
void PI_update(void);

void move_motor(int16_t encoder_ticks, uint8_t _dir);



/* -------------------------------------------------------------------------------------------------------------
 *  Internal-Use Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

//Applys the electronic break for whichever direction the motor is currently in.
void apply_electronic_break(void);

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void);

// Sets up encoder interface to read motor speed
void encoder_init(void);

// Sets up ADC to measure motor current
void ADC_init(void);

#endif /* MOTOR_H_ */
