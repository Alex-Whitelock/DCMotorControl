/* Delay Functions and SysTick Interrupt
 *
 * Provides a known-duration delay, that puts the processor to sleep while waiting
 * Allows for registering a callback function for the SysTick IRQ
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f0xx.h"
#include <stdlib.h>

volatile uint32_t delay_tics;
typedef void (*callback_TypeDef)(void);
callback_TypeDef SysTick_callback;

// Initializes the SysTick timer to 1000Hz, gives 1mS delay resolution
void delay_init(void);

// Delays the specified number of milliseconds
void delay_ms(uint32_t delay);

// Accepts a function pointer to a void function without parameters
// Registers as a callback to be invoked at every SysTick interrupt
void SysTick_setCallback(callback_TypeDef func);

// Clears the SysTick callback
void SysTick_clearCallback(void);

#endif /* DELAY_H_ */
