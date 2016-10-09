/* Delay Functions and SysTick Interrupt
 *
 * Provides a known-duration delay, that puts the processor to sleep while waiting
 * Allows for registering a callback function for the SysTick IRQ
 */

#include "delay.h"

// Initializes the SysTick timer to 1000Hz, gives 1mS delay resolution
void delay_init(void) {
    SysTick_Config(SystemCoreClock/1000);   // Divide system clock by 1000 to get 1mS
    NVIC_SetPriority(SysTick_IRQn, 1);      // Set to high-priority
    delay_tics = 0;
    SysTick_callback = NULL;
}

// Delays the specified number of milliseconds
void delay_ms(uint32_t delay) {
    delay_tics = delay;
    while(delay_tics) {     // Loop until delay_tics == 0
        __WFI();            // Put processor to sleep until interrupt occurs
    }
}

// Accepts a function pointer to a void function without parameters
// Registers as a callback to be invoked at every SysTick interrupt
void SysTick_setCallback(callback_TypeDef func) {
    SysTick_callback = func;
}

// Clears the SysTick callback
void SysTick_clearCallback(void) {
    SysTick_callback = NULL;
}

void SysTick_Handler(void) {
    if(delay_tics)          // If delay_tics is nonzero, decrement it
        delay_tics--;
    if(SysTick_callback)    // If a callback function has been registered, call it
        SysTick_callback();
}
