/*
 * MotorCommunication.h
 *
 *  Created on: Oct 12, 2016
 *      Author: wiblack
 */

#ifndef MOTORCOMMUNICATION_H_
#define MOTORCOMMUNICATION_H_

#include "stm32f0xx.h"
#include <stdlib.h>
#include "stm32f0xx_usart.h"//note need to ensure that this is not ignored by eclipse
#include "motor.h"
#include "MotionControl.h"

void UART_Init(uint32_t speed);

volatile int isArmed;
volatile int is_stm_controlled;
volatile int is_in_ScanningMode;
void UART_PutChar(unsigned char c);
void UART_PutStr(char *str);
void UART_ClearRxBuffer(void);
//void LED_init(void);
//uint8_t UART_Test(void);
//uint8_t UART_SetBaud(uint32_t speed);
uint8_t UART_SetName(char *name);
uint8_t UART_SetPin(char *pin);
void UART_Delay(uint32_t delay);
void set_STM_cotrolled(uint8_t set);





#endif /* MOTORCOMMUNICATION_H_ */
