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

void UART_Init(uint32_t speed);



#endif /* MOTORCOMMUNICATION_H_ */
