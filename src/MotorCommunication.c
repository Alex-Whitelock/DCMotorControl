/*
 * MotorCommunication.c
 *
 *  Created on: Oct 12, 2016
 *      Author: wiblack
 */

#include "MotorCommunication.h"

void UART_Init(uint32_t speed){


	/* USARTx configured as follow:
						- BaudRate = speed parameter above
						- Word Length = 8 Bits
						- One Stop Bit
						- No parity
						- Hardware flow control disabled (RTS and CTS signals)
						- Receive and transmit enabled
			*/

	RCC->AHBENR|=RCC_AHBENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->CR1 =0;//RESET AND ENSURE THAT WE  are working with 8 bits
	//aslso pairty control is disabled. and in even parity
		//PA3_is the usart2_RX data input
		//PA2 is the usart2_TX data output
	GPIOA->MODER |= 1<<5;//Enable 2 for alternate function mode for e pa2
	GPIOA->MODER |= 1<<7;//ENABLE PA3 for alternate function mode.
	GPIOA->AFR[0] |= (1<<8);//choose AF1 for pin 2
	GPIOA ->AFR[0] |= (1<<12);//choose af1 for pin 3

	//send a clock to the usart register

	USART2->CR1|=1<<5;
	USART2->CR1|=USART_CR1_RE;//reciever enable
	USART2->CR1|=USART_CR1_TE;//transmitter enable
	USART2->CR1|=USART_CR1_UE;//usart enable


	USART2->BRR = SystemCoreClock/speed;


	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 2); // Set to medium priority
	//isArmed=0;

	}
