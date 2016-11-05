/*
 * MotorCommunication.c
 *
 *  Created on: Oct 12, 2016
 *      Author: wiblack
 */

#include "MotorCommunication.h"

#define UART_RX_BUFFER_LENGTH	16																		//maximum number of characters to hold in the receive buffer
#define	UART_TIMEOUT_MAX			94000

char UART_rx_buffer[UART_RX_BUFFER_LENGTH];													//used by the IRQ handler
uint8_t UART_rx_counter = 0; 																				//used by the IRQ handler
char UART_msg[UART_RX_BUFFER_LENGTH];																//variable that contains the latest string received on the RX pin
uint8_t new_UART_msg = 0;
uint8_t *transmitBuffer;


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

//This is the interupt handler for the USART
void USART2_IRQHandler(void)
{
//	int x;

	//int i;

	int16_t  encoder_ticks = 0;
	uint8_t direction = 0;
	uint8_t is_motor_go =0;
	uint8_t motor_speed =0;
	uint8_t is_motor_stop = 0;

	//int i;
	//char armedReceive [40];
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
		/* Read one byte from the receive data register */
    UART_rx_buffer[UART_rx_counter] = USART_ReceiveData(USART2);


    //Most of this code will have to be changed once w
    if(UART_rx_counter == 1) {

    	GPIOC->ODR ^= GPIO_ODR_9;
//    	encoder_ticks = encoder_ticks |(UART_rx_buffer[0] << 8);
//
//
//    	encoder_ticks = encoder_ticks | UART_rx_buffer[1];

    	if(UART_rx_buffer[0] == 'a'){
    		encoder_ticks = 12800;

    	} else if (UART_rx_buffer[0] == 'b') {
    		is_motor_go = 0;
    		encoder_ticks = 6400;
    	} else if(UART_rx_buffer[0] == 'c') {
    		is_motor_go = 0;
    		encoder_ticks = 3200;
    	} else if(UART_rx_buffer[0] == 'd') {
    		is_motor_go = 0;
    		encoder_ticks = 1600;
    	} else if(UART_rx_buffer[0] == 'e') {
    		is_motor_go = 0;
    		encoder_ticks = 800;
    	} else if(UART_rx_buffer[0] == 'x') {
    		is_motor_go = 1;
    		motor_speed = 100;
    	} else if(UART_rx_buffer[0] == 'y') {
    		is_motor_go = 1;
    		motor_speed = 50;
    	} else if(UART_rx_buffer[0] == 'z') {
    		is_motor_go = 1;
    		motor_speed = 25;
    	}
    	else if(UART_rx_buffer[0] == 's') {
    		is_motor_stop = 1;
    	} else {
    		is_motor_go = 0;
    		encoder_ticks = 0;
    	}




    	if( UART_rx_buffer[1] == '0'){
    		direction = 0;
    	} else if( UART_rx_buffer[1] == '1') {
    		direction = 1;
    	}
    	else {
    		direction = 0;
    	}

    	if(is_motor_stop) {
    		motor_stop();
    	}
    	else {
			if(is_motor_go && (UART_rx_counter == 1)){
				motor_go(motor_speed,direction);
			}
			else {

				if((encoder_ticks > 0 ) && (UART_rx_counter == 1)) {
					move_motor(encoder_ticks, direction);
				}
			}
    	}

    	encoder_ticks = 0;
    	direction =0;

    }



    if(UART_rx_counter == 1){
    	UART_rx_counter = 0;
    } else {
    	GPIOC->ODR ^= GPIO_ODR_9;
    	UART_rx_counter ++;
    }


		/* if the last character received is the LF ('\r' or 0x0a) character OR if the UART_RX_BUFFER_LENGTH (40) value has been reached ...*/
//    if((UART_rx_counter + 1 == UART_RX_BUFFER_LENGTH) || (UART_rx_buffer[UART_rx_counter] == 0x0a))
//    {
//      new_UART_msg = 1;
//			for(x=0; x <= UART_rx_counter; x++)											//copy each character in the UART_rx_buffer to the UART_msg variable
//				UART_msg[x] = UART_rx_buffer[x];
//			UART_msg[x-1] = '\0';																		//terminate with NULL character
//      memset(UART_rx_buffer, 0, UART_RX_BUFFER_LENGTH);				//clear UART_rx_buffer
//      UART_rx_counter = 0;
//    }
//    else
//    {
//			UART_rx_counter++;
//    }
  }
}

//used to send a string of information
void UART_PutStr(char *str)
{
	unsigned int i=0;

	do
	{
		UART_PutChar(str[i]);
		i++;
	}while(str[i]!='\0');
}



//used to send characters
void UART_PutChar(unsigned char ch)
{
	/* Put character on the serial line */ //I don't know if this will wrk
	USART_SendData(USART2, (ch & (uint16_t)0x01FF));
	/* Loop until transmit data register is empty *///this should work
	while( !(USART2->ISR & 0x00000040) );
}

/*********************************************************************************************
probablly don't want to use this as brent gave us a better delay function
*********************************************************************************************/
void UART_Delay(uint32_t delay)
{
	uint32_t x;

	for(x=0; x<delay; x++)
	{
		//do nothing
	}
}
/****
 *  this sets the default pin name as well. did not use

 *
 */

uint8_t UART_SetName(char *name)
{
	uint32_t timeout = UART_TIMEOUT_MAX;
	char buf[20];

	UART_ClearRxBuffer();																//clear rx buffer

	if(strlen(name) > 13)																//error - name more than 20 characters
		return 0x01;

	UART_PutStr(buf);																		//AT command for SET NAME

	while(UART_rx_counter < 9)													//wait for "OKsetname" message, i.e. 9 chars
	{
		timeout--;
		UART_Delay(1000);																	//wait +/- 100us just to give interrupt time to service incoming message
		if (timeout == 0)
			return 0x02;																		//if the timeout delay is exeeded, exit with error code
	}
	if(strcmp(UART_rx_buffer, "OKsetname") == 0)
		return 0x00;																			//success
	else
		return 0x03;																			//unknown return AT msg from UART
}


/***
 * sets the default pin name
 *
 *
 */
uint8_t UART_SetPin(char *pin)
{
	uint32_t timeout = UART_TIMEOUT_MAX;
	char buf[20];

	UART_ClearRxBuffer();																//clear rx buffer

	if((strlen(pin) < 4) || (strlen(pin) > 4))
		return 0x01;																			//error - too few or many characetrs in pin

	UART_PutStr(buf);																		//AT command for SET PIN

	while(UART_rx_counter < 8)													//wait for "OKsetpin" message, i.e. 8 chars
	{
		timeout--;
		UART_Delay(1000);																	//wait +/- 100us just to give interrupt time to service incoming message
		if (timeout == 0)
			return 0x02;																		//if the timeout delay is exeeded, exit with error code
	}
	if(strcmp(UART_rx_buffer, "OKsetPIN") == 0)
		return 0x00;																			//success
	else
		return 0x03;																			//unknown return AT msg from UART
}

