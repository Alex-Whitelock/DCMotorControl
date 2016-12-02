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
uint8_t priorityLevel = 2;


void UART_Init(uint32_t speed){


	/* USARTx configured as follow:
						- BaudRate = speed parameter above
						- Word Length = 8 Bits
						- One Stop Bit
						- No parity
						- Hardware flow control disabled (RTS and CTS signals)
						- Receive and transmit enabled
	*/
	isArmed = 0;
	is_stm_controlled = 0;
	is_in_ScanningMode = 1;//Start in scanning mode.

	RCC->AHBENR|=RCC_AHBENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->CR1 =0;//RESET AND ENSURE THAT WE  are working with 8 bits
	//also parity control is disabled. and in even parity
		//PA3_is the usart2_RX data input
		//PA2 is the usart2_TX data output
	//GPIOA->MODER |= 1<<5;//Enable 2 for alternate function mode for e pa2
	//GPIOA->MODER |= 1<<7;//ENABLE PA3 for alternate function mode.

	GPIOA-> MODER |= 1<<29; //ENABLE PA14 For alternate function mode.
	GPIOA-> MODER |= 1<<31;//ENABLE PA15 for alternate function mode.

	GPIOA->PUPDR |= (2<<28); //set pa14 into pull down mode so pi doesn't get random interupts.
	GPIOA->PUPDR |= (2<<30); // set pa15 in

	//GPIOA->AFR[0] |= (1<<8);//choose AF1 for pin 2
	//GPIOA ->AFR[0] |= (1<<12);//choose af1 for pin 3

	//need to set this up to be af1 in afr high register.
	GPIOA->AFR[1]  |= (1<<24); //put pa14 into af1
	GPIOA -> AFR[1] |= (1 << 28 );// put pa15 into af1

	//send a clock to the usart register

	USART2->CR1|=1<<5;
	USART2->CR1 &= ~USART_CR1_RE;//reciever disable
	USART2->CR1 &= ~USART_CR1_TE;//transmitter diable
	//USART2->CR1|=USART_CR1_UE;//usart enable

	USART2->BRR = SystemCoreClock/speed;

	//USART2->CR1 &= ~(1<<2);

//	NVIC_EnableIRQ(USART2_IRQn);
//	NVIC_SetPriority(USART2_IRQn, priorityLevel); // Set to medium priority
	//isArmed=0;
	//is_stm_controlled = 1; //When the stm board is turned on the initial stm control should be turned on.

	}

void activate_USART(void){
	USART2->CR1 |= USART_CR1_RE; //rx enable
	USART2->CR1 |= USART_CR1_TE; //tx enable
	USART2->CR1|=USART_CR1_UE;//usart enable
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn,priorityLevel);


}

//This is the interupt handler for the USART
void USART2_IRQHandler(void)
{
	uint16_t  encoder_ticks = 0;
	uint8_t encoder_ticks_high = 0;
	uint16_t encoder_ticks_low =0;
	uint8_t direction = 0;
	uint8_t quadrant = 0;
	uint8_t motor_speed =0;
	uint8_t pir_information = 0;
	uint8_t gear_high = 0;
	uint8_t gear_low = 0;
	char instruction[12];

	NVIC_DisableIRQ(USART2_IRQn);

  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
		/* Read one byte from the receive data register */
    UART_rx_buffer[UART_rx_counter] = USART_ReceiveData(USART2);

    //Once we get to this part we need to decide which kind of instruction we are doing.
    if(UART_rx_counter == 4){
    	GPIOC->ODR ^= GPIO_ODR_9;

    	if(UART_rx_buffer[0] == 1){
    		//ask for PIR information.
    		pir_information = get_pir_information();
    		gear_high = (gear_position>>8) & 0xff;
    		gear_low = gear_position & 0xff;

    		instruction[0] = 1;

    		if(pir_information == 0){
    			pir_information = 0xf0;
    			instruction[1] = pir_information;
    		} else
    			instruction[1] = pir_information;
    		if(gear_high == 0){
    			instruction[2] = 0xff;
    		} else
    			instruction[2] = gear_high;
    		if(gear_low == 0) {
    			instruction[3] = 0xff;
    			instruction[4] = 0x02;
    		} else{
    			instruction[3] = gear_low;
    			instruction[4] = 0x01; //Tell the pi that it is 255 instead of zero.
    		}

    		instruction[5] = '\0';
    		UART_PutStr(instruction);

    		//UART_PutStr("Hello from 1\0");

    	} else if(UART_rx_buffer[0] == 2) {
    		//transfer total control to pi. Ensure that the  motor does not turn due to
    		//pir sensors.
    		isArmed = 0;
    		is_stm_controlled = 0;//make it so that the stm is not controlled.
    		set_STM_cotrolled(0);
    		//UART_PutStr("Hello from 2\0");

    	} else if(UART_rx_buffer[0] == 3) {
    		//This will move the motor.
    		if(is_stm_controlled == 0){
    			encoder_ticks = 0;//Zero out the encoder_ticks to prep for bit manipulation
    			encoder_ticks_high = UART_rx_buffer[1] & 0xff;
    			encoder_ticks_low = UART_rx_buffer[2] & 0xff;
    			encoder_ticks = (encoder_ticks_high << 8) | encoder_ticks_low;
    			direction = UART_rx_buffer[3];

    			move_motor(encoder_ticks, direction);
    		}

    		instruction[0] = 2;
    		instruction[1] = 0xff;
    		instruction[2] = 0xff;
    		instruction[3] = 0xff;
    		instruction[4] = 0x02;
    		instruction[5] = '\0';
    		//tell the stm that the last value is 0 not 255 doesn't really matter for this instruction though

    		//This tell the pi that it now has control of the motor again.
    		UART_PutStr(instruction);

    		//UART_PutStr("Hello from 3\0");

    	} else if(UART_rx_buffer[0] == 4) {
    		//transfer total control from pi to stm.

    		set_STM_cotrolled(1);

    		//UART_PutStr("Hello from 4\0");
    	} else if(UART_rx_buffer[0] == 5) {
    		// set the motor speed
    		if(is_stm_controlled == 0){
				motor_speed = UART_rx_buffer[1] & 0xff;
				if(motor_speed > 200){
					motor_speed = 0; //could probably get rid of this.
				} else {
					//move motor. Be sure to ask
					direction = UART_rx_buffer[2] & 0xff;
					motor_go(motor_speed, direction);
				}
    		}

    		//UART_PutStr("Hello from 5\0");
    	} else if(UART_rx_buffer[0] == 6) {
    		//reset method goes here.
    		if(is_stm_controlled == 0){
    			if(UART_rx_buffer[2] == 1){
    				reset_motor();
    			} else if(UART_rx_buffer[2] == 0){
    				quadrant = UART_rx_buffer[1] & 0xff;
    				go_to_quadrant(quadrant);

    			} else{
    				//no-op
    			}

    			instruction[0] = 2;
				instruction[1] = 0xff;
				instruction[2] = 0xff;
				instruction[3] = 0xff;
				instruction[4] = 0x02;
				instruction[5] = '\0';


				UART_PutStr(instruction); // Put the strin

    		}
    		//UART_PutStr("Hello from 6\0");
    	} else if(UART_rx_buffer[0] == 7) {
    		//set a stop command.
    		motor_stop();

    		//UART_PutStr("Hello from 7\0");
    	} else if(UART_rx_buffer[0] == 8) {
    		if(is_stm_controlled == 0) {

    			if(is_stm_controlled == 0){
					encoder_ticks_high = UART_rx_buffer[1] & 0xff;
					encoder_ticks_low = UART_rx_buffer[2] & 0xff;
					encoder_ticks = (encoder_ticks_high << 8) | encoder_ticks_low;
					direction = UART_rx_buffer[4] & 0x1;
					motor_speed = UART_rx_buffer[3] & 0xff;
					pi_move_motor(encoder_ticks, motor_speed, direction);
    			}



				instruction[0] = 2;
				instruction[1] = 0xff;
				instruction[2] = 0xff;
				instruction[3] = 0xff;
				instruction[4] = 0x02;
				instruction[5] = '\0';

				UART_PutStr(instruction); // Put the strin

    		}

    	} else if(UART_rx_buffer[0] == 9) {
    		//turn on scanning mode.

    		if(UART_rx_buffer[1] == 1){
    			is_in_ScanningMode = 1;

    		} else
    			is_in_ScanningMode = 0;

    	} else {
    		//no-op
    	}

    }



    if(UART_rx_counter >= 4){
    	UART_rx_counter = 0;
    } else {
    	//GPIOC->ODR ^= GPIO_ODR_9;
    	UART_rx_counter ++;
    }

    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, priorityLevel);


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

void set_STM_cotrolled(uint8_t set){
	if(set == 1){
		isArmed = 1;
		is_stm_controlled = 1;
	} else if( set ==0) {
		isArmed =0;
		is_stm_controlled = 0;
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
	}while(str[i]!='\0'); //Wait  <-- this is why you can't use Zeros? if all instructions are set at 5 or 4 bytes as they were why not just loop that known amount of times??
}



//used to send characters
void UART_PutChar(unsigned char ch)
{
	/* Put character on the serial line */ //I don't know if this will work
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

