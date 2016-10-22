/*
 * MotionControl.c
 *
 *  Created on: Apr 13, 2016
 *      Author: wiblack
 */




#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f0xx.h"
#include <string.h>
#include "MotionControl.h"
#include "motor.h"
#include "delay.h"

//this is the global variable that keeps track of the motor position.
int MOTOR_POSITION;

//these are a bunch of constants that are used to define motor position.
//we were using a semi stepper motor driver.
//static const uint16_t NORTH =0;
//static const uint16_t NORTH_EAST=200;
//static const uint16_t EAST =400;
//static const uint16_t SOUTH_EAST= 600;
//static const uint16_t SOUTH= 800;
//static const uint16_t SOUTH_WEST=1000;
//static const uint16_t WEST=1200;
//static const uint16_t NORTH_WEST=1400;
static const uint16_t NORTH =0;
static const uint16_t NORTH_EAST=1600;
static const uint16_t EAST =3200;
static const uint16_t SOUTH_EAST= 4800;
static const uint16_t SOUTH= 6400;
static const uint16_t SOUTH_WEST=8000;
static const uint16_t WEST=9600;
static const uint16_t NORTH_WEST=11200;
//MOTOR_POSITIONS;
//0:North;
//200: North East
//400: East
//600: South East
//800: South
//1000: South West
//1200: West
//1400: North West

void motion_init(){
	// initialize all of the motion controls
	MOTOR_POSITION=0;

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB2ENR|= RCC_APB2ENR_SYSCFGCOMPEN;
	//GPIOB->MODER |=
	GPIOB ->MODER &= ~(GPIO_MODER_MODER4|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7 );//Put all of the motions sensors
  // GPIOB->MODER |=(GPIO_MODER_MODER4_1 |GPIO_MODER_MODER5_1 |GPIO_MODER_MODER6_1|GPIO_MODER_MODER7_1  );
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR4_1|GPIO_PUPDR_PUPDR5_1|GPIO_PUPDR_PUPDR6_1|GPIO_PUPDR_PUPDR7_1);

//


}

//This method senses motions and moves the motor accordingly to the right position from wherever it's last postion was

void sense_motion(){
	//Priority Level:
	//north Sensor :1
	// East Sensor: 2
	//South Sensor: 3
	//West Sensor: 4

	//define each sensor from the inputs from their respective pins
	int northSensor = ((GPIOB -> IDR)>>4)&1;
	int eastSensor = ((GPIOB -> IDR)>>5)&1;
	int southSensor = ((GPIOB -> IDR)>>6)&1;
    int westSensor = ((GPIOB -> IDR)>>7)&1;
    GPIOC->ODR ^=(1<<9);//toggle the light for testing purposes.

    if(northSensor && eastSensor){
    	if(MOTOR_POSITION==NORTH){
    		move_motor(1600,0);


    	}
    	else if(MOTOR_POSITION == NORTH_EAST){
    		//do nothing
    	}
    	else if(MOTOR_POSITION==EAST){
    		move_motor(1600,1);//move it counter clock wise
    	}
    	else if(MOTOR_POSITION==SOUTH_EAST){
    		move_motor(3200,1);//move the motor 400 steps counter clockwise
    	}
    	else if(MOTOR_POSITION==SOUTH){
    		move_motor(4800,1);
    	}
    	else if(MOTOR_POSITION==SOUTH_WEST)
    		move_motor(6400,0);
    	else if(MOTOR_POSITION==WEST){
    		move_motor(4800,0);
    	}
    	else if(MOTOR_POSITION==NORTH_WEST)
    		move_motor(3200,0);


    	MOTOR_POSITION=NORTH_EAST;


    }
    else if(eastSensor && southSensor){

    			if(MOTOR_POSITION==NORTH){
    	    		move_motor(4800,0);


    	    	}
    	    	else if(MOTOR_POSITION == NORTH_EAST){
    	    		move_motor(3200,0);
    	    	}
    	    	else if(MOTOR_POSITION==EAST){
    	    		move_motor(1600,0);//move it counter clock wise
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH_EAST){
    	    		//do nothing
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH){
    	    		move_motor(1600,1);
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH_WEST)
    	    		move_motor(3200,1);
    	    	else if(MOTOR_POSITION==WEST){
    	    		move_motor(4800,1);
    	    	}
    	    	else if(MOTOR_POSITION==NORTH_WEST)
    	    		move_motor(6400,0);

    		MOTOR_POSITION=SOUTH_EAST;

    }
    else if(westSensor && southSensor){

    			if(MOTOR_POSITION==NORTH){
    	    		move_motor(600,1);


    	    	}
    	    	else if(MOTOR_POSITION == NORTH_EAST){
    	    		move_motor(6400,0);
    	    	}
    	    	else if(MOTOR_POSITION==EAST){
    	    		move_motor(4800,0);//move it counter clock wise
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH_EAST){
    	    		move_motor(3200,0);
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH){
    	    		move_motor(1600,0);
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH_WEST){
    	    		//move_motor(400,1);
    	    	}
    	    	else if(MOTOR_POSITION==WEST){
    	    		move_motor(1600,1);
    	    	}
    	    	else if(MOTOR_POSITION==NORTH_WEST)
    	    		move_motor(3200,1);

    		MOTOR_POSITION=SOUTH_WEST;

    }
    else if(westSensor && northSensor){

    			if(MOTOR_POSITION==NORTH){
    	    		move_motor(1600,1);


    	    	}
    	    	else if(MOTOR_POSITION == NORTH_EAST){
    	    		move_motor(3200,1);
    	    	}
    	    	else if(MOTOR_POSITION==EAST){
    	    		move_motor(4800,1);//move it counter clock wise
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH_EAST){
    	    		move_motor(6400,0);
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH){
    	    		move_motor(4800,0);
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH_WEST){
    	    		//move_motor(400,1);
    	    		move_motor(3200,0);
    	    	}
    	    	else if(MOTOR_POSITION==WEST){
    	    		move_motor(200,0);
    	    	}
    	    	else if(MOTOR_POSITION==NORTH_WEST){
    	    		//do nothing.
    	    	}

    		MOTOR_POSITION= NORTH_WEST;

    }
    else if( northSensor){

    			if(MOTOR_POSITION==NORTH){
    	    		//move_motor(200,1);


    	    	}
    	    	else if(MOTOR_POSITION == NORTH_EAST){
    	    		move_motor(1600,1);
    	    	}
    	    	else if(MOTOR_POSITION==EAST){
    	    		move_motor(3200,1);//move it counter clock wise
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH_EAST){
    	    		move_motor(4800,1);
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH){
    	    		move_motor(6400,0);
    	    	}
    	    	else if(MOTOR_POSITION==SOUTH_WEST){
    	    		//move_motor(400,1);
    	    		move_motor(4800,0);
    	    	}
    	    	else if(MOTOR_POSITION==WEST){
    	    		move_motor(6400,0);
    	    	}
    	    	else if(MOTOR_POSITION==NORTH_WEST){
    	    		//do nothing.
    	    		move_motor(1600,0);
    	    	}

    		MOTOR_POSITION= NORTH;

    }
    else if( eastSensor){

       			if(MOTOR_POSITION==NORTH){
       	    		move_motor(3200,0);


       	    	}
       	    	else if(MOTOR_POSITION == NORTH_EAST){
       	    		move_motor(1600,0);
       	    	}
       	    	else if(MOTOR_POSITION==EAST){
       	    		//do nothing
       	    	}
       	    	else if(MOTOR_POSITION==SOUTH_EAST){
       	    		move_motor(1600,1);
       	    	}
       	    	else if(MOTOR_POSITION==SOUTH){
       	    		move_motor(3200,1);
       	    	}
       	    	else if(MOTOR_POSITION==SOUTH_WEST){
       	    		//move_motor(400,1);
       	    		move_motor(4800,1);
       	    	}
       	    	else if(MOTOR_POSITION==WEST){
       	    		move_motor(6400,0);
       	    	}
       	    	else if(MOTOR_POSITION==NORTH_WEST){
       	    		//do nothing.
       	    		move_motor(4800,0);
       	    	}

       		MOTOR_POSITION= EAST;

       }

    else if( southSensor){

       			if(MOTOR_POSITION==NORTH){
       	    		move_motor(6400,0);


       	    	}
       	    	else if(MOTOR_POSITION == NORTH_EAST){
       	    		move_motor(4800,0);
       	    	}
       	    	else if(MOTOR_POSITION==EAST){
       	    		move_motor(3200,0);//move it counter clock wise
       	    	}
       	    	else if(MOTOR_POSITION==SOUTH_EAST){
       	    		move_motor(1600,0);
       	    	}
       	    	else if(MOTOR_POSITION==SOUTH){
       	    		//move_motor(400,1);
       	    	}
       	    	else if(MOTOR_POSITION==SOUTH_WEST){

       	    		move_motor(1600,1);
       	    	}
       	    	else if(MOTOR_POSITION==WEST){
       	    		move_motor(3200,1);
       	    	}
       	    	else if(MOTOR_POSITION==NORTH_WEST){
       	    		//do nothing.
       	    		move_motor(4800,1);
       	    	}

       		MOTOR_POSITION= SOUTH;

       }
    else if( westSensor){

       			if(MOTOR_POSITION==NORTH){
       	    		move_motor(3200,1);


       	    	}
       	    	else if(MOTOR_POSITION == NORTH_EAST){
       	    		move_motor(4800,1);
       	    	}
       	    	else if(MOTOR_POSITION==EAST){
       	    		move_motor(6400,0);//move it counter clock wise
       	    	}
       	    	else if(MOTOR_POSITION==SOUTH_EAST){
       	    		move_motor(4800,0);
       	    	}
       	    	else if(MOTOR_POSITION==SOUTH){
       	    		move_motor(3200,0);
       	    	}
       	    	else if(MOTOR_POSITION==SOUTH_WEST){

       	    		move_motor(1600,0);
       	    	}
       	    	else if(MOTOR_POSITION==WEST){
       	    		//move_motor(400,1);
       	    	}
       	    	else if(MOTOR_POSITION==NORTH_WEST){
       	    		//do nothing.
       	    		move_motor(1600,1);
       	    	}

       		MOTOR_POSITION= WEST;

       }

   // delay_ms(3000);

}
//resets the motor to the north position based upon whatever postion it was in last
//This method will have to be changed to a different method once we get the proper sensors. At the moment it should be fine, but I will
//need to work on it.
void motor_reset(){

			if(MOTOR_POSITION==NORTH){
				//move_motor(200,1);


			}
			else if(MOTOR_POSITION == NORTH_EAST){
				move_motor(200,1);
			}
			else if(MOTOR_POSITION==EAST){
				move_motor(400,1);//move it counter clock wise
			}
			else if(MOTOR_POSITION==SOUTH_EAST){
				move_motor(600,1);
			}
			else if(MOTOR_POSITION==SOUTH){
				move_motor(800,0);
			}
			else if(MOTOR_POSITION==SOUTH_WEST){
				//move_motor(400,1);
				move_motor(600,0);
			}
			else if(MOTOR_POSITION==WEST){
				move_motor(400,0);
			}
			else if(MOTOR_POSITION==NORTH_WEST){
				//do nothing.
				move_motor(200,0);
			}

		MOTOR_POSITION= NORTH;

}
