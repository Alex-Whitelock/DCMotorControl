/*
 * MotionControl.h
 *
 *  Created on: Apr 13, 2016
 *      Author: wiblack
 */

#ifndef MOTIONCONTROL_H_
#define MOTIONCONTROL_H_

#include "stm32f0xx.h"
#include <stdlib.h>

void motion_init();
//void move_motor(int steps,int direction);
//void EXTI4_15_IRQHandler(void);
void sense_motion();
void motor_reset();

#endif /* MOTIONCONTROL_H_ */


static const uint16_t NORTH =0;
static const uint16_t NORTH_EAST=1600;
static const uint16_t EAST =3200;
static const uint16_t SOUTH_EAST= 4800;
static const uint16_t SOUTH= 6400;
static const uint16_t SOUTH_WEST=8000;
static const uint16_t WEST=9600;
static const uint16_t NORTH_WEST=11200;
