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
