/*
 * motor.c
 *
 *  Created on: Mar 21, 2016
 *      Author: wiblack
 */


/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
#include "motor.h"

volatile uint8_t Kp=1;            // Proportional gain
volatile uint8_t Ki=1;    // Set default integral gain

// Sets up the entire motor drive system
void motor_init(void) {
	gear_position = 0;
    pwm_init();
    encoder_init();
    ADC_init();
    RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    GPIOD->MODER &= ~(3<<2); //genearl purpose input mode for pd2
    GPIOD->PUPDR |= (1<<2); // put it into pull-up mode since encoder is active high.

}

//under no circumstances change the direction of the motor while it is moving.
void motor_go(uint8_t _target_rpm, uint8_t _dir) {

	if(_dir == 0){
			GPIOC->ODR |= (1 << 11);  // Set PA4 to high
			GPIOC->ODR &= ~(1 << 12); // Set PA5 to low

		}
		else{
			GPIOC->ODR &= ~(1 << 11);  // Set PA4 to low
			GPIOC->ODR |= (1 << 12); // Set PA5 to high
		}

	dir = _dir;

	target_rpm = _target_rpm;

}

void motor_stop(void) {


	//May want to add some efficient stoping.
	int temp_rpm = target_rpm;
	target_rpm = 0;

	if(temp_rpm < 50){
		apply_electronic_break();
	}

}




void move_motor(int16_t encoder_ticks, uint8_t _dir){


	target_rpm = 0;
	motor_ticks = 0;// intitialize the motor ticks to 0.
	dir = _dir;//set the global direction to the new one.

	if(dir == 0){
		GPIOC->ODR |= (1 << 11);  // Set PA4 to high
		GPIOC->ODR &= ~(1 << 12); // Set PA5 to low

	}
	else{
		GPIOC->ODR &= ~(1 << 11);  // Set PA4 to low
		GPIOC->ODR |= (1 << 12); // Set PA5 to high
	}

	set_initial_target_rpm(encoder_ticks);

	int16_t half_encoder_ticks = encoder_ticks/2; //get half of the encoder ticks.
	halved_ticks = 0;



	overshot = 0;
	while(!overshot){

		if(dir == 0){
			if(halved_ticks >= half_encoder_ticks){
				if(target_rpm > 30){
					halved_ticks = 0;
					half_encoder_ticks = half_encoder_ticks >> 1; //reduces the new half way point by 2.
					target_rpm = target_rpm/2 ; //reduce the target rpm by half
				}
			else{

						halved_ticks = 0;
						target_rpm = 20;
				}
			}

			if(motor_ticks >= ((int16_t)(.9 * encoder_ticks))){
				overshot = 1;
			}
		}
		else{

				if((~(halved_ticks)+1) >= half_encoder_ticks){

					if(target_rpm > 30){
						halved_ticks = 0;
						half_encoder_ticks = half_encoder_ticks /2 ;
						target_rpm = target_rpm >> 1 ;
					}
					else{
						halved_ticks = 0;
						target_rpm = 20;
					}
				}

			if((~(motor_ticks)+1) >= ((int16_t)(.9 * encoder_ticks))){
				overshot = 1;
			}

		}
	}


	target_rpm = 0;
	apply_electronic_break();
//	delay_ms(500);//wait a second.
//		if(dir == 0){
//				dir = 1;
//				GPIOC->ODR &= ~(1 << 11);  // Set PA4 to low
//				delay_ms(1);
//				GPIOC->ODR |= (1 << 12);
//				target_rpm = 5;
//
//			}
//			else{
//				dir = 0;
//				//Set PA4 to high
//				GPIOC->ODR |= (1 << 11);
//				delay_ms(1);
//				GPIOC->ODR &= ~(1 << 12);
//				// Set PA5 to low
//				target_rpm = 5;
//			}
//
//		overshot = 0;
//		while(overshot != 1){
//			if(dir == 1){
//				if(motor_ticks <= (encoder_ticks)){
//						overshot = 1;
//					}
//			}
//			else{
//
//				if((~(motor_ticks)+1) <= encoder_ticks){
//								overshot = 1;
//				}
//			}
//
//		}
//
//	motor_ticks = 0;
//	target_rpm = 0;
//	apply_electronic_break();

}

void pi_move_motor(int16_t encoder_ticks, uint8_t _speed, uint8_t _dir){

	target_rpm = 0;
	motor_ticks = 0;// Initialize the motor ticks to 0.
	dir = _dir;//set the global direction to the new one.

	if(dir == 0){
		GPIOC->ODR |= (1 << 11);  // Set PA4 to high
		GPIOC->ODR &= ~(1 << 12); // Set PA5 to low

	}
	else{
		GPIOC->ODR &= ~(1 << 11);  // Set PA4 to low
		GPIOC->ODR |= (1 << 12); // Set PA5 to high
	}

	if(_speed > 200) {
		target_rpm = 200;
	} else
		target_rpm = _speed;

	overshot = 0;
		while(!overshot){

			if(dir == 0){

				if(motor_ticks >= (encoder_ticks)){
					overshot = 1;
				}

			}
			else{

				if((~(motor_ticks)+1) >= encoder_ticks){
					overshot = 1;
				}
			}
		}
	target_rpm = 0;


	motor_stop();



}

void set_initial_target_rpm(int16_t encoder_ticks){
	if(encoder_ticks > 6400){
		target_rpm = max_rpm;
	}
	else if(encoder_ticks <= 6400 && encoder_ticks>3200){
		target_rpm = max_rpm/2;
	}
	else if(encoder_ticks <= 3200 && encoder_ticks >1600){
		target_rpm = max_rpm/4;
	}
	else{
		target_rpm = 20;
	}


}

void apply_electronic_break(void){
	GPIOC ->ODR |= (1<<11);  //Set PA4 to high to set direction pins to same high value.
	GPIOC->ODR |= (1 << 12);
}


// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void) {

    /// TODO: Set up a pin for H-bridge PWM output (TIMER 14 CH1)
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |=GPIO_MODER_MODER4_1;//pa4
	GPIOA->AFR[0] |= (4<<16);

    /// TODO: Set up a few GPIO output pins for direction control
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  // Set up the c gpio to be used, turn on the clock for C gpio.
	GPIOC->MODER |=GPIO_MODER_MODER11_0; // Set the PA4 to general purpose output mode
	GPIOC->MODER |=GPIO_MODER_MODER12_0; // Set the PA5 to general purpose output mode


    /// TODO: Initialize one direction pin to high, the other low

	GPIOC->ODR |= (1 << 11);  // Set Pc4 to high
	GPIOC->ODR &= ~(1 << 12); // Set Pc5 to low


    /* Hint: These pins are processor outputs, inputs to the H-bridge
     *       they can be ordinary 3.3v pins.
     *       If you hook up the motor and the encoder reports you are
     *       running in reverse, either swap the direction pins or the
     *       encoder pins. (we'll only be using forward speed in this lab)
     */

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->CR1 = 0;                         // Clear control registers
    TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM14->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM14->PSC = 1;                         // Run timer on 24Mhz
    TIM14->ARR = 1200;                      // PWM at 20kHz
    TIM14->CCR1 = 0;                        // Start PWM at 0% duty cycle

    TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty) {
    if(duty <= 100) {
        TIM14->CCR1 = ((uint32_t)duty*TIM14->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

void calibrate(){

	if(((GPIOD->IDR >> 2) & 0X1)==0){
		GPIOC->ODR ^= GPIO_ODR_8;
		gear_position = 0;
		return ;
	}
	volatile uint8_t findingNorth = 1;

	motor_go(20,0); // Start the motor moving at 10 rpm
	while(findingNorth) {

		if(((GPIOD->IDR >> 2) & 0X1)==0){
			motor_stop();
			findingNorth  = 0;
			gear_position = 0;
			 GPIOC->ODR ^= GPIO_ODR_8;

		}
	}


}

void go_to_quadrant(uint8_t quadrant){
	int16_t quadrant_ticks = quadrant * 1600;//This gives the gear position we are looking for.
	int16_t to_move = gear_position - quadrant_ticks;
	if((quadrant > 7) || (quadrant < 0)) {
		return;
	}

	//First check to see if this is Case I or Case II

	int aTicks = -1;
	int bTicks = -1;
	if (gear_position < quadrant_ticks) {
		//Case I
		//Calculate both paths
		aTicks = gear_position + (12799 - quadrant_ticks);
		bTicks = quadrant_ticks - gear_position;


	} else {
		//case II
		aTicks = gear_position - quadrant_ticks;
		bTicks = (12799 - gear_position) + quadrant_ticks;

	}
	if (aTicks > bTicks) {

		move_motor(bTicks,0);
	} else {

		move_motor(aTicks,1);
	}
}

void reset_motor(){

	go_to_quadrant(0);

}

// Sets up encoder interface to read motor speed
void encoder_init(void) {

    /// TODO: Set up encoder input pins (TIMER 3 CH1 and CH2)
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER6_1;//Set pc6 to alternate function mode  Sets it to tim3_ch1
	GPIOC->MODER |= GPIO_MODER_MODER7_1;//Set PC7 to alternate function mode. Sets it to tim3_ch2
	//Don't need to set the alternate function mode they both only apply to tim3 channels


    /* Hint: MAKE SURE THAT YOU USE 5V TOLERANT PINS FOR THE ENCODER INPUTS!
     *       You'll fry the processor otherwise, read the lab to find out why!
     */

    // Set up encoder interface (TIM3 encoder input mode)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CCMR1 = 0;    //Clear control registers
    TIM3->CCER = 0;
    TIM3->SMCR = 0;
    TIM3->CR1 = 0;

    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM3->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM3->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM3->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    TIM3->CR1 |= TIM_CR1_CEN;                               // Enable timer


    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    /// TODO: Select PSC and ARR values that give an appropriate interrupt rate

    /* Hint: See section in lab on sampling rate!
     *       Recommend choosing a sample rate that gives 2:1 ratio between encoder value
     *       and target speed. (Example: 200 RPM = 400 Encoder count for interrupt period)
     *       This is so your system will match the lab solution
     */
    TIM6->PSC = 47; // Change this!
    TIM6->ARR = 37500; // Change this!

    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(17);          // Enable interrupt in NVIC
    NVIC_SetPriority(17,1);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {
    /* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
    motor_speed = (TIM3->CNT - 0x7FFF);
    TIM3->CNT = 0x7FFF; // Reset back to center point



    // Call the PI update function
    PI_update();


    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}



void ADC_init(void) {

    /// TODO: Configure a pin for ADC input (used for current monitoring)
	 RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	 RCC->APB2ENR |=RCC_APB2ENR_ADCEN;
	 GPIOB->MODER |= GPIO_MODER_MODER0;//This sets PB0 to a analog pin.

    // Configure ADC to 8-bit continuous-run mode, (asynchronous clock mode)
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 = 0;
    ADC1->CFGR1 |= (ADC_CFGR1_CONT);        // Set to continuous mode and 12-bit resolution
    //already set to 12 bit resolution mode.

    /// TODO: Enable the proper channel for the ADC pin you are using
    ADC1->CHSELR |= 1<<8; // Change this!     Changed this to channel 8 for PB0

    ADC1->CR = 0;
    ADC1->CR |= ADC_CR_ADCAL;               // Perform self calibration
    while(ADC1->CR & ADC_CR_ADCAL);         // Delay until calibration is complete

    ADC1->CR |= ADC_CR_ADEN;                // Enable ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY));    // Wait until ADC ready
    ADC1->CR |= ADC_CR_ADSTART;             // Signal conversion start
}

void PI_update(void) {

    /* Run PI control loop
     *
     * Make sure to use the indicated variable names. This allows STMStudio to monitor
     * the condition of the system!
     *
     * target_rpm -> target motor speed in RPM
     * motor_speed -> raw motor speed in encoder counts
     * error -> error signal (difference between measured speed and target)
     * error_integral -> integrated error signal
     * Kp -> Proportional Gain
     * Ki -> Integral Gain
     * output -> raw output signal from PI controller
     * duty_cycle -> used to report the duty cycle of the system
     * adc_value -> raw ADC counts to report current
     *
     */

    /// TODO: calculate error signal and write to "error" variable
	if(dir == 0)
		error= (2*target_rpm)-motor_speed;
	else{
		error = (2*target_rpm)+motor_speed;
	}

	    //Do this to keep track of the total ticks.
		motor_ticks = motor_ticks + motor_speed;
		halved_ticks = halved_ticks + motor_speed;

		gear_position = gear_position + motor_speed;// ad the moto speed

		if(gear_position < 0){
			gear_position = 12800 - ((~gear_position)+1);
		}
		gear_position = gear_position % 12800;


    /* Hint: Remember that your calculated motor speed may not be directly in RPM!
     *       You will need to convert the target or encoder speeds to the same units.
     *       I recommend converting to whatever units result in larger values, gives
     *      more resolution.
     */

    /// TODO: Calculate integral portion of PI controller, write to "error_integral" variable
	error_integral= (Ki * error) + error_integral;

    /// TODO: Clamp the value of the integral to a limited positive range
	if( error_integral >= 3200)
		error_integral = 3200;   	// Set the integral error to the max
	else if(error_integral <= 0)
		error_integral = 0;			// Set the integral error to the min


    /* Hint: The value clamp is needed to prevent excessive "windup" in the integral.
     *       You'll read more about this for the post-lab. The exact value is arbitrary
     *       but affects the PI tuning.
     *       Recommend that you clamp between 0 and 3200 (what is used in the lab solution)
     */

    /// TODO: Calculate proportional portion, add integral and write to "output" variable


    int16_t output = Kp*error + error_integral;

    /* Because the calculated values for the PI controller are significantly larger than
     * the allowable range for duty cycle, you'll need to divide the result down into
     * an appropriate range. (Maximum integral clamp / X = 100% duty cycle)
     *
     * Hint: If you chose 3200 for the integral clamp you should divide by 32 (right shift by 5 bits),
     *       this will give you an output of 100 at maximum integral "windup".
     *
     * This division also turns the above calculations into pseudo fixed-point. This is because
     * the lowest 5 bits act as if they were below the decimal point until the division where they
     * were truncated off to result in an integer value.
     *
     * Technically most of this is arbitrary, in a real system you would want to use a fixed-point
     * math library. The main difference that these values make is the difference in the gain values
     * required for tuning.
     */

     /// TODO: Divide the output into the proper range for output adjustment
    output = output >> 5;


     /// TODO: Clamp the output value between 0 and 100
    if(output >= 100)
    	output = 100;	// set output to max
    else if(output <= 0)
    	output = 0;		// set output to min

    pwm_setDutyCycle(output);
    duty_cycle = output;            // For debug viewing

    // Read the ADC value for current monitoring, actual conversion into meaningful units
    // will be performed by STMStudio
    if(ADC1->ISR & ADC_ISR_EOC) {   // If the ADC has new data for us
        adc_value = ADC1->DR;       // Read the motor current for debug viewing
    }
}
