/*
 * DIP_NG.c
 *
 *  Created on: 17.10.2011
 *      Author: gageik
 */
// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011 Nils Gageik

#include "QUAD_TWI_6DOF.h"

extern int tc_ticks;

unsigned int last_led_toggle_time = 0;
unsigned int last_beep_toggle_time = 0;


void toggle_led(){

	if (tc_ticks - last_led_toggle_time > 500){
		last_led_toggle_time = tc_ticks;
		gpio_tgl_gpio_pin(RUNTIME_LED);
	}

}
void beep(){
	if (tc_ticks - last_beep_toggle_time > 500){
			last_beep_toggle_time = tc_ticks;
			gpio_tgl_gpio_pin(BEEPER_PIN);
		}
}

void beep_off(){
	gpio_clr_gpio_pin(BEEPER_PIN);

}
/*
void beeper_alarm_on(){
	gpio_set_gpio_pin(BEEPER_PIN);
}

void beeper_alarm_off(){
	gpio_clr_gpio_pin(BEEPER_PIN);
}
*/
