/*
 * Beeper+LED.h
 *
 *  Created on: 17.10.2011
 *      Author: gageik
 */

#ifndef BEEPERLED_H_
#define BEEPERLED_H_

#include "basics.h"

#define BEEPER_PIN		28

void toggle_led(void);
//void beeper_alarm_on(void);
//void beeper_alarm_off(void);
void beep();
void beep_off();

#endif /* BEEPERLED_H_ */
