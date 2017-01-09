/*
 * DIP_NG.h
 *
 *  Created on: 04.08.2011
 *      Author: gageik
 */

#ifndef DIP_NG_H_
#define DIP_NG_H_

#ifdef DISPLAY_ON
void myDIP_init(void);
void renew_display();
void setDisplay(void);
#endif

void init_button_irq(void);

#endif /* DIP_NG_H_ */
