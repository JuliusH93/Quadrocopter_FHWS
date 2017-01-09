/*
 * TC_NG.h
 *
 *  Created on: 04.08.2011
 *      Author: gageik
 */

#ifndef TC_NG_H_
#define TC_NG_H_

// TC Defines
#define FPBA    					FOSC0
#define TC_CHANNEL    				0

void myTC_init(void);
void wait_i (unsigned int j);
int wait_ms(unsigned int wait);

#endif /* TC_NG_H_ */
