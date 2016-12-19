/*
 * BlCtrl.h
 *
 *  Created on: 09.12.2011
 *      Author: gageik
 */

#ifndef BLCTRL_H_
#define BLCTRL_H_

#define BLCTRL_ENGINE1_TWI_ADDRESS		0x29	// Engine TWI address
#define BLCTRL_ENGINE2_TWI_ADDRESS		0x2A
#define BLCTRL_ENGINE3_TWI_ADDRESS		0x2B
#define BLCTRL_ENGINE4_TWI_ADDRESS		0x2C

//#define BLCTRL_2

void set_engine(MotorControl* Motoren);
void stop_engine();
void engine_init_procedure(void);
void blctrl_init(void);

#endif /* BLCTRL_H_ */
