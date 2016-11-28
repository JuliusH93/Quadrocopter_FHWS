/*
 *  TC_NG.h
 *
 *  Created on: 04.08.2011
 *      Author: gageik
 */
// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011 Nils Gageik

// This Software uses the AVR Framework:
// Copyright (c) 2009 Atmel Corporation. All rights reserved.

// ******************************************
// *		TC driver						*
// ******************************************
#ifndef TC_NG_H_
#define TC_NG_H_

// TC Defines
#define FPBA    					FOSC0
#define TC_CHANNEL    				0

void myTC_init(void);
void wait_i (unsigned int j);
int wait_ms(unsigned int wait);

#endif /* TC_NG_H_ */
