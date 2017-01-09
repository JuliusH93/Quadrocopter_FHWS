/*
 * QUAD_TWI_6DOF.h
 *
 *  Created on: 16.03.2012
 *      Author: gageik
 */

#ifndef QUAD_TWI_6DOF_H_
#define QUAD_TWI_6DOF_H_

// General Includes (Standard)
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

// HW Includes (Framework)
#include <avr32/io.h>
#include "board.h"
#include "compiler.h"
#include "dip204.h"
#include "intc.h"
#include "tc.h"
#include "gpio.h"
#include "pm.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "spi.h"
#include "pwm.h"
#include "flashc.h"
#include "power_clocks_lib.h"
#include "cycle_counter.h"
#include "pdca.h"


// Overall/General Defines
#include "basics.h"
#include "topics.h"

// Sourcefiles
#include "myMath.h"
#include "Beeper+LED.h"
#include "Miscellaneous.h"

#include "Control\BLCTRL_NG.h"

#include "IMU\ITG3200.h"
#include "IMU\ADXL345.h"
#include "IMU\LSM303DLM.h"
#include "IMU\L3G4200D.h"
#include "IMU\MPU6000.h"
#include "IMU\IMU_Selector.h"

// Drivers for Interfaces and HW
#include "Drivers\DIP_NG.h"
#include "Drivers\TC_NG.h"
#include "Drivers\TWI_NG.h"
#include "Drivers\ADC_NG.h"
#include "Drivers\USART_NG.h"				// Header for both: USART_PDCA.c and USART_NG.c (old, obsolete version)

#endif /* QUAD_TWI_6DOF_H_ */
