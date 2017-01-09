/*
 * miscellaneous.c
 *
 *  Created on: 16.03.2012
 *      Author: gageik
 */

#ifndef MISCELLANEOUS_C_
#define MISCELLANEOUS_C_

#include "QUAD_TWI_6DOF.h"

unsigned int jitter_counter;
extern unsigned int tc_ticks;
extern unsigned int last_Sample_time;
extern char debug_line[DEBUG_BUFFER_SIZE];

double winkel = 0.2;


volatile double lz_test1,lz_test2;

void SetProcessorFrequency(volatile avr32_pm_t* pm)
// Set CPU to 60MHz
{
  pm_switch_to_osc0(pm, FOSC0, OSC0_STARTUP);
  pm_pll_setup(pm,  0, 9, 1,  0, 16);
  pm_pll_set_option(pm, 0, 1, 1, 0);
  pm_pll_enable(pm,0);
  pm_wait_for_pll0_locked(pm) ;
  pm_cksel(pm, 0, 0, 0, 0, 0, 0);
  //pm_cksel(pm, 1, 0, 0, 0, 0, 0);
  flashc_set_wait_state(1);
  pm_switch_to_clock(pm, AVR32_PM_MCCTRL_MCSEL_PLL0);
}


void jitter_debug(){

	if (tc_ticks - last_Sample_time > SAMPLE_TIME)
					jitter_counter++;


			 if (jitter_counter == 10){
					 USART_schreiben("10th Lost Sample");
						jitter_counter++;
			 }

}

#endif /* MISCELLANEOUS_C_ */
