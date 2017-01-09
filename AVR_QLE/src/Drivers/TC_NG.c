/*
 * myTC.c
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
#include "..\atmel.h"			//Mandantory

// ******************************************
// *		TC driver						*
// ******************************************
#include "..\QUAD_TWI_6DOF.h"

// Timer Counter
volatile extern avr32_tc_t *tc;

// Timer Counter in ms
extern unsigned int tc_ticks;
extern char rollroundtime;


/*! \brief TC interrupt.
 */
__attribute__((__interrupt__))
static void tc_irq(void)
// Increase Timer Counter every mili second
{
  // Increment the ms seconds counter
  tc_ticks = tc_ticks + 1;  		//32bit unsigned int => 1ms = 1 bit => 49,7days run time

  // Clear the interrupt flag. This is a side effect of reading the TC SR.
   tc_read_sr(&AVR32_TC, TC_CHANNEL);
}



void myTC_init(){

	// Timer Configs
	// Options for waveform generation.
	static const tc_waveform_opt_t WAVEFORM_OPT =
	{
	.channel  = TC_CHANNEL,                        // Channel selection.

	.bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
	.beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
	.bcpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOB.
	.bcpb     = TC_EVT_EFFECT_NOOP,                // RB compare effect on TIOB.

	.aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
	.aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
	.acpc     = TC_EVT_EFFECT_NOOP,                // RC compare effect on TIOA: toggle.
	.acpa     = TC_EVT_EFFECT_NOOP,                // RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

	.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
	.enetrg   = FALSE,                             // External event trigger enable.
	.eevt     = 0,                                 // External event selection.
	.eevtedg  = TC_SEL_NO_EDGE,                    // External event edge selection.
	.cpcdis   = FALSE,                             // Counter disable when RC compare.
	.cpcstop  = FALSE,                             // Counter clock stopped with RC compare.

	.burst    = FALSE,                             // Burst signal selection.
	.clki     = FALSE,                             // Clock inversion.
	.tcclks   = TC_CLOCK_SOURCE_TC4                // Internal source clock 3, connected to fPBA / 8.
	};

	static const tc_interrupt_t TC_INTERRUPT =
	{
	.etrgs = 0,
	.ldrbs = 0,
	.ldras = 0,
	.cpcs  = 1,
	.cpbs  = 0,
	.cpas  = 0,
	.lovrs = 0,
	.covfs = 0
	};


  // *****************   Timer Setup ***********************************************
  // Initialize the timer/counter.
  tc_init_waveform(tc, &WAVEFORM_OPT);         // Initialize the timer/counter waveform.

  // Set the compare triggers.
  // Remember TC counter is 16-bits, so counting second is not possible with fPBA = 12 MHz.
  // We configure it to count ms.
  // We want: (1/(fPBA/8)) * RC = 0.001 s, hence RC = (fPBA/8) / 1000 = 1500 to get an interrupt every 1 ms.
  //tc_write_rc(tc, TC_CHANNEL, (FPBA / 8) / 1000); // Set RC value.

  tc_write_rc(tc, TC_CHANNEL, (CPU_SPEED / 32) / 1000); // Set RC value.

  tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);

  // Start the timer/counter.
  tc_start(tc, TC_CHANNEL);                    // And start the timer/counter.

  // *******************************************************************************

  INTC_register_interrupt(&tc_irq, AVR32_TC_IRQ0, AVR32_INTC_INT3);
  // Priority 2: Higher then others (TWI, USART)

  // Je höher der Wert desto höher die Priorität: AVR32_INTC_INT2 vor AVR32_INTC_INT0

}

void wait_i(unsigned int j){
	int i,k;
	for (i=0;i<j;i++)
		k++;
}

int wait_ms(unsigned int wait){

	volatile int i;
	unsigned int t = tc_ticks;
	unsigned int end_time = wait + tc_ticks;

	while(t < end_time){
		t = tc_ticks;
		i++;
	}

	return i;
}
