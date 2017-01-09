/*
 * myUSART.c
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
// *		USART driver					*
// ******************************************
#include "..\QUAD_TWI_6DOF.h"

#ifndef DMA_PDCA

char new_line[2];
char debug_line[350];

#define RECEIVEBUFFER_LEN 512
char ringBuffer[RECEIVEBUFFER_LEN];



int writeBufferPos=0, readBufferPos=0;


#ifdef MESSAGE_ON
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
#pragma handler = AVR32_TC_IRQ_GROUP, 1
__interrupt
#endif
static void usart_irq(void)
{
	int x,result;
	result=usart_read_char(USART_COMM,&x);

	if (result != USART_SUCCESS){
		usart_reset_status(USART_COMM);
	}else{
		ringBuffer[writeBufferPos]=(char)x;

		writeBufferPos++;
		if (writeBufferPos > RECEIVEBUFFER_LEN-1)
			writeBufferPos = 0;
	}

}
#endif




void myUSART_init(){

	// USART Map
	static const gpio_map_t USART_1_GPIO_MAP =
		{
			 {USART_COMM_RX_PIN, USART_COMM_RX_FUNCTION},
			 {USART_COMM_TX_PIN, USART_COMM_TX_FUNCTION}
		};

	// USART options.
	static const usart_options_t USART_OPTIONS =
			{
				.baudrate     = USART_BAUDRATE,
				.charlength   = 8,
				.paritytype   = USART_NO_PARITY,
				.stopbits     = USART_1_STOPBIT,
				.channelmode  = USART_NORMAL_CHMODE
			};

	// Assign GPIO to USART COMM
	gpio_enable_module(USART_1_GPIO_MAP, sizeof(USART_1_GPIO_MAP) / sizeof(USART_1_GPIO_MAP[0]));


	// -------  INIT USART	----------------------------------------------------------
	// Initialize USART COMM in RS232 mode.
	usart_init_rs232(USART_COMM, &USART_OPTIONS, CPU_SPEED);


	#ifdef MESSAGE_ON
	 INTC_register_interrupt(&usart_irq, USART_COMM_IRQ, AVR32_INTC_INT0);
	 // Enable USART Rx interrupt.
	 USART_COMM->ier = AVR32_USART_IER_RXRDY_MASK;
	#endif
}

void USART_schreiben(char* line){
	usart_write_line(USART_COMM, line);
}

void USART_schreiben_binary(char* data,int len){
	while(len >0){
		int c = *(data++);
		int timeout = USART_DEFAULT_TIMEOUT;
		do
		  {
		    if (!timeout--) break;
		  } while (usart_write_char(USART_COMM, c) != USART_SUCCESS);
		len--;
	}
}

void USART_schreibe_char(char x){
	usart_putchar(USART_COMM,(int)x);
}

void USART_schreibe_int(int value){

	sprintf(debug_line, "%d \n", value);

	usart_write_line(USART_COMM, debug_line);
}



void USART_schreibe_float(float value){

	sprintf(debug_line, "%3.3f \n", value);
	usart_write_line(USART_COMM, debug_line);
}

void USART_schreibe_double(double value){

	sprintf(debug_line, "%3.3f \n", value);
	usart_write_line(USART_COMM, debug_line);
}

void USART_new_line(){
	usart_write_line(USART_COMM, new_line);
}

void sendDebugValues(double value1, double value2, double value3, double value4, double value5, double value6){
	sprintf(debug_line, "$, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, #\n", value1, value2, value3, value4, value5, value6);
	usart_write_line(USART_COMM, debug_line);
}

int USART_lesen(char* c){
	int result;
	USART_COMM->ier = 0; //Disable USART IRQ
	if (writeBufferPos==readBufferPos){
		result = USART_RX_EMPTY;
	}else{
		*c = ringBuffer[readBufferPos++];
		readBufferPos %= RECEIVEBUFFER_LEN;
		result= USART_SUCCESS;
	}
	USART_COMM->ier = AVR32_USART_IER_RXRDY_MASK;			// TESTEN!

	return result;
}


void USART_paint_value(short value){

	if (value > 10)
		value = 10;

	for (;value > 0; value--)
		USART_schreibe_char('x');

	USART_new_line();
}

void Debug_Msg(char character){
	#ifdef DEBUG_MSG
		USART_schreibe_char(character);
	#endif
}

#endif
