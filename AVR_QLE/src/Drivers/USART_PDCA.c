/*
 *  USART_PDCA.c
 *
 *  Created on: 01.01.2013
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
// *		USART PDCA driver				*
// ******************************************
#include "..\QUAD_TWI_6DOF.h"

// This c-file substitutes USART_NG.c to add PDCA DMA functions
// This module has been designed to be compatible to the current USART_NG driver
// It is to send/receive serial data using PDCA (DMA: direct memory access)

// This USART is for communication/debug with the ground station
// The driver can also support the RX Usart for Remote without Remote-API (not in use)


#ifdef DMA_PDCA

char new_line[2];
char debug_line[350];


// Basic USART API: Redefined for PDCA/DMA
void myUSART_init(){

	//**********************  USART SETUPS  **************************************************************
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

	// Assign GPIO to USART 1
	gpio_enable_module(USART_1_GPIO_MAP, sizeof(USART_1_GPIO_MAP) / sizeof(USART_1_GPIO_MAP[0]));


	// -------  INIT USART	----------------------------------------------------------
	// Initialize USART 1 in RS232 mode.
	usart_init_rs232(USART_COMM, &USART_OPTIONS, CPU_SPEED);
	//*************************************************************************************************

	//************************** PDCA SETUPS (DMA) ****************************************************
	pdca_dma_usart_comm_init();
	//*************************************************************************************************
}


// Basic USART API
void USART_schreiben(char* line){
	pdca_write_line2(line);

	pdca_dma_usart_comm_process();						// Progress via USART
}

void USART_schreiben_binary(char* data,int len){
	pdca_dma_usart_comm_sent(data,(unsigned int)len);

	pdca_dma_usart_comm_process();						// Progress via USART
}

void USART_schreibe_char(char x){
	pdca_usart_comm_put_char(x);

	pdca_dma_usart_comm_process();						// Progress via USART
}


void USART_schreibe_int(int value){

	sprintf(debug_line, "%d \n", value);
	pdca_write_line(debug_line);

	pdca_dma_usart_comm_process();						// Progress via USART
}


void USART_schreibe_float(float value){

	sprintf(debug_line, "%3.3f \n", value);
	pdca_write_line(debug_line);

	pdca_dma_usart_comm_process();						// Progress via USART
}

void USART_schreibe_double(double value){

	sprintf(debug_line, "%3.3f \n", value);
	pdca_write_line(debug_line);

	pdca_dma_usart_comm_process();						// Progress via USART
}

void USART_new_line(){
	pdca_write_newline();

	pdca_dma_usart_comm_process();						// Progress via USART
}

void sendDebugValues(double value1, double value2, double value3, double value4, double value5, double value6){
	sprintf(debug_line, "$, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, #\n", value1, value2, value3, value4, value5, value6);
	pdca_write_line(debug_line);

	pdca_dma_usart_comm_process();						// Progress via USART
}


int USART_lesen(char* c){					// obsolete
	return 0;
}


void USART_paint_value(short value){

	if (value > 10)
		value = 10;

	for (;value > 0; value--)
		pdca_usart_comm_put_char('x');

	pdca_write_newline();

	pdca_dma_usart_comm_process();						// Progress via USART
}

void Debug_Msg(char character){
	#ifdef DEBUG_MSG
			USART_schreibe_char(character);
	#endif
}

#endif






