/*
 * PDCA_DMA_NG.c
 *
 *  Created on: 22.02.2013
 *      Author: Gageik
 */

#include "..\QUAD_TWI_6DOF.h"

// PDCA DMA USART COMM
// PDCA Drivers for USART Communication with GROUNDSTATION
// These functions are dedicated for USART Communication with GROUNDSTATION

#ifdef DMA_PDCA

pdca_Daten pdca_Werte;

// ************					INIT PDCA DMA		************************************************************
void pdca_dma_usart_comm_init(){

	pdca_Werte.usart_rx_current_index = 0;
	pdca_Werte.usart_tx_current_index = 0;
	pdca_Werte.usart_tx_load_state = PDCA_STATE_BUFFER_1;
	pdca_Werte.usart_rx_load_state = PDCA_STATE_BUFFER_1;

	// TODO: to be tested
	//AVR32_HMATRIX.mcfg[AVR32_HMATRIX_MASTER_CPU_INSN] = 0x1;

	// PDCA channel options for USART TX (Transfer USART Debug)
	static const pdca_channel_options_t PDCA_OPTIONS_TX =
	{
		.addr = (void *)pdca_Werte.usart_tx_buffer_1,            // memory address
		.pid = USART_COMM_PDCA_PID_TX,           				// select peripheral - data are transmit on USART TX line.
		.size = 0,              // transfer counter
		.r_addr = (void *)pdca_Werte.usart_tx_buffer_2,         // next memory address
		.r_size = 0,            // next transfer counter
		.transfer_size = PDCA_TRANSFER_SIZE_BYTE  				// select size of the transfer
	 };

	  // Init PDCA channel with the pdca_options.
	  pdca_init_channel(PDCA_CHANNEL_USART_TX_COMM, &PDCA_OPTIONS_TX); // init PDCA channel with options.


	  static const pdca_channel_options_t PDCA_OPTIONS_RX =
	  {
	    .addr = (void *)pdca_Werte.usart_rx_buffer_1,          // memory address
	    .pid = USART_COMM_PDCA_PID_RX,         					  // select peripheral - data are transmit on USART TX line.
	    .size = PDCA_RX_BUFFER_SIZE,            				  // transfer counter
	    .r_addr = (void *)pdca_Werte.usart_rx_buffer_2,          // next memory address
	    .r_size = PDCA_RX_BUFFER_SIZE,                   // next transfer counter
	    .transfer_size = PDCA_TRANSFER_SIZE_BYTE  // select size of the transfer
	  };

	  // Init PDCA channel with the pdca_options.
	  pdca_init_channel(PDCA_CHANNEL_USART_RX_COMM, &PDCA_OPTIONS_RX); // init PDCA channel with options.

	  // Enable now the transfer.
	  pdca_enable(PDCA_CHANNEL_USART_TX_COMM);
	  pdca_enable(PDCA_CHANNEL_USART_RX_COMM);
}



// ************					SENT PDCA DMA		************************************************************
// Sends String-Data as Char via PDCA DMA
// The data is to be stored correctly in one of two buffers
// Parameters are pointer to char-Array (String) and length/size in Byte
void pdca_dma_usart_comm_sent(char* data, unsigned int size){

	unsigned int lauf_index = minimum_ui(size, (unsigned int)PDCA_TX_BUFFER_SIZE - pdca_Werte.usart_tx_current_index);
	unsigned int i;

	switch(pdca_Werte.usart_tx_load_state){

	case PDCA_STATE_BUFFER_1:					// Speichere in Buffer
		for (i = 0; i < lauf_index; i++){
			pdca_Werte.usart_tx_buffer_1[pdca_Werte.usart_tx_current_index] = data[i];
			pdca_Werte.usart_tx_current_index++;
		}
		break;

	case PDCA_STATE_BUFFER_2:
		for (i = 0; i < lauf_index; i++){
			pdca_Werte.usart_tx_buffer_2[pdca_Werte.usart_tx_current_index] = data[i];
			pdca_Werte.usart_tx_current_index++;
		}
		break;

	case PDCA_STATE_BUFFER_3:
		for (i = 0; i < lauf_index; i++){
			pdca_Werte.usart_tx_buffer_3[pdca_Werte.usart_tx_current_index] = data[i];
			pdca_Werte.usart_tx_current_index++;
		}
		break;
	}
}


// Processes the data from the correct buffer
void pdca_dma_usart_comm_process(){

	switch(pdca_Werte.usart_tx_load_state){

		case PDCA_STATE_BUFFER_1:		// Buffer 3 Loaded, Buffer 1 Reloaded, Buffer 2 to be written next

			if (!pdca_get_reload_size(PDCA_CHANNEL_USART_TX_COMM) && pdca_Werte.usart_tx_current_index){
				pdca_reload_channel(PDCA_CHANNEL_USART_TX_COMM, (void *)pdca_Werte.usart_tx_buffer_1, pdca_Werte.usart_tx_current_index);
				pdca_Werte.usart_tx_current_index = 0;
				pdca_Werte.usart_tx_load_state = PDCA_STATE_BUFFER_2;
			}
			break;

		case PDCA_STATE_BUFFER_2:	// Buffer 1 Loaded, Buffer 2 Reloaded, Buffer 3 to be written
			if (!pdca_get_reload_size(PDCA_CHANNEL_USART_TX_COMM) && pdca_Werte.usart_tx_current_index){
				pdca_reload_channel(PDCA_CHANNEL_USART_TX_COMM, (void *)pdca_Werte.usart_tx_buffer_2, pdca_Werte.usart_tx_current_index);
				pdca_Werte.usart_tx_current_index = 0;
				pdca_Werte.usart_tx_load_state = PDCA_STATE_BUFFER_3;
			}
			break;

		case PDCA_STATE_BUFFER_3:
			if (!pdca_get_reload_size(PDCA_CHANNEL_USART_TX_COMM) && pdca_Werte.usart_tx_current_index){
				pdca_reload_channel(PDCA_CHANNEL_USART_TX_COMM, (void *)pdca_Werte.usart_tx_buffer_3, pdca_Werte.usart_tx_current_index);
				pdca_Werte.usart_tx_current_index = 0;
				pdca_Werte.usart_tx_load_state = PDCA_STATE_BUFFER_1;
			}
		break;

		}
}


// ************					RECEIVE PDCA DMA		************************************************************

// Receive Data from pdca and write into dataStream: To be executed two times to withdraw all data (optional)
// This can be necessary as data is saved in two different buffers (with not connected arrays): But not required (just read next time)
dataStream pdca_usart_comm_rec_data(void){

	dataStream mydata;		// New data is written with size to dataStream (return value)

	// Amount of Bytes in Buffer
	unsigned int size_load = PDCA_RX_BUFFER_SIZE - pdca_get_load_size(PDCA_CHANNEL_USART_RX_COMM);


	switch(pdca_Werte.usart_rx_load_state){

	case PDCA_STATE_BUFFER_1:
		if (!pdca_get_reload_size(PDCA_CHANNEL_USART_RX_COMM)){
			mydata.data = &pdca_Werte.usart_rx_buffer_1[pdca_Werte.usart_rx_current_index];
			mydata.size = PDCA_RX_BUFFER_SIZE - pdca_Werte.usart_rx_current_index;
			pdca_Werte.usart_rx_current_index = 0;
		}
		else if (size_load > pdca_Werte.usart_rx_current_index){
			mydata.data = &pdca_Werte.usart_rx_buffer_1[pdca_Werte.usart_rx_current_index];
			mydata.size = size_load - pdca_Werte.usart_rx_current_index;
			pdca_Werte.usart_rx_current_index = size_load;
		}
		else {
			mydata.size = 0;
		}

		break;

	case PDCA_STATE_BUFFER_2:
		if (!pdca_get_reload_size(PDCA_CHANNEL_USART_RX_COMM)){
			mydata.data = &pdca_Werte.usart_rx_buffer_2[pdca_Werte.usart_rx_current_index];
			mydata.size = PDCA_RX_BUFFER_SIZE - pdca_Werte.usart_rx_current_index;
			pdca_Werte.usart_rx_current_index = 0;
		}
		else if (size_load > pdca_Werte.usart_rx_current_index){
			mydata.data = &pdca_Werte.usart_rx_buffer_2[pdca_Werte.usart_rx_current_index];
			mydata.size = size_load - pdca_Werte.usart_rx_current_index;
			pdca_Werte.usart_rx_current_index = size_load;
		}
		else {
			mydata.size = 0;
		}


		break;
	}

	return mydata;
}


// Sets the current/last buffer free: To be executed by user after reading and processing is finished
// So the data does not need to be copied, user can process data and afterwards same space will be overwritten
void pdca_usart_comm_rec_set_free(void){

	if(!pdca_get_reload_size(PDCA_CHANNEL_USART_RX_COMM)){

		switch(pdca_Werte.usart_rx_load_state){


		case PDCA_STATE_BUFFER_1:				// Current write to Buffer 2, next write to 1 (Reload)
			pdca_reload_channel(PDCA_CHANNEL_USART_RX_COMM, (void*)pdca_Werte.usart_rx_buffer_1, (unsigned int)PDCA_RX_BUFFER_SIZE);
			pdca_Werte.usart_rx_load_state = PDCA_STATE_BUFFER_2;

		break;

		case PDCA_STATE_BUFFER_2:
			pdca_reload_channel(PDCA_CHANNEL_USART_RX_COMM, (void*)pdca_Werte.usart_rx_buffer_2, (unsigned int)PDCA_RX_BUFFER_SIZE);
			pdca_Werte.usart_rx_load_state = PDCA_STATE_BUFFER_1;
		break;

		}

	}
}




// ************					API PDCA DMA		************************************************************
// Send a Stream
void pdca_dma_sent_usart_comm_stream(dataStream* stream){
	pdca_dma_usart_comm_sent(stream->data, stream->size);
}

// Sends one character (1 Byte: char)
void pdca_usart_comm_put_char(char data){
	char datum = data;			// TODO: simplify

	pdca_dma_usart_comm_sent(&datum, 1);
}

// Write a line via pdca and add a new line at the end
void pdca_write_line(char *string)
{
  int i = 0;

  while (string[i] != '\0')
	  i++;

	pdca_dma_usart_comm_sent(string, i);
	pdca_write_newline();
}

// Write a line via pdca
void pdca_write_line2(char *string)
{
  int i = 0;

  while (string[i] != '\0')
	  i++;

	pdca_dma_usart_comm_sent(string, i);
}

// Write a line via pdca with iterative put_char derived from AVR Framework
void pdca_write_line_put(char *string)
{
  while (*string != '\0')
	  pdca_usart_comm_put_char(*string++);
}

// Write a new line ASCI symbol
void pdca_write_newline(void){
	char newline = '\n';

	pdca_usart_comm_put_char(newline);
}


#endif
