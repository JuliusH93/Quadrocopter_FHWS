/*
 * PDCA_DMA_NG.h
 *
 *  Created on: 22.02.2013
 *      Author: Gageik
 */

#ifndef PDCA_DMA_NG_H_
#define PDCA_DMA_NG_H_

//! The channel instance for the USART example, here PDCA channel 0 (highest priority).
#define PDCA_CHANNEL_USART_TX_COMM			3			// TX for USART Groundstation Communication
#define PDCA_CHANNEL_USART_RX_COMM			4			// RX for USART Groundstation Communication
#define PDCA_CHANNEL_USART_RX_REMOTE		2			// RX for RC Satellite Remote Receiver
#define PDCA_CHANNEL_USART_RX_INTERN		0			// RX for USART Intern PC (nano ITX) Communcation
#define PDCA_CHANNEL_USART_TX_INTERN		1			// TX for USART Intern PC (nano ITX) Communcation

#define PDCA_STATE_BUFFER_1					0
#define PDCA_STATE_BUFFER_2					1
#define PDCA_STATE_BUFFER_3					2

#define PDCA_RX_BUFFER_SIZE					100			// 2 Buffer
#define PDCA_TX_BUFFER_SIZE					200			// 3 Buffer

// PID Param
typedef struct {
	char usart_tx_buffer_1[PDCA_TX_BUFFER_SIZE];
	char usart_tx_buffer_2[PDCA_TX_BUFFER_SIZE];
	char usart_tx_buffer_3[PDCA_TX_BUFFER_SIZE];

	char usart_rx_buffer_1[PDCA_RX_BUFFER_SIZE];
	char usart_rx_buffer_2[PDCA_RX_BUFFER_SIZE];

	char usart_tx_load_state;								// Process Buffer: PDCA_STATE_LOAD = Process Load Buffer (1),  new data to 2 (Reload Buffer)
	char usart_rx_load_state;								// Read from this buffer currently

	unsigned int usart_tx_current_index;					// Gibt Anzahl Bytes an: Läuft von 1 bis PDCA_TX_BUFFER_SIZE oder 0 wenn leer
	unsigned int usart_rx_current_index;					// Gibt Bufferstand an
} pdca_Daten;

// Init
void pdca_dma_usart_comm_init(void);

// Sent
void pdca_dma_usart_comm_sent(char* data, unsigned int size);
void pdca_dma_usart_comm_process(void);

// Receive
dataStream pdca_usart_comm_rec_data(void);
void pdca_usart_comm_rec_set_free(void);

// API
void pdca_write_line(char *string);
void pdca_write_line2(char *string);
void pdca_write_line_put(char *string);
void pdca_write_newline(void);
void pdca_dma_sent_usart_comm_stream(dataStream* stream);
void pdca_usart_comm_put_char(char data);

#endif /* PDCA_DMA_NG_H_ */
