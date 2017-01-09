/*
 * myUSART.h
 *
 *  Created on: 04.08.2011
 *      Author: gageik
 */
// USART FOR DEBUGGING/COMMUNICATION

#ifndef MYUSART_H_
#define MYUSART_H_

#ifdef DMA_PDCA
#include "PDCA_USART_DMA.h"
#endif


// USART Settings
#define USART_BAUDRATE  	    	57600
//#define USART_BAUDRATE  	    	115200

// Debugging
#define USART_COMM             			(&AVR32_USART0)
#define USART_COMM_RX_PIN        		AVR32_USART0_RXD_0_0_PIN
#define USART_COMM_RX_FUNCTION   		AVR32_USART0_RXD_0_0_FUNCTION
#define USART_COMM_TX_PIN        		AVR32_USART0_TXD_0_0_PIN
#define USART_COMM_TX_FUNCTION   		AVR32_USART0_TXD_0_0_FUNCTION
#define USART_COMM_IRQ           		AVR32_USART0_IRQ
#define USART_COMM_PDCA_PID_TX			AVR32_PDCA_PID_USART0_TX
#define USART_COMM_PDCA_PID_RX			AVR32_PDCA_PID_USART0_RX

void myUSART_init(void);
void USART_schreiben(char* line);
void USART_schreiben_binary(char* data,int len);
void USART_schreibe_char(char x);
void USART_new_line(void);
void sendDebugValues(double value1, double value2, double value3, double value4, double value5, double value6);
int USART_lesen(char* x);
void USART_paint_value(short value);
void USART_schreibe_float(float value);
void USART_schreibe_int(int value);
void USART_schreibe_double(double value);
void Debug_Msg(char character);


#endif /* MYUSART_H_ */
