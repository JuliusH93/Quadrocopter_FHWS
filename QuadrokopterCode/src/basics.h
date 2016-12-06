/*
 * basics.h
 *
 *  Created on: 12.04.2011
 *      Author: gageik
 */

#ifndef BASICS_H_
#define BASICS_H_

#define DEBUG_BUFFER_SIZE		200
#define RECEIVE_DATA_LENGTH		6
#define CPU_SPEED				12000000		// This needs to be the oscillator fequency

#define DEBUG_MSG_IMU_RAW
#define DEBUG_MSG_TWI

// Sensor Rohdaten
typedef struct {
	double sensor_1;
	double sensor_2;
	// Das ist nur ein Beispiel: Bennent Sie besser!
} sensorDaten_raw;

void dip_write_string(char* string, int zeile);
void USART_schreiben(char* string);

#endif /* BASICS_H_ */
