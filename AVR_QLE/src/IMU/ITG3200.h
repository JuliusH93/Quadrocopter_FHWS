/*
 * ITG3200.h
 *
 *  Created on: 13.12.2011
 *      Author: gageik
 */

// ITG Gyroscope

#ifndef ITG3200_H_
#define ITG3200_H_

#include "..\QUAD_TWI_6DOF.h"

#define ITG_SCALE_FACTOR	32.8f
//#define ITG_SCALE_FACTOR	35.8f
//#define ITG_SCALE_FACTOR	16.4f

#define ITG_TWI_ADDRESS      		0x68    // ITG TWI address
//#define ITG_TWI_ADDRESS      		0x69    // ITG TWI address, single version address
#define ITG_SENSING_ADDR_START 		0x1D    // Start->Address of ITG Sensor Values
#define ITG_INIT_DLPF_ADDR_START 	0x16    // Configuration Address for ITG
#define ITG_INIT_PM_ADDR_START 		0x3E    // PowerManagement Setup: X Gyro as reference clock (recommended)
#define ITG_SMPLRT_DIV				0x15	// Sample Rate Divider: 8 -> 8ms per Sample bei 1kHz

#define ITG_ABSOLUTE_MAXIMUM_VALUE  500		// If a value is higher then this limit, it is discarded as error and the previous value is forwarded



int itg3200_init(void);
void itg3200_read(sensorDaten_raw* sensorWerte_raw);
int itg3200_read_ex(sensorDaten_raw* sensorWerte_raw);
void itg3200_cond(sensorDaten_raw* sensorWerte_raw);

#endif /* ITG3200_H_ */
