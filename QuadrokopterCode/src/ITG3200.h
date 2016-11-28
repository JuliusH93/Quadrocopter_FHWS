/*
 * ITG3200.h
 *
 *  Created on: 13.12.2011
 *      Author: gageik
 */

// ITG Gyroscope
#ifndef ITG3200_H_
#define ITG3200_H_

#include "basics.h"

#define ITG_SCALE_FACTOR	10.0f

#define ITG_TWI_ADDRESS      		0x68    // ITG TWI address
#define ITG_SENSING_ADDR_START 		0x1D    // Start->Address of ITG Sensor Values
#define ITG_INIT_DLPF_ADDR_START 	0x16    // Configuration Address for ITG
#define ITG_INIT_PM_ADDR_START 		0x3E    // PowerManagement Setup: X Gyro as reference clock (recommended)
#define ITG_SMPLRT_DIV				0x15	// Sample Rate Divider: 8 -> 8ms per Sample bei 1kHz

int itg3200_init(void);
void itg3200_read(sensorDaten_raw* sensorWerte_raw);
int itg3200_read_ex(sensorDaten_raw* sensorWerte_raw);
void itg3200_cond(sensorDaten_raw* sensorWerte_raw);

#endif /* ITG3200_H_ */
