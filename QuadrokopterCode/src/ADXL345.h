/*
 * adxl345.h
 *
 *  Created on: 13.12.2011
 *      Author: gageik
 */

// ADXL Accelerometer


#ifndef ADXL345_H_
#define ADXL345_H_

#include "basics.h"
#include "ADXL345.h"

#define ADXL_SCALE_FACTOR	1.0f

int adxl345_init(void);
void adxl345_read(sensorDaten_raw* sensorWerte_raw);
int adxl345_read_ex(sensorDaten_raw* sensorWerte_raw);
void adxl345_cond(sensorDaten_raw* sensorWerte_raw);

#define ADXL_TWI_ADDRESS      		0x53    // ADXL TWI address
#define ADXL_SENSING_ADDR_START 	0x32    // Start->Address of ADXL Sensor Values
#define ADXL_POWER_CTRL_ADR	 		0x2D    // Configuration Address for ADXL (POWER_CTRL)
#define ADXL_DATA_FORMAT_ADR 		0x31    // Configuration Address for ADXL (FORMAT_ADR)
#define ADXL_BW_RATE_ADR			0x2C	// Set Frequency to 800 Hz


#endif /* ADXL345_H_ */
