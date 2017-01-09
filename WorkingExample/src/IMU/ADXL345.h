/*
 * adxl345.h
 *
 *  Created on: 13.12.2011
 *      Author: gageik
 */

// ADXL Accelerometer


#ifndef ADXL345_H_
#define ADXL345_H_

#include "..\QUAD_TWI_6DOF.h"

#define ADXL_SCALE_FACTOR	3.9f

int adxl345_init(void);
void adxl345_read(sensorDaten_raw* sensorWerte_raw);
int adxl345_read_ex(sensorDaten_raw* sensorWerte_raw);
void adxl345_cond(sensorDaten_raw* sensorWerte_raw);

#define ADXL_TWI_ADDRESS      		0x53    // ADXL TWI address
#define ADXL_SENSING_ADDR_START 	0x32    // Start->Address of ADXL Sensor Values
#define ADXL_POWER_CTRL_ADR	 		0x2D    // Configuration Address for ADXL (POWER_CTRL)
#define ADXL_DATA_FORMAT_ADR 		0x31    // Configuration Address for ADXL (FORMAT_ADR)
#define ADXL_BW_RATE_ADR			0x2C	// Set Frequency to 800 Hz

#define ADXL_ABSOLUTE_MAXIMUM_VALUE 2000	// If a value is higher then this limit, it is discarded as error and the previous value is forwarded


#endif /* ADXL345_H_ */
