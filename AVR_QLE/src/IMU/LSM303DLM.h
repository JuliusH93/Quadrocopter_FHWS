/*
 * LSM303DLM.h
 *
 *  Created on: 21.09.2012
 *      Author: Gageik
 */

// Valid for LSM303DLH and LSM303LM (new version)

#include "..\QUAD_TWI_6DOF.h"

#ifndef LSM303DLH_H_
#define LSM303DLH_H_

//#define LSM_ACC_SCALE_FACTOR				16.0
#define LSM_ACC_SCALE_FACTOR				8.0
#define LSM_ACC_ABSOLUTE_MAXIMUM_VALUE		4000.0
#define LSM_MAG_SCALE_FACTOR				1.0

#ifdef MINI_IMU_V2
#define	LSM_ACC_TWI_ADDRESS				0x19
#define	LSM_MAG_TWI_ADDRESS				0x1E
#define LSM_CTRL1_REG_VALUE				0x57
#define LSM_CTRL4_REG_VALUE				0x98
#else
#define	LSM_ACC_TWI_ADDRESS				0x18
#define	LSM_MAG_TWI_ADDRESS				0x1E
#define LSM_CTRL1_REG_VALUE				0x2F
#define LSM_CTRL4_REG_VALUE				0x90
#endif

#define LSM_CTRL1_REG_ADR				0x20
#define LSM_CTRL4_REG_ADR				0x23
#define LSM_ACC_SENSING_ADDR_START		0xA8			// Acc Multiple Read Address


#define LSM_MAG_SENSING_ADDR_START		0x83			// Mag Multiple Read Address
#define LSM_MAG_CRA_REG_ADR				0x00
#define LSM_MAG_CRB_REG_ADR				0x01
#define LSM_MAG_MR_REG_ADR				0x02

#define LSM_MAG_CRA_REG_VALUE			0x3C			// Output Rate
#define LSM_MAG_CRB_REG_VALUE			0xA0			// Gain: Maximal Scale
#define LSM_MAG_MR_REG_VALUE			0x00			// Mode: Continuos

// Functions Accelerometer
int lsm303_acc_init(void);
void lsm303_acc_read(sensorDaten_raw* sensorWerte_raw);
int lsm303_acc_read_ex(sensorDaten_raw* sensorWerte_raw);
void lsm303_acc_cond(sensorDaten_raw* sensorWerte_raw);

// Functions Magnetometer
int lsm303_mag_init(void);
void lsm303_mag_read(sensorDaten_raw* sensorWerte_raw);
int lsm303_mag_read_ex(sensorDaten_raw* sensorWerte_raw);
void lsm303_mag_cond(sensorDaten_raw* sensorWerte_raw);


#endif /* LSM303DLH_H_ */
