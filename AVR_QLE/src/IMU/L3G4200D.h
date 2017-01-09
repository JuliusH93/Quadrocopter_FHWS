/*
 * L3G4200D.h
 *
 *  Created on: 24.09.2012
 *      Author: Gageik
 */

#include "..\QUAD_TWI_6DOF.h"

#ifndef L3G4200D_H_
#define L3G4200D_H_

#ifdef MINI_IMU_V2
#define	L3G_GYRO_TWI_ADDRESS			0x6B
#else
#define	L3G_GYRO_TWI_ADDRESS			0x69
#endif

//#define L3G_GYRO_SCALE					17.5
#define L3G_GYRO_SCALE					70.0


#define	L3G_ABSOLUTE_MAXIMUM_VALUE		2000.0

// Register Adresses
#define L3G_CTRL1_REG_ADR				0x20
#define L3G_CTRL2_REG_ADR				0x21
#define L3G_CTRL4_REG_ADR				0x23
#define L3G_CTRL5_REG_ADR				0x24

//Register Values
#define	L3G_CTRL_REG1					0x4F
#define	L3G_CTRL_REG2					0x29
#define	L3G_CTRL_REG4					0xA0
#define	L3G_CTRL_REG5					0x00

#define L3G_GYRO_SENSING_ADDR_START		0xA8

int l3g_init(void);
void l3g_read(sensorDaten_raw* sensorWerte_raw);
int l3g_read_ex(sensorDaten_raw* sensorWerte_raw);
void l3g_cond(sensorDaten_raw* sensorWerte_raw);

#endif /* L3G4200D_H_ */
