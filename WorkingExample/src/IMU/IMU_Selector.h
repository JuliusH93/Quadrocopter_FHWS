/*
 * imu_selector.h
 *
 *  Created on: 15.11.2013
 *      Author: Gageik
 */

#ifndef IMU_SELECTOR_H_
#define IMU_SELECTOR_H_

#include "..\QUAD_TWI_6DOF.h"


#define IMU_MPU			0
#define IMU_ITG			1
#define IMU_L3G			2

#define IMU_ADXL		1
#define IMU_LSM			2

#define IMU_MAX_NUMBER	3

void pick_imu();
char detect_imu_gyro();
char detect_imu_acc();
int Init_Gyro_Sensor();
int Init_Acc_Sensor();
int Init_Mag_Sensor();

int read_ex_IMU_Sensors(sensorDaten_raw* nWert);
void read_IMU_Sensors(sensorDaten_raw* nWert);
void cond_IMU_Sensors(sensorDaten_raw* nWert);
int read_ex_Acc_Sensor(sensorDaten_raw* nWert);



#endif /* IMU_SELECTOR_H_ */
