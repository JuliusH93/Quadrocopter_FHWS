/*
 * MPU6000.h
 *
 *  Created on: 12.11.2013
 *      Author: Gageik
 */

#ifndef MPU6000_H_
#define MPU6000_H_

#define MPU_GYRO_SCALE_FACTOR	32.8
#define MPU_ACC_SCALE_FACTOR	8.192

#define MPU_TWI_ADDR			0x68

#define MPU_PM_ADDR				0x6B
#define MPU_SMPLRT_DIV_ADDR		0x19
#define MPU_CONFIG_ADDR			0x1A
#define MPU_GYRO_CONFIG_ADDR	0x1B
#define MPU_ACC_CONFIG_ADDR		0x1C
#define MPU_FIFO_CONFIG_ADDR	0x23
#define MPU_FIFO_EN_CONFIG_ADDR	0x6A

#define MPU_SENS_START_ADDR		0x3B
#define MPU_GYRO_START_ADDR_	0x43

#define MPU_FIFO_COUNT_ADDR		0x72
#define MPU_FIFO_ADDR			0x74
#define MPU_STATUS_ADDR			0x3A

#define MPU_PACKAGE_RECEIVE_DATA_LENGTH				120

#define MPU6000_GYRO_ABSOLUTE_MAXIMUM_VALUE			1000
#define MPU6000_ACC_ABSOLUTE_MAXIMUM_VALUE			4000

void mpu6000_acc_read(sensorDaten_raw* sensorWerte_raw);
int mpu6000_acc_read_ex(sensorDaten_raw* sensorWerte_raw);
void mpu6000_gyro_read(sensorDaten_raw* sensorWerte_raw);
int mpu6000_gyro_read_ex(sensorDaten_raw* sensorWerte_raw);
void mpu6000_gyro_cond(sensorDaten_raw* sensorWerte_raw);
void mpu6000_acc_cond(sensorDaten_raw* sensorWerte_raw);

int mpu6000_init(void);

// FIFO is obsolete now
#define MPU_NO_FIFO				// Adding FIFO will change the whole SP design


/*
void mpu6000_read(sensorDaten_raw* sensorWerte_raw);
int mpu6000_read_ex(sensorDaten_raw* sensorWerte_raw);
void mpu6000_cond(sensorDaten_raw* sensorWerte_raw);

#ifdef MPU_NO_FIFO
#define MPU_MAX_PACKAGE_SIZE	14
#define MPU_ACC_X_H		0
#define MPU_ACC_X_L		1
#define MPU_ACC_Y_H		2
#define MPU_ACC_Y_L		3
#define MPU_ACC_Z_H		4
#define MPU_ACC_Z_L		5
#define MPU_GYRO_X_H	8
#define MPU_GYRO_X_L	9
#define MPU_GYRO_Y_H	10
#define MPU_GYRO_Y_L	11
#define MPU_GYRO_Z_H	12
#define MPU_GYRO_Z_L	13

#else
#define MPU_MAX_PACKAGE_SIZE	12
#define MPU_ACC_X_H		0
#define MPU_ACC_X_L		1
#define MPU_ACC_Y_H		2
#define MPU_ACC_Y_L		3
#define MPU_ACC_Z_H		4
#define MPU_ACC_Z_L		5
#define MPU_GYRO_X_H	6
#define MPU_GYRO_X_L	7
#define MPU_GYRO_Y_H	8
#define MPU_GYRO_Y_L	9
#define MPU_GYRO_Z_H	10
#define MPU_GYRO_Z_L	11
#endif
*/
#endif /* MPU6000_H_ */
