/*
 * imu_selector.c
 *
 *  Created on: 15.11.2013
 *      Author: Gageik
 */
// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011 Nils Gageik

#include "..\QUAD_TWI_6DOF.h"

char imu_detected_gyro;
char imu_detected_acc;
char imu_detected_mag;				// To be added:		TODO

/* Define Priority here */
char gyro_priority[IMU_MAX_NUMBER] = {IMU_ITG, IMU_MPU, IMU_L3G};
char acc_priority[IMU_MAX_NUMBER] = {IMU_LSM, IMU_ADXL, IMU_MPU};
char mag_priority[IMU_MAX_NUMBER] = {IMU_LSM, IMU_LSM, IMU_LSM};

extern char debug_line[DEBUG_BUFFER_SIZE];


void pick_imu(){

#ifdef IMU_AUTO_DETECTION
	imu_detected_gyro = detect_imu_gyro();
	imu_detected_acc  = detect_imu_acc();
#else
	imu_detected_gyro = gyro_priority[0];
	imu_detected_acc = acc_priority[0];

#endif

	sprintf(debug_line,"\n Pick IMU Gyro = %d | Acc = %d", imu_detected_gyro, imu_detected_acc);
	USART_schreiben(debug_line);
}

char detect_imu_gyro(){
	// Return 255 -> No Device detected
	// Otherwise return detected device id

	twi_package_t test_packet;
	char test_data;


	int i = 0;

	while (i < IMU_MAX_NUMBER){

		switch(gyro_priority[i]){

		case IMU_MPU:
			test_packet.chip = MPU_TWI_ADDR;
			test_packet.addr_length = EEPROM_ADDR_LGT;
			test_packet.length = 1;
			test_packet.addr = 0x75;				// WHO AM I ADDRESS
			test_packet.buffer = &test_data;

			if (twi_master_read_ex_success(&test_packet)){
				wait_ms(2);
				if (test_data == 104){
					return IMU_MPU;			// MPU erkannt
				}
			}
		break;


		case IMU_ITG:
			test_packet.chip = ITG_TWI_ADDRESS;
			test_packet.addr_length = EEPROM_ADDR_LGT;
			test_packet.length = 1;
			test_packet.addr = 0x00;
			test_packet.buffer = &test_data;

			if (twi_master_read_ex_success(&test_packet)){
				wait_ms(2);
				if (test_data == 105){
					return IMU_ITG;			// ITG erkannt
				}
			}
		break;

		case IMU_L3G:
			test_packet.chip = L3G_GYRO_TWI_ADDRESS;
			test_packet.addr_length = EEPROM_ADDR_LGT;
			test_packet.length = 1;
			test_packet.addr = 0x0F;
			test_packet.buffer = &test_data;

			if (twi_master_read_ex_success(&test_packet)){
				wait_ms(2);
				if (test_data == 212 || test_data == 211){
					return IMU_L3G;			// L3G erkannt
				}
			}
		break;

		}

		i++;
	}

	return 255;
}


char detect_imu_acc(){
	// Return 255 -> No Device detected
	// Otherwise return detected device id

	twi_package_t test_packet;
	char test_data;


	int i = 0;

	while (i < IMU_MAX_NUMBER){

		switch(acc_priority[i]){

		case IMU_MPU:
			test_packet.chip = MPU_TWI_ADDR;
			test_packet.addr_length = EEPROM_ADDR_LGT;
			test_packet.length = 1;
			test_packet.addr = 0x75;				// WHO AM I ADDRESS
			test_packet.buffer = &test_data;

			if (twi_master_read_ex_success(&test_packet)){
				wait_ms(2);
				if (test_data == 104){
					return IMU_MPU;			// MPU erkannt
				}
			}
		break;

		case IMU_ADXL:
			test_packet.chip = ADXL_TWI_ADDRESS;
			test_packet.addr_length = EEPROM_ADDR_LGT;
			test_packet.length = 1;
			test_packet.addr = 0x00;
			test_packet.buffer = &test_data;

			if (twi_master_read_ex_success(&test_packet)){
				wait_ms(2);
				if (test_data == 229){
					return IMU_ADXL;			// ADXL erkannt
				}
			}
		break;


		case IMU_LSM:
			test_packet.chip = LSM_ACC_TWI_ADDRESS;
			test_packet.addr_length = EEPROM_ADDR_LGT;
			test_packet.length = 1;
			test_packet.addr = 0x0F;
			test_packet.buffer = &test_data;

			if (twi_master_read_ex_success(&test_packet)){
				wait_ms(2);
				if ((test_data == 51) || (test_data = 50)){		// Altes Board hat 50
					return IMU_LSM;			// LSM erkannt
				}
			}
		break;

		}

		i++;
	}

	return 255;
}

// Init the detected sensors: Acc + Gyro
int Init_IMU_Sensors(){
	// Returns Error Code

	// ***** PICK IMU HERE
	pick_imu();

	int error = 0;
	error = error + Init_Gyro_Sensor();
	error = error + Init_Acc_Sensor();
	error = error + Init_Mag_Sensor();

	return error;
}

// Init the detected gyro sensor
int Init_Gyro_Sensor(){
	int error = 0;

	switch (imu_detected_gyro){

	case IMU_MPU:
		error = mpu6000_init();
	break;

	case IMU_ITG:
		error = itg3200_init();
	break;

	case IMU_L3G:
		error = l3g_init();
	break;

	default:
		sprintf(debug_line,"\nNo Gyro detected and can be used\n");
		USART_schreiben(debug_line);

	}

	return error;
}

// Init the detected accelero sensor
int Init_Acc_Sensor(){
	int error = 0;

	switch (imu_detected_acc){

	case IMU_MPU:
		error = mpu6000_init();
	break;

	case IMU_ADXL:
		error = adxl345_init();
	break;

	case IMU_LSM:
		error = lsm303_acc_init();
	break;

	default:
		sprintf(debug_line,"\nNo Acc detected and can be used\n");
		USART_schreiben(debug_line);

	}

	return error;
}


int Init_Mag_Sensor(){
	#ifdef LSM_MAG_SENSOR
	return lsm303_mag_init();
	#else
	return 0;
	#endif
}

// non-blocking Read of Values of Defined Sensors: Used in Calibrate
int read_ex_IMU_Sensors(sensorDaten_raw* nWert){
	int error = 0;

	switch (imu_detected_gyro){

		case IMU_MPU:
			error = mpu6000_gyro_read_ex(nWert);
		break;

		case IMU_ITG:
			error = itg3200_read_ex(nWert);
		break;

		case IMU_L3G:
			error = l3g_read_ex(nWert);
		break;

		}

	error = error + read_ex_Acc_Sensor(nWert);

	#ifdef LSM_MAG_SENSOR
	error = error + lsm303_mag_read_ex(nWert);
	#endif

	return error;
}

// non blocking: Only Acc needed for Manual Calibration
int read_ex_Acc_Sensor(sensorDaten_raw* nWert){
	int error = 0;

	switch (imu_detected_acc){

		case IMU_MPU:
			error = error + mpu6000_acc_read_ex(nWert);
		break;

		case IMU_ADXL:
			error = error + adxl345_read_ex(nWert);
		break;

		case IMU_LSM:
			error = error + lsm303_acc_read_ex(nWert);
		break;
	}

	return error;
}


// blocking/resetting reading of the values of defined Sensors: Used in Sensoring
void read_IMU_Sensors(sensorDaten_raw* nWert){

	switch (imu_detected_gyro){

		case IMU_MPU:
			mpu6000_gyro_read(nWert);
		break;

		case IMU_ITG:
			itg3200_read(nWert);
		break;

		case IMU_L3G:
			l3g_read(nWert);
		break;

		}

	switch (imu_detected_acc){

		case IMU_MPU:
			mpu6000_acc_read(nWert);
		break;

		case IMU_ADXL:
			adxl345_read(nWert);
		break;

		case IMU_LSM:
			lsm303_acc_read(nWert);
		break;

	}

	#ifdef LSM_MAG_SENSOR
	lsm303_mag_read(nWert);
	#endif

}



// Condition the defined Sensors: Sensor depending conditioning
void cond_IMU_Sensors(sensorDaten_raw* nWert){

	switch (imu_detected_gyro){

		case IMU_MPU:
			mpu6000_gyro_cond(nWert);
		break;

		case IMU_ITG:
			itg3200_cond(nWert);
		break;

		case IMU_L3G:
			l3g_cond(nWert);
		break;

		}

	switch (imu_detected_acc){

		case IMU_MPU:
			mpu6000_acc_cond(nWert);
		break;

		case IMU_ADXL:
			adxl345_cond(nWert);
		break;

		case IMU_LSM:
			lsm303_acc_cond(nWert);
		break;

	}

	#ifdef LSM_MAG_SENSOR
	lsm303_mag_cond(nWert);
	#endif
}
