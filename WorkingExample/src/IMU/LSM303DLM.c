/*
 * LSM303DLM.c
 *
 *  Created on: 21.09.2012
 *      Author: Gageik
 */
// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011 Nils Gageik


// Accelerometer and Magnetometer
// Valid for LSM303DLH and LSM303LM (new version)

#include "..\QUAD_TWI_6DOF.h"

twi_package_t packet_lsm_acc, packet_lsm_mag;

char data_received_lsm_acc[RECEIVE_DATA_LENGTH];
char data_received_lsm_mag[RECEIVE_DATA_LENGTH];

const U8 setup_lsm_ctrl1_reg_a[1] =  { LSM_CTRL1_REG_VALUE};
const U8 setup_lsm_ctrl4_reg_a[1] =  { LSM_CTRL4_REG_VALUE};

const U8 setup_lsm_mag_cra_reg[1] =  { LSM_MAG_CRA_REG_VALUE };
const U8 setup_lsm_mag_crb_reg[1] =  { LSM_MAG_CRB_REG_VALUE };
const U8 setup_lsm_mag_mr_reg[1] = { LSM_MAG_MR_REG_VALUE };


extern char debug_line[DEBUG_BUFFER_SIZE];

//* LSM Accelerometer *///////////////////////////////////////////////////////////////////////////////
//***************************************************************************************************

void lsm303_acc_read(sensorDaten_raw* sensorWerte_raw){
		// Reads from TWI, writes to SensorDaten_raw_double Structure
		// Blocking Read
		// 2 Bytes, scaled

		 // Read Twice, if reset was necessary
		 twi_read_twice(2,&packet_lsm_acc);

		 sensorWerte_raw->acc_x_s = data_received_lsm_acc[0] | ( data_received_lsm_acc[1] << 8);
		 sensorWerte_raw->acc_y_s = data_received_lsm_acc[2] | ( data_received_lsm_acc[3] << 8);
		 sensorWerte_raw->acc_z_s = data_received_lsm_acc[4] | ( data_received_lsm_acc[5] << 8);
}


int lsm303_acc_read_ex(sensorDaten_raw* sensorWerte_raw){
		// Reads from TWI, writes to SensorDaten_raw_double Structure
		// Non blocking Read
		// 2 Bytes, scaled

		 // If Read Error					To detect if there is no sensor, return error
		 if (twi_read_ex(2,&packet_lsm_acc)){
			 return 1;						// Error
		 }

		 sensorWerte_raw->acc_x_s = data_received_lsm_acc[0] | ( data_received_lsm_acc[1] << 8);
		 sensorWerte_raw->acc_y_s = data_received_lsm_acc[2] | ( data_received_lsm_acc[3] << 8);
		 sensorWerte_raw->acc_z_s = data_received_lsm_acc[4] | ( data_received_lsm_acc[5] << 8);

		return 0;
}


void lsm303_acc_cond(sensorDaten_raw* sensorWerte_raw){
		// Conditions ADXL Raw Values accordingly

		sensorWerte_raw->acc_x_f = ((double)sensorWerte_raw->acc_x_s) / (double)LSM_ACC_SCALE_FACTOR;
		sensorWerte_raw->acc_y_f = ((double)sensorWerte_raw->acc_y_s) / (double)LSM_ACC_SCALE_FACTOR;
		sensorWerte_raw->acc_z_f = ((double)sensorWerte_raw->acc_z_s) / (double)LSM_ACC_SCALE_FACTOR;

		//sensorWerte_raw->acc_x_f = my_saturate(sensorWerte_raw->acc_x_f, LSM_ACC_ABSOLUTE_MAXIMUM_VALUE);
		//sensorWerte_raw->acc_y_f = my_saturate(sensorWerte_raw->acc_y_f, LSM_ACC_ABSOLUTE_MAXIMUM_VALUE);
		//sensorWerte_raw->acc_z_f = my_saturate(sensorWerte_raw->acc_z_f, LSM_ACC_ABSOLUTE_MAXIMUM_VALUE);

		#ifdef DEBUG_MSG_IMU
			sprintf(debug_line,"TWI A_L: %2.2f | %2.2f | %2.2f || ", sensorWerte_raw->acc_x_f, sensorWerte_raw->acc_y_f, sensorWerte_raw->acc_z_f);
			USART_schreiben(debug_line);
		#endif
}


int lsm303_acc_init(void){
	// Setup the TWI packages

	int status = 0;			// for debug, return USART error


    // --------- 	Setup LSM303	--------------------------------------
	// Set CTRL1 Register
	// TWI chip address to communicate with
	packet_lsm_acc.chip = LSM_ACC_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_lsm_acc.addr = LSM_CTRL1_REG_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_lsm_acc.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_lsm_acc.buffer = (void*) setup_lsm_ctrl1_reg_a;
	// How many bytes do we want to write
	packet_lsm_acc.length = 1;

	// Init LSM, Set Power On
	status += twi_master_write_ex_edit(&AVR32_TWI, &packet_lsm_acc);

	// Set CTRL4 Register
	// TWI chip address to communicate with
	packet_lsm_acc.chip = LSM_ACC_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_lsm_acc.addr = LSM_CTRL4_REG_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_lsm_acc.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_lsm_acc.buffer = (void*) setup_lsm_ctrl4_reg_a;
	// How many bytes do we want to write
	packet_lsm_acc.length = 1;

	// Init LSM, Set Power On
	status += twi_master_write_ex_edit(&AVR32_TWI, &packet_lsm_acc);

	#ifdef DEBUG_MSG_TWI
	if (status != 0){
		sprintf(debug_line, "\n!! TWI LSM Acc Sensor INIT Error %d !!\n", status);
		USART_schreiben(debug_line);
	}
	#endif

	// Read Package
	// TWI chip address to communicate with
	packet_lsm_acc.chip = LSM_ACC_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_lsm_acc.addr = LSM_ACC_SENSING_ADDR_START;
	// Length of the TWI data address segment (1-3 bytes)
	packet_lsm_acc.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_lsm_acc.buffer = (void*) data_received_lsm_acc;
	// How many bytes do we want to write
	packet_lsm_acc.length = 6;
	return status;

	// ------------------------------------------------------------------
}
//*****************************************************************************************


//* LSM Magnetometer *///////////////////////////////////////////////////////////////////////////////
//***************************************************************************************************
void lsm303_mag_read(sensorDaten_raw* sensorWerte_raw){
		// Reads from TWI, writes to SensorDaten_raw_double Structure
		// Blocking Read
		// 2 Bytes, scaled

		 // Read Twice, if reset was necessary
		 twi_read_twice(4,&packet_lsm_mag);


		 sensorWerte_raw->mag_x_s = data_received_lsm_mag[1] | ( data_received_lsm_mag[0] << 8);
		 sensorWerte_raw->mag_z_s = data_received_lsm_mag[3] | ( data_received_lsm_mag[2] << 8);
		 sensorWerte_raw->mag_y_s = data_received_lsm_mag[5] | ( data_received_lsm_mag[4] << 8);
}


int lsm303_mag_read_ex(sensorDaten_raw* sensorWerte_raw){
		// Reads from TWI, writes to SensorDaten_raw_double Structure
		// Non blocking Read
		// 2 Bytes, scaled

		 // If Read Error					To detect if there is no sensor, return error
		 if (twi_read_ex(4,&packet_lsm_mag)){
			 return 1;						// Error
		 }

		 //Switch Z-Axis (Rechts-System)
		 sensorWerte_raw->mag_x_s = data_received_lsm_mag[1] | ( data_received_lsm_mag[0] << 8);
		 sensorWerte_raw->mag_z_s = data_received_lsm_mag[3] | ( data_received_lsm_mag[2] << 8);
		 sensorWerte_raw->mag_y_s = data_received_lsm_mag[5] | ( data_received_lsm_mag[4] << 8);

		return 0;
}


void lsm303_mag_cond(sensorDaten_raw* sensorWerte_raw){
		// Conditions ADXL Raw Values accordingly

		sensorWerte_raw->mag_x_f = ((double)sensorWerte_raw->mag_x_s) * LSM_MAG_SCALE_FACTOR;
		sensorWerte_raw->mag_y_f = ((double)sensorWerte_raw->mag_y_s) * LSM_MAG_SCALE_FACTOR;
		sensorWerte_raw->mag_z_f = ((double)sensorWerte_raw->mag_z_s) * LSM_MAG_SCALE_FACTOR;


		#ifdef DEBUG_MSG_IMU
			sprintf(debug_line,"TWI M_L: %2.2f | %2.2f | %2.2f || ", sensorWerte_raw->mag_x_f, sensorWerte_raw->mag_y_f, sensorWerte_raw->mag_z_f);
			USART_schreiben(debug_line);
		#endif
}



int lsm303_mag_init(void){
	// Setup the TWI packages

	int status = 0;			// for debug, return USART error


    // --------- 	Setup LSM303 Magnetometer	--------------------------------------
	// Set CRA: Output Rate
	// TWI chip address to communicate with
	packet_lsm_mag.chip = LSM_MAG_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_lsm_mag.addr = LSM_MAG_CRA_REG_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_lsm_mag.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_lsm_mag.buffer = (void*) setup_lsm_mag_cra_reg;
	// How many bytes do we want to write
	packet_lsm_mag.length = 1;

	status += twi_master_write_ex_edit(&AVR32_TWI, &packet_lsm_mag);

	// Set CRB:  Set Gain (Magnetic Maximum Scale)
	// TWI chip address to communicate with
	packet_lsm_mag.chip = LSM_MAG_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_lsm_mag.addr = LSM_MAG_CRB_REG_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_lsm_mag.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_lsm_mag.buffer = (void*) setup_lsm_mag_crb_reg;
	// How many bytes do we want to write
	packet_lsm_mag.length = 1;

	status += twi_master_write_ex_edit(&AVR32_TWI, &packet_lsm_mag);

	// Set MR:  Set Continuos Mode
	// TWI chip address to communicate with
	packet_lsm_mag.chip = LSM_MAG_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_lsm_mag.addr = LSM_MAG_MR_REG_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_lsm_mag.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_lsm_mag.buffer = (void*) setup_lsm_mag_mr_reg;
	// How many bytes do we want to write
	packet_lsm_mag.length = 1;

	status += twi_master_write_ex_edit(&AVR32_TWI, &packet_lsm_mag);

	#ifdef DEBUG_MSG_TWI
	if (status != 0){
		sprintf(debug_line, "\n!! TWI LSM Mag Sensor INIT Error %d !!\n", status);
		USART_schreiben(debug_line);
	}
	#endif

	// Read Package
	// TWI chip address to communicate with
	packet_lsm_mag.chip = LSM_MAG_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_lsm_mag.addr = LSM_MAG_SENSING_ADDR_START;
	// Length of the TWI data address segment (1-3 bytes)
	packet_lsm_mag.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_lsm_mag.buffer = (void*) data_received_lsm_mag;
	// How many bytes do we want to write
	packet_lsm_mag.length = 6;
	return status;

	// ------------------------------------------------------------------
}

