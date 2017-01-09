/*
 * L3G4200D.c
 *
 *  Created on: 24.09.2012
 *      Author: Gageik
 */
// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011 Nils Gageik


// ST 3 axis Gyro from Pololu IMU01a Board

#include "L3G4200D.h"
#include "..\QUAD_TWI_6DOF.h"

twi_package_t packet_l3g;

char data_received_l3g[RECEIVE_DATA_LENGTH];

const U8 setup_l3g_ctrl1_reg_g[1] =  { L3G_CTRL_REG1};
const U8 setup_l3g_ctrl2_reg_g[1] =  { L3G_CTRL_REG2};
const U8 setup_l3g_ctrl4_reg_g[1] =  { L3G_CTRL_REG4};
const U8 setup_l3g_ctrl5_reg_g[1] =  { L3G_CTRL_REG5};


extern char debug_line[DEBUG_BUFFER_SIZE];


void l3g_read(sensorDaten_raw* sensorWerte_raw){
		// Reads from TWI, writes to SensorDaten_raw_double Structure
		// Blocking Read
		// 2 Bytes, scaled

		 // Read Twice, if reset was necessary
		 twi_read_twice(3,&packet_l3g);

		 sensorWerte_raw->gyro_x_s = data_received_l3g[0] | ( data_received_l3g[1] << 8);
		 sensorWerte_raw->gyro_y_s = data_received_l3g[2] | ( data_received_l3g[3] << 8);
		 sensorWerte_raw->gyro_z_s = data_received_l3g[4] | ( data_received_l3g[5] << 8);
}


int l3g_read_ex(sensorDaten_raw* sensorWerte_raw){
		// Reads from TWI, writes to SensorDaten_raw_double Structure
		// Non blocking Read
		// 2 Bytes, scaled

		 // If Read Error					To detect if there is no sensor, return error
		 if (twi_read_ex(3,&packet_l3g)){
			 return 1;						// Error
		 }

		sensorWerte_raw->gyro_x_s = data_received_l3g[0] | ( data_received_l3g[1] << 8);
		sensorWerte_raw->gyro_y_s = data_received_l3g[2] | ( data_received_l3g[3] << 8);
		sensorWerte_raw->gyro_z_s = data_received_l3g[4] | ( data_received_l3g[5] << 8);

		return 0;
}


void l3g_cond(sensorDaten_raw* sensorWerte_raw){
		// Conditions ADXL Raw Values accordingly

		sensorWerte_raw->gyro_x_f = ((double)sensorWerte_raw->gyro_x_s) * (double)L3G_GYRO_SCALE / 1000.0;
		sensorWerte_raw->gyro_y_f = ((double)sensorWerte_raw->gyro_y_s) * (double)L3G_GYRO_SCALE / 1000.0;
		sensorWerte_raw->gyro_z_f = ((double)sensorWerte_raw->gyro_z_s) * (double)L3G_GYRO_SCALE / 1000.0;

		sensorWerte_raw->gyro_x_f = my_saturate(sensorWerte_raw->gyro_x_f, (double) L3G_ABSOLUTE_MAXIMUM_VALUE);
		sensorWerte_raw->gyro_y_f = my_saturate(sensorWerte_raw->gyro_y_f, (double) L3G_ABSOLUTE_MAXIMUM_VALUE);
		sensorWerte_raw->gyro_z_f = my_saturate(sensorWerte_raw->gyro_z_f, (double) L3G_ABSOLUTE_MAXIMUM_VALUE);


		#ifdef DEBUG_MSG_IMU
			sprintf(debug_line,"TWI G: %2.2f | %2.2f | %2.2f || ", sensorWerte_raw->gyro_x_f, sensorWerte_raw->gyro_y_f, sensorWerte_raw->gyro_z_f);
			USART_schreiben(debug_line);
		#endif
}


int l3g_init(void){
	// Setup the TWI packages

	int status = 0;			// for debug, return USART error


    // --------- 	Setup L3G4200D Gyro	--------------------------------------
	// Set CTRL1 Register: Frequency and Cut-Off Frequenz
	// TWI chip address to communicate with
	packet_l3g.chip = L3G_GYRO_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_l3g.addr = L3G_CTRL1_REG_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_l3g.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_l3g.buffer = (void*) setup_l3g_ctrl1_reg_g;
	// How many bytes do we want to write
	packet_l3g.length = 1;

	status += twi_master_write_ex_edit(&AVR32_TWI, &packet_l3g);

	// Set CTRL2 Register: High Pass Filter
	// TWI chip address to communicate with
	packet_l3g.chip = L3G_GYRO_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_l3g.addr = L3G_CTRL2_REG_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_l3g.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_l3g.buffer = (void*) setup_l3g_ctrl2_reg_g;
	// How many bytes do we want to write
	packet_l3g.length = 1;

	status += twi_master_write_ex_edit(&AVR32_TWI, &packet_l3g);

	// Set CTRL4 Register
	// TWI chip address to communicate with
	packet_l3g.chip = L3G_GYRO_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_l3g.addr = L3G_CTRL4_REG_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_l3g.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_l3g.buffer = (void*) setup_l3g_ctrl4_reg_g;
	// How many bytes do we want to write
	packet_l3g.length = 1;

	status += twi_master_write_ex_edit(&AVR32_TWI, &packet_l3g);

	// Set CTRL5 Register
	// TWI chip address to communicate with
	packet_l3g.chip = L3G_GYRO_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_l3g.addr = L3G_CTRL5_REG_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_l3g.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_l3g.buffer = (void*) setup_l3g_ctrl5_reg_g;
	// How many bytes do we want to write
	packet_l3g.length = 1;

	status += twi_master_write_ex_edit(&AVR32_TWI, &packet_l3g);

	#ifdef DEBUG_MSG_TWI
	if (status != 0){
		sprintf(debug_line, "\n!! TWI L3G Sensor INIT Error %d !!\n", status);
		USART_schreiben(debug_line);
	}
	#endif

	// Read Package
	// TWI chip address to communicate with
	packet_l3g.chip = L3G_GYRO_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_l3g.addr = L3G_GYRO_SENSING_ADDR_START;
	// Length of the TWI data address segment (1-3 bytes)
	packet_l3g.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_l3g.buffer = (void*) data_received_l3g;
	// How many bytes do we want to write
	packet_l3g.length = 6;
	return status;

	// ------------------------------------------------------------------
}
