/*
 * ADXL345.c
 *
 *  Created on: 13.12.2011
 *      Author: gageik
 */

// ADXL Accelerometer

#include "basics.h"
#include "AVR_Framework.h"
#include "TWI_NG.h"
#include "ADXL345.h"

char data_received_adxl[RECEIVE_DATA_LENGTH];			// Contains Data after read

twi_package_t packet_adxl;

extern char debug_line[DEBUG_BUFFER_SIZE];

// Setup Value (datas) for ADXL to startup
const char setup_data_POWER_CTRL[1] =  { 0x08};
const char setup_data_DATA_FORMAT[1] =  {	0x0B};
const char setup_data_BW_RATE[1] = { 0x0D};

void adxl345_read(sensorDaten_raw* sensorWerte_raw){
		// Reads from TWI, writes to SensorDaten_raw_double Structure
		// Blocking Read
		// 2 Bytes, scaled

		 // Read Twice, if reset was necessary
		 twi_read(0,&packet_adxl);

		 // Compose bytes here
		 // !!!!!!


		#ifdef DEBUG_MSG_IMU_RAW
		 sprintf(debug_line,"TWI: A %x  %x  ", data_received_adxl[0], data_received_adxl[1]);
		 dip_write_string(debug_line, 1);
		 sprintf(debug_line,"TWI: A %x  %x  ", data_received_adxl[2], data_received_adxl[3]);
		 dip_write_string(debug_line, 2);
		 sprintf(debug_line,"TWI: A %x  %x  ", data_received_adxl[4], data_received_adxl[5]);
		 dip_write_string(debug_line, 3);
		#endif
}


int adxl345_read_ex(sensorDaten_raw* sensorWerte_raw){
		// Reads from TWI, writes to SensorDaten_raw_double Structure
		// Non blocking Read
		// 2 Bytes, scaled

		 // If Read Error					To detect if there is no sensor, return error
		 if (twi_read_ex(0,&packet_adxl)){
			 return 1;						// Error
		 }

		 // Read Twice, if reset was necessary
		 twi_read(0,&packet_adxl);

		 // Compose bytes here
		 // !!!!!!

		#ifdef DEBUG_MSG_IMU_RAW
		 sprintf(debug_line,"TWI: A %x  %x  ", data_received_adxl[0], data_received_adxl[1]);
		 dip_write_string(debug_line, 1);
		 sprintf(debug_line,"TWI: A %x  %x  ", data_received_adxl[2], data_received_adxl[3]);
		 dip_write_string(debug_line, 2);
		 sprintf(debug_line,"TWI: A %x  %x  ", data_received_adxl[4], data_received_adxl[5]);
		 dip_write_string(debug_line, 3);
		#endif

		return 0;
}


void adxl345_cond(sensorDaten_raw* sensorWerte_raw){
		// Conditions ADXL Raw Values accordingly



}


int adxl345_init(void){
	// Setup the TWI packages

	int status = 0;			// for debug, return USART error

	// --------- 	Setup ADXL345	--------------------------------------
	// Set POWER_CTRL Register
	// TWI chip address to communicate with
	packet_adxl.chip = ADXL_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_adxl.addr = ADXL_POWER_CTRL_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_adxl.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_adxl.buffer = (void*) setup_data_POWER_CTRL;
	// How many bytes do we want to write
	packet_adxl.length = 1;

	// Init ADXL, Set Power On
	//while (status != TWI_SUCCESS)
		status += twi_master_write_ex_edit(&AVR32_TWI, &packet_adxl);


	// Set DATA_FORMAT Register
	// TWI chip address to communicate with
	packet_adxl.chip = ADXL_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_adxl.addr = ADXL_DATA_FORMAT_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_adxl.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_adxl.buffer = (void*) setup_data_DATA_FORMAT;
	// How many bytes do we want to write
	packet_adxl.length = 1;

	// Set ADXL Data Format
	//while (status != TWI_SUCCESS)
		status += twi_master_write_ex_edit(&AVR32_TWI, &packet_adxl);


	// Set BW_RATE Register (Sample Rate)
	// TWI chip address to communicate with
	packet_adxl.chip = ADXL_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_adxl.addr = ADXL_BW_RATE_ADR;
	// Length of the TWI data address segment (1-3 bytes)
	packet_adxl.addr_length = EEPROM_ADDR_LGT;
	// Where to find the data to be written
	packet_adxl.buffer = (void*) setup_data_BW_RATE;
	// How many bytes do we want to write
	packet_adxl.length = 1;

	// Set ADXL Data Format
	//while (status != TWI_SUCCESS)
		status += twi_master_write_ex_edit(&AVR32_TWI, &packet_adxl);

	#ifdef DEBUG_MSG_TWI
	if (status != 0){
		sprintf(debug_line, "\n!! TWI ADXL Sensor INIT Error %d !!\n", status);
		USART_schreiben(debug_line);
	}
	#endif

	// TWI chip address to communicate with
	packet_adxl.chip = ADXL_TWI_ADDRESS ;
	// Length of the TWI data address segment (1-3 bytes)
	packet_adxl.addr_length = EEPROM_ADDR_LGT;
	// How many bytes do we want to write
	packet_adxl.length = RECEIVE_DATA_LENGTH;
	// TWI address/commands to issue to the other chip (node)
	packet_adxl.addr = ADXL_SENSING_ADDR_START;
	// Where to find the data to be written
	packet_adxl.buffer = (void*) data_received_adxl;

	return status;

	// ------------------------------------------------------------------
}
