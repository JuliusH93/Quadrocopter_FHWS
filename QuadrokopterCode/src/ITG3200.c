/*
 * ITG3200.c
 *
 *  Created on: 13.12.2011
 *      Author: gageik
 */

// ITG Gyroscope and IMU3000

#include "basics.h"
#include "AVR_Framework.h"
#include "TWI_NG.h"
#include "ITG3200.h"

twi_package_t packet_itg;

extern char debug_line[DEBUG_BUFFER_SIZE];

char data_received_itg[RECEIVE_DATA_LENGTH];

const U8 setup_data_itg_PM[1] = { 0x01};
const U8 setup_data_dlpf_itg[1] =  { 0x11};
const U8 setup_data_itg_SMPLRT[1] = { 0x07};

void itg3200_read(sensorDaten_raw* sensorWerte_raw){
		 // Reads from TWI

		 // Read Twice, if reset was necessary
		 twi_read(1, &packet_itg);

		 // Compose bytes here
		 // !!!!!!

		#ifdef DEBUG_MSG_IMU_RAW
		 sprintf(debug_line,"TWI: G %x  %x  ", data_received_itg[0], data_received_itg[1]);
		 dip_write_string(debug_line, 1);
		 sprintf(debug_line,"TWI: G %x  %x  ", data_received_itg[2], data_received_itg[3]);
		 dip_write_string(debug_line, 2);
		 sprintf(debug_line,"TWI: G %x  %x  ", data_received_itg[4], data_received_itg[5]);
		 dip_write_string(debug_line, 3);
		#endif
}

int itg3200_read_ex(sensorDaten_raw* sensorWerte_raw){
		 // Reads from TWI, not blocking

		 // If Read Error
		 		 if (twi_read_ex(1, &packet_itg)){
		 			 return 1;					// Error
		 		 }

		 // Compose bytes here
		 // !!!!!!

		#ifdef DEBUG_MSG_IMU_RAW
		 sprintf(debug_line,"TWI: G %x  %x  ", data_received_itg[0], data_received_itg[1]);
		 dip_write_string(debug_line, 1);
		 sprintf(debug_line,"TWI: G %x  %x  ", data_received_itg[2], data_received_itg[3]);
		 dip_write_string(debug_line, 2);
		 sprintf(debug_line,"TWI: G %x  %x  ", data_received_itg[4], data_received_itg[5]);
		 dip_write_string(debug_line, 3);
		#endif

		return 0;
}


void itg3200_cond(sensorDaten_raw* sensorWerte_raw){
		// Conditions ITG Raw Values accordingly


}


int itg3200_init(void){
		// Initialize ITG

		int status = 0;		// For Debug / Error Message

		// --------- 	Setup ITG3200	--------------------------------------
		// DLPF, Full Scale (Digital Low Pass Filter and Scale)
		// TWI chip address to communicate with
		packet_itg.chip = ITG_TWI_ADDRESS;
		// TWI address/commands to issue to the other chip (node)
		packet_itg.addr = ITG_INIT_DLPF_ADDR_START;
		// Length of the TWI data address segment (1-3 bytes)
		packet_itg.addr_length = EEPROM_ADDR_LGT;
		// Where to find the data to be written
		packet_itg.buffer = (void*) setup_data_dlpf_itg;
		// How many bytes do we want to write
		packet_itg.length = 1;

		// Set ITG Digital Filter and Scale
		//while (status != TWI_SUCCESS)
			status += twi_master_write_ex_edit(&AVR32_TWI, &packet_itg);

		// TWI chip address to communicate with
		packet_itg.chip = ITG_TWI_ADDRESS;
		// TWI address/commands to issue to the other chip (node)
		packet_itg.addr = ITG_INIT_PM_ADDR_START;
		// Length of the TWI data address segment (1-3 bytes)
		packet_itg.addr_length = EEPROM_ADDR_LGT;
		// Where to find the data to be written
		packet_itg.buffer = (void*) setup_data_itg_PM;
		// How many bytes do we want to write
		packet_itg.length = 1;


		// Set ITG PM, x Gyro as Reference
		//while (status != TWI_SUCCESS)
			status += twi_master_write_ex_edit(&AVR32_TWI, &packet_itg);


		// TWI chip address to communicate with
		packet_itg.chip = ITG_TWI_ADDRESS;
		// TWI address/commands to issue to the other chip (node)
		packet_itg.addr = ITG_SMPLRT_DIV;
		// Length of the TWI data address segment (1-3 bytes)
		packet_itg.addr_length = EEPROM_ADDR_LGT;
		// Where to find the data to be written
		packet_itg.buffer = (void*) setup_data_itg_SMPLRT;
		// How many bytes do we want to write
		packet_itg.length = 1;


		// Set ITG Sample Rate (here 8ms)
		//while (status != TWI_SUCCESS)
			status += twi_master_write_ex_edit(&AVR32_TWI, &packet_itg);

		#ifdef DEBUG_MSG_TWI
		if (status != 0){
			sprintf(debug_line, "\n!! TWI ITG Sensor INIT Error %d !!\n", status);
			USART_schreiben(debug_line);
		}
		#endif

		// TWI chip address to communicate with
		packet_itg.chip = ITG_TWI_ADDRESS;
		// TWI address/commands to issue to the other chip (node)
		packet_itg.addr = ITG_SENSING_ADDR_START;
		// Length of the TWI data address segment (1-3 bytes)
		packet_itg.addr_length = EEPROM_ADDR_LGT;
		// Where to find the data to be written
		packet_itg.buffer = (void*) data_received_itg;
		// How many bytes do we want to write
		packet_itg.length = RECEIVE_DATA_LENGTH;

		return status;

}
