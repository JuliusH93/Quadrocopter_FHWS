/*
 * MPU6000.c
 *
 *  Created on: 12.11.2013
 *      Author: Gageik
 */
// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011 Nils Gageik


#include "..\QUAD_TWI_6DOF.h"

const U8 setup_data_mpu_pm[1] 			= { 0x03};		// z-Axis as reference (power mode clock source)
const U8 setup_data_mpu_smplrt_div[1] 	= { 0x09};		// Sampleratedivider -> 10ms (For Testing)
const U8 setup_data_mpu_config[1] 		= { 0x03};		// 1kHz, 8.5ms Delay (DLPF, Tiefpassfilter)
const U8 setup_data_mpu_g_cfg[1] 		= { 0x10};		// 1000 °/s
const U8 setup_data_mpu_a_cfg[1] 		= { 0x08};		// +/- 4g
const U8 setup_data_fifo_cfg[1] 		= { 0x78};		// Acc+Gyro -> FIFO
const U8 setup_data_fifo_cfg_en[1] 		= { 0x40};		// FIFO aktivieren
const U8 setup_data_fifo_reset[1] 		= { 0x04};		// FIFO aktivieren

extern unsigned int tc_ticks;

extern char debug_line[DEBUG_BUFFER_SIZE];

char data_received_mpu_acc[RECEIVE_DATA_LENGTH];
char data_received_mpu_gyro[RECEIVE_DATA_LENGTH];

twi_package_t packet_mpu6000_acc;
twi_package_t packet_mpu6000_gyro;

/*
#ifdef MPU_NO_FIFO
char data_received_mpu[MPU_MAX_PACKAGE_SIZE];
#else
char data_received_mpu[MPU_MAX_PACKAGE_SIZE];
#endif
*/

//char fifo_size[2];
//char fifo_ov;

#ifndef MPU_NO_FIFO
twi_package_t packet_mpu6000_fifo;
twi_package_t packet_mpu6000_fifo_enable;
twi_package_t packet_mpu6000_fifo_reset;
twi_package_t packet_mpu6000_status;
#endif

// Initially Reset FIFO
//char fifo_reset = 1;


void mpu6000_acc_read(sensorDaten_raw* sensorWerte_raw){

	twi_read_twice(6, &packet_mpu6000_acc);	 		 // Read Twice, if reset was necessary

	sensorWerte_raw->acc_x_s = data_received_mpu_acc[1] | ( data_received_mpu_acc[0] << 8);
	sensorWerte_raw->acc_y_s = data_received_mpu_acc[3] | ( data_received_mpu_acc[2] << 8);
	sensorWerte_raw->acc_z_s = data_received_mpu_acc[5] | ( data_received_mpu_acc[4] << 8);
}

int mpu6000_acc_read_ex(sensorDaten_raw* sensorWerte_raw){

	// If Read Error
	 if (twi_read_ex(6, &packet_mpu6000_acc)){
		 return 1;					// Error
	 }

	 sensorWerte_raw->acc_x_s = data_received_mpu_acc[1] | ( data_received_mpu_acc[0] << 8);
	 sensorWerte_raw->acc_y_s = data_received_mpu_acc[3] | ( data_received_mpu_acc[2] << 8);
	 sensorWerte_raw->acc_z_s = data_received_mpu_acc[5] | ( data_received_mpu_acc[4] << 8);

	 return 0;
}


void mpu6000_gyro_read(sensorDaten_raw* sensorWerte_raw){
	twi_read_twice(7, &packet_mpu6000_gyro);	 		 // Read Twice, if reset was necessary

	sensorWerte_raw->gyro_x_s = data_received_mpu_gyro[1] | ( data_received_mpu_gyro[0] << 8);
	sensorWerte_raw->gyro_y_s = data_received_mpu_gyro[3] | ( data_received_mpu_gyro[2] << 8);
	sensorWerte_raw->gyro_z_s = data_received_mpu_gyro[5] | ( data_received_mpu_gyro[4] << 8);
}

int mpu6000_gyro_read_ex(sensorDaten_raw* sensorWerte_raw){

	// If Read Error
	 if (twi_read_ex(7, &packet_mpu6000_gyro)){
		 return 1;					// Error
	 }

	sensorWerte_raw->gyro_x_s = data_received_mpu_gyro[1] | ( data_received_mpu_gyro[0] << 8);
	sensorWerte_raw->gyro_y_s = data_received_mpu_gyro[3] | ( data_received_mpu_gyro[2] << 8);
	sensorWerte_raw->gyro_z_s = data_received_mpu_gyro[5] | ( data_received_mpu_gyro[4] << 8);

	 return 0;
}

void mpu6000_gyro_cond(sensorDaten_raw* sensorWerte_raw){

	sensorWerte_raw->gyro_x_f = (double)sensorWerte_raw->gyro_x_s / MPU_GYRO_SCALE_FACTOR;
	sensorWerte_raw->gyro_y_f = (double)sensorWerte_raw->gyro_y_s / MPU_GYRO_SCALE_FACTOR;
	sensorWerte_raw->gyro_z_f = (double)sensorWerte_raw->gyro_z_s / MPU_GYRO_SCALE_FACTOR;

	sensorWerte_raw->gyro_x_f = my_saturate(sensorWerte_raw->gyro_x_f, (double) MPU6000_GYRO_ABSOLUTE_MAXIMUM_VALUE);
	sensorWerte_raw->gyro_y_f = my_saturate(sensorWerte_raw->gyro_y_f, (double) MPU6000_GYRO_ABSOLUTE_MAXIMUM_VALUE);
	sensorWerte_raw->gyro_z_f = my_saturate(sensorWerte_raw->gyro_z_f, (double) MPU6000_GYRO_ABSOLUTE_MAXIMUM_VALUE);

	#ifdef DEBUG_MSG_IMU
	 sprintf(debug_line,"TWI: MPU-G %2.2f | %2.2f | %2.2f |\n", sensorWerte_raw->gyro_x_f, sensorWerte_raw->gyro_y_f, sensorWerte_raw->gyro_z_f);
	 USART_schreiben(debug_line);
	#endif
}


void mpu6000_acc_cond(sensorDaten_raw* sensorWerte_raw){

	sensorWerte_raw->acc_x_f = ((double)sensorWerte_raw->acc_x_s) / (double)MPU_ACC_SCALE_FACTOR;
	sensorWerte_raw->acc_y_f = ((double)sensorWerte_raw->acc_y_s) / (double)MPU_ACC_SCALE_FACTOR;
	sensorWerte_raw->acc_z_f = ((double)sensorWerte_raw->acc_z_s) / (double)MPU_ACC_SCALE_FACTOR;

	sensorWerte_raw->acc_x_f = my_saturate(sensorWerte_raw->acc_x_f, (double) MPU6000_ACC_ABSOLUTE_MAXIMUM_VALUE);
	sensorWerte_raw->acc_y_f = my_saturate(sensorWerte_raw->acc_y_f, (double) MPU6000_ACC_ABSOLUTE_MAXIMUM_VALUE);
	sensorWerte_raw->acc_z_f = my_saturate(sensorWerte_raw->acc_z_f, (double) MPU6000_ACC_ABSOLUTE_MAXIMUM_VALUE);

	#ifdef DEBUG_MSG_IMU
	 sprintf(debug_line,"TWI: MPU-A %2.2f | %2.2f | %2.2f |\n", sensorWerte_raw->acc_x_f, sensorWerte_raw->acc_y_f, sensorWerte_raw->acc_z_f);
	 USART_schreiben(debug_line);
	#endif
}



int mpu6000_init(void){
		// Initialize MPU 6000: Both Acc & Gyro

		twi_package_t packet_mpu6000;

		int status = 0;		// For Debug / Error Message

		packet_mpu6000.chip = MPU_TWI_ADDR;
		packet_mpu6000.addr_length = EEPROM_ADDR_LGT;
		packet_mpu6000.length = 1;

		// --------- 	Setup MPU6000	--------------------------------------
		// PM
		packet_mpu6000.addr = MPU_PM_ADDR;
		packet_mpu6000.buffer = (void*) setup_data_mpu_pm;
		status += twi_master_write_ex_edit(&AVR32_TWI, &packet_mpu6000);

		wait_ms(2);

		// Samplerate Divider
		packet_mpu6000.addr = MPU_SMPLRT_DIV_ADDR;
		packet_mpu6000.buffer = (void*) setup_data_mpu_smplrt_div;
		status += twi_master_write_ex_edit(&AVR32_TWI, &packet_mpu6000);

		wait_ms(2);

		// Config Reg: 1kHz, DLPF 8.5ms
		packet_mpu6000.addr = MPU_CONFIG_ADDR;
		packet_mpu6000.buffer = (void*) setup_data_mpu_config;
		status += twi_master_write_ex_edit(&AVR32_TWI, &packet_mpu6000);

		wait_ms(2);

		// Gyro Config: Skalierungsfaktor
		packet_mpu6000.addr = MPU_GYRO_CONFIG_ADDR;
		packet_mpu6000.buffer = (void*) setup_data_mpu_g_cfg;
		status += twi_master_write_ex_edit(&AVR32_TWI, &packet_mpu6000);

		wait_ms(2);

		// Acc Config: Skalierungsfaktor
		packet_mpu6000.addr = MPU_ACC_CONFIG_ADDR;
		packet_mpu6000.buffer = (void*) setup_data_mpu_a_cfg;
		status += twi_master_write_ex_edit(&AVR32_TWI, &packet_mpu6000);

		wait_ms(2);

		#ifndef MPU_NO_FIFO
		// FIFO Config: Acc+Gyro -> FIFO
		packet_mpu6000.addr = MPU_FIFO_CONFIG_ADDR;
		packet_mpu6000.buffer = (void*) setup_data_fifo_cfg;
		status += twi_master_write_ex_edit(&AVR32_TWI, &packet_mpu6000);

		// FIFO Enable
		packet_mpu6000.addr = MPU_FIFO_EN_CONFIG_ADDR;
		packet_mpu6000.buffer = (void*) setup_data_fifo_cfg_en;
		status += twi_master_write_ex_edit(&AVR32_TWI, &packet_mpu6000);
		#endif

		#ifdef DEBUG_MSG_TWI
		if (status != 0){
			sprintf(debug_line, "\n!! TWI MPU6000 Sensor INIT Error %d !!\n", status);
			USART_schreiben(debug_line);
		}
		#endif

		#ifndef MPU_NO_FIFO
		// Set Sensing Start Addr
		packet_mpu6000.addr = MPU_FIFO_ADDR;
		// Amout of Bytes to be Read
		packet_mpu6000.length = MPU_PACKAGE_RECEIVE_DATA_LENGTH;

		//Setup FIFO Read Zählerstand Package
		packet_mpu6000_fifo.chip = MPU_TWI_ADDR;
		packet_mpu6000_fifo.addr_length = EEPROM_ADDR_LGT;
		packet_mpu6000_fifo.addr = MPU_FIFO_COUNT_ADDR;
		packet_mpu6000_fifo.length = 2;								// FIFO Zählerstand in 2 Bytes als unsigned short
		packet_mpu6000_fifo.buffer = (void*) fifo_size;

		//Setup FIFO Read Status Package
		packet_mpu6000_status.chip = MPU_TWI_ADDR;
		packet_mpu6000_status.addr_length = EEPROM_ADDR_LGT;
		packet_mpu6000_status.addr = MPU_STATUS_ADDR;
		packet_mpu6000_status.length = 1;
		packet_mpu6000_status.buffer = (void*) &fifo_ov;

		//Setup FIFO Reset Package
		packet_mpu6000_fifo_reset.chip = MPU_TWI_ADDR;
		packet_mpu6000_fifo_reset.addr_length = EEPROM_ADDR_LGT;
		packet_mpu6000_fifo_reset.addr = MPU_FIFO_EN_CONFIG_ADDR;
		packet_mpu6000_fifo_reset.length = 1;
		packet_mpu6000_fifo_reset.buffer = (void*) setup_data_fifo_reset;

		//FIFO Enable Package
		packet_mpu6000_fifo_enable.chip = MPU_TWI_ADDR;
		packet_mpu6000_fifo_enable.addr_length = EEPROM_ADDR_LGT;
		packet_mpu6000_fifo_enable.addr = MPU_FIFO_EN_CONFIG_ADDR;
		packet_mpu6000_fifo_enable.length = 1;
		packet_mpu6000_fifo_enable.buffer = (void*) setup_data_fifo_cfg_en;
		#endif

		/*
		#ifdef MPU_NO_FIFO
		packet_mpu6000.addr = MPU_SENS_START_ADDR;
		packet_mpu6000.length = MPU_MAX_PACKAGE_SIZE;
		packet_mpu6000.buffer = (void*) data_received_mpu;
		#endif
		*/

		// Create Single Packet for MPU Acc
		packet_mpu6000_acc.chip = MPU_TWI_ADDR;
		packet_mpu6000_acc.addr_length = EEPROM_ADDR_LGT;
		packet_mpu6000_acc.length = RECEIVE_DATA_LENGTH;
		packet_mpu6000_acc.addr = MPU_SENS_START_ADDR;
		packet_mpu6000_acc.buffer = data_received_mpu_acc;

		// Create Single Packet for MPU Gyro
		packet_mpu6000_gyro.chip = MPU_TWI_ADDR;
		packet_mpu6000_gyro.addr_length = EEPROM_ADDR_LGT;
		packet_mpu6000_gyro.length = RECEIVE_DATA_LENGTH;
		packet_mpu6000_gyro.addr = MPU_GYRO_START_ADDR_;
		packet_mpu6000_gyro.buffer = data_received_mpu_gyro;

		return status;
}


// Obsolete Functions
/*
void mpu6000_read(sensorDaten_raw* sensorWerte_raw){
		 // Reads from TWI, blocking

		#ifndef MPU_NO_FIFO
		// **************** FIFO Overflow Check ***********************
		// Read FIFO Ooverflow Reg.
		twi_read_twice(5, &packet_mpu6000_status); 	 // Read Twice, if reset was necessary

		// USART_schreibe_float((float)data_received_mpu[0]);

		 if (fifo_ov >= 16)			// Overflow occured -> datasheet p. 28
			 fifo_reset = 1;

		// ***************** FIFO SIZE Check ***************************
		// Read FIFO Size
		twi_read_twice(5, &packet_mpu6000_fifo);	 // Read Twice, if reset was necessary

		short fifo_count = fifo_size[1] | ( fifo_size[0] << 8);

		//USART_schreibe_float((float) fifo_count);

		// *************** READ NEW DATA ******************************
		if (fifo_count > MPU_PACKAGE_RECEIVE_DATA_LENGTH){

				if (fifo_count > MPU_MAX_PACKAGE_SIZE)
					fifo_count = MPU_MAX_PACKAGE_SIZE;
				else
					fifo_count = fifo_count - (fifo_count % MPU_PACKAGE_RECEIVE_DATA_LENGTH);	// Auf ganzzahlige Vielfache von MPU_PACKAGE_RECEIVE_DATA_LENGTH bringen

				USART_schreibe_float((float) fifo_count);

				packet_mpu6000.length = (unsigned int)fifo_count;
		#endif

				twi_read_twice(5, &packet_mpu6000);	 		 // Read Twice, if reset was necessary

				sensorWerte_raw->acc_x_s = data_received_mpu[MPU_ACC_X_L] | ( data_received_mpu[MPU_ACC_X_H] << 8);
				sensorWerte_raw->acc_y_s = data_received_mpu[MPU_ACC_Y_L] | ( data_received_mpu[MPU_ACC_Y_H] << 8);
				sensorWerte_raw->acc_z_s = data_received_mpu[MPU_ACC_Z_L] | ( data_received_mpu[MPU_ACC_Z_H] << 8);

				sensorWerte_raw->gyro_x_s = data_received_mpu[MPU_GYRO_X_L] | ( data_received_mpu[MPU_GYRO_X_H] << 8);
				sensorWerte_raw->gyro_y_s = data_received_mpu[MPU_GYRO_Y_L] | ( data_received_mpu[MPU_GYRO_Y_H] << 8);
				sensorWerte_raw->gyro_z_s = data_received_mpu[MPU_GYRO_Z_L] | ( data_received_mpu[MPU_GYRO_Z_H] << 8);

		#ifndef MPU_NO_FIFO
				}
		#endif
}

int mpu6000_read_ex(sensorDaten_raw* sensorWerte_raw){
		 // Reads from TWI, not blocking

		#ifndef MPU_NO_FIFO
		char error = 0;

		// **************** FIFO Overflow Check ***********************
		// Read FIFO Ooverflow Reg.
		if (twi_read_ex(5, &packet_mpu6000_status)){
						 return 1;					// Error
					 }

		 //USART_schreibe_float((float)data_received_mpu[0]);

		 if (fifo_ov >= 16)			// Overflow occured -> datasheet p. 28
			 fifo_reset = 1;

		if (fifo_reset){
			USART_schreiben("FIFO Reset");
			error = twi_master_write_ex_edit(&AVR32_TWI, &packet_mpu6000_fifo_reset);
			error = error + twi_master_write_ex_edit(&AVR32_TWI, &packet_mpu6000_fifo_enable);

			if (error){
						return 1;					// Error
								 }
			fifo_reset = 0;
		}

		// ***************** FIFO SIZE Check ***************************
		 // Read FIFO Size
		 if (twi_read_ex(5, &packet_mpu6000_fifo)){
			 			 return 1;					// Error
			 		 }

		 short fifo_count = fifo_size[1] | ( fifo_size[0] << 8);

		 //USART_schreibe_float((float) fifo_count);

		// *************** READ NEW DATA ******************************
		if (fifo_count > MPU_PACKAGE_RECEIVE_DATA_LENGTH){

				if (fifo_count > MPU_MAX_PACKAGE_SIZE)
					fifo_count = MPU_MAX_PACKAGE_SIZE;
				else
					fifo_count = fifo_count - (fifo_count % MPU_PACKAGE_RECEIVE_DATA_LENGTH);	// Auf ganzzahlige Vielfache von MPU_PACKAGE_RECEIVE_DATA_LENGTH bringen

				packet_mpu6000.length = (unsigned int)fifo_count;

		#endif
				// If Read Error
		 		 if (twi_read_ex(5, &packet_mpu6000)){
		 			 return 1;					// Error
		 		 }

				sensorWerte_raw->acc_x_s = data_received_mpu[MPU_ACC_X_L] | ( data_received_mpu[MPU_ACC_X_H] << 8);
				sensorWerte_raw->acc_y_s = data_received_mpu[MPU_ACC_Y_L] | ( data_received_mpu[MPU_ACC_Y_H] << 8);
				sensorWerte_raw->acc_z_s = data_received_mpu[MPU_ACC_Z_L] | ( data_received_mpu[MPU_ACC_Z_H] << 8);

				sensorWerte_raw->gyro_x_s = data_received_mpu[MPU_GYRO_X_L] | ( data_received_mpu[MPU_GYRO_X_H] << 8);
				sensorWerte_raw->gyro_y_s = data_received_mpu[MPU_GYRO_Y_L] | ( data_received_mpu[MPU_GYRO_Y_H] << 8);
				sensorWerte_raw->gyro_z_s = data_received_mpu[MPU_GYRO_Z_L] | ( data_received_mpu[MPU_GYRO_Z_H] << 8);
		#ifndef MPU_NO_FIFO
				}
		#endif

		return 0;
}


void mpu6000_cond(sensorDaten_raw* sensorWerte_raw){
		// Conditions ITG Raw Values accordingly

		sensorWerte_raw->gyro_x_f = (double)sensorWerte_raw->gyro_x_s / MPU_GYRO_SCALE_FACTOR;
		sensorWerte_raw->gyro_y_f = (double)sensorWerte_raw->gyro_y_s / MPU_GYRO_SCALE_FACTOR;
		sensorWerte_raw->gyro_z_f = (double)sensorWerte_raw->gyro_z_s / MPU_GYRO_SCALE_FACTOR;

		sensorWerte_raw->gyro_x_f = my_saturate(sensorWerte_raw->gyro_x_f, (double) MPU6000_GYRO_ABSOLUTE_MAXIMUM_VALUE);
		sensorWerte_raw->gyro_y_f = my_saturate(sensorWerte_raw->gyro_y_f, (double) MPU6000_GYRO_ABSOLUTE_MAXIMUM_VALUE);
		sensorWerte_raw->gyro_z_f = my_saturate(sensorWerte_raw->gyro_z_f, (double) MPU6000_GYRO_ABSOLUTE_MAXIMUM_VALUE);

		sensorWerte_raw->acc_x_f = ((double)sensorWerte_raw->acc_x_s) / (double)MPU_ACC_SCALE_FACTOR;
		sensorWerte_raw->acc_y_f = ((double)sensorWerte_raw->acc_y_s) / (double)MPU_ACC_SCALE_FACTOR;
		sensorWerte_raw->acc_z_f = ((double)sensorWerte_raw->acc_z_s) / (double)MPU_ACC_SCALE_FACTOR;

		sensorWerte_raw->acc_x_f = my_saturate(sensorWerte_raw->acc_x_f, (double) MPU6000_ACC_ABSOLUTE_MAXIMUM_VALUE);
		sensorWerte_raw->acc_y_f = my_saturate(sensorWerte_raw->acc_y_f, (double) MPU6000_ACC_ABSOLUTE_MAXIMUM_VALUE);
		sensorWerte_raw->acc_z_f = my_saturate(sensorWerte_raw->acc_z_f, (double) MPU6000_ACC_ABSOLUTE_MAXIMUM_VALUE);

		#ifdef DEBUG_MSG_IMU
		 sprintf(debug_line,"TWI: MPU %2.2f | %2.2f | %2.2f |\n", sensorWerte_raw->gyro_x_f, sensorWerte_raw->gyro_y_f, sensorWerte_raw->gyro_z_f);
		 USART_schreiben(debug_line);
		#endif
}
*/
