/*
 * BlCtrl.c
 *
 *  Created on: 09.12.2011
 *      Author: gageik
 */
// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011 Nils Gageik


// Module for the 4 Brushless Controllers (Motors)

#include "QUAD_TWI_6DOF.h"

// Bl-Controller Packete
twi_package_t packet_blctrl1;
twi_package_t packet_blctrl2;
twi_package_t packet_blctrl3;
twi_package_t packet_blctrl4;

unsigned char stellwert_blctrl_engine1;
unsigned char stellwert_blctrl_engine2;
unsigned char stellwert_blctrl_engine3;
unsigned char stellwert_blctrl_engine4;

extern unsigned int tc_ticks;

void stop_engine(){

	MotorControl Motorn;

	Motorn.stellwert_blctrl_engine1 = 0;
	Motorn.stellwert_blctrl_engine2 = 0;
	Motorn.stellwert_blctrl_engine3 = 0;
	Motorn.stellwert_blctrl_engine4 = 0;

	set_engine(&Motorn);
}



void set_engine(MotorControl* Motoren){
	// Set the 4 Motors according to parameters Stellwert1 till Stellwert4

	 stellwert_blctrl_engine1 = Motoren->stellwert_blctrl_engine1;
	 stellwert_blctrl_engine2 = Motoren->stellwert_blctrl_engine2;
	 stellwert_blctrl_engine3 = Motoren->stellwert_blctrl_engine3;
	 stellwert_blctrl_engine4 = Motoren->stellwert_blctrl_engine4;

     // NONFLYING setzt die Stellwerte immer auf 0 damit der Copter nie starten kann
     #ifdef NONFLYING
		 Motoren.stellwert_blctrl_engine1 = 0;
		 Motoren.stellwert_blctrl_engine2 = 0;
		 Motoren.stellwert_blctrl_engine3 = 0;
		 Motoren.stellwert_blctrl_engine4 = 0;
     #endif

	 // For BLCTRL 2.0
	 packet_blctrl1.addr = (unsigned int) stellwert_blctrl_engine1;
	 packet_blctrl2.addr = (unsigned int) stellwert_blctrl_engine2;
	 packet_blctrl3.addr = (unsigned int) stellwert_blctrl_engine3;
	 packet_blctrl4.addr = (unsigned int) stellwert_blctrl_engine4;

	 // Try till bus was successfully reseted and packet arrived
	 while(twi_debug_status(twi_master_write_ex_edit(&AVR32_TWI, &packet_blctrl1), 11));

	 while(twi_debug_status(twi_master_write_ex_edit(&AVR32_TWI, &packet_blctrl2), 12));

	 while(twi_debug_status(twi_master_write_ex_edit(&AVR32_TWI, &packet_blctrl3), 13));

	 while(twi_debug_status(twi_master_write_ex_edit(&AVR32_TWI, &packet_blctrl4), 14));
}

void blctrl_init(void){
	// Initializes the TWI Packets for the BLCTRL

	// --------- 	Setup TWI Packet for BlCtrl 1 ---------------------------------
	// TWI chip address to communicate with
	packet_blctrl1.chip = BLCTRL_ENGINE1_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_blctrl1.addr = (unsigned int) 0;				// this used BLCTRL 2.0
	// Length of the TWI data address segment (1-3 bytes)
	packet_blctrl1.addr_length = 1;
	// Where to find the data to be written
	packet_blctrl1.buffer = &stellwert_blctrl_engine1;							// this used old BLCTRL
	// How many bytes do we want to write
	packet_blctrl1.length = 1;
	// --------- 	Setup TWI Packet for BlCtrl 2 ---------------------------------
	// TWI chip address to communicate with
	packet_blctrl2.chip = BLCTRL_ENGINE2_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_blctrl2.addr = (unsigned int) 0;
	// Length of the TWI data address segment (1-3 bytes)
	packet_blctrl2.addr_length = 1;
	// Where to find the data to be written
	packet_blctrl2.buffer = &stellwert_blctrl_engine2;
	// How many bytes do we want to write
	packet_blctrl2.length = 1;
	// --------- 	Setup TWI Packet for BlCtrl 3 ---------------------------------
	// TWI chip address to communicate with
	packet_blctrl3.chip = BLCTRL_ENGINE3_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_blctrl3.addr = (unsigned int) 0;
	// Length of the TWI data address segment (1-3 bytes)
	packet_blctrl3.addr_length = 1;
	// Where to find the data to be written
	packet_blctrl3.buffer = &stellwert_blctrl_engine3;
	// How many bytes do we want to write
	packet_blctrl3.length = 1;
	// --------- 	Setup TWI Packet for BlCtrl 4 ---------------------------------
	// TWI chip address to communicate with
	packet_blctrl4.chip = BLCTRL_ENGINE4_TWI_ADDRESS;
	// TWI address/commands to issue to the other chip (node)
	packet_blctrl4.addr = (unsigned int) 0;
	// Length of the TWI data address segment (1-3 bytes)
	packet_blctrl4.addr_length = 1;
	// Where to find the data to be written
	packet_blctrl4.buffer = &stellwert_blctrl_engine4;
	// How many bytes do we want to write
	packet_blctrl4.length = 1;
	// ------------------------------------------------------------------
}
