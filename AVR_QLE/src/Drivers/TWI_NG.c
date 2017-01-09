/*
 * TWI_NG.c
 *
 *  Created on: 05.08.2011
 *      Author: gageik
 */
// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011 Nils Gageik

// This Software uses the AVR Framework:
// Copyright (c) 2009 Atmel Corporation. All rights reserved.
#include "..\atmel.h"			//Mandantory

// ******************************************
// *		TWI & IMU driver				*
// ******************************************
#include "..\QUAD_TWI_6DOF.h"


//! Pointer to the instance 1 of the TWI registers for IT.
static volatile avr32_twi_t *twi_inst1 = &AVR32_TWI;

//! Pointer to the applicative TWI transmit buffer.
static const unsigned char *volatile twi_tx_data = NULL;
//! Pointer to the applicative TWI receive buffer.
static volatile unsigned char *volatile twi_rx_data = NULL;

//! Remaining number of bytes to transmit.
static volatile int twi_tx_nb_bytes = 0;
//! Remaining number of bytes to receive.
static volatile int twi_rx_nb_bytes = 0;

//! Add NACK boolean.
static volatile Bool twi_nack = FALSE;

//! IT mask.
static volatile unsigned long twi_it_mask;


// State Parameter to know: In the one but last twi communication was an error, repeat!
int last_twi_read_error = 0;

unsigned int amount_twi_resets = 0;

char twi_deactivate = 0;			//If too many TWI errors occured, set TWI off	(emergency off)

// Debugging/Printf Array
extern char debug_line[DEBUG_BUFFER_SIZE];

// TWI Vars
twi_options_t opt;

// Debugging
char Test1_twi[1];
char Test2_twi[1];

extern unsigned int tc_ticks;

extern steuerDaten steuerWerte;
extern sensorDaten sensorWerte;

extern short engine_on;

char buffer1[RECEIVE_DATA_LENGTH], buffer2[RECEIVE_DATA_LENGTH], buffer3[RECEIVE_DATA_LENGTH];

// Debugging value;
int equal_counter = 0;

/*! \brief TWI interrupt handler.
 */
__attribute__((__interrupt__))
static void twi_master_inst1_interrupt_handler(void)
{
  // get masked status register value
  int status = twi_inst1->sr & twi_it_mask;

  // this is a NACK
  if (status & AVR32_TWI_SR_NACK_MASK)
  {
    goto nack;
  }
  // this is a RXRDY
  else if (status & AVR32_TWI_SR_RXRDY_MASK)
  {
    // get data from Receive Holding Register
    *twi_rx_data = twi_inst1->rhr;
    twi_rx_data++;
    // last byte to receive
    if(--twi_rx_nb_bytes==1)
    {
      // set stop bit
      twi_inst1->cr = AVR32_TWI_STOP_MASK;
    }
    // receive complete
    if (twi_rx_nb_bytes==0)
    {
      // finish the receive operation
      goto complete;
    }
  }
  // this is a TXRDY
  else if (status & AVR32_TWI_SR_TXRDY_MASK)
  {
    // decrease transmited bytes number
    twi_tx_nb_bytes--;
    // no more bytes to transmit
    if (twi_tx_nb_bytes <= 0)
    {
      // enable TXCOMP IT and unmask all others IT
      twi_it_mask = AVR32_TWI_IER_TXCOMP_MASK;
      twi_inst1->ier = twi_it_mask;
    }
    else
    {
      // put the byte in the Transmit Holding Register
      twi_inst1->thr = *twi_tx_data++;
    }
  }
  // this is a TXCOMP
  else if (status & AVR32_TWI_SR_TXCOMP_MASK)
  {
    // finish the transmit operation
    goto complete;
  }

  return;

nack:
  twi_nack = TRUE;

complete:
  // disable all interrupts
  twi_disable_interrupt(twi_inst1);

  return;
}


/*! \brief Set the twi bus speed in cojunction with the clock frequency
 *
 * \param twi    Base address of the TWI (i.e. &AVR32_TWI).
 * \param speed  The desired twi bus speed
 * \param pba_hz The current running PBA clock frequency
 * \return TWI_SUCCESS
 */
static int twi_set_speed(volatile avr32_twi_t *twi, unsigned int speed, unsigned long pba_hz)
{
  unsigned int ckdiv = 0;
  unsigned int c_lh_div;

  c_lh_div = pba_hz / (speed * 2) - 4;

  // cldiv must fit in 8 bits, ckdiv must fit in 3 bits
  while ((c_lh_div > 0xFF) && (ckdiv < 0x7))
  {
    // increase clock divider
    ckdiv++;
    // divide cldiv value
    c_lh_div /= 2;
  }

  // set clock waveform generator register
  twi->cwgr = ((c_lh_div << AVR32_TWI_CWGR_CLDIV_OFFSET) |
              (c_lh_div << AVR32_TWI_CWGR_CHDIV_OFFSET) |
              (ckdiv << AVR32_TWI_CWGR_CKDIV_OFFSET));

  return TWI_SUCCESS;
}


int twi_master_init(volatile avr32_twi_t *twi, const twi_options_t *opt)
{
  Bool global_interrupt_enabled = Is_global_interrupt_enabled();
  int status = TWI_SUCCESS;

  // Disable TWI interrupts
  if (global_interrupt_enabled) Disable_global_interrupt();
  twi->idr = ~0UL;
  twi->sr;

  // Reset TWI
  twi->cr = AVR32_TWI_CR_SWRST_MASK;
  if (global_interrupt_enabled) Enable_global_interrupt();

  // Dummy read in SR
  twi->sr;

  // Disable all interrupts
  Disable_global_interrupt();

  // Register TWI handler on level 2
  INTC_register_interrupt( &twi_master_inst1_interrupt_handler, AVR32_TWI_IRQ, AVR32_INTC_INT2);

  // Enable all interrupts
  Enable_global_interrupt();

  // Select the speed
  twi_set_speed(twi, opt->speed, opt->pba_hz);

  // Probe the component
  //status = twi_probe(twi, opt->chip);

  return status;
}





void twi_disable_interrupt(volatile avr32_twi_t *twi)
{
  Bool global_interrupt_enabled = Is_global_interrupt_enabled();

  if (global_interrupt_enabled) Disable_global_interrupt();
  twi->idr = ~0UL;
  twi->sr;
  if (global_interrupt_enabled) Enable_global_interrupt();
}


int twi_probe(volatile avr32_twi_t *twi, char chip_addr)
{
  twi_package_t package;
  char data[1] = {0};

  // data to send
  package.buffer = data;
  // chip address
  package.chip = chip_addr;
  // frame length
  package.length = 1;
  // address length
  package.addr_length = 0;
  // internal chip address
  package.addr = 0;
  // perform a master write access
  return (twi_master_write(twi, &package));
}


int twi_master_read(volatile avr32_twi_t *twi, const twi_package_t *package)
{
  // check argument
  if (package->length == 0)
  {
    return TWI_INVALID_ARGUMENT;
  }

  while( twi_is_busy() ) {};

  twi_nack = FALSE;

  // set read mode, slave address and 3 internal address byte length
  twi->mmr = (package->chip << AVR32_TWI_MMR_DADR_OFFSET) |
             ((package->addr_length << AVR32_TWI_MMR_IADRSZ_OFFSET) & AVR32_TWI_MMR_IADRSZ_MASK) |
             (1 << AVR32_TWI_MMR_MREAD_OFFSET);

  // set internal address for remote chip
  twi->iadr = package->addr;

  // get a pointer to applicative data
  twi_rx_data = package->buffer;

  // get a copy of nb bytes to read
  twi_rx_nb_bytes = package->length;

  // Enable master transfer
  twi->cr =  AVR32_TWI_CR_MSEN_MASK;

  // Send start condition
  twi->cr = AVR32_TWI_START_MASK;

  // only one byte to receive
  if(twi_rx_nb_bytes == 1)
  {
    // set stop bit
    twi->cr = AVR32_TWI_STOP_MASK;
  }

  // mask NACK and RXRDY interrupts
  twi_it_mask = AVR32_TWI_IER_NACK_MASK | AVR32_TWI_IER_RXRDY_MASK;

  // update IMR through IER
  twi->ier = twi_it_mask;

  // get data
  while (!twi_nack && twi_rx_nb_bytes);

  // Disable master transfer
  twi->cr =  AVR32_TWI_CR_MSDIS_MASK;

  if( twi_nack )
    return TWI_RECEIVE_NACK;

  return TWI_SUCCESS;
}


int twi_master_write(volatile avr32_twi_t *twi, const twi_package_t *package)
{
  // No data to send
  if (package->length == 0)
  {
    return TWI_INVALID_ARGUMENT;
  }

  while( twi_is_busy() ) {};

  twi_nack = FALSE;

  // Enable master transfer, disable slave
  twi->cr =   AVR32_TWI_CR_MSEN_MASK
#ifndef AVR32_TWI_180_H_INCLUDED
            | AVR32_TWI_CR_SVDIS_MASK
#endif
            ;

  // set write mode, slave address and 3 internal address byte length
  twi->mmr = (0 << AVR32_TWI_MMR_MREAD_OFFSET) |
             (package->chip << AVR32_TWI_MMR_DADR_OFFSET) |
             ((package->addr_length << AVR32_TWI_MMR_IADRSZ_OFFSET) & AVR32_TWI_MMR_IADRSZ_MASK);

  // set internal address for remote chip
  twi->iadr = package->addr;

  // get a pointer to applicative data
  twi_tx_data = package->buffer;

  // get a copy of nb bytes to write
  twi_tx_nb_bytes = package->length;

  // put the first byte in the Transmit Holding Register
  twi_inst1->thr = *twi_tx_data++;

  // mask NACK and TXRDY interrupts
  twi_it_mask = AVR32_TWI_IER_NACK_MASK | AVR32_TWI_IER_TXRDY_MASK;

  // update IMR through IER
  twi->ier = twi_it_mask;

  // send data
  while (!twi_nack && twi_tx_nb_bytes);

  // Disable master transfer
  twi->cr =  AVR32_TWI_CR_MSDIS_MASK;

  if( twi_nack )
    return TWI_RECEIVE_NACK;

  return TWI_SUCCESS;
}


//! This function is not blocking.
int twi_master_write_ex(volatile avr32_twi_t *twi, const twi_package_t *package)
{
  int status = TWI_SUCCESS;

  if( twi_nack )
    status = TWI_RECEIVE_NACK;  // Previous transaction returns a NACK

  else if( twi_tx_nb_bytes )
    return TWI_BUSY;          // Still transmitting...

  // No data to send
  if (package->length == 0)
  {
    return TWI_INVALID_ARGUMENT;
  }

  twi_nack = FALSE;

  // Enable master transfer, disable slave
  twi->cr =   AVR32_TWI_CR_MSEN_MASK
#ifndef AVR32_TWI_180_H_INCLUDED
            | AVR32_TWI_CR_SVDIS_MASK
#endif
            ;

  // set write mode, slave address and 3 internal address byte length
  twi->mmr = (0 << AVR32_TWI_MMR_MREAD_OFFSET) |
             (package->chip << AVR32_TWI_MMR_DADR_OFFSET) |
             ((package->addr_length << AVR32_TWI_MMR_IADRSZ_OFFSET) & AVR32_TWI_MMR_IADRSZ_MASK);

  // set internal address for remote chip
  twi->iadr = package->addr;

  // get a pointer to applicative data
  twi_tx_data = package->buffer;

  // get a copy of nb bytes to write
  twi_tx_nb_bytes = package->length;

  // put the first byte in the Transmit Holding Register
  twi_inst1->thr = *twi_tx_data++;

  // mask NACK and TXRDY interrupts
  twi_it_mask = AVR32_TWI_IER_NACK_MASK | AVR32_TWI_IER_TXRDY_MASK;

  // update IMR through IER
  twi->ier = twi_it_mask;

  return status;
}


Bool twi_is_busy(void)
{
  if( !twi_nack && ( twi_rx_nb_bytes || twi_tx_nb_bytes ) )
    return TRUE;          // Still receiving/transmitting...

  else
    return FALSE;
}

void twi_read_multiple(int index, const twi_package_t *package){
	// Reads after an SW Reset TWI_READ_MULTIPLE_TIMES to overcome corrupted data
	// Blocking

	int current_twi_read_error = twi_debug_status(twi_master_read_ex(&AVR32_TWI, package), index);

	while 	( current_twi_read_error != 0 || last_twi_read_error > 0){
		if (current_twi_read_error){
			last_twi_read_error = last_twi_read_error + TWI_READ_MULTIPLE_TIMES;
		}
		else {
			if (last_twi_read_error > 0)
				last_twi_read_error--;
		}

		current_twi_read_error = twi_debug_status(twi_master_read_ex(&AVR32_TWI, package), index);
	}
}

// Request two times the same values and compare them
// - they are equal: take them (done)
// - they are unequal: read another time
// -> 3 readings: take allways the middle value
void twi_save_read(int index, twi_package_t *package){

	int i;
	int sum1,sum2;

	// First Read
	package->buffer = (void*) buffer1;
	twi_read(index, package);

	// Second Read
	package->buffer = (void*) buffer2;
	twi_read(index, package);

	for	(i = 0; i < RECEIVE_DATA_LENGTH;i++){
		sum1 = sum1 + (int)buffer1[i];
		sum2 = sum2 + (int)buffer2[i];
	}

	if (sum1 == sum2){		// Values are good
		return;
		equal_counter++;
	}

	equal_counter--;
}

void twi_read_twice(int index, const twi_package_t *package){
	// Read two times after reset to eliminate wrong twi packages
	// Is blocking

	int current_twi_read_error = twi_debug_status(twi_master_read_ex(&AVR32_TWI, package), index);

	// Read until received two correct packages
	while ( current_twi_read_error != 0 || last_twi_read_error != 0){
		last_twi_read_error = current_twi_read_error;
		current_twi_read_error = twi_debug_status(twi_master_read_ex(&AVR32_TWI, package), index);
	}
}

void twi_read(int index, const twi_package_t *package){
	// While and Reset till successfull read
	// Is Blocking

	while(twi_debug_status(twi_master_read_ex(&AVR32_TWI, package), index));
}

int twi_read_ex(int index, const twi_package_t *package){
	// Read only one time, not blocking

	return twi_debug_status(twi_master_read_ex(&AVR32_TWI, package), index);
}


// TWI Debug function
int twi_debug_status(int status, int index){
	// In case of errors, stop & reset TWI and return error value
	// Return 1 for error, 0 for no error

	if(status<0){

		#ifdef DEBUG_MSG_TWI

			sprintf(debug_line,"ID: %d S: %d ", index, status);		// Output ID for debugging
			USART_schreiben(debug_line);

			switch(status){
				case -1:
					USART_schreiben("TWI_INVALID_ARGUMENT:");
				break;

				case -2:
					USART_schreiben("TWI_ARBITRATION_LOST:");
				break;

				case -3:
					USART_schreiben("TWI_NO_CHIP_FOUND:");
				  break;

				case -4:
					USART_schreiben("TWI_RECEIVE_OVERRUN:");
				  break;

				case -5:
					USART_schreiben("TWI_RECEIVE_NACK:");
				  break;

				case -6:
					USART_schreiben("TWI_SEND_OVERRUN:");
				  break;

				case -7:
					USART_schreiben("TWI_SEND_NACK:");
				  break;

				case -8:
					USART_schreiben("TWI_BUSY:");
				  break;

				case TWI_DEACTIVATED:
					USART_schreiben("TWI_DEACTIVATE:");
					steuerWerte.engine_on = 0;
					return 1;		// Proceed WITHOUT TWI
				  break;

				case TWI_RECEIVE_NACK_TIMEOUT:
					USART_schreiben("TWI_NACK_TIMEOUT:");
					break;

				case TWI_BUSY_TIMEOUT:
					USART_schreiben("TWI_BUSY_TIMEOUT:");
					break;

				}
		#endif

		myTWI_Reset();

		return 1;
	}

	if (amount_twi_resets > 0)
		amount_twi_resets--;

	return 0;


}

void myTWI_Reset(void){

	//  Stop TWI, RESET and RE_INIT
	// Stop and Init two Times, cleared more software errors

	amount_twi_resets++;

	if (amount_twi_resets > TWI_MAX_RESETS){

				gpio_clr_gpio_pin(SENSOR_CALIBRATE_LED);	// Clears Sensor LED (LED 2) to show, that there was a fatal TWI Error

				#ifdef DEBUG_MSG_TWI
				USART_schreiben("To many TWI Resets, HW Reset");
				#endif


				if (TWI_Sensor_Hardware_Reset() == 0){				// Hardware Reset Suceed
							gpio_set_gpio_pin(INIT_READY_LED);		// Set Init LED to show, that there was a fatal TWI Error resolved (LED 1 On, LED 2 Off)

							amount_twi_resets = 0;
							return;
				}


			twi_deactivate = 1;

			gpio_clr_gpio_pin(SENSOR_CALIBRATE_LED);	// Clears Init LED to show, that there was a fatal TWI Error

			#ifdef DEBUG_MSG_TWI
			USART_schreiben(" failed, Deactivate TWI");
			#endif

			return;
			}

	myTWI_stop(&AVR32_TWI);
	myTWI_init();

	// Two times was more effective
	myTWI_stop(&AVR32_TWI);
	myTWI_init();

	#ifdef DEBUG_MSG_TWI
	gpio_clr_gpio_pin(INIT_READY_LED);				// Clears Init LED to show, that there was a simple TWI Error resolved by SW Reset

	USART_schreiben(",TWI Reseted\n");
	#endif

}

void myTWI_stop(volatile avr32_twi_t *twi){
	// Stop and Clear TWI

	unsigned int i;

	twi_clear(twi);

	// Stop Signal Hardcoded, best working
	// Stop Signal is rising edge of SDA during SCL high phase
	// Send 8 times (suggested)

	for (i = 0; i < 10; i++){
		gpio_clr_gpio_pin(AVR32_TWI_SDA_0_0_PIN);	// set SDA L
		wait_i(100);
		gpio_set_gpio_pin(AVR32_TWI_SCL_0_0_PIN);	//set SCL H
		wait_i(100);
		gpio_set_gpio_pin(AVR32_TWI_SDA_0_0_PIN);	// set SDA L
		wait_i(100);
		gpio_clr_gpio_pin(AVR32_TWI_SCL_0_0_PIN);	//set SCL L
		wait_i(100);
	}



	twi_clear(twi);

}


void myTWI_init(){
	// Initialize TWI

	// Don't clear GPIO PINS SDA & SCL nor Pull Up


	// Clear Reset Pin
	gpio_clr_gpio_pin(I2C_RESET_CLEAR_PIN);

	// TWI Map
	static const gpio_map_t TWI_GPIO_MAP =
	{
	  {AVR32_TWI_SDA_0_0_PIN, AVR32_TWI_SDA_0_0_FUNCTION},
	  {AVR32_TWI_SCL_0_0_PIN, AVR32_TWI_SCL_0_0_FUNCTION}
	};


	// TWI options settings
	opt.pba_hz = CPU_SPEED;
	opt.speed = TWI_SPEED;
	opt.chip = 0x01;

	// TWI gpio pins configuration
	gpio_enable_module(TWI_GPIO_MAP, sizeof(TWI_GPIO_MAP) / sizeof(TWI_GPIO_MAP[0]));


	// ********************************		TWI INIT		*************************
	// initialize TWI driver with options
	twi_master_init(&AVR32_TWI, &opt);
	// Uses IRQ on level 2:		AVR32_INTC_INT1
	// *******************************************************************************
}




//! This function is not blocking.
// Edited
int twi_master_write_ex_edit(volatile avr32_twi_t *twi, const twi_package_t *package){
	  // New Function, use this to WRITE: Not Blocking

	// Compute Time till timeout
	unsigned int t = tc_ticks + TWI_TIME_OUT*(amount_twi_resets+1);

	  if(twi_deactivate)
	  		return TWI_DEACTIVE;

	  // No data to send
	  if (package->length == 0)
	  {
	    return TWI_INVALID_ARGUMENT;
	  }

	  while( twi_is_busy() ){
		  if (tc_ticks > t)
		  		  return TWI_BUSY_TIMEOUT;
	  }

	  twi_nack = FALSE;

	  // Enable master transfer, disable slave
	  twi->cr =   AVR32_TWI_CR_MSEN_MASK
	#ifndef AVR32_TWI_180_H_INCLUDED
	            | AVR32_TWI_CR_SVDIS_MASK
	#endif
	            ;

	  // set write mode, slave address and 3 internal address byte length
	  twi->mmr = (0 << AVR32_TWI_MMR_MREAD_OFFSET) |
	             (package->chip << AVR32_TWI_MMR_DADR_OFFSET) |
	             ((package->addr_length << AVR32_TWI_MMR_IADRSZ_OFFSET) & AVR32_TWI_MMR_IADRSZ_MASK);

	  // set internal address for remote chip
	  twi->iadr = package->addr;

	  // get a pointer to applicative data
	  twi_tx_data = package->buffer;

	  // get a copy of nb bytes to write
	  twi_tx_nb_bytes = package->length;

	  // put the first byte in the Transmit Holding Register
	  twi_inst1->thr = *twi_tx_data++;

	  // mask NACK and TXRDY interrupts
	  twi_it_mask = AVR32_TWI_IER_NACK_MASK | AVR32_TWI_IER_TXRDY_MASK;

	  // update IMR through IER
	  twi->ier = twi_it_mask;

	  // send data
	  while (!twi_nack && twi_tx_nb_bytes){
		  if (tc_ticks > t)
		  	  	return TWI_RECEIVE_NACK_TIMEOUT;
	  }

	  // Disable master transfer
	  twi->cr =  AVR32_TWI_CR_MSDIS_MASK;

	  if( twi_nack )
	    return TWI_RECEIVE_NACK;

	  return TWI_SUCCESS;
}



void twi_clear(volatile avr32_twi_t *twi){
	// Clears TWI State Values

	Disable_global_interrupt();

	twi_disable_interrupt(twi);

	twi->idr = ~0UL;
	twi->sr;

	// Reset TWI
	twi->cr = AVR32_TWI_CR_SWRST_MASK;

	// set stop bit
	twi_inst1->cr = AVR32_TWI_STOP_MASK;


	twi_tx_nb_bytes = 0;
	twi_rx_nb_bytes = 0;
	twi_nack = 0;
	twi_tx_data = NULL;
	twi_rx_data = NULL;

	Enable_global_interrupt();
}

int twi_master_read_ex(volatile avr32_twi_t *twi, const twi_package_t *package)
// New Function, use this to READ: Not Blocking
{

	if(twi_deactivate)
		return TWI_DEACTIVE;

	// Compute Time till timeout
	unsigned int t = tc_ticks + TWI_TIME_OUT*(amount_twi_resets+1);

  // check argument
  if (package->length == 0)
  {
    return TWI_INVALID_ARGUMENT;
  }

  while( twi_is_busy() ) {
	  if (tc_ticks > t)
		  return TWI_BUSY_TIMEOUT;

  };

  twi_nack = FALSE;

  // set read mode, slave address and 3 internal address byte length
  twi->mmr = (package->chip << AVR32_TWI_MMR_DADR_OFFSET) |
             ((package->addr_length << AVR32_TWI_MMR_IADRSZ_OFFSET) & AVR32_TWI_MMR_IADRSZ_MASK) |
             (1 << AVR32_TWI_MMR_MREAD_OFFSET);

  // set internal address for remote chip
  twi->iadr = package->addr;

  // get a pointer to applicative data
  twi_rx_data = package->buffer;

  // get a copy of nb bytes to read
  twi_rx_nb_bytes = package->length;

  // Enable master transfer
  twi->cr =  AVR32_TWI_CR_MSEN_MASK;

  // Send start condition
  twi->cr = AVR32_TWI_START_MASK;

  // only one byte to receive
  if(twi_rx_nb_bytes == 1)
  {
    // set stop bit
    twi->cr = AVR32_TWI_STOP_MASK;
  }

  // mask NACK and RXRDY interrupts
  twi_it_mask = AVR32_TWI_IER_NACK_MASK | AVR32_TWI_IER_RXRDY_MASK;

  // update IMR through IER
  twi->ier = twi_it_mask;

  // get data
  while (!twi_nack && twi_rx_nb_bytes){
	  if (tc_ticks > t)
	  	return TWI_RECEIVE_NACK_TIMEOUT;
  }

  // Disable master transfer
  twi->cr =  AVR32_TWI_CR_MSDIS_MASK;

  if( twi_nack )
    return TWI_RECEIVE_NACK;

  return TWI_SUCCESS;
}

int twi_master_read_ex_success(const twi_package_t *package){

	if (twi_master_read_ex(&AVR32_TWI, package) == TWI_SUCCESS)
		return 1;

	return 0;
}


int TWI_Sensor_Hardware_Reset(){

	int error;

	myTWI_stop(&AVR32_TWI);
	myTWI_init();							// Init TWI, maybe obsolete

	gpio_set_gpio_pin(I2C_RESET_CLEAR_PIN);		// Set GND Pin to how -> VCC and GND on 3V -> Sensor off

	wait_ms(350);

	gpio_clr_gpio_pin(I2C_RESET_CLEAR_PIN);		// Set GND Pin back to GND to start Sensor

	wait_ms(10);

	myTWI_stop(&AVR32_TWI);
	myTWI_init();								// Init TWI, maybe obsolete

	// Init Sensors again after Reset
	error = Init_IMU_Sensors();

	wait_ms(10);

	// Return Negative Value, if Init Failed (reset failed)
	return error;


}



