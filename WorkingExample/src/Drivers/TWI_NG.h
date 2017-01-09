/*
 * TWI_NG.h
 *
 *  Modified on: 05.08.2011
 *      Author: gageik
 *      Based on AVR Framework
 */

#ifndef TWI_NG_H_
#define TWI_NG_H_


#include "../QUAD_TWI_6DOF.h"			// All Header Files

#define TWI_TIME_OUT			7


/*! \name Error Codes for the Module
 */
//! @{
#define TWI_DEACTIVE				2		// When Deactive do not stay in the while
#define TWI_SUCCESS              	0
#define TWI_INVALID_ARGUMENT    	-1
#define TWI_ARBITRATION_LOST    	-2
#define TWI_NO_CHIP_FOUND       	-3
#define TWI_RECEIVE_OVERRUN     	-4
#define TWI_RECEIVE_NACK        	-5
#define TWI_SEND_OVERRUN        	-6
#define TWI_SEND_NACK           	-7
#define TWI_BUSY                	-8
#define TWI_DEACTIVATED				-9
#define TWI_RECEIVE_NACK_TIMEOUT	-10
#define TWI_BUSY_TIMEOUT			-11
//! @}

//------------------  TWI/I2C Configurations  ------------------------------------------------------------------------
#define EEPROM_ADDR_LGT       		1       // Address length of the EEPROM memory
#define TWI_SPEED             		400000  // 400000 war sonst immer (bis 9.2013)
//#define TWI_SPEED             		150000  // Verbesserung? Nein
//#define TWI_SPEED             		200000  // Speed of TWI: 200000 = 400kHz | 50.000 = 100kHz	Gyro-Ausreiﬂer bei 200000
#define RECEIVE_DATA_LENGTH			6		// Bytes to receive from ADXL
//----------------------------------------------------------------------------------------------------------------------

#define TWI_MAX_RESETS				20		// In Error Case how many TWI SW Resets are performed until a HW Reset is executed
#define TWI_READ_MULTIPLE_TIMES		5		// In Error Case how many packages to discard
#define HW_RESET_TIME_DELAY			300


/*!
 * \brief Input parameters when initializing the twi module mode
 */
typedef struct
{
  //! The PBA clock frequency.
  unsigned long pba_hz;
  //! The baudrate of the TWI bus.
  unsigned long speed;
  //! The desired address.
  char chip;
} twi_options_t;

/*!
 * \brief Information concerning the data transmission
 */
typedef struct
{
  //! TWI chip address to communicate with.
  char chip;
  //! TWI address/commands to issue to the other chip (node).
  unsigned int addr;
  //! Length of the TWI data address segment (1-3 bytes).
  int addr_length;
  //! Where to find the data to be written.
  void *buffer;
  //! How many bytes do we want to write.
  unsigned int length;
} twi_package_t;


/*!
 * \brief Pointer on TWI slave application routines
 */
typedef struct
{
  //! Routine to receiv data from TWI master
  void (*rx)(U8);
  //! Routine to transmit data to TWI master
  U8 (*tx)(void);
  //! Routine to signal a TWI STOP
  void (*stop)(void);
} twi_slave_fct_t;


void myTWI_init(void);
void twi_clear(volatile avr32_twi_t *twi);
void myTWI_stop(volatile avr32_twi_t *twi);
int twi_debug_status(int status, int index);
void twi_read(int index, const twi_package_t *package);
int twi_read_ex (int index, const twi_package_t *package);
void twi_read_twice(int index, const twi_package_t *package);
int twi_master_write_ex_edit(volatile avr32_twi_t *twi, const twi_package_t *package);
int twi_master_read_ex(volatile avr32_twi_t *twi, const twi_package_t *package);
int twi_master_read_ex_success(const twi_package_t *package);
void twi_read_multiple(int index, const twi_package_t *package);
void myTWI_Reset(void);
int TWI_Sensor_Hardware_Reset(void);
void twi_save_read(int index, twi_package_t *package);


// TWI Functions from Framework
/*!
 * \brief Initialize the twi master module
 *
 * \param twi   Base address of the TWI (i.e. &AVR32_TWI).
 * \param *opt  Options for initializing the twi module
 *              (see \ref twi_options_t)
 */
extern int twi_master_init(volatile avr32_twi_t *twi, const twi_options_t *opt);


/*!
 * \brief Initialize the twi slave module
 *
 * \param twi   Base address of the TWI (i.e. &AVR32_TWI).
 * \param *opt  Options for initializing the twi module
 *              (see \ref twi_options_t)
 * \param *slave_fct  Pointer on application fonctions
 */
extern int twi_slave_init(volatile avr32_twi_t *twi, const twi_options_t *opt, const twi_slave_fct_t *slave_fct);


/*!
 * \brief Test if a chip answers for a given twi address
 *
 * \param twi        Base address of the TWI (i.e. &AVR32_TWI).
 * \param chip_addr  Address of the chip which is searched for
 * \return TWI_SUCCESS if a chip was found, error code otherwhise
 */
extern int twi_probe(volatile avr32_twi_t *twi, char chip_addr);

/*!
 * \brief Disable all TWI interrupts
 *
 * \param twi        Base address of the TWI (i.e. &AVR32_TWI).
 *
 */
extern void twi_disable_interrupt(volatile avr32_twi_t *twi);

/*!
 * \brief Read multiple bytes from a TWI compatible slave device
 *
 * \param twi     Base address of the TWI (i.e. &AVR32_TWI).
 * \param package Package information and data
 *                (see \ref twi_package_t)
 * \return TWI_SUCCESS if all bytes were read, error code otherwhise
 */
extern int twi_master_read(volatile avr32_twi_t *twi, const twi_package_t *package);

/*!
 * \brief Write multiple bytes to a TWI compatible slave device
 *
 * \param twi       Base address of the TWI (i.e. &AVR32_TWI).
 * \param *package  Package information and data
 *                  (see \ref twi_package_t)
 * \return TWI_SUCCESS if all bytes were written, error code otherwhise
 */
extern int twi_master_write(volatile avr32_twi_t *twi, const twi_package_t *package);

/*!
 * \brief Write multiple bytes to a TWI compatible slave device. This function is not blocking.
 *
 * The function does not wait that all the bytes are written.
 *
 * \param twi       Base address of the TWI (i.e. &AVR32_TWI).
 * \param *package  Package information and data
 *                  (see \ref twi_package_t)
 * \return TWI_SUCCESS if all bytes were written, error code otherwhise
 */
extern int twi_master_write_ex(volatile avr32_twi_t *twi, const twi_package_t *package);

/*!
 * \brief Test if a TWI read/write is pending.
 *
 * \return TRUE if a write/read access is pending, FALSE otherwhise
 */
extern Bool twi_is_busy(void);



#endif /* TWI_NG_H_ */
