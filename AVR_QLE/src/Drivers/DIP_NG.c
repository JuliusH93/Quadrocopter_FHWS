/*
 * DIP_NG.c
 *
 *  Created on: 04.08.2011
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

#include "..\QUAD_TWI_6DOF.h"

// ******************************************
// *		Display & Buttons driver		*
// ******************************************


// LCD Variables
char line_d[150] = "                    ";		//for LCD only
char line_d0[20] = "                   ";

short dip_index;

extern steuerDaten steuerWerte;
extern controllerDaten controllerWerte;
extern sensorDaten sensorWerte;

extern Quaternion Quat;

extern double voltage;
extern double voltage1;

extern double mean_value;

// ***** Eingabezustände zur DIP-Steuerung *******************
extern short display_modus;

extern unsigned int tc_ticks;
extern unsigned int last_Remote_time;
extern unsigned int last_DP_time;
extern unsigned int last_button_time;


extern unsigned long hoehe;

unsigned int last_time_display_switch;

#ifdef DISPLAY_ON
void setDisplay(){
	// Operate on Display

	if (tc_ticks > last_DP_time + RENEW_DISPLAY_TIME){
	  last_DP_time = tc_ticks;

	  renew_display();
	  }

	return;
}


void renew_display(){
	// Write Display, Display has several pages (DISPLAY_MODI) and there is always only one line set each time
	// display_modus	Current Page
	// dip_index		Current Line, so every sample only one line is written (save maximal delay per sample)

	double acc_betrag;
	acc_betrag = sqrt(sensorWerte.acc_x*sensorWerte.acc_x+sensorWerte.acc_y*sensorWerte.acc_y+sensorWerte.acc_z*sensorWerte.acc_z);

	// Needs 2-3ms processing time (coz of sprintf etc.)
	dip_index = dip_index + 1;

	if (dip_index == 4)		// Maximal 4 lines
		dip_index = 0;

	switch(dip_index){

	case 0:
		dip204_set_cursor_position(1,1);
		dip204_write_string(line_d0);
		break;

	case 1:
		dip204_set_cursor_position(1,2);
		dip204_write_string(line_d0);
		break;

	case 2:
		dip204_set_cursor_position(1,3);
		dip204_write_string(line_d0);
		break;

	case 3:
		dip204_set_cursor_position(1,4);
		dip204_write_string(line_d0);
		break;
	}


	switch (display_modus){

	// 0
	case 0:

		switch(dip_index){

		case 0:

			break;

		case 1:
			snprintf(line_d,20,"Ax: %2.2f,%2.2f", sensorWerte.acc_x, sensorWerte.acc_x_dpf);
			dip204_set_cursor_position(1,2);
			dip204_write_string(line_d);
			break;

		case 2:
			snprintf(line_d,20,"Ay: %2.2f,%2.2f", sensorWerte.acc_y, sensorWerte.acc_y_dpf);
			dip204_set_cursor_position(1,3);
			dip204_write_string(line_d);
			break;

		case 3:
			snprintf(line_d,20,"Betrag: %2.2f", acc_betrag);
			dip204_set_cursor_position(1,4);
			dip204_write_string(line_d);
			break;
		}


		dip204_hide_cursor();
		break;


	case 1:

		switch(dip_index){

		case 0:

			break;

		case 1:
			snprintf(line_d,20,"Gx: %2.2f", sensorWerte.gyro_x);
			dip204_set_cursor_position(1,2);
			dip204_write_string(line_d);
			break;

		case 2:
			snprintf(line_d,20,"Gy: %2.2f", sensorWerte.gyro_y);
			dip204_set_cursor_position(1,3);
			dip204_write_string(line_d);
			break;

		case 3:
			snprintf(line_d,20,"Gz: %2.2f", sensorWerte.gyro_z);
			dip204_set_cursor_position(1,4);
			dip204_write_string(line_d);
			break;
		}

		dip204_hide_cursor();
		break;


	case 2:

		switch(dip_index){

		case 0:
			snprintf(line_d,20,"Wxyz G,A,K");
			dip204_set_cursor_position(1,1);
			dip204_write_string(line_d);
			break;

		case 1:
			snprintf(line_d,20,"%2.1f,%2.1f,%2.1f", sensorWerte.winkel_x_gyro, sensorWerte.roll_acc, sensorWerte.winkel_x_kalman);
			dip204_set_cursor_position(1,2);
			dip204_write_string(line_d);
			break;

		case 2:
			snprintf(line_d,20,"%2.1f,%2.1f,%2.1f", sensorWerte.winkel_y_gyro, sensorWerte.pitch_acc, sensorWerte.winkel_y_kalman);
			dip204_set_cursor_position(1,3);
			dip204_write_string(line_d);
			break;

		case 3:
			snprintf(line_d,20,"%2.1f", sensorWerte.winkel_z_gyro);
			dip204_set_cursor_position(1,4);
			dip204_write_string(line_d);
			break;
		}

		dip204_hide_cursor();
		break;

	case 3:

		switch(dip_index){

		case 0:
			snprintf(line_d,20,"W:%2.2f,S:%d", sensorWerte.winkel_y_gyro, controllerWerte.pid_roll.state);
			dip204_set_cursor_position(1,1);
			dip204_write_string(line_d);
			break;

		case 1:
			snprintf(line_d,20,"P:%2.2f", controllerWerte.pid_roll.p);
			dip204_set_cursor_position(1,2);
			dip204_write_string(line_d);
			break;

		case 2:
			snprintf(line_d,20,"I:%2.2f", controllerWerte.pid_roll.i);
			dip204_set_cursor_position(1,3);
			dip204_write_string(line_d);
			break;

		case 3:
			snprintf(line_d,20,"D:%2.2f", controllerWerte.pid_roll.d);
			dip204_set_cursor_position(1,4);
			dip204_write_string(line_d);
			break;
		}

		dip204_hide_cursor();
		break;

	case 4:

		switch(dip_index){

		case 0:
			snprintf(line_d,20,"W:%2.2f", sensorWerte.winkel_z_gyro);
			dip204_set_cursor_position(1,1);
			dip204_write_string(line_d);
			break;

		case 1:
			snprintf(line_d,20,"P:%3.3f", controllerWerte.pid_yaw.p);
			dip204_set_cursor_position(1,2);
			dip204_write_string(line_d);
			break;

		case 2:
			snprintf(line_d,20,"I:%2.2f", controllerWerte.pid_yaw.i);
			dip204_set_cursor_position(1,3);
			dip204_write_string(line_d);
			break;

		case 3:
			snprintf(line_d,20,"D:%2.2f", controllerWerte.pid_yaw.d);
			dip204_set_cursor_position(1,4);
			dip204_write_string(line_d);
			break;
		}

		dip204_hide_cursor();
		break;

	case 5:

		switch(dip_index){

		case 0:
			snprintf(line_d,20,"Wy:%2.2f", sensorWerte.winkel_y_gyro);
			dip204_set_cursor_position(1,1);
			dip204_write_string(line_d);
			break;

		case 1:
			snprintf(line_d,20,"S:%2.2f,P:%2.2f", controllerWerte.soll_angle_roll, controllerWerte.roll_p_anteil);
			dip204_set_cursor_position(1,2);
			dip204_write_string(line_d);
			break;

		case 2:
			snprintf(line_d,20,"F:%2.2f", controllerWerte.fehlerintegral_roll);
			dip204_set_cursor_position(1,3);
			dip204_write_string(line_d);
			break;

		case 3:
			snprintf(line_d,20,"I:%2.2f,D:%2.2f", controllerWerte.roll_i_anteil, controllerWerte.roll_d_anteil);
			dip204_set_cursor_position(1,4);
			dip204_write_string(line_d);
			break;
		}

		dip204_hide_cursor();
		break;

	case 6:

			switch(dip_index){

			case 0:
				snprintf(line_d,20,"Q01:%1.3f,%1.3f", Quat.q0, Quat.q1);
				dip204_set_cursor_position(1,1);
				dip204_write_string(line_d);
				break;

			case 1:
				snprintf(line_d,20,"Q23:%1.3f,%1.3f", Quat.q2, Quat.q3);
				dip204_set_cursor_position(1,2);
				dip204_write_string(line_d);
				break;

			case 2:
				snprintf(line_d,20,"RP:%2.1f,%2.1f", sensorWerte.roll_angle, sensorWerte.pitch_angle);
				dip204_set_cursor_position(1,3);
				dip204_write_string(line_d);
				break;

			case 3:
				snprintf(line_d,20,"Y:%2.1f", sensorWerte.yaw_angle);
				dip204_set_cursor_position(1,4);
				dip204_write_string(line_d);

				break;
			}

		dip204_hide_cursor();
		break;


		case 7:

			switch(dip_index){

			case 0:
				snprintf(line_d,20,"Wqxyz G,A,K");
				dip204_set_cursor_position(1,1);
				dip204_write_string(line_d);
				break;

			case 1:
				snprintf(line_d,20,"%2.1f,%2.1f,%2.1f", sensorWerte.winkel_x_gyro, sensorWerte.roll_acc, sensorWerte.winkel_x_kalman);
				dip204_set_cursor_position(1,2);
				dip204_write_string(line_d);
				break;

			case 2:
				snprintf(line_d,20,"%2.1f,%2.1f,%2.1f", sensorWerte.winkel_y_gyro, sensorWerte.pitch_acc, sensorWerte.winkel_y_kalman);
				dip204_set_cursor_position(1,3);
				dip204_write_string(line_d);
				break;

			case 3:
				snprintf(line_d,20,"%2.1f", sensorWerte.yaw_angle);
				dip204_set_cursor_position(1,4);
				dip204_write_string(line_d);
				break;
			}


			dip204_hide_cursor();
			break;
	}

	//USART_schreiben(line_d);

	return;
}


// ************************************ IRQS DECLARATION *************************************************

/*!
 * \brief ////////////						PUSH BUTTONS				//////////////////////
 */
#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif
static void dip204_example_PB_int_handler(void)				// For Debug, Obsolete to Fly
{

  /* display all available chars */
  if (gpio_get_pin_interrupt_flag(GPIO_PUSH_BUTTON_0))
  {
	#ifdef ENGINE_WITH_SWITCH_DEBUG

		  if (steuerWerte.engine_on){
			  steuerWerte.engine_on = 0;
		  }
		  else{
			  steuerWerte.engine_on = 1;
		  }
	#endif


	  gpio_clear_pin_interrupt_flag(GPIO_PUSH_BUTTON_0);

  }

  if (gpio_get_pin_interrupt_flag(GPIO_PUSH_BUTTON_1))
  {

	  setRPY(10, 0, 0);

	  gpio_clear_pin_interrupt_flag(GPIO_PUSH_BUTTON_1);
  }

  if (gpio_get_pin_interrupt_flag(GPIO_PUSH_BUTTON_2))
  {

	setRPY(-10, 0, 0);
	gpio_clear_pin_interrupt_flag(GPIO_PUSH_BUTTON_2);
  }
}


/*!
* \brief ////////////						JOYSTICK				////////////////////// */
__attribute__((__interrupt__))
static void dip204_example_Joy_int_handler(void)
{

	  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_UP))
	  {
		  if ((tc_ticks - last_time_display_switch) > 300){
		  		last_time_display_switch = tc_ticks;
		  		display_modus++;
		  		display_modus = display_modus % DISPLAY_MODI;
		  }

		  gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_UP);
	  }
	  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_DOWN))
	  {
		  if ((tc_ticks - last_time_display_switch) > 300){
		  	  last_time_display_switch = tc_ticks;
			  display_modus = DISPLAY_MODI - 1 + display_modus;
			  display_modus = display_modus % DISPLAY_MODI;
		  }

		  gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_DOWN);
	  }
	  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_LEFT))
	  {




		  gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_LEFT);
	  }
	  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_RIGHT))
	  {




		  gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_RIGHT);
	  }
}
#endif


// Button Interrupt
__attribute__((__interrupt__))
static void Joy_int_handler(){

			if (tc_ticks > last_button_time + 500){
			  			  steuerWerte.button_clicked = steuerWerte.button_clicked + 1;
			  			  last_button_time = tc_ticks;
			  		  }

			gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_PUSH);
}

#ifdef DISPLAY_ON

/*!
 * \brief function to configure push button to generate IT upon rising edge
 */
void dip204_example_configure_push_buttons_IT(void)
{
  gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_0 , GPIO_RISING_EDGE);

  gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_1 , GPIO_RISING_EDGE);

  gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_2 , GPIO_RISING_EDGE);

  /* Disable all interrupts */
  Disable_global_interrupt();
  /* register PB0 handler on level 1 */
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_PUSH_BUTTON_1/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_PUSH_BUTTON_2/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_PB_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_PUSH_BUTTON_0/8), AVR32_INTC_INT1);
  /* Enable all interrupts */
  Enable_global_interrupt();
}


/*!
 * \brief function to configure joystick to generate IT upon falling edge
 */
void dip204_example_configure_joystick_IT(void)
{
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_UP , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_DOWN , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_RIGHT , GPIO_FALLING_EDGE);
  gpio_enable_pin_interrupt(GPIO_JOYSTICK_LEFT , GPIO_FALLING_EDGE);

  /* Disable all interrupts */
  Disable_global_interrupt();
  /* register PB0 handler on level 1 */
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_UP/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_DOWN/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_RIGHT/8), AVR32_INTC_INT1);
  INTC_register_interrupt( &dip204_example_Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_LEFT/8), AVR32_INTC_INT1);
  /* Enable all interrupts */
  Enable_global_interrupt();
}


// ************************************ END IRQS ******************************************************


void myDIP_init(){
	// Initialize Display

	// LCD Map
	static const gpio_map_t DIP204_SPI_GPIO_MAP =
		{
		  {DIP204_SPI_SCK_PIN,  DIP204_SPI_SCK_FUNCTION },  // SPI Clock.
		  {DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION},  // MISO.
		  {DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION},  // MOSI.
		  {DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
	};


	// --- SPI for DIP (LCD) -------------------
	  // add the spi options driver structure for the LCD DIP204
	  spi_options_t spiOptions =
	  {
	    .reg          = DIP204_SPI_NPCS,
	    .baudrate     = 1000000,
	    .bits         = 8,
	    .spck_delay   = 0,
	    .trans_delay  = 0,
	    .stay_act     = 1,
	    .spi_mode     = 0,
	    .modfdis      = 1
	  };


	// -------- ASSIGN GPIO MAP   ---------------------------------------------------------------------------

	// Assign I/Os to SPI
	gpio_enable_module(DIP204_SPI_GPIO_MAP, sizeof(DIP204_SPI_GPIO_MAP) / sizeof(DIP204_SPI_GPIO_MAP[0]));


	// ---- INIT DIP  ---------------------------------------------------------------
	// SPI Setup for DIP
	// Initialize as master
	spi_initMaster(DIP204_SPI, &spiOptions);

	// Set selection mode: variable_ps, pcs_decode, delay
	spi_selectionMode(DIP204_SPI, 0, 0, 0);

	// Enable SPI
	spi_enable(DIP204_SPI);

	// setup chip registers
	spi_setupChipReg(DIP204_SPI, &spiOptions, CPU_SPEED);
	// -------------------------------------------------------------------------------


	// configure local push buttons
	dip204_example_configure_push_buttons_IT();

	// configure local joystick
	dip204_example_configure_joystick_IT();

	// initialize delay driver
	delay_init( CPU_SPEED );

	// initialize LCD
	dip204_init(backlight_PWM, TRUE);
}
#endif

void init_button_irq(){

	  gpio_enable_pin_interrupt(GPIO_JOYSTICK_PUSH , GPIO_FALLING_EDGE);
	  INTC_register_interrupt( &Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_PUSH/8), AVR32_INTC_INT1);

}
