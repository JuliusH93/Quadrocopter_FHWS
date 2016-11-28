/********************************************************
 Name          : main.c
 Author        : Nils Gageik
 Copyright     : Not really
 Description   : LR Labor Template Version
 **********************************************************/

/*! \page License
 * Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 */

// Include Files
#include "AVR_Framework.h"
#include "basics.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "TWI_NG.h"
#include "TC_NG.h"

sensorDaten_raw sensorWerte;

int counter = 0;
char debug_line[DEBUG_BUFFER_SIZE];

__attribute__((__interrupt__))
static void Joy_int_handler(void)
{
  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_UP))
  {
	  counter++;
    /* allow new interrupt : clear the IFR flag */
    gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_UP);
  }
  if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_DOWN))
  {
	  counter--;
    /* allow new interrupt : clear the IFR flag */
    gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_DOWN);
  }
if (gpio_get_pin_interrupt_flag(GPIO_JOYSTICK_PUSH))
  {
	counter = 0;
    /* allow new interrupt : clear the IFR flag */
    gpio_clear_pin_interrupt_flag(GPIO_JOYSTICK_PUSH);
  }
}


int main(void) {

	// Map SPI Pins
	static const gpio_map_t DIP204_SPI_GPIO_MAP =
	  {
	    {DIP204_SPI_SCK_PIN,  DIP204_SPI_SCK_FUNCTION },  // SPI Clock.
	    {DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION},  // MISO.
	    {DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION},  // MOSI.
	    {DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION}   // Chip Select NPCS.
	  };

	  // Switch the CPU main clock to oscillator 0
	  pcl_switch_to_osc(PCL_OSC0, CPU_SPEED, OSC0_STARTUP); // Switch main clock to external oscillator 0 (crystal), 12 MHz


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



	  // SPI Inits: Assign I/Os to SPI
	  gpio_enable_module(DIP204_SPI_GPIO_MAP,
	                       sizeof(DIP204_SPI_GPIO_MAP) / sizeof(DIP204_SPI_GPIO_MAP[0]));

	  // Initialize as master
	  spi_initMaster(DIP204_SPI, &spiOptions);

	  // Set selection mode: variable_ps, pcs_decode, delay
	  spi_selectionMode(DIP204_SPI, 0, 0, 0);

	  // Enable SPI
	  spi_enable(DIP204_SPI);

	  // setup chip registers
	  spi_setupChipReg(DIP204_SPI, &spiOptions, FOSC0);

	  // initialize delay driver
	  delay_init( FOSC0 );

	  // initialize LCD
	  dip204_init(backlight_PWM, TRUE);

	  // Initialisiere die Interrupt Vector Tabelle:
	  // Nur einmal ausf�hren vor dem Registrieren der Interrupts!

	  // Disable all interrupts.
	  Disable_global_interrupt();

	  // init the interrupts
	  INTC_init_interrupts();

	  // Enable all interrupts.
	  Enable_global_interrupt();

	  // Init Button Interrupts
	  gpio_enable_pin_interrupt(GPIO_JOYSTICK_UP , GPIO_FALLING_EDGE);
	  gpio_enable_pin_interrupt(GPIO_JOYSTICK_DOWN , GPIO_FALLING_EDGE);
	  gpio_enable_pin_interrupt(GPIO_JOYSTICK_PUSH , GPIO_FALLING_EDGE);

	  Disable_global_interrupt();
	    /* register PB0 handler on level 1 */
	  INTC_register_interrupt
	  (&Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_UP/8) ,  AVR32_INTC_INT1);

	  INTC_register_interrupt
	  (&Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_DOWN/8) ,  AVR32_INTC_INT1);

	  INTC_register_interrupt
	  (&Joy_int_handler, AVR32_GPIO_IRQ_0 + (GPIO_JOYSTICK_PUSH/8), AVR32_INTC_INT1);

	  /* Enable all interrupts */
	  Enable_global_interrupt();


		 // Display default message.
		dip204_set_cursor_position(1,1);
		dip204_write_string("Quadrocopter");
		dip204_hide_cursor();


	    unsigned int counter = 0;

	    while (1)
	    {
				counter++;
				//sensorWerte_raw = TODO;
				//handleNewSensorData(sensorWerte);
				sprintf(debug_line,"%d", counter);
				dip_write_sensor_data("test1","test2",2);
				wait_ms(100);
	    }

	    return 0;
}

void dip_write_sensor_data(char* input1, char* input2, int zeile)
{
	dip204_set_cursor_position(1,zeile);
	dip204_write_string(input1);
	dip204_set_cursor_position(1,zeile + 1);
	dip204_write_string(input2);
}

void handleNewSensorData(sensorWerte_raw sensorWerte)
{
	//TODO
}

void USART_schreiben(char* string){
}

void dip_write_string(char* string, int zeile){
    dip204_set_cursor_position(1,zeile);
    dip204_write_string(string);
}
