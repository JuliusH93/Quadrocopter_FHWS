/*
 * ADC_NG.c
 *
 *  Created on: 31.08.2011
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

volatile avr32_adc_t *adc = &AVR32_ADC; // ADC IP registers address

void myADC_init(){

	//gpio_clr_gpio_pin(ADC_0_PIN);
	//gpio_clr_gpio_pin(ADC_1_PIN);


	static const gpio_map_t ADC_GPIO_MAP =
	{
			{ADC_0_PIN, 0},
			{ADC_1_PIN, 0},
			{ADC_2_PIN, 0}
	};

	// Assign and enable GPIO pins to the ADC function.
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) / sizeof(ADC_GPIO_MAP[0]));

	// configure ADC
	// Lower the ADC clock to match the ADC characteristics (because we configured
	// the CPU clock to 12MHz, and the ADC clock characteristics are usually lower;
	// cf. the ADC Characteristic section in the datasheet).
	AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;

	adc_configure(adc);

	adc_enable(adc, ADC_0_channel);
	adc_enable(adc, ADC_1_channel);
	adc_enable(adc, ADC_2_channel);



	//gpio_enable_gpio_pin(ADC_0_PIN);
	//gpio_enable_gpio_pin(ADC_1_PIN);
}

// launch conversion on all enabled channels
void adc_set_all(){
	adc_start(adc);
}



short adc_get_0(){
	// Finish ADC measurement, close and return value

	short adc_value;
	adc_value = (short) adc_get_value(adc, ADC_0_channel);

	return adc_value;
}



short adc_get_1(){

	short adc_value;
	adc_value = (short) adc_get_value(adc, ADC_1_channel);

	return adc_value;
}

short adc_get_2(){

	short adc_value;
	adc_value = (short) adc_get_value(adc, ADC_2_channel);

	return adc_value;
}
