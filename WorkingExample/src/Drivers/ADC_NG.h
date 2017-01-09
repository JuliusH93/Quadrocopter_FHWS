/*
 * ADC_NG.h
 *
 *  Created on: 31.08.2011
 *      Author: gageik
 */

#ifndef ADC_NG_H_
#define ADC_NG_H_


// ADC Channels
#define ADC_0_channel	0
#define ADC_1_channel	1
#define ADC_2_channel	2

#define ADC_0_PIN		AVR32_PIN_PA21
#define ADC_1_PIN		AVR32_PIN_PA22
#define ADC_2_PIN		AVR32_PIN_PA23

void myADC_init(void);
void adc_set_all(void);			// Open ADCs for all channels
short adc_get_0(void);			// Receive value
short adc_get_1(void);			// Receive value
short adc_get_2(void);			// Receive value

#endif /* ADC_NG_H_ */
