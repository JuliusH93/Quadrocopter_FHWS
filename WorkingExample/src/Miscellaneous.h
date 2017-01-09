/*
 * Miscellaneous.h
 *
 *  Created on: 16.03.2012
 *      Author: gageik
 */

#ifndef MISCELLANEOUS_H_
#define MISCELLANEOUS_H_

void SetProcessorFrequency(volatile avr32_pm_t* pm);
void jitter_debug(void);

#define LAUFZEIT_TEST_RUNS		100000

#endif /* MISCELLANEOUS_H_ */
