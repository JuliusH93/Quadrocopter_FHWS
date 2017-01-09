/*
 * topics.h
 *
 *  Created on: 17.06.2014
 *      Author: Gageik
 */

#ifndef TOPICS_H_
#define TOPICS_H_

typedef struct {
	double roll_angle;
	double pitch_angle;
	double yaw_angle;
	double roll_delta_angle;
	double pitch_delta_angle;
	double yaw_delta_angle;
} SignalProcessed;

typedef struct {
	unsigned char stellwert_blctrl_engine1;
	unsigned char stellwert_blctrl_engine2;
	unsigned char stellwert_blctrl_engine3;
	unsigned char stellwert_blctrl_engine4;
} MotorControl;				// Stellwerte für die Motoren (zwischen 0 und 255)


#endif /* TOPICS_H_ */
