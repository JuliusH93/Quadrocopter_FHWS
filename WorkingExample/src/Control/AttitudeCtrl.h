/*
 * AttitudeCtrl.h
 *
 *  Created on: 05.01.2012
 *      Author: gageik
 */

#ifndef ATTITUDECTRL_H_
#define ATTITUDECTRL_H_

#include "../QUAD_TWI_6DOF.h"			// All Header Files

#define INT_CUTOFF					0.999f
#define	ATTITUDE_PART				0.9f

#define INTEGRAL_SCHRANKE			500.0f


#define MAX_HEIGHT					250		// Maximale einstellbare Höhe des Höhenreglers

#define P_YAW						5
#define I_YAW						0.005
#define D_YAW						2

#ifndef FLUG
#define YAW_K					  	0.5f
#else
#define YAW_K					  	1.0f
#endif

// Zustandseinteilung (Grad degree)
#define	PHI_CRITICAL					4.0f
#define PHI_SAVE						1.0f

#define ATTITUDE_K					  0.5f

// PID Values for Roll & Pitch
#define	P_AUFBAU 					12
#define I_AUFBAU			   		0.02
#define D_AUFBAU					2

// Gut Geflogen
#ifndef BIG_ENGINE
#define P_FLUG	 					4.0f
#define I_FLUG				   		0.02f
#define D_FLUG						0.8f
#endif


#ifdef BIG_ENGINE
#define P_FLUG	 					2.0f
#define I_FLUG				   		0.01f
#define D_FLUG						0.4f
#endif

#define P_Thilo						3.4f
#define I_Thilo						0.02f
#define D_Thilo						0.5f

//#define P_ALEX						4.47
//#define D_ALEX						0.67

#define P_ALEX						1.5
#define D_ALEX						0.26

// Further PID Parameters
#define HIGH_INTEGRAL_ERROR_AREA		 5.0f
#define LOW_INTEGRAL_ERROR_AREA			 1.0f

MotorControl Attitude_Controlling(SignalProcessed* SP_Data);
void Attitude_Control_Init(void);
void Attitude_Control(double winkel_x, double winkel_y, double winkel_z, double angular_rate_x, double angular_rate_y, double angular_rate_z);

#endif /* ATTITUDECTRL_H_ */
