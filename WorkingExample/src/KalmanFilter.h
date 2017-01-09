/*
 * Kalman.h
 *
 *  Created on: 02.09.2011
 *      Author: gageik
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "basics.h"
#include "topics.h"

#ifdef	COMP_MODE_VERY_FAST
#define QUAT_CORRECT_INTERVALL		100		// In Factors of SAMPLE_TIME: 200 was good
#define MEASUREMENT_NOISE_ACC		1000
#else
#ifdef		COMP_MODE_FAST
#define QUAT_CORRECT_INTERVALL		200		// In Factors of SAMPLE_TIME: 200 was good
#define MEASUREMENT_NOISE_ACC		2000

#else
#define QUAT_CORRECT_INTERVALL		500		// In Factors of SAMPLE_TIME: 200 was good
#define MEASUREMENT_NOISE_ACC		40000	// 40.000 GUT GEFLOGEN
#endif
#endif

//#define PROCESS_NOISE				1
//#define MEASUREMENT_NOISE_BIAS		5

#define SAMPLE_TIME_k	SAMPLE_TIME
#define ALPHA_k			0.9f
//#define ALPHA_k			0

#define MEASUREMENT_NOISE_GYRO		10.0f
//#define MEASUREMENT_NOISE_ACC		30000
//#define MEASUREMENT_NOISE_ACC		40000	// 40.000 GUT GEFLOGEN
//#define MEASUREMENT_NOISE_ACC		2000
//#define MEASUREMENT_NOISE_ACC		400000
#define PROCESS_NOISE_ANGLE			0.01f
#define PROCESS_NOISE_RATE			1000.0f
#define P_INIT_UNCERTAINTY			10.0f

#define	KALMAN_MEAS_ACC_ALPHA			0.99
#define	KALMAN_MEAS_ACC_ALPHA_LOW		0.93
#define KALMAN_MEAS_ACC_ALPHA_HIGH		0.996f

#define	KALMAN_FILTER_ANGLE_CUT_OFF		6.0
#define	MAX_KALMAN_WEIGHT				1000000.0

SignalProcessed KalmanFiltering(void);
void Kalman_Init(void);
void KalmanFilter(void);
void Kalman_Measure_Roll(double acc, double gyro);
void Kalman_Measure_Pitch(double acc, double gyro);
void Kalman_Estimate_Roll(void);
void Kalman_Estimate_Pitch(void);
double Kalman_Correct_Roll(void);
double Kalman_Correct_Pitch(void);
void Kalman_Set_R_old(sensorDaten* Sensoren, controllerDaten* Controller );
void Kalman_Set_RP(double roll, double pitch);


typedef struct {

	double x_k[2];			// Angle, Angular Rate
	double xe_k[2];
	double y_k[2];

	double P_k[2][2];
	double Pe_k[2][2];
	double K_k[2][2];

	//debug
	double a;
	double b;
	double c;
	double g;
	double s;
	double acc_betrag;
	double noise;

} Kalman;

typedef struct{

	double acc_measure_noise;

} Parameter;

typedef struct {
	double P_roll_k;
	double P_pitch_k;

	double K_roll_k;
	double K_pitch_k;

	double x_roll_k;
	double x_pitch_k;
} Kalman1D;

#endif /* KALMAN_H_ */
