// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011-2014 Nils Gageik

/*
 * KalmanFilter.c
 *
 *  Created on: 02.09.2011
 *      Author: gageik
 */

#include "QUAD_TWI_6DOF.h"


// KalmanFilter 2D, 2x2 Matrix Multiplication
// Used for Attitude Control

// Zustandsraummodell
double A_k[2][2] = { { 1.0f, (double)SAMPLE_TIME_k / 1000.0f }, { 0.0f, 1.0f } };			// System
double C_k[2][2] = { { 1.0f, 0.0f}, { 0.0f, 1.0f } };								// Measurement
double At_k[2][2];		// transponierte, to safe processing time
double Ct_k[2][2];

// Process Noise and Measurement Noise
double Q_k[2][2] = { { PROCESS_NOISE_ANGLE, 0.0f}, { 0.0f, PROCESS_NOISE_RATE } };
double R_k_Pitch[2][2] = { { MEASUREMENT_NOISE_ACC, 0.0f}, { 0.0f, MEASUREMENT_NOISE_GYRO } };
double R_k_Roll[2][2] = { { MEASUREMENT_NOISE_ACC, 0.0f}, { 0.0f, MEASUREMENT_NOISE_GYRO } };

double Einheitsmatrix[2][2] = { { 1.0f, 0.0f}, { 0.0f, 1.0f } };

// Kalman Filter Valies, exclusive Parameters for every Axis
Kalman roll_k;
Kalman pitch_k;
Parameter KF_Parameter;

extern MeanFilter MF_Acc_X_Mean;
extern MeanFilter MF_Acc_X_Mean_Delayed;
extern MeanFilter MF_Acc_Y_Mean;
extern MeanFilter MF_Acc_Y_Mean_Delayed;
extern MeanFilter MF_Acc_Z_Mean;
extern MeanFilter MF_Acc_Z_Mean_Delayed;

extern sensorDaten sensorWerte;
extern controllerDaten controllerWerte;

// Variable for Fly Observer (Fly Noise 2 Axis)
double	fly_noise[2];	// Roll, Pitch

// Temporäre Variablen (für Berechnung)		(Temporary Calculating Variables, Matrixes)
double Temp1_k[2][2];
double Temp2_k[2][2];
double Temp3_k[2][2];

double Temp_v1_k[2];							// (Vectors)
double Temp_v2_k[2];

// Last Remote Steering Commands
double last_roll_remote;
double last_pitch_remote;

// For debugging Messages
extern char debug_line[DEBUG_BUFFER_SIZE];

SignalProcessed KalmanFiltering(){

	// Calculate RPY Angles from Current Orientation Quaternion for Control
	calcRPY();

	SignalProcessed Out;

	KalmanFilter();

	Debug_Msg('K');

	// Correct Current Quaternion Orientation using KalmanFilter values
	correctQuat(sensorWerte.winkel_x_kalman, sensorWerte.winkel_y_kalman, sensorWerte.yaw_angle);

	Out.roll_angle = sensorWerte.roll_angle;
	Out.pitch_angle = sensorWerte.pitch_angle;
	Out.yaw_angle = sensorWerte.yaw_angle;

	Out.pitch_delta_angle = sensorWerte.pitch_delta_angle;
	Out.roll_delta_angle = sensorWerte.roll_delta_angle;
	Out.yaw_delta_angle = sensorWerte.yaw_delta_angle;

	return Out;
}


void Kalman_Init(){
	// Setup all Variables, Vectors and Matrices
	Transponiert2D(A_k, At_k);
	Transponiert2D(C_k, Ct_k);


	KF_Parameter.acc_measure_noise = (double) MEASUREMENT_NOISE_ACC;

	roll_k.x_k[0] = 0.0f;
	roll_k.x_k[1] = 0.0f;

	pitch_k.x_k[0] = 0.0f;
	pitch_k.x_k[1] = 0.0f;

	roll_k.a = 0;
	roll_k.b = 0;
	roll_k.c = 0;
	roll_k.g = 0;
	roll_k.acc_betrag = 0;
	roll_k.noise = 0;

	pitch_k.a = 0;
	pitch_k.b = 0;
	pitch_k.c = 0;
	pitch_k.g = 0;
	pitch_k.acc_betrag = 0;
	pitch_k.noise = 0;


	roll_k.P_k[0][0] = P_INIT_UNCERTAINTY;
	roll_k.P_k[0][1] = 0.0f;
	roll_k.P_k[1][0] = 0.0f;
	roll_k.P_k[1][1] = P_INIT_UNCERTAINTY;


	pitch_k.P_k[0][0] = P_INIT_UNCERTAINTY;
	pitch_k.P_k[0][1] = 0.0f;
	pitch_k.P_k[1][0] = 0.0f;
	pitch_k.P_k[1][1] = P_INIT_UNCERTAINTY;
}

void KalmanFilter(){
	 // Operate on Kalman Filter

	 Kalman_Measure_Roll(sensorWerte.roll_acc, sensorWerte.roll_delta_angle);
	 Kalman_Measure_Pitch(sensorWerte.pitch_acc, sensorWerte.pitch_delta_angle);

	 //sendDebugValues(sensorWerte.winkel_y_kalman, sensorWerte.winkel_y_acc, sensorWerte.acc_y_bias, sensorWerte.acc_y, 0, 0);

     Kalman_Set_R_old(&sensorWerte, &controllerWerte);		// Gut geflogen

	 Kalman_Estimate_Roll();
	 Kalman_Estimate_Pitch();

	 sensorWerte.winkel_x_kalman = Kalman_Correct_Roll();
	 sensorWerte.winkel_y_kalman = Kalman_Correct_Pitch();
}


void Kalman_Set_R_neu(sensorDaten* Sensoren, controllerDaten* Controller ){

	double ar_x_acc, ar_y_acc;			// Angular Rate form Acc

	ar_x_acc = (Sensoren->roll_acc_old - Sensoren->roll_acc) / (double)(SAMPLE_TIME)  * 1000.0f;
	ar_y_acc = (Sensoren->pitch_acc_old - Sensoren->pitch_acc) / (double)(SAMPLE_TIME)  * 1000.0f;

	roll_k.a = ar_x_acc;
	roll_k.b = ar_y_acc;

	R_k_Roll[0][0] = roll_k.noise;
	R_k_Pitch[0][0] = roll_k.noise;
}


// Aktuell Beste Version
void Kalman_Set_R_old(sensorDaten* Sensoren, controllerDaten* Controller ){

	float a,b,c;		// (A)cceleration Change Part, (B)etrag Part

	// Betrag Part (Difference to expected Gravitation Vector only)
	float acc_betrag = sqrt( Sensoren->acc_x*Sensoren->acc_x
			+ Sensoren->acc_y*Sensoren->acc_y + Sensoren->acc_z*Sensoren->acc_z );

	if (acc_betrag > 900 && acc_betrag < 1100)
				b = 1;			//1
		else if (acc_betrag > 800 && acc_betrag < 1200)
				b = 2;			//3
		else if (acc_betrag > 700 && acc_betrag < 1300)
				b = 3;			//10
		else if (acc_betrag > 600 && acc_betrag < 1400)
				b = 4;
		else if (acc_betrag > 500 && acc_betrag < 1500)
				b = 5;
		else if (acc_betrag > 490 && acc_betrag < 1600)
				b = 6;
		else if (acc_betrag > 480 && acc_betrag < 1700)
				b = 7;
		else if (acc_betrag > 450 && acc_betrag < 1800)
				b = 8;
		else if (acc_betrag > 400 && acc_betrag < 1900)
				b = 10;
		else if (acc_betrag > 350 && acc_betrag < 2000)
				b = 15;
		else if (acc_betrag > 300 && acc_betrag < 2200)
				b = 20;
		else
				b = 50;


	// Acceleration Change Part
	a = (MF_Acc_X_Mean.value - MF_Acc_X_Mean_Delayed.value)*(MF_Acc_X_Mean.value - MF_Acc_X_Mean_Delayed.value)
			+ (MF_Acc_Y_Mean.value - MF_Acc_Y_Mean_Delayed.value) * (MF_Acc_Y_Mean.value - MF_Acc_Y_Mean_Delayed.value)
			+ (MF_Acc_Z_Mean.value - MF_Acc_Z_Mean_Delayed.value) * (MF_Acc_Z_Mean.value - MF_Acc_Z_Mean_Delayed.value);

	if (a > 100000.0f)
		a  = 100;
	else
		a = 1;

	c = abs(Controller->error_pitch) + abs(Controller->error_roll) + abs(Controller->error_yaw);

	roll_k.acc_betrag = acc_betrag;
	roll_k.b = b;
	roll_k.a = a;
	roll_k.c = c;

	roll_k.noise = (roll_k.a+roll_k.b+roll_k.c)* KF_Parameter.acc_measure_noise;

	R_k_Roll[0][0] = roll_k.noise;
	R_k_Pitch[0][0] = roll_k.noise;
}


void Kalman_Measure_Roll(double acc, double gyro){
	// Sets Measurements for Roll into the Kalman Equation

	roll_k.y_k[0] = acc;
	roll_k.y_k[1] = gyro;
}

void Kalman_Measure_Pitch(double acc, double gyro){
	// Sets Measurements for Pitch into the Kalman Equation

	pitch_k.y_k[0] = acc;
	pitch_k.y_k[1] = gyro;
}

////////////////////////////////////////////////////////////////////////////
//	********************	Roll Axis	************************************
////////////////////////////////////////////////////////////////////////////
void Kalman_Estimate_Roll(){
	// Estimate Next State

	// ***	Project State Ahead	********************************************
	// x_e = A x
	Matrix_mal_Vektor_2D (A_k, roll_k.x_k, roll_k.xe_k);

	// A P A'T
	Matrixmultiplikation2D(A_k,roll_k.P_k, Temp1_k);// A P 			-> TEMP1
	Matrixmultiplikation2D(Temp1_k, At_k, Temp2_k);	// TEMP1 A'T 	-> TEMP2

	// ***	Project Kalman Uncertainty *************************************
	// P = A P A'T + Q
	Matrixaddition2D(Temp2_k,Q_k,roll_k.Pe_k);				// TEMP2 + Q	-> P
}

double Kalman_Correct_Roll(){
	// Correct Estimation with Measurement

	//R_k[0][0] = fly_noise[0];

	// ***	Compute Kalman Gain	********************************************
	// C P C'T
	Matrixmultiplikation2D(roll_k.Pe_k,Ct_k, Temp3_k);// P C'T		-> TEMP3
	Matrixmultiplikation2D(C_k, Temp3_k, Temp1_k);	// C TEMP3  	-> TEMP1

	// C P C'T + R
	Matrixaddition2D(Temp1_k,R_k_Roll,Temp2_k);			// TEMP1 + R	-> TEMP2

	// (C P C'T + R)^-1
	Matrix_Inverse(Temp2_k, Temp1_k);				// TEMP2^-1		-> TEMP1

	// K = P C'T (C P C'T + R)^-1
	Matrixmultiplikation2D(Temp3_k, Temp1_k, roll_k.K_k);
	// *********************************************************************


	// ***	Correct State **************************************************
	// C x
	Matrix_mal_Vektor_2D(C_k, roll_k.xe_k, Temp_v1_k);// C xe		-> TEMPv1

	//  y - C x
	Skalar_mal_Vektor(-1.0f, Temp_v1_k);			// -TEMPv1		-> TEMPv1
	Vektoraddition2D(roll_k.y_k, Temp_v1_k, Temp_v2_k);// y + TEMPv1-> TEMPv2

	// K (y - C x)									K_K TEMPv2		-> TEMPv1
	Matrix_mal_Vektor_2D(roll_k.K_k, Temp_v2_k, Temp_v1_k);

	// x = x + K (y - C x)
	Vektoraddition2D(roll_k.xe_k, Temp_v1_k, roll_k.x_k);
	// *********************************************************************

	// ***	Update Error Covariance ****************************************
	// K C
	Matrixmultiplikation2D(roll_k.K_k, C_k, Temp1_k);// K C			-> TEMP1

	// I - K C
	Skalar_mal_Matrix(-1.0f, Temp1_k);				// -TEMP1		-> TEMP1
	Matrixaddition2D(Einheitsmatrix, Temp1_k, Temp2_k);// T + TEMP1	-> TEMP2

	// P = (I - K C) P
	Matrixmultiplikation2D(Temp2_k,roll_k.Pe_k,roll_k.P_k);

	// *********************************************************************

	return roll_k.x_k[0];
}

////////////////////////////////////////////////////////////////////////////
//	********************	Pitch Axis	************************************
////////////////////////////////////////////////////////////////////////////
void Kalman_Estimate_Pitch(){
	// Estimate Next State

	// ***	Project State Ahead	********************************************
		// x_e = A x
		Matrix_mal_Vektor_2D (A_k, pitch_k.x_k, pitch_k.xe_k);

		// A P A'T
		Matrixmultiplikation2D(A_k,pitch_k.P_k, Temp1_k);// A P 		-> TEMP1
		Matrixmultiplikation2D(Temp1_k, At_k, Temp2_k);	// TEMP1 A'T 	-> TEMP2

		// ***	Project Kalman Uncertainty *************************************
		// P = A P A'T + Q
		Matrixaddition2D(Temp2_k,Q_k,pitch_k.Pe_k);		// TEMP2 + Q	-> P

}

double Kalman_Correct_Pitch(){
	// Correct Estimation with Measurement

	//R_k[0][0] = fly_noise[1];

	// ***	Compute Kalman Gain	********************************************
		// C P C'T
		Matrixmultiplikation2D(pitch_k.Pe_k,Ct_k, Temp3_k);// P C'T		-> TEMP3
		Matrixmultiplikation2D(C_k, Temp3_k, Temp1_k);	// C TEMP3  	-> TEMP1

		// C P C'T + R
		Matrixaddition2D(Temp1_k,R_k_Pitch,Temp2_k);			// TEMP1 + R	-> TEMP2

		// (C P C'T + R)^-1
		Matrix_Inverse(Temp2_k, Temp1_k);				// TEMP2^-1		-> TEMP1

		// K = P C'T (C P C'T + R)^-1
		Matrixmultiplikation2D(Temp3_k, Temp1_k, pitch_k.K_k);
		// *********************************************************************


		// ***	Correct State **************************************************
		// C x
		Matrix_mal_Vektor_2D(C_k, pitch_k.xe_k, Temp_v1_k);// C xe		-> TEMPv1

		//  y - C x
		Skalar_mal_Vektor(-1.0f, Temp_v1_k);			// -TEMPv1		-> TEMPv1
		Vektoraddition2D(pitch_k.y_k, Temp_v1_k, Temp_v2_k);// y + TEMPv1-> TEMPv2

		// K (y - C x)									K_k TEMPv2		-> TEMPv1
		Matrix_mal_Vektor_2D(pitch_k.K_k, Temp_v2_k, Temp_v1_k);

		// x = x + K (y - C x)
		Vektoraddition2D(pitch_k.xe_k, Temp_v1_k, pitch_k.x_k);
		// *********************************************************************

		// ***	Update Error Covariance ****************************************
		// K C
		Matrixmultiplikation2D(pitch_k.K_k, C_k, Temp1_k);// K C			-> TEMP1

		// I - K C
		Skalar_mal_Matrix(-1.0f, Temp1_k);				// -TEMP1		-> TEMP1
		Matrixaddition2D(Einheitsmatrix, Temp1_k, Temp2_k);// I + TEMP1	-> TEMP2

		// P = (I - K C) P
		Matrixmultiplikation2D(Temp2_k,pitch_k.Pe_k,pitch_k.P_k);

		// *********************************************************************

		return pitch_k.x_k[0];
}

void Kalman_Set_RP(double roll, double pitch){

	sensorWerte.winkel_x_kalman = roll;
	sensorWerte.winkel_y_kalman = pitch;

	roll_k.x_k[0] = roll;
	pitch_k.x_k[0] = pitch;

	sprintf(debug_line, "Set KF: Roll = %3.3f; Pitch = %3.3f;", roll, pitch);
	USART_schreiben(debug_line);
}
