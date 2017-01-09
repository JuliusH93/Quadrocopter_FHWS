// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011-2014 Nils Gageik

/*
 * SignalProcessing.c
 *
 *  Created on: 20.01.2012
 *      Author: gageik
 */

#ifndef SIGNALPROCESSING_C_
#define SIGNALPROCESSING_C_

#include "QUAD_TWI_6DOF.h"

extern steuerDaten steuerWerte;
extern sensorDaten sensorWerte;

Quaternion Quat;					// Orientation Quaternion
Quaternion Acc_Quat;
Quaternion Mag_Quat;
Quaternion Mag_Quat_Init;				// Quaternion, das den Init_Mag_Sensor in die z-Ebene rotiert
double RPY_Acc[3];
double RPY_Mag[3];

double acc_noise  = 30.0;

extern controllerDaten controllerWerte;

extern MeanFilter MF_Infrared_Height;
extern MeanFilter MF_Ultrasonic_Height;
extern MeanFilter MF_Height;
extern MeanFilter MF_Acc_X_Mean;
extern MeanFilter MF_Acc_X_Mean_Delayed;
extern MeanFilter MF_Acc_Y_Mean;
extern MeanFilter MF_Acc_Y_Mean_Delayed;
extern MeanFilter MF_Acc_Z_Mean;
extern MeanFilter MF_Acc_Z_Mean_Delayed;
extern MeanFilter MF_Mag_Yaw;

extern AusreisserFilter Gyro_X_Ausreisser_Filter;
extern AusreisserFilter Gyro_Y_Ausreisser_Filter;
extern AusreisserFilter Gyro_Z_Ausreisser_Filter;

sensorDaten_raw sensorWerte_raw;

// Kalman Filter Valies, exclusive Parameters for every Axis
extern Kalman roll_k;
extern Kalman pitch_k;

int gravitation_vector[3] = {0, 0, 1000};

extern char debug_line[DEBUG_BUFFER_SIZE];

extern unsigned int tc_ticks;
extern unsigned int last_SignalProcessing_time;	// Last tc_tick Count for Signal Processing


/// ---------------------        C A L I B R A T E			--------------------------- /////////////////////////////////////
//  **************************************************************************************************************************
void calibrate(void){
	// Calculate Bias with Gravitation Compensation

	// ---- Calibration ----------------------------
	// Calibrate Sensor
	sensorDaten_raw neuerWert;
	int error = 0;
	int i;
	unsigned int t,ct;
	short dummy = 1;

	double z_Ebene_Vector[3];

	sensorWerte.acc_x_bias = 0.0f;
	sensorWerte.acc_y_bias = 0.0f;
	sensorWerte.acc_z_bias = 0.0f;

	sensorWerte.gyro_x_bias = 0.0f;
	sensorWerte.gyro_y_bias = 0.0f;
	sensorWerte.gyro_z_bias = 0.0f;

	controllerWerte.fehlerintegral_roll 	= 0.0f;
	controllerWerte.fehlerintegral_pitch 	= 0.0f;
	controllerWerte.fehlerintegral_yaw 		= 0.0f;
	sensorWerte.winkel_x_gyro = 0.0f;
	sensorWerte.winkel_y_gyro = 0.0f;
	sensorWerte.winkel_z_gyro = 0.0f;

	ct = tc_ticks;

	// Read Fix Calibrated Values
	sensorWerte.acc_sensivity_x = 1000.0;
	sensorWerte.acc_sensivity_y = 1000.0;
	sensorWerte.acc_sensivity_z = 1000.0;

	sensorWerte.mag_sensivity_x = 250.0;
	sensorWerte.mag_sensivity_y = 250.0;
	sensorWerte.mag_sensivity_z = 250.0;


	#ifdef CALIBRATION_DEBUG
	sprintf(debug_line,"\n Acc Sensivity (x/y/z): %3.4f/%3.4f/%3.4f \n", sensorWerte.acc_sensivity_x, sensorWerte.acc_sensivity_y,
			sensorWerte.acc_sensivity_z);
	USART_schreiben(debug_line);

	sprintf(debug_line,"\n Mag Sensivity (x/y/z): %3.4f/%3.4f/%3.4f \n", sensorWerte.mag_sensivity_x, sensorWerte.mag_sensivity_y,
				sensorWerte.mag_sensivity_z);
	USART_schreiben(debug_line);

	sprintf(debug_line,"\n Acc Bias (x/y/z): %3.4f/%3.4f/%3.4f \n", sensorWerte.acc_x_bias, sensorWerte.acc_y_bias,
			sensorWerte.acc_z_bias);
	USART_schreiben(debug_line);

	sprintf(debug_line,"\n Mag Bias (x/y/z): %3.4f/%3.4f/%3.4f \n", sensorWerte.mag_x_bias, sensorWerte.mag_y_bias,
				sensorWerte.mag_z_bias);
		USART_schreiben(debug_line);
	#endif

	sprintf(debug_line,"\n Start Sensor");
	USART_schreiben(debug_line);

	t  = tc_ticks;
	while(tc_ticks - t < 1000){
		dummy = (dummy + 1)%13;}		//Wait 0.5s

	gpio_set_gpio_pin(SENSOR_CALIBRATE_LED);

	sprintf(debug_line," Calibration \n");
	USART_schreiben(debug_line);

	for (i = 0; i < BIAS_SAMPLES; i++){

			t  = tc_ticks;

			error = read_ex_IMU_Sensors(&neuerWert);

			cond_IMU_Sensors(&neuerWert);


			if (error){
					steuerWerte.regler_on = 0;
					sprintf(debug_line,"\nTWI Calibration Error \n");
					USART_schreiben(debug_line);
					gpio_clr_gpio_pin(SENSOR_CALIBRATE_LED);
					return;
				}

			#ifdef ACC_DYNAMIC_BIAS
			sensorWerte.acc_x_bias = sensorWerte.acc_x_bias + neuerWert.acc_x_f;
			sensorWerte.acc_y_bias = sensorWerte.acc_y_bias + neuerWert.acc_y_f;
			sensorWerte.acc_z_bias = sensorWerte.acc_z_bias + neuerWert.acc_z_f;
			#endif

			sensorWerte.gyro_x_bias = sensorWerte.gyro_x_bias + neuerWert.gyro_x_f;
			sensorWerte.gyro_y_bias = sensorWerte.gyro_y_bias + neuerWert.gyro_y_f;
			sensorWerte.gyro_z_bias = sensorWerte.gyro_z_bias + neuerWert.gyro_z_f;

			sensorWerte.Mag_Vec_Init[0] = sensorWerte.Mag_Vec_Init[0] + neuerWert.mag_x_f;
			sensorWerte.Mag_Vec_Init[1] = sensorWerte.Mag_Vec_Init[1] + neuerWert.mag_y_f;
			sensorWerte.Mag_Vec_Init[2] = sensorWerte.Mag_Vec_Init[2] + neuerWert.mag_z_f;


			#ifdef DISPLAY_ON

			if (i == BIAS_SAMPLES / 100){
				 dip204_set_cursor_position(1,1);
				 dip204_write_string("  Att. Contr. 01%   ");
			}

			if (i == BIAS_SAMPLES / 10){
				 dip204_set_cursor_position(1,1);
				 dip204_write_string("  Att. Contr. 10%   ");
			}

			if (i == BIAS_SAMPLES / 2){
				 dip204_set_cursor_position(1,1);
				 dip204_write_string("  Att. Contr. 50%   ");

			}
			#endif

			if (i == BIAS_SAMPLES / 2)
				 gpio_clr_gpio_pin(SENSOR_CALIBRATE_LED);

			// Messungen im 1ms takt
			while(tc_ticks == t)
				dummy = (dummy + 1)%13;
	}


	 #ifdef DISPLAY_ON
	 dip204_set_cursor_position(1,1);
	 dip204_write_string("  Att. Contr. 100%  ");
	 dip204_hide_cursor();
	#else
	 USART_schreiben("Calibration Finished!\n");
	 #endif

	gpio_set_gpio_pin(SENSOR_CALIBRATE_LED);

	// Init Mag Vectors considering Bias and Sensivity
	sensorWerte.Mag_Vec_Init[0] = (sensorWerte.Mag_Vec_Init[0] / (double)BIAS_SAMPLES - sensorWerte.mag_x_bias)
			/ sensorWerte.mag_sensivity_x;
	sensorWerte.Mag_Vec_Init[1] = (sensorWerte.Mag_Vec_Init[1] / (double)BIAS_SAMPLES - sensorWerte.mag_y_bias)
			/ sensorWerte.mag_sensivity_y;
	sensorWerte.Mag_Vec_Init[2] = (sensorWerte.Mag_Vec_Init[2] / (double)BIAS_SAMPLES - sensorWerte.mag_z_bias)
			/ sensorWerte.mag_sensivity_z;

	normalize_vector(sensorWerte.Mag_Vec_Init);

	// Compute Mag Initial Quaternion to transform Mag_Vector into z-axis
	z_Ebene_Vector[0] = sensorWerte.Mag_Vec_Init[0];
	z_Ebene_Vector[1] = sensorWerte.Mag_Vec_Init[1];
	z_Ebene_Vector[2] = 0.0;

	Mag_Quat_Init = Quat_from_Vectors(z_Ebene_Vector,sensorWerte.Mag_Vec_Init);

	// Rotate Mag_Vector_Init into z-plane
	Q_rotate_Vec(sensorWerte.Mag_Vec_Init, Mag_Quat_Init, sensorWerte.Mag_Vec_Init);

	#ifdef ACC_DYNAMIC_BIAS
	sensorWerte.acc_x_bias = sensorWerte.acc_x_bias / (double)BIAS_SAMPLES;
	sensorWerte.acc_y_bias = sensorWerte.acc_y_bias / (double)BIAS_SAMPLES;
	sensorWerte.acc_z_bias = sensorWerte.acc_z_bias / (double)BIAS_SAMPLES + sensorWerte.acc_sensivity_z;
	#endif

	// Init Acc Vector
	sensorWerte.Acc_Vec_Init[0] = 0.0;
	sensorWerte.Acc_Vec_Init[1] = 0.0;
	sensorWerte.Acc_Vec_Init[2] = -1.0;

	sensorWerte.gyro_x_bias = sensorWerte.gyro_x_bias / (double)BIAS_SAMPLES;
	sensorWerte.gyro_y_bias = sensorWerte.gyro_y_bias / (double)BIAS_SAMPLES;
	sensorWerte.gyro_z_bias = sensorWerte.gyro_z_bias / (double)BIAS_SAMPLES;


	// Init Quaternion
	Quat.q0 = 1.0f;
	Quat.q1 = 0.0f;
	Quat.q2 = 0.0f;
	Quat.q3 = 0.0f;

	//Init SP Variables
	sensorWerte.gyro_x = 0.0f;
	sensorWerte.gyro_y = 0.0f;
	sensorWerte.gyro_z = 0.0f;

	// Set Times to zero
	last_SignalProcessing_time = tc_ticks;

	sprintf(debug_line,"Calibration Time: %i \n", tc_ticks - ct);
	USART_schreiben(debug_line);

	// Init Feuersuche-Temp Values
	sensorWerte.temp_max_winkel = 0;
	sensorWerte.temp_max_aussen = 10;
}



/// ---------------------        S I G N A L - P R O C E S S I N G 			-------------------- /////////////////////////////////////
//  XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX


SignalProcessed SignalProcessing(){

	Debug_Msg('S');

	SignalProcessed Out;

	// Get Values from Sensors and Calibrate
	Sensoring();

	// Rotate Current Quaternion according to new Gyro Values
	Quat = rotateQuat(Quat, sensorWerte.gyro_x_sampletimed, sensorWerte.gyro_y_sampletimed, sensorWerte.gyro_z_sampletimed);

	Out.roll_angle = sensorWerte.roll_angle;
	Out.pitch_angle = sensorWerte.pitch_angle;
	Out.yaw_angle = sensorWerte.yaw_angle;

	Out.roll_delta_angle = sensorWerte.gyro_y;
	Out.pitch_delta_angle = sensorWerte.gyro_x;
	Out.yaw_delta_angle = sensorWerte.gyro_z;

	return Out;
}

// Complementary Quaternionenfilter
SignalProcessed CQF(){

	SignalProcessed Out;

	// Correct Quaternion using Acc & Mag Vector (Measurements)
	correctQuat2();

	// Calculate RPY Angles from Current Orientation Quaternion for Control
	calcRPY();			// Maybe obsolete

	Out.roll_angle = sensorWerte.roll_angle;
	Out.pitch_angle = sensorWerte.pitch_angle;
	Out.yaw_angle = sensorWerte.yaw_angle;

	Out.roll_delta_angle = sensorWerte.roll_delta_angle;
	Out.pitch_delta_angle = sensorWerte.pitch_delta_angle;
	Out.yaw_delta_angle = sensorWerte.yaw_delta_angle;

	return Out;
}



void Sensoring(){
	 // Get SensorValues, Bias, Integrate and Process

	 sensorDaten_raw sensorWerte_raw1, sensorWerte_raw2;

	 // **********  I M U ***************************************************************************************************
	 // IMU Readings for Attitude
	 // Read Twice to overcome data corruption coz of bus errors on communication line
	 read_IMU_Sensors(&sensorWerte_raw1);
	 read_IMU_Sensors(&sensorWerte_raw2);

	 sensorWerte.RohWerte.acc_x_s = myMedian(sensorWerte_raw1.acc_x_s, sensorWerte_raw2.acc_x_s, sensorWerte.RohWerte.acc_x_s);
	 sensorWerte.RohWerte.acc_y_s = myMedian(sensorWerte_raw1.acc_y_s, sensorWerte_raw2.acc_y_s, sensorWerte.RohWerte.acc_y_s);
	 sensorWerte.RohWerte.acc_z_s = myMedian(sensorWerte_raw1.acc_z_s, sensorWerte_raw2.acc_z_s, sensorWerte.RohWerte.acc_z_s);

	#ifndef GYRO_AUSREISSER_FILTER
	 sensorWerte.RohWerte.gyro_x_s = myMedian(sensorWerte_raw1.gyro_x_s, sensorWerte_raw2.gyro_x_s, sensorWerte.RohWerte.gyro_x_s);
	 sensorWerte.RohWerte.gyro_y_s = myMedian(sensorWerte_raw1.gyro_y_s, sensorWerte_raw2.gyro_y_s, sensorWerte.RohWerte.gyro_y_s);
	 sensorWerte.RohWerte.gyro_z_s = myMedian(sensorWerte_raw1.gyro_z_s, sensorWerte_raw2.gyro_z_s, sensorWerte.RohWerte.gyro_z_s);
	#else
	 sensorWerte.RohWerte.gyro_x_s = Calc_New_AusreisserFilter(&Gyro_X_Ausreisser_Filter, sensorWerte_raw1.gyro_x_s, sensorWerte_raw2.gyro_x_s);
	 sensorWerte.RohWerte.gyro_y_s = Calc_New_AusreisserFilter(&Gyro_Y_Ausreisser_Filter, sensorWerte_raw1.gyro_y_s, sensorWerte_raw2.gyro_y_s);
	 sensorWerte.RohWerte.gyro_z_s = Calc_New_AusreisserFilter(&Gyro_Z_Ausreisser_Filter, sensorWerte_raw1.gyro_z_s, sensorWerte_raw2.gyro_z_s);
	#endif

	 sensorWerte.RohWerte.mag_x_s = myMedian(sensorWerte_raw1.mag_x_s, sensorWerte_raw2.mag_x_s, sensorWerte.RohWerte.mag_x_s);
	 sensorWerte.RohWerte.mag_y_s = myMedian(sensorWerte_raw1.mag_y_s, sensorWerte_raw2.mag_y_s, sensorWerte.RohWerte.mag_y_s);
	 sensorWerte.RohWerte.mag_z_s = myMedian(sensorWerte_raw1.mag_z_s, sensorWerte_raw2.mag_z_s, sensorWerte.RohWerte.mag_z_s);

	  Conditioning();
}


void Conditioning(){
// Conditions the Sensor Values: Coming from Raw Values to calibrated, mean filtered values

		// Sensor Condition of Each Sensor
		cond_IMU_Sensors(&sensorWerte.RohWerte);

		// Bias-Compensation
		sensorWerte.acc_x = (sensorWerte.RohWerte.acc_x_f - sensorWerte.acc_x_bias) * 1000 / sensorWerte.acc_sensivity_x;
		sensorWerte.acc_y = (sensorWerte.RohWerte.acc_y_f - sensorWerte.acc_y_bias) * 1000 / sensorWerte.acc_sensivity_y;
		sensorWerte.acc_z = (sensorWerte.RohWerte.acc_z_f - sensorWerte.acc_z_bias) * 1000 / sensorWerte.acc_sensivity_z;

		sensorWerte.gyro_x = sensorWerte.RohWerte.gyro_x_f - sensorWerte.gyro_x_bias;
		sensorWerte.gyro_y = sensorWerte.RohWerte.gyro_y_f - sensorWerte.gyro_y_bias;
		sensorWerte.gyro_z = sensorWerte.RohWerte.gyro_z_f - sensorWerte.gyro_z_bias;

		sensorWerte.mag_x = (sensorWerte.RohWerte.mag_x_f - sensorWerte.mag_x_bias) * 1000 / sensorWerte.mag_sensivity_x;
		sensorWerte.mag_y = (sensorWerte.RohWerte.mag_y_f - sensorWerte.mag_y_bias) * 1000 / sensorWerte.mag_sensivity_y;
		sensorWerte.mag_z = (sensorWerte.RohWerte.mag_z_f - sensorWerte.mag_z_bias) * 1000 / sensorWerte.mag_sensivity_z;

		sensorWerte.mag_magnitude = sqrt(sensorWerte.mag_x * sensorWerte.mag_x
				+ sensorWerte.mag_y * sensorWerte.mag_y
				+ sensorWerte.mag_z * sensorWerte.mag_z);

		/*
		sensorWerte.mag_magnitude = sqrt(sensorWerte.RohWerte.mag_x_f * sensorWerte.RohWerte.mag_x_f
				+ sensorWerte.RohWerte.mag_y_f * sensorWerte.RohWerte.mag_y_f
				+ sensorWerte.RohWerte.mag_z_f * sensorWerte.RohWerte.mag_z_f);
				*/

		fill_vector(sensorWerte.mag_x, sensorWerte.mag_y, sensorWerte.mag_z, sensorWerte.Mag_Vec);
		fill_vector(sensorWerte.acc_x, sensorWerte.acc_y, sensorWerte.acc_z, sensorWerte.Acc_Vec);

		normalize_vector(sensorWerte.Mag_Vec);
		normalize_vector(sensorWerte.Acc_Vec);

		// Digital Lowpass Filter
		sensorWerte.acc_x_dpf = (double)(iir1((float)sensorWerte.acc_x));
		sensorWerte.acc_y_dpf = (double)(iir2((float)sensorWerte.acc_y));

		// Convert Gyro Values according to SAMPLE_TIME (ms)
		sensorWerte.gyro_x_sampletimed = sensorWerte.gyro_x * SAMPLE_TIME / 1000.f;
		sensorWerte.gyro_y_sampletimed = sensorWerte.gyro_y * SAMPLE_TIME / 1000.f;
		sensorWerte.gyro_z_sampletimed = sensorWerte.gyro_z * SAMPLE_TIME / 1000.f;

		sensorWerte.winkel_x_gyro = sensorWerte.winkel_x_gyro + sensorWerte.gyro_x_sampletimed;
		sensorWerte.winkel_y_gyro = sensorWerte.winkel_y_gyro + sensorWerte.gyro_y_sampletimed;
		sensorWerte.winkel_z_gyro = sensorWerte.winkel_z_gyro + sensorWerte.gyro_z_sampletimed;

		// For KalmanVarianz Computation: Calc Two Means: Current, and delayed (by Filter_size shifted)
		Calc_New_Mean(&MF_Acc_X_Mean_Delayed, Calc_New_Mean(&MF_Acc_X_Mean, sensorWerte.acc_x_dpf));
		Calc_New_Mean(&MF_Acc_Y_Mean_Delayed, Calc_New_Mean(&MF_Acc_Y_Mean, sensorWerte.acc_y_dpf));
		Calc_New_Mean(&MF_Acc_Z_Mean_Delayed, Calc_New_Mean(&MF_Acc_Z_Mean, sensorWerte.acc_z));

		// Saturate, coz asin needs -1,+1
		sensorWerte.acc_x = my_saturate(sensorWerte.acc_x_dpf, sensorWerte.acc_sensivity_x);
		sensorWerte.acc_y = my_saturate(MF_Acc_Y_Mean.value, sensorWerte.acc_sensivity_y);

		sensorWerte.roll_acc_old = sensorWerte.roll_acc;
		sensorWerte.pitch_acc_old = sensorWerte.pitch_acc;

		sensorWerte.roll_acc =  asin (sensorWerte.acc_y / 1000) * -180.0f / PI;
		sensorWerte.pitch_acc =  asin (sensorWerte.acc_x / 1000) * 180.0f / PI;
}


Quaternion rotateQuat2(Quaternion myQuat, double x, double y, double z){
	// Rotate Quaternion According to three Angles x,y,z
	// Atheel Version
	double norm;          		  // vector norm
	double q0_D, q1_D, q2_D, q3_D; // quaternion derrivative from gyroscopes elements

	double gx_rad_h=x*PI/360;
	double gy_rad_h=y*PI/360;
	double gz_rad_h=z*PI/360;


	// Compute the quaternion derrivative measured by gyroscopes
	q0_D =-myQuat.q1 * gx_rad_h - myQuat.q2 * gy_rad_h - myQuat.q3 * gz_rad_h;
	q1_D = myQuat.q0 * gx_rad_h + myQuat.q2 * gz_rad_h - myQuat.q3 * gy_rad_h;
	q2_D = myQuat.q0 * gy_rad_h - myQuat.q1 * gz_rad_h + myQuat.q3 * gx_rad_h;
	q3_D = myQuat.q0 * gz_rad_h + myQuat.q1 * gy_rad_h - myQuat.q2 * gx_rad_h;

	// integrate the estimated quaternion derrivative
	myQuat.q0 += q0_D;
	myQuat.q1 += q1_D;
	myQuat.q2 += q2_D;
	myQuat.q3 += q3_D;

	// Normalise quaternion
	norm = sqrt(myQuat.q0*myQuat.q0 + myQuat.q1*myQuat.q1 + myQuat.q2*myQuat.q2 + myQuat.q3*myQuat.q3);
	myQuat.q0 /= norm;
	myQuat.q1 /= norm;
	myQuat.q2 /= norm;
	myQuat.q3 /= norm;

	return myQuat;
}


Quaternion rotateQuat(Quaternion myQuat, double x, double y, double z){
	// Calculate Quaternion from RPY
	// from Book Kuipers

	Quaternion dQuat, rQ;

	double betrag;

	double cdx = cos(x* (double)PI / 360.0f);
	double sdx = sin(x* (double)PI / 360.0f);
	double cdy = cos(y* (double)PI / 360.0f);
	double sdy = sin(y* (double)PI / 360.0f);
	double cdz = cos(z* (double)PI / 360.0f);
	double sdz = sin(z* (double)PI / 360.0f);

	//% Transforms RPY EulerAngles into Quaternion
	//% dx angle of rotation about x-axis (using RAD) | cdx = cos(dx) etc.
	//% dy angle of rotation about y-axis
	//% dz angle of rotation about z-axis

	// Compute dq
	dQuat.q0 = cdz*cdy*cdx + sdz*sdy*sdx;
	dQuat.q1 = cdz*cdy*sdx - sdz*sdy*cdx;
	dQuat.q2 = cdz*sdy*cdx + sdz*cdy*sdx;
	dQuat.q3 = sdz*cdy*cdx - cdz*sdy*sdx;

	betrag = sqrt(dQuat.q0*dQuat.q0+dQuat.q1*dQuat.q1+dQuat.q2*dQuat.q2+dQuat.q3*dQuat.q3);
	dQuat.q0 = dQuat.q0 / betrag;
	dQuat.q1 = dQuat.q1 / betrag;
	dQuat.q2 = dQuat.q2 / betrag;
	dQuat.q3 = dQuat.q3 / betrag;

	// Update Orientation-Quaternion: Quaternionenprodukt:
	// rQ = myQuat x dQuat
	rQ = Quatprodukt(myQuat, dQuat);

	//Normalise
	betrag = sqrt(rQ.q0*rQ.q0+rQ.q1*rQ.q1+rQ.q2*rQ.q2+rQ.q3*rQ.q3);
	rQ.q0 = rQ.q0 / betrag;
	rQ.q1 = rQ.q1 / betrag;
	rQ.q2 = rQ.q2 / betrag;
	rQ.q3 = rQ.q3 / betrag;

	return rQ;
}


void correctQuat(double x, double y, double z){
	// Correct the Quaternion usign RPY from Kalmanfilter
	// Does not correct every cycle to overcome instability & noise feedback

	double old_x, old_y, old_z;		// Current, drifted, old values
	double delta_x_old, delta_y_old;
	double delta_x, delta_y;

	sensorWerte.kalman_correct_index++;


	if (sensorWerte.kalman_correct_index == QUAT_CORRECT_INTERVALL){

		// Save current, drifted values
		old_x = sensorWerte.roll_angle;
		old_y = sensorWerte.pitch_angle;
		old_z = sensorWerte.yaw_angle;

		//sendDebugValues(Quat.q0,Quat.q1,Quat.q2,Quat.q3,0,0);
		Quat = calcQuat(x, y, z);
		calcRPY();		// Renew RPY Angles for correct delta computation

		delta_x_old = sensorWerte.roll_angle_old - sensorWerte.roll_angle;
		delta_y_old = sensorWerte.pitch_angle_old - sensorWerte.pitch_angle;
		//delta_z_old = sensorWerte.yaw_angle_old - sensorWerte.yaw_angle;

		delta_x = sensorWerte.roll_angle - old_x;
		delta_y = sensorWerte.pitch_angle - old_y;
		//delta_z = sensorWerte.yaw_angle - old_z;

		sensorWerte.kalman_correct_index = 0;
	}
}

void correctQuat2(){
	// New Correction Approach with Quaternion Vectors rotating back

	double compensation_cut_on = (double)1.0 - (double)COMPENSATION_CUT_OFF;


	static Quaternion compensation_Quat = {1.0, 0.0, 0.0, 0.0};

	//static double Mag_Vec_Mean[3] = {0.0, 0.0, 0.0};
	static double Acc_Vec_Mean[3] = {0.0, 0.0, 0.0};
	//static double Mag_Trans_Mean[3] = {0.0, 0.0, 0.0};
	static double Acc_Trans_Mean[3] = {0.0, 0.0, 0.0};

	// Transform Initial Measurements according to current Orientation
	Q_rotate_Vec(sensorWerte.Acc_Vec_Init, Quat, sensorWerte.Acc_Vec_trans);

	// Rotate Mag Vector according to Orientation and transform in z-plane
	Q_rotate_Vec(sensorWerte.Mag_Vec, Q_konjugiert(Quat), sensorWerte.Mag_Vec_trans);
	Q_rotate_Vec(sensorWerte.Mag_Vec_trans, Mag_Quat_Init, sensorWerte.Mag_Vec_trans);


	//Exponential_Vektor_Fusion(compensation_cut_on, COMPENSATION_CUT_OFF, Mag_Vec_Mean, sensorWerte.Mag_Vec_Init);
	Exponential_Vektor_Fusion(compensation_cut_on, COMPENSATION_CUT_OFF, Acc_Vec_Mean, sensorWerte.Acc_Vec);
	//Exponential_Vektor_Fusion(compensation_cut_on, COMPENSATION_CUT_OFF, Mag_Trans_Mean, sensorWerte.Mag_Vec_trans);
	Exponential_Vektor_Fusion(compensation_cut_on, COMPENSATION_CUT_OFF, Acc_Trans_Mean, sensorWerte.Acc_Vec_trans);

	// Compute Quaternion from transformed current Vectors to measured Vectors for compensation
	Acc_Quat = Quat_from_Vectors(Acc_Vec_Mean, Acc_Trans_Mean);
	Mag_Quat = Quat_from_Vectors(sensorWerte.Mag_Vec_trans, sensorWerte.Mag_Vec_Init);

	// Compute RPY for debugging
	calcRPY2(Acc_Quat, RPY_Acc);
	calcRPY2(Mag_Quat, RPY_Mag);

	// Compute Compensation Quaternion
	compensation_Quat = compensate_Drift(Acc_Quat, Mag_Quat);

	//Transform (Update) Mean Filterd Vectors according to compensation Quaternion			TODO: Überprüfen!
	//Q_rotate_Vec(Mag_Vec_Mean, Q_konjugiert(compensation_Quat), Mag_Vec_Mean);
	Q_rotate_Vec(Acc_Vec_Mean, Q_konjugiert(compensation_Quat), Acc_Vec_Mean);

	Quat = Quatprodukt(Quat, compensation_Quat);
}



Quaternion calcQuat(double x, double y, double z){
	// Computes Quaternion from Euler Angles (RPY - y,x,z)
	double d;

	Quaternion Quat2;

	double cdx = cos(x* (double)PI / 360.0f);
	double sdx = sin(x* (double)PI / 360.0f);
	double cdy = cos(y* (double)PI / 360.0f);
	double sdy = sin(y* (double)PI / 360.0f);
	double cdz = cos(z* (double)PI / 360.0f);
	double sdz = sin(z* (double)PI / 360.0f);

	Quat2.q0 = (cdx*cdy*cdz+sdx*sdy*sdz);
	Quat2.q1 = (sdx*cdy*cdz-cdx*sdy*sdz);
	Quat2.q2 = (cdx*sdy*cdz+sdx*cdy*sdz);
	Quat2.q3 = (cdx*cdy*sdz-sdx*sdy*cdz);

	//Normalise
	d = sqrt(Quat2.q0*Quat2.q0+Quat2.q1*Quat2.q1+Quat2.q2*Quat2.q2+Quat2.q3*Quat2.q3);
	Quat2.q0 = Quat2.q0 / d;
	Quat2.q1 = Quat2.q1 / d;
	Quat2.q2 = Quat2.q2 / d;
	Quat2.q3 = Quat2.q3 / d;

	return Quat2;
}



void calcRPY(){
	// Compute RPY Angular Rate (roll_angle & roll_delta_angle):
	// More accurate then using gyro x/y/z, In fact these are the transformed gyro x/y/z
	// Take them from Quat, write them to sensorWerte

	// RPY formular from Wiki
	double x,y,z;

	// Compute Roll/Pitch/Yaw Angle from Quat
	x = atan2( (2*(Quat.q0*Quat.q1 + Quat.q2*Quat.q3)), (1-2*(Quat.q1*Quat.q1+Quat.q2*Quat.q2)) ) * 180.0f / PI;

	y = asin( (2*(Quat.q0*Quat.q2 - Quat.q3*Quat.q1)) ) * 180.0f / PI;

	z = atan2( (2*(Quat.q0*Quat.q3 + Quat.q1*Quat.q2)), (1-2*(Quat.q2*Quat.q2+Quat.q3*Quat.q3)) ) * 180.0f / PI;


	// Compute Angular Rate with reference to actual orientation
	// -> Transformed Gyro Value
	sensorWerte.roll_delta_angle 	= (x - sensorWerte.roll_angle)	 * 1000 / SAMPLE_TIME;
	sensorWerte.pitch_delta_angle 	= (y - sensorWerte.pitch_angle)	 * 1000 / SAMPLE_TIME;
	sensorWerte.yaw_delta_angle 	= (z - sensorWerte.yaw_angle)	 * 1000 / SAMPLE_TIME;


	// Save last RPY angle for next step
	sensorWerte.roll_angle = x;
	sensorWerte.pitch_angle = y;
	sensorWerte.yaw_angle = z;
}

void calcRPY2(Quaternion myQuat, double* RPY){
	// Compute RPY Angular Rate (roll_angle & roll_delta_angle) from a Quaternion, write to RPY 3x1 vector
	// RPY formular from Wiki

	// Compute Roll/Pitch/Yaw Angle from Quat
	RPY[0] = atan2( (2*(myQuat.q0*myQuat.q1 + myQuat.q2*myQuat.q3)), (1-2*(myQuat.q1*myQuat.q1+myQuat.q2*myQuat.q2)) ) * 180.0f / PI;

	RPY[1] = asin( (2*(myQuat.q0*myQuat.q2 - myQuat.q3*myQuat.q1)) ) * 180.0f / PI;

	RPY[2] = atan2( (2*(myQuat.q0*myQuat.q3 + myQuat.q1*myQuat.q2)), (1-2*(myQuat.q2*myQuat.q2+myQuat.q3*myQuat.q3)) ) * 180.0f / PI;
}

// Other implementation, not better then calcRPY()
void calcRPY_SM(){

	double x,y,z;

	 double m21 = 2 * Quat.q1 * Quat.q2  + 2 * Quat.q0 * Quat.q3;
	 double m11 = 2 * Quat.q0 * Quat.q0 - 1 + 2 * Quat.q1 * Quat.q1;
	 double m31 = 2 * Quat.q1 * Quat.q3  - 2 * Quat.q0 * Quat.q2;
	 double m32 = 2 * Quat.q2 * Quat.q3  + 2 * Quat.q0 *Quat.q1;
	 double m33 = 2 * Quat.q0 * Quat.q0 - 1 + 2 * Quat.q3 * Quat.q3;

	 x  = atan2(m32, m33) * 180.0f / PI;
	 y = atan2(-m31, sqrt(m11*m11 + m21*m21)) * 180.0f / PI;
	 z  = atan2(m21, m11) * 180.0f / PI;


	 // Compute Angular Rate with reference to actual orientation
	// -> Transformed Gyro Value
	sensorWerte.roll_delta_angle 	= (x - sensorWerte.roll_angle)	 * 1000 / SAMPLE_TIME;
	sensorWerte.pitch_delta_angle 	= (y - sensorWerte.pitch_angle)	 * 1000 / SAMPLE_TIME;
	sensorWerte.yaw_delta_angle 	= (z - sensorWerte.yaw_angle)	 * 1000 / SAMPLE_TIME;


	// Save last RPY angle for next step
	sensorWerte.roll_angle = x;
	sensorWerte.pitch_angle = y;
	sensorWerte.yaw_angle = z;
}

void setRPY(double roll, double pitch, double yaw){

	// Set RPY angles
	sensorWerte.roll_angle = roll;
	sensorWerte.pitch_angle = pitch;
	sensorWerte.yaw_angle = yaw;

	sensorWerte.roll_angle_old = roll;
	sensorWerte.pitch_angle_old = pitch;

	Kalman_Set_RP(roll, pitch);

	Quat = calcQuat(roll, pitch, yaw);
}


Quaternion compensate_Drift(Quaternion Quat_Acc, Quaternion Quat_Mag){
	// return Compensation Quaternion

	static char my_toogle = 1;
	Quaternion compensation_Quat = {1.0, 0.0, 0.0, 0.0};

	#define ACC_NOISE		1000
	#define MAG_NOISE		100

	#define ACC_MAX_COMP_ANGLE	0.001		// 0.5 Grad / 1000 / (57 = 180 / PI)
	#define MAG_MAX_COMP_ANGLE	0.0001


	double rot_angle_acc, rot_angle_mag;


	rot_angle_acc = acos(Quat_Acc.q0);

	if (rot_angle_acc > ACC_MAX_COMP_ANGLE)
		rot_angle_acc = ACC_MAX_COMP_ANGLE;

	Quat_Acc.q0 = cos(rot_angle_acc);


	rot_angle_mag = acos(Quat_Mag.q0);

	if (rot_angle_mag > MAG_MAX_COMP_ANGLE)
		rot_angle_mag = MAG_MAX_COMP_ANGLE;

	Quat_Mag.q0 = cos(rot_angle_mag);


	// Toggle between Acc und Mag compensation: To avoid cross effects
	if (my_toogle){
		compensation_Quat = Q_normalise_fix_q0_zero_q3(Quat_Acc);
		#ifdef LSM_MAG_SENSOR
		my_toogle = 0;
		#endif
	}
	else {
		compensation_Quat = Q_normalise_fix_q0_zero12(Quat_Mag);
		my_toogle = 1;
	}

	Q_normalise(compensation_Quat);

	return compensation_Quat;
}
#endif /* SIGNALPROCESSING_C_ */
