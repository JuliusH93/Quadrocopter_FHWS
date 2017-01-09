/*
 * AttitudeCtrl.c
 *
 *  Created on: 05.01.2012
 *      Author: gageik
 */
// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011 Nils Gageik

#include "../QUAD_TWI_6DOF.h"			// All Header Files

controllerDaten controllerWerte; // Controlling Values

extern steuerDaten steuerWerte;

extern unsigned int tc_ticks;
unsigned int last_Controller_time;		// Last tc_tick Count for Controlling
extern char debug_line[DEBUG_BUFFER_SIZE];

void Attitude_Control_Init(){


	#ifndef FLUG
	controllerWerte.pid_roll.p = P_AUFBAU;
	controllerWerte.pid_roll.i = I_AUFBAU;
	controllerWerte.pid_roll.d = D_AUFBAU;
	controllerWerte.pid_pitch.p = P_AUFBAU;
	controllerWerte.pid_pitch.i = I_AUFBAU;
	controllerWerte.pid_pitch.d = D_AUFBAU;
	#endif


	#ifdef FLUG
	controllerWerte.pid_roll.p = P_FLUG;
	controllerWerte.pid_roll.i = I_FLUG;
	controllerWerte.pid_roll.d = D_FLUG;
	controllerWerte.pid_pitch.p = P_FLUG;
	controllerWerte.pid_pitch.i = I_FLUG;
	controllerWerte.pid_pitch.d = D_FLUG;
	#endif

	#ifdef ALEX_PD
	controllerWerte.pid_roll.p = P_ALEX;
	controllerWerte.pid_roll.d = D_ALEX;
	controllerWerte.pid_pitch.p = P_ALEX;
	controllerWerte.pid_pitch.d = D_ALEX;
	#endif

	controllerWerte.pid_yaw.p = (double)YAW_K * (double)P_YAW;
	controllerWerte.pid_yaw.i = (double)YAW_K * (double)I_YAW;
	controllerWerte.pid_yaw.d = (double)YAW_K * (double)D_YAW;
}


MotorControl Attitude_Controlling(SignalProcessed* SP_Data)
{

	Attitude_Control(SP_Data->roll_angle, SP_Data->pitch_angle, SP_Data->yaw_angle,
			SP_Data->roll_delta_angle, SP_Data->pitch_delta_angle, SP_Data->yaw_delta_angle);		// Regelschleife


	MotorControl Output;
	Output.stellwert_blctrl_engine1 = controllerWerte.stellwertM1;
	Output.stellwert_blctrl_engine2 = controllerWerte.stellwertM2;
	Output.stellwert_blctrl_engine3 = controllerWerte.stellwertM3;
	Output.stellwert_blctrl_engine4 = controllerWerte.stellwertM4;

	Debug_Msg('A');

	return Output;
}



// Controller of RPY Attitude Angles (Roll Pitch Yaw)
void Attitude_Control(double winkel_x, double winkel_y, double winkel_z, double angular_rate_x, double angular_rate_y, double angular_rate_z){

	char ctrl_nan = 0;
	// Prüfe auf NANs
	if (winkel_x != winkel_x)											{	winkel_x = 0;			ctrl_nan = 1;}
	if (winkel_y != winkel_y) 											{	winkel_y = 0;			ctrl_nan = 2;}
	if (winkel_z != winkel_z)											{	winkel_z = 0;			ctrl_nan = 3;}
	if (angular_rate_x != angular_rate_x)								{	angular_rate_x = 0;		ctrl_nan = 4;}
	if (angular_rate_y != angular_rate_y) 								{	angular_rate_y = 0;		ctrl_nan = 5;}
	if (angular_rate_z != angular_rate_z)								{	angular_rate_z = 0;		ctrl_nan = 6;}
	if (controllerWerte.soll_angle_roll != controllerWerte.soll_angle_roll)		{	controllerWerte.soll_angle_roll = 0	;	ctrl_nan = 7;}
	if (controllerWerte.soll_angle_pitch != controllerWerte.soll_angle_pitch)	{	controllerWerte.soll_angle_pitch = 0;	ctrl_nan = 8;}
	if (controllerWerte.soll_angle_yaw != controllerWerte.soll_angle_yaw)		{	controllerWerte.soll_angle_yaw = 0;		ctrl_nan = 9;}

	if(ctrl_nan){
		USART_schreiben("NAN ACTRL ");
		USART_schreibe_char(ctrl_nan);		// NAN Error ID
	}

	double roll = 0;
	double yaw = 0;
	double pitch = 0;
	double error_change_roll, error_change_pitch, error_change_yaw;
	double new_error_roll, new_error_pitch, new_error_yaw;
	double yaw1, yaw2, yaw3;
	double M1 = 0;
	double M2 = 0;
	double M3 = 0;
	double M4 = 0;

	double integral_cutoff = INT_CUTOFF;
	double rest_gas;				// Wieviel von max_gas für yaw bleibt
	double add_gas;
	double gas = controllerWerte.gas;
	double max_controller_out;

	new_error_roll = winkel_x + controllerWerte.soll_angle_roll;
	new_error_pitch = winkel_y + controllerWerte.soll_angle_pitch;
	new_error_yaw = winkel_z - controllerWerte.soll_angle_yaw;

	// Verwende für Yaw kürzesten "Weg"
	if (new_error_yaw > 180.0)
		new_error_yaw = new_error_yaw - 360.0;

	if (new_error_yaw < -180.0)
		new_error_yaw = new_error_yaw + 360.0;


	error_change_roll = new_error_roll - controllerWerte.error_roll;
	error_change_pitch = new_error_pitch - controllerWerte.error_pitch;
	error_change_yaw = new_error_yaw - controllerWerte.error_yaw;

	controllerWerte.error_roll = new_error_roll;
	controllerWerte.error_pitch = new_error_pitch;
	controllerWerte.error_yaw = new_error_yaw;


	// Idea: Only Integrate when Error is Small ( I-Part to improve convergence)
	// Cut-Off, if error is not very small (could be old part of movement)

	if (abs(controllerWerte.error_roll) < HIGH_INTEGRAL_ERROR_AREA){
		if(abs(controllerWerte.error_roll) < LOW_INTEGRAL_ERROR_AREA){
			integral_cutoff = 1;
		}
		else {
			controllerWerte.integral_index_roll++;
			if (controllerWerte.integral_index_roll > 5){
				integral_cutoff = INT_CUTOFF;
				controllerWerte.integral_index_roll = 0;
				}
		}
			controllerWerte.fehlerintegral_roll = integral_cutoff * controllerWerte.fehlerintegral_roll
					+ controllerWerte.error_roll;
		}

	if (abs(controllerWerte.error_pitch) < HIGH_INTEGRAL_ERROR_AREA){
		if(abs(controllerWerte.error_pitch) < LOW_INTEGRAL_ERROR_AREA){
			integral_cutoff = 1;
		}
		else {
			controllerWerte.integral_index_pitch++;			// TODO: OBSOLETE??!!
			if (controllerWerte.integral_index_pitch > 5){
				integral_cutoff = INT_CUTOFF;
				controllerWerte.integral_index_pitch = 0;
				}
		}
			controllerWerte.fehlerintegral_pitch = integral_cutoff * controllerWerte.fehlerintegral_pitch
					+ controllerWerte.error_pitch;
		}

	if (abs(controllerWerte.error_yaw) < HIGH_INTEGRAL_ERROR_AREA){
		if(abs(controllerWerte.error_yaw) < LOW_INTEGRAL_ERROR_AREA){
			integral_cutoff = 1;
		}
		else {
			controllerWerte.integral_index_yaw++;
			if (controllerWerte.integral_index_yaw > 5){
				integral_cutoff = INT_CUTOFF;
				controllerWerte.integral_index_yaw = 0;
				}
		}
			controllerWerte.fehlerintegral_yaw = integral_cutoff * controllerWerte.fehlerintegral_yaw
					+ controllerWerte.error_yaw;
		}


	if (controllerWerte.fehlerintegral_roll > INTEGRAL_SCHRANKE)
		controllerWerte.fehlerintegral_roll = INTEGRAL_SCHRANKE;

	if (controllerWerte.fehlerintegral_roll < -INTEGRAL_SCHRANKE)
			controllerWerte.fehlerintegral_roll = -INTEGRAL_SCHRANKE;

	if (controllerWerte.fehlerintegral_pitch > INTEGRAL_SCHRANKE)
		controllerWerte.fehlerintegral_pitch = INTEGRAL_SCHRANKE;

	if (controllerWerte.fehlerintegral_pitch < -INTEGRAL_SCHRANKE)
			controllerWerte.fehlerintegral_pitch = -INTEGRAL_SCHRANKE;

	if (controllerWerte.fehlerintegral_yaw > INTEGRAL_SCHRANKE)
		controllerWerte.fehlerintegral_yaw = INTEGRAL_SCHRANKE;

	if (controllerWerte.fehlerintegral_yaw < -INTEGRAL_SCHRANKE)
		controllerWerte.fehlerintegral_yaw = -INTEGRAL_SCHRANKE;


	// Roll Controller
	controllerWerte.roll_p_anteil = controllerWerte.pid_roll.p * controllerWerte.error_roll;
	controllerWerte.roll_i_anteil = controllerWerte.pid_roll.i * controllerWerte.fehlerintegral_roll;
	controllerWerte.roll_d_anteil = controllerWerte.pid_roll.d * angular_rate_x;			// negativ
	roll = 12.0f / BATTERY_VOLTAGE * ATTITUDE_K * (controllerWerte.roll_p_anteil + controllerWerte.roll_i_anteil + controllerWerte.roll_d_anteil);

	#ifdef FLUG
	// Pitch Controller
	controllerWerte.pitch_p_anteil = controllerWerte.pid_pitch.p * controllerWerte.error_pitch;
	controllerWerte.pitch_i_anteil = controllerWerte.pid_pitch.i * controllerWerte.fehlerintegral_pitch;
	controllerWerte.pitch_d_anteil = controllerWerte.pid_pitch.d * angular_rate_y;			// negativ
	pitch = 12.0f / BATTERY_VOLTAGE * ATTITUDE_K * (controllerWerte.pitch_p_anteil + controllerWerte.pitch_i_anteil + controllerWerte.pitch_d_anteil);
	#endif

	//Yaw Controller
	controllerWerte.fehlerintegral_yaw = integral_cutoff * controllerWerte.fehlerintegral_yaw + controllerWerte.error_yaw;
	yaw1 = controllerWerte.pid_yaw.p * controllerWerte.error_yaw;
	yaw2 = controllerWerte.pid_yaw.i * controllerWerte.fehlerintegral_yaw;
	yaw3 = controllerWerte.pid_yaw.d * angular_rate_z;			// negativ
	yaw = 12.0f / BATTERY_VOLTAGE * (yaw1 + yaw2 + yaw3);

	// Fix Added Gas scaled to voltage
	add_gas = 12.0f / BATTERY_VOLTAGE * MINIMAL_GAS;

	// Limit Controller Output		nach unten				nach oben
	max_controller_out = minimum(add_gas + gas, MAX_GAS - add_gas - gas - NULL_GAS);

	// Divide Gas first to Roll, rest to Yaw
	if (betrag(roll)+betrag(yaw) >= max_controller_out){
		roll = sign(roll) * minimum(ATTITUDE_PART*max_controller_out,betrag(roll));
		rest_gas = max_controller_out - betrag(roll);						// Here Gas free for Yaw Controll
		yaw = sign(yaw) * minimum(rest_gas,betrag(yaw));
	}

	// Divide Gas first to Pitch, rest to Yaw
	if (betrag(pitch)+betrag(yaw) >= max_controller_out){
			pitch = sign(pitch) * minimum(ATTITUDE_PART*(double)max_controller_out,betrag(pitch));
			rest_gas = max_controller_out - betrag(pitch);
			yaw = sign(yaw) * minimum(rest_gas,betrag(yaw));
	}

	#ifndef FLUG
		gas = gas - (double)TEST_GAS_ABZUG;
	#endif


	M1 = NULL_GAS + add_gas + gas - roll + yaw;
	M2 = NULL_GAS + add_gas + gas + pitch - yaw ;
	M3 = NULL_GAS + add_gas + gas + roll + yaw;
	M4 = NULL_GAS + add_gas + gas - pitch - yaw;


	// So the Motors dont turn off:
	if (M1 < NULL_GAS)
		M1 = NULL_GAS;

	if (M2 < NULL_GAS)
		M2 = NULL_GAS;

	if (M3 < NULL_GAS)
		M3 = NULL_GAS;

	if (M4 < NULL_GAS)
		M4 = NULL_GAS;


	if (M1 > MAX_GAS)
		M1 = MAX_GAS;

	if (M2 > MAX_GAS)
		M2 = MAX_GAS;

	if (M3 > MAX_GAS)
		M3 = MAX_GAS;

	if (M4 > MAX_GAS)
		M4 = MAX_GAS;


	controllerWerte.stellwertM1 = (unsigned char)M1;
	controllerWerte.stellwertM2 = (unsigned char)M2;
	controllerWerte.stellwertM3 = (unsigned char)M3;
	controllerWerte.stellwertM4 = (unsigned char)M4;


	#ifdef DEBUG_MSG
	sprintf(debug_line,"W:%d,%d,%d,%d:",controllerWerte.stellwertM1,controllerWerte.stellwertM2,
			controllerWerte.stellwertM3,controllerWerte.stellwertM4);
	USART_schreiben(debug_line);
	#endif
}


