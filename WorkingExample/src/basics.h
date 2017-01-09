/*
 * basics.h
 *
 *  Created on: 12.04.2011
 *      Author: gageik
 */

#ifndef BASICS_H_
#include <stdint.h>

//#define FLUG									// USE 3DOF ?!:	Switch between 3DOF == FLUG and 2DOF

// Kompensations Modi zum Experimentieren
//#define		COMP_MODE_VERY_FAST
#define		COMP_MODE_FAST
//#define		COMP_MODE_NORMAL

// Timing and Configuration Parameters
#define RENEW_DISPLAY_TIME			201
#define SAMPLE_TIME					10			// US: 5ms

#define BIAS_SAMPLES				2000		// 500 are good

#define	BATTERY_VOLTAGE				12.0f
#define SCHUB_GAS					133.0f		// Gut für 12V
#define NULL_GAS					3.0f		// Minimalgas jedes Motors (damit nicht ausgeht)
#define MAX_GAS						255.0f		// maximales Gas pro Motor

//#define	MINIMAL_GAS					40.0f		// 40: Startgas bei Einschalten (zzgl. Knüppel & Reglergas)
#define	MINIMAL_GAS					90.0f		// 40: Startgas bei Einschalten (zzgl. Knüppel & Reglergas)

// Programming Parameters
#define DISPLAY_MODI 				8			// Number of Display States
#define DIP_START_MODUS				2


#define DMA_PDCA								// Use PDCA DMA to Send/Receive Data
#define DEBUG_BUFFER_SIZE			200

// Debugging Modi
#define DEBUG_MSG_TWI 							// 	USART Debug Messages for TWI: Outcomment to set off
//#define DEBUG_MSG								// 	Activates Global Debugging Messages: Letters
//#define DEBUG_MSG_IMU							//  USART Debug Messages for IMU
#define DEBUG_INFORMATIONS

#define IMU_AUTO_DETECTION
//#define LSM_MAG_SENSOR
#define MINI_IMU_V2		// Ob Version 2 oder Version 1 benutzt wird für L3G & LSM

//#define KALMAN_FILTER							// Use Kalman Filter
#define NEW_ORIENTATION							// Use New -> Quaternion Based Rotation (Relativ), Old -> KF Based (Absolute)

//#define ALEX_PD
//#define NONFLYING							// Motoren werden zwar angesteurt, jedoch immer auf 0 gesetzt

#define CALIBRATION_DEBUG
#define ACC_DYNAMIC_BIAS			// MAG_MINMAX_CALIBRATION = In alle Richtungen drehen und das Minimum + Maximum nehmen -> nicht gut

#define ENGINE_WITH_SWITCH_DEBUG			// 	Set Motor On by PB0, remove when Fly
#define DISPLAY_ON							// 	Display OFF: Outcomment this line

//#define TEST_GAS_ABZUG		40
#define TEST_GAS_ABZUG			0


#define GYRO_AUSREISSER_FILTER
#define GYRO_AUSREISSER_SCHRANKE	1000			// durch ca. 30 (entspricht Skalierungsfaktor) teilen -> [°/s]

#define DIFFERENTIAL_FILTER_SIZE		10

#define CPU_SPEED						60000000			//Macht Probleme mit I2C
//#define CPU_SPEED						12000000


#define GRAVITATIONS_KONSTANTE		9.81f


//define LED PINS
#define	INIT_READY_LED					AVR32_PIN_PB19
#define	SENSOR_CALIBRATE_LED			AVR32_PIN_PB20
#define	REMOTE_CALIBRATE_LED			AVR32_PIN_PB21
#define	RUNTIME_LED						AVR32_PIN_PB22

#define I2C_RESET_CLEAR_PIN				40

// Automation States
#define AUTOMATION_SWITCH_OFF				0
#define AUTOMATION_SWITCH_MID				1
#define AUTOMATION_SWITCH_ON				2

// Ring buffer für D-Anteil Buffersize (OF)
#define OF_D_BUFFERSIZE 5

// PID Param
typedef struct {
	double p;
	double i;
	double d;

	short state;
} pid_param;

typedef struct {
	double x;
	double y;
} position2D;

// Steuerdaten
typedef struct {
	char landeregler_on;			// Controlled Landing
	char startregler_on;			// Controlled Starting
	char hoehenregler_on;			// Activate Height Control
	char regler_on;					// Activates Controller
	char engine_on;					// Activates Motors
	char calibrate_on;				// Should the system be calibrated?
	char button_clicked;			// count the button clicks for Accelerometer calibration, Activates Position Hold
	char positionsregler_on;
	char fully_autonomous_flight_on;			// FlightCtrl takes over: Yes / No
	char fully_autonomous_flight_execute;		// Execute FlightCtrled Flight
	char landeregler_state;
	char fully_autonomous_hold_position;		// Quadcopter holds position in fully autonomous FlightCtrl
	char collision_avoidance_on;
	char autonomous_landing;
	char manual_calibration_state; 				// keeps the state for manual calibration
} steuerDaten;

typedef struct{
	int firstrun;
	double soll_pos;						// Positions Sollwert
	double of_pos;
	double of_delta_pos;
	double stellwert_winkel;				// Output -> Stellwinkel (Sollwinkel für Attitude Control)
	double pos_p_anteil;
	double pos_d_anteil;
	double pos_i_anteil;
	double error;
	double d_ring_buffer[OF_D_BUFFERSIZE];
	int d_ring_pos;
	double fehlerintegral;
	unsigned int boost_timeout;
	double boost_error_last_timeout;
	double boost_current_factor;
} positionController;

// ControllerDaten
typedef struct {
	unsigned char stellwertM1; // Wertebereich: 0 bis 255
	unsigned char stellwertM2;	//
	unsigned char stellwertM3;
	unsigned char stellwertM4;
	double fehlerintegral_roll;
	double fehlerintegral_yaw;
	double fehlerintegral_pitch;
	double fehlerintegral_height;
	//double fehlerintegral_pos_x;
	//double fehlerintegral_pos_y;

	double error_roll;
	double error_yaw;
	double error_pitch;
	double error_height;
	//double error_pos_x;
	//double error_pos_y;

	double soll_angle_roll;
	double soll_angle_pitch;
	double soll_angle_yaw;

	double soll_height;
	double max_soll_height;

	double roll_p_anteil;
	double roll_i_anteil;
	double roll_d_anteil;

	double pitch_p_anteil;
	double pitch_i_anteil;
	double pitch_d_anteil;

	double height_p_anteil;
	double height_i_anteil;
	double height_d_anteil;
	double height_starting_integration_factor;
	char neuer_height_stellwert;

	positionController forthCtrl;
	positionController strafeCtrl;

	double gas;
	double height_gas;
	double schwebe_gas;

	pid_param pid_roll;
	pid_param pid_pitch;
	pid_param pid_yaw;
	pid_param pid_height;
    pid_param pid_pos;
    pid_param pid_landing;

	int integral_index_roll;
	int integral_index_pitch;
	int integral_index_yaw;
	int integral_index_height;

	char end_landing_state;
} controllerDaten;

// Sensor Rohdaten
typedef struct {
	short gyro_x_s;
	short gyro_y_s;
	short gyro_z_s;
	short acc_x_s;
	short acc_y_s;
	short acc_z_s;
	short ap_twi_s;
	short mag_x_s;
	short mag_y_s;
	short mag_z_s;

	double gyro_x_f;
	double gyro_y_f;
	double gyro_z_f;
	double acc_x_f;
	double acc_y_f;
	double acc_z_f;
	double mag_x_f;
	double mag_y_f;
	double mag_z_f;
	double ap_twi_f;
} sensorDaten_raw;

// Airpressure Daten
typedef struct {
	int AC1;
	int AC2;
	int AC3;
	unsigned int AC4;
	unsigned int AC5;
	unsigned int AC6;
	int B1;
	int B2;
	int MB;
	int MC;
	int MD;
	unsigned int UT;
	unsigned long UP;
	long temp;
	long pressure;
	double start_height;
	char read_toggle;
} Airpressure_data;

// Sensordaten
typedef struct {
	sensorDaten_raw RohWerte;

	double gyro_x;
	double gyro_y;
	double gyro_z;

	double gyro_x_sampletimed;
	double gyro_y_sampletimed;
	double gyro_z_sampletimed;

	double acc_x;
	double acc_y;
	double acc_z;

	double mag_x;
	double mag_y;
	double mag_z;

	double gyro_x_last;
	double gyro_y_last;
	double gyro_z_last;

	double acc_x_last;
	double acc_y_last;
	double acc_z_last;

	double gyro_x_bias;
	double gyro_y_bias;
	double gyro_z_bias;
	double acc_x_bias;
	double acc_y_bias;
	double acc_z_bias;

	double mag_x_bias;
	double mag_y_bias;
	double mag_z_bias;

	double winkel_x_gyro;
	double winkel_y_gyro;
	double winkel_z_gyro;

	double gyro_rotated[3];		// x,y,z rotated

	double winkel_x_acc_iir_filtered;

	double winkel_x_kalman;
	double winkel_y_kalman;
	double winkel_z_kalman;

	double kalman_correct_index;

	// RPY Angles from Q
	double roll_angle;
	double pitch_angle;
	double yaw_angle;

	// RPY Angular Rates transformed with Q
	// -> Used as more accurate Gyro-Outputs
	double roll_delta_angle;
	double pitch_delta_angle;
	double yaw_delta_angle;

	// Old RPY Angles for Drift Compensation
	double roll_angle_old;
	double pitch_angle_old;
	double yaw_angle_old;

	// Angles from Accelerometer
	double roll_acc;
	double pitch_acc;
	double roll_acc_old;
	double pitch_acc_old;

	// Depth Pass Filter
	double acc_x_dpf;
	double acc_y_dpf;
	double acc_z_dpf;

	// Initial Vectors: Ursprüngliche Ausrichtung
	double Mag_Vec_Init[3];
	double Acc_Vec_Init[3];

	// Current Measured Vectors
	double Mag_Vec[3];
	double Acc_Vec[3];

	// Current expected/transformed Vectors
	double Mag_Vec_trans[3];
	double Acc_Vec_trans[3];

	// Sensivity of Acc: Fix Calibrated
	double acc_sensivity_x;
	double acc_sensivity_y;
	double acc_sensivity_z;

	// Sensivity of Mag: Fix Calibrated
	double mag_sensivity_x;
	double mag_sensivity_y;
	double mag_sensivity_z;
	double mag_magnitude;

	//********************************************************************
	//####################################################################
	//Matthias MAG Drift-Kompensation ADD_ON
	double winkel_z_mag;
	//double winkel_z_mag_old;
	double winkel_z_mag_offset;
	double mag_vector_length;
	double mag_vector_length_old;
	double mag_length_delta;
	double tmp_yaw_gyro;
	double yaw_filter_alpha;
	double yaw_filtered_tmp;
	int yaw_filter_modi;
	double yaw_filter_mag_trust_value;
	double MAG_THRESHOLD_VALUE;
	double yaw_unfiltered;
	double yaw_filtered;
	char filter_ready;
	char drift_ready;

	double dMag_abs;
	double gyro_abs;

	double height;		// Hoehe, current valid
	double height_ir;	// InfraRed
	double height_ir_short;	// InfraRed Short Distance (4-30cm)
	double height_ir_mid;	// InfraRed Short Distance (20-150cm)
	double height_ir_long;	// InfraRed Long Distance (100-550cm)
	double height_us;	// UltraSonic
	double height_ap;	// AirPressure

	Airpressure_data ap_data;

	int surface_quality;
	int of_surface_quality_scale;
	int8_t raw_dx;		// raw x for OF Sensor
	int8_t raw_dy;		// raw y for OF Sensor

	// Position (from OF)			[cm]
	position2D position;				// Allgemeine Position in Zentimetern, körperfest: DIESE WIRD BENUTZT
	position2D position_of;				// Position in Zentimeter, Körperfest: von OF ADNS
	position2D position_of_delta_od;	// Änderung der Position im letzten OD-Sample, Körperfest

	double temp_max_aussen;			// maximale Temperatur im Außenbereich
	double temp_max_winkel;			// Winkel der maximalen Temperatur in Grad von 0 bis 360

} sensorDaten;

// Quaternion
typedef struct {
	double q0;
	double q1;
	double q2;
	double q3;
} Quaternion ;

// Actuator
typedef struct {
	int pwm_duty_time_servo_roll;
	int pwm_duty_time_servo_pitch;			// Duty Time für Pitch Servo
	int pwm_duty_max;						// Maximale Duty Time
	int pwm_duty_time_roll_mid;				// Duty Time der Mittelstellung (Nullstellung) des Roll Servos (Verarbeitet / Umgerechnet in Cycles)
	int pwm_duty_time_pitch_mid;
	int pwm_duty_raw_med_roll;				// Duty Time der Mittelstellung (Nullstellung) des Roll Servos (Rohwert in Microsekunden)
	int pwm_duty_raw_med_pitch;
} actuatorDaten ;

typedef struct {
	char *data;
	unsigned int size;
} dataStream;


#ifdef FLUG
#define PROJEKT_NAME 		"  3 DOF INIT"
#else
#define PROJEKT_NAME 		"  2 DOF INIT"
#endif

#define BASICS_H_
#endif /* BASICS_H_ */
