/*
 * SignalProcessing.h
 *
 *  Created on: 20.01.2012
 *      Author: gageik
 */

#ifndef SIGNALPROCESSING_H_
#define SIGNALPROCESSING_H_

#include "basics.h"

/*
 * x -> Roll Axis (For Gyro and Acc)
 * y -> Pitch Axis (For Gyro and Acc)
 * Acc and Gyro use different reference system: x and y are switched so Acc_x is Angle around x-axis
 */


#ifdef	COMP_MODE_VERY_FAST
#define COMPENSATION_CUT_OFF 			0.002
#else
#ifdef		COMP_MODE_FAST
#define COMPENSATION_CUT_OFF 			0.0015
#else
#define COMPENSATION_CUT_OFF 			0.001
#endif
#endif


//#define QUAT_CORRECT_INTERVALL		10			// In Factors of SAMPLE_TIME
//#define QUAT_CORRECT_INTERVALL			10		// In Factors of SAMPLE_TIME
//#define QUAT_CORRECT_INTERVALL		1		// In Factors of SAMPLE_TIME

#define DRIFT_COMPENSATION_ALPHA	0.01f
//#define DRIFT_COMPENSATION_ALPHA	0

#define MANUAL_CALIBRATION_INACTIVATED							0
#define MANUAL_CALIBRATION_WAIT_FOR_TIME_OUT					1
#define MANUAL_CALIBRATION_MAG_RUNNING							2
#define MANUAL_CALIBRATION_ACC_RUNNING							3

#define	MANUAL_CALIBRATION_TIMEOUT								2000


SignalProcessed SignalProcessing(void);
SignalProcessed CQF();
void Sensoring(void);

void calibrate(void);
int Init_IMU_Sensors(void);

void setRPY(double roll, double pitch, double yaw);

Quaternion rotateQuat(Quaternion myQuat, double x, double y, double z);
Quaternion rotateQuat2(Quaternion myQuat, double x, double y, double z);
Quaternion calcQuatfromFW(double x, double y, double z);
void calcRPY(void);
void calcRPY2(Quaternion myQuat, double* RPY);
void calcRPY_SM(void);
Quaternion calcQuat(double x, double y, double z);
void correctQuat(double x, double y, double z);
void correctQuat2();
void Conditioning();

Quaternion compensate_Drift(Quaternion Quat_Acc, Quaternion Quat_Mag);
#endif /* SIGNALPROCESSING_H_ */
