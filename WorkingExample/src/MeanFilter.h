/*
 * MeanFilter.h
 *
 *  Created on: 02.02.2012
 *      Author: gageik
 */

#ifndef MEANFILTER_H_
#define MEANFILTER_H_

#include "QUAD_TWI_6DOF.h"

#define INFRARED_MEANFILTER_SIZE			7			// 10 ursprünglich
#define ULTRASONIC_MEANFILTER_SIZE			5
#define HEIGHT_MEANFILTER_SIZE				5			// For Height Ctrl
#define AP_HEIGHT_MEANFILTER_SIZE			30			// For Height Ctrl
#define MAG_YAW_MEAN_FILTER_SIZE			100
#define GYRO_DRIFT_MEAN_FILTER_SIZE			1000

#define ACC_MEAN_FILTER_SIZE			20


#define NCoef 10


typedef struct {
	double* values;
	double size;

	double value;
	short index;
} MeanFilter;

typedef struct {
	double d_result;
	short s_last_values_a;
	short s_last_values_b;
	double d_last_values_a;
	double d_last_values_b;
} AusreisserFilter;


void Init_MeanFilters();
void Init_Mean_Filter(MeanFilter* my_struct, double my_array[], int size);
void Init_Ausreisser_Filter();
double Calc_New_Mean (MeanFilter *Filter, double neuerWert);
short Calc_New_AusreisserFilter (AusreisserFilter* Gyro_Aussreisser_Filter, short new_a, short new_b);
float iir1(float NewSample) ;
float iir2(float NewSample) ;
void Init_Field(double my_array[], int size);






#endif /* MEANFILTER_H_ */
