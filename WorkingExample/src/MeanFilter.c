/*
 * MeanFilter.c
 *
 *  Created on: 02.02.2012
 *      Author: gageik
 */


/// ********			MeanFilter MF			****************

#include "MeanFilter.h"


double MF_Acc_X_Mean_Values[ACC_MEAN_FILTER_SIZE];
double MF_Acc_X_Mean_Delayed_Values[ACC_MEAN_FILTER_SIZE];
double MF_Acc_Y_Mean_Values[ACC_MEAN_FILTER_SIZE];
double MF_Acc_Y_Mean_Delayed_Values[ACC_MEAN_FILTER_SIZE];
double MF_Acc_Z_Mean_Values[ACC_MEAN_FILTER_SIZE];
double MF_Acc_Z_Mean_Delayed_Values[ACC_MEAN_FILTER_SIZE];
double MF_MAG_YAW_Mean_Values[MAG_YAW_MEAN_FILTER_SIZE];
double MF_GYRO_DRIFT_Mean_Values[GYRO_DRIFT_MEAN_FILTER_SIZE];

MeanFilter MF_Acc_X_Mean;
MeanFilter MF_Acc_X_Mean_Delayed;
MeanFilter MF_Acc_Y_Mean;
MeanFilter MF_Acc_Y_Mean_Delayed;
MeanFilter MF_Acc_Z_Mean;
MeanFilter MF_Acc_Z_Mean_Delayed;
MeanFilter MF_Mag_Yaw;
MeanFilter MF_Gyro_Drift;

AusreisserFilter Gyro_X_Ausreisser_Filter;
AusreisserFilter Gyro_Y_Ausreisser_Filter;
AusreisserFilter Gyro_Z_Ausreisser_Filter;


void Init_MeanFilters(){

	// Acc
	Init_Mean_Filter(&MF_Acc_X_Mean, MF_Acc_X_Mean_Values, ACC_MEAN_FILTER_SIZE);
	Init_Mean_Filter(&MF_Acc_X_Mean_Delayed, MF_Acc_X_Mean_Delayed_Values, ACC_MEAN_FILTER_SIZE);
	Init_Mean_Filter(&MF_Acc_Y_Mean, MF_Acc_Y_Mean_Values, ACC_MEAN_FILTER_SIZE);
	Init_Mean_Filter(&MF_Acc_Y_Mean_Delayed, MF_Acc_Y_Mean_Delayed_Values, ACC_MEAN_FILTER_SIZE);
	Init_Mean_Filter(&MF_Acc_Z_Mean, MF_Acc_Z_Mean_Values, ACC_MEAN_FILTER_SIZE);
	Init_Mean_Filter(&MF_Acc_Z_Mean_Delayed, MF_Acc_Z_Mean_Delayed_Values, ACC_MEAN_FILTER_SIZE);

	// Mag
	Init_Mean_Filter(&MF_Mag_Yaw, MF_MAG_YAW_Mean_Values, MAG_YAW_MEAN_FILTER_SIZE);
	Init_Mean_Filter(&MF_Gyro_Drift, MF_GYRO_DRIFT_Mean_Values,GYRO_DRIFT_MEAN_FILTER_SIZE);
}

void Init_Mean_Filter(MeanFilter* my_struct, double my_array[], int size){
	int i;

	// Setup Array:
	my_struct->values = my_array;

	// Setup Size
	my_struct->size = size;

	// Set all Entries to zero: Might be a problem otherwise
	for (i = 0; i < size; i++)
		my_array[i] = 0.0;
}


void Init_Ausreisser_Filter(){
	Gyro_X_Ausreisser_Filter.d_result = 0;
	Gyro_X_Ausreisser_Filter.s_last_values_a = 0;
	Gyro_X_Ausreisser_Filter.s_last_values_b = 0;
	Gyro_X_Ausreisser_Filter.d_last_values_a = 0;
	Gyro_X_Ausreisser_Filter.d_last_values_b = 0;

	Gyro_Y_Ausreisser_Filter.d_result = 0;
	Gyro_Y_Ausreisser_Filter.s_last_values_a = 0;
	Gyro_Y_Ausreisser_Filter.s_last_values_b = 0;
	Gyro_Y_Ausreisser_Filter.d_last_values_a = 0;
	Gyro_Y_Ausreisser_Filter.d_last_values_b = 0;

	Gyro_Z_Ausreisser_Filter.d_result = 0;
	Gyro_Z_Ausreisser_Filter.s_last_values_a = 0;
	Gyro_Z_Ausreisser_Filter.s_last_values_b = 0;
	Gyro_Z_Ausreisser_Filter.d_last_values_a = 0;
	Gyro_Z_Ausreisser_Filter.d_last_values_b = 0;
}

double Calc_New_Mean (MeanFilter *Filter, double neuerWert){
	// Returns old value -> for cascading mean filters

	double old = Filter->values[Filter->index];

	Filter->value = Filter->value + ( neuerWert - Filter->values[Filter->index] ) / Filter->size;
	Filter->values[Filter->index] = neuerWert;

	// Next Index
	Filter->index = Filter->index + 1;

	if (Filter->index >= Filter->size)
		Filter->index = 0;

	return old;
}

#define GYRO_AUSREISSER_SCHRANKE	1000			// DURCH ca. 30 (Skalierungsfaktor entspricht °/s)

// Entfernt Ausreisser aus 2-2 Messungen (Speichert die beiden Alten und Benutzt die beiden Neuen)
short Calc_New_AusreisserFilter (AusreisserFilter* Ausreisser_Filter, short new_a, short new_b){

		char miss_neu_a = 0, miss_neu_b = 0;

		short output = 0;
		double d = 0;

		double d_new_a = (double) new_a;
		double d_new_b = (double) new_b;


		d = fabs(d_new_a - d_new_b);
		if (d > GYRO_AUSREISSER_SCHRANKE){				// Neuste Werte eines Samples passen nicht -> Fehler

			d = fabs(d_new_a - Ausreisser_Filter->d_last_values_a);
			if (d > GYRO_AUSREISSER_SCHRANKE){
				miss_neu_a++;
			}

			d = fabs(d_new_a - Ausreisser_Filter->d_last_values_b);
			if (d > GYRO_AUSREISSER_SCHRANKE){
				miss_neu_a++;
			}

			d = fabs(new_b - Ausreisser_Filter->d_last_values_a);
			if (d > GYRO_AUSREISSER_SCHRANKE){
				miss_neu_b++;
			}

			d = fabs(new_b - Ausreisser_Filter->d_last_values_b);
			if (d > GYRO_AUSREISSER_SCHRANKE){
				miss_neu_b++;
			}

			// Einer von beiden Werten ok?
			if (miss_neu_a < miss_neu_b && miss_neu_a <= 1){
				output = new_a;
			}
			else if (miss_neu_b < miss_neu_a && miss_neu_b <= 1){
				output = new_b;
			}
			else {

				// Vermutlich beide neuen Werte korrupt: Wähle Werte, der zu allen Werten die geringste Differenz besitzt
				double e1 = euclidean_norm(d_new_a, d_new_b);
				double e2 = euclidean_norm(d_new_a, Ausreisser_Filter->d_last_values_a);
				double e3 = euclidean_norm(d_new_a, Ausreisser_Filter->d_last_values_b);
				double e4 = euclidean_norm(d_new_a, Ausreisser_Filter->d_result);
				double e5 = euclidean_norm(d_new_b, Ausreisser_Filter->d_last_values_a);
				double e6 = euclidean_norm(d_new_b, Ausreisser_Filter->d_last_values_b);
				double e7 = euclidean_norm(d_new_b, Ausreisser_Filter->d_result);
				double e8 = euclidean_norm(Ausreisser_Filter->d_last_values_a, Ausreisser_Filter->d_last_values_b);
				double e9 = euclidean_norm(Ausreisser_Filter->d_last_values_a, Ausreisser_Filter->d_result);
				double e0 = euclidean_norm(Ausreisser_Filter->d_last_values_b, Ausreisser_Filter->d_result);

				double error_neu_a = e1 + e2 + e3 + e4;
				double error_neu_b = e1 + e5 + e6 + e7;
				double error_alt_a = e2 + e5 + e8 + e9;
				double error_alt_b = e3 + e6 + e8 + e0;

				// Wähle besten Wert aus
				if ((error_neu_a < error_neu_b) && (error_neu_a < error_alt_a) && (error_neu_a < error_alt_b)){
					output = new_a;
				}
				else if ((error_neu_b < error_alt_a) && (error_neu_b < error_alt_b)){
					output = new_b;
				}
				else if (error_alt_b < error_alt_a){
					output = Ausreisser_Filter->s_last_values_b;
				}
				else {
					output = Ausreisser_Filter->s_last_values_a;
				}
			}

		}

		else{			// Kein Fehler: Nimm Mittelwert aus neuster Messung
				double zwischen = ((double)new_a + (double) new_b) / (double)2.0;
				output = (short) zwischen;
		}

		Ausreisser_Filter->s_last_values_a = new_a;
		Ausreisser_Filter->s_last_values_b = new_b;
		Ausreisser_Filter->d_last_values_a = d_new_a;
		Ausreisser_Filter->d_last_values_b = d_new_b;
		Ausreisser_Filter->d_result = (double)output;
		return output;
}


/**************************************************************
WinFilter version 0.8
http://www.winfilter.20m.com
akundert@hotmail.com

Filter type: Low Pass
Filter model: Bessel
Filter order: 10
Sampling Frequency: 200 Hz
Cut Frequency: 50.000000 Hz
Coefficents Quantization: float
**************************************************************/

float ACoef[NCoef+1] = {
        0.00000136066890274772,
        0.00001360668902747718,
        0.00006123010062364732,
        0.00016328026832972619,
        0.00028574046957702086,
        0.00034288856349242504,
        0.00028574046957702086,
        0.00016328026832972619,
        0.00006123010062364732,
        0.00001360668902747718,
        0.00000136066890274772
    };

    float BCoef[NCoef+1] = {
        1.00000000000000000000,
        -5.67888828856097480000,
        14.95350906682947000000,
        -23.95894210563200500000,
        25.79459735858526100000,
        -19.45323330227612100000,
        10.38729863176669500000,
        -3.87117355817402630000,
        0.96230415075654718000,
        -0.14389737879329331000,
        0.00981839230923440410
    };
    /*

float ACoef2[NCoef+1] = {
        0.00035875827717070694,
        0.00358758277170706950,
        0.01614412247268181100,
        0.04305099326048483100,
        0.07533923820584845200,
        0.09040708584701814800,
        0.07533923820584845200,
        0.04305099326048483100,
        0.01614412247268181100,
        0.00358758277170706950,
        0.00035875827717070694
    };

    float BCoef2[NCoef+1] = {
        1.00000000000000000000,
        -1.77859231024663970000,
        2.25022237712217080000,
        -1.89657326451937650000,
        1.20365855835611280000,
        -0.56969847463017997000,
        0.20229442448798388000,
        -0.05232345319200532700,
        0.00936831559819767750,
        -0.00104018814467048420,
        0.00005410916143060020
    };


float ACoef[NCoef+1] = {
        0.00203244654808066050,
        0.02032446548080660600,
        0.09146009466362972600,
        0.24389358576967929000,
        0.42681377509693874000,
        0.51217653011632647000,
        0.42681377509693874000,
        0.24389358576967929000,
        0.09146009466362972600,
        0.02032446548080660600,
        0.00203244654808066050
    };

    float BCoef[NCoef+1] = {
        1.00000000000000000000,
        0.07170986180841441800,
        0.84156328558345250000,
        -0.04440260257806187500,
        0.21813753934859514000,
        -0.02484818391382418900,
        0.02088311394035699900,
        -0.00241468932312067630,
        0.00063189550185437256,
        -0.00004438599340740831,
        0.00000266712872394848
    };

    */

float iir1(float NewSample) {


    static float y[NCoef+1]; //output samples
    static float x[NCoef+1]; //input samples
    int n;

    //shift the old samples
    for(n=NCoef; n>0; n--) {
       x[n] = x[n-1];
       y[n] = y[n-1];
    }

    //Calculate the new output
    x[0] = NewSample;
    y[0] = ACoef[0] * x[0];
    for(n=1; n<=NCoef; n++)
        y[0] += ACoef[n] * x[n] - BCoef[n] * y[n];

    return y[0];
}

float iir2(float NewSample) {


    static float y[NCoef+1]; //output samples
    static float x[NCoef+1]; //input samples
    int n;

    //shift the old samples
    for(n=NCoef; n>0; n--) {
       x[n] = x[n-1];
       y[n] = y[n-1];
    }

    //Calculate the new output
    x[0] = NewSample;
    y[0] = ACoef[0] * x[0];
    for(n=1; n<=NCoef; n++)
        y[0] += ACoef[n] * x[n] - BCoef[n] * y[n];

    return y[0];
}

