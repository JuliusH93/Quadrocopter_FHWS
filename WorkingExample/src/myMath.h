/*
 * myMath.h
 *
 *  Created on: 05.08.2011
 *      Author: gageik
 */

#ifndef MYMATH_H_
#define MYMATH_H_

#include "basics.h"

#define DURCH_WURZEL_2					0.707f
#define PI    							3.14159265358979323846f
#define MAL_PI_DURCH_180				0.01745329
#define MAL_180_DURCH_PI				57.2957795

double euclidean_norm (double a, double b);
double diff_norm (double a, double b);
short round_s (double zahl);
double maximum(double a, double b);
double minimum(double a, double b);
unsigned int minimum_ui(unsigned int a, unsigned int b);
double limit_max(double x, double max);
double limit_min(double x, double min);
double sign(double a);
double betrag(double z);
double my_saturate(double x, double grenze);
double saturate(double x, double grenze);
int betrag_i(int a);
double scalar_product(double* A, double* B);
void fill_vector(double a, double b, double c, double* V);
void cross_product (double* A, double* B, double* O);
void normalize_vector (double* V);

void Matrixmultiplikation (double * M1, double * M2, double *c);
void Transponiert2D (double Matrix[2][2], double R[2][2]);
void Matrixmultiplikation2D (double M1[2][2], double M2[2][2], double R[2][2]);
void Matrixaddition2D (double M1[2][2], double M2[2][2], double R[2][2]);
void Skalar_mal_Matrix (double s, double R[2][2]);
void Skalar_mal_Vektor (double s, double* R);
void Skalar_mal_Vektor3D(double s, double* R);
void Exponential_Vektor_Fusion(double a, double b, double* R, double* S);
void Matrix_mal_Vektor_2D (double M[2][2], double* V, double * Vr);

double Determinante_2D (double M[2][2]);
void Matrix_Inverse (double M[2][2], double R[2][2]);
Quaternion Quatprodukt(Quaternion r, Quaternion dQuat);
double number_potent (double value, unsigned int exponent);
short myMedian(short a, short b, short alt);
void Vektoraddition2D(double* V1, double* V2, double* V);
void Vektoraddition3D(double* V1, double* V2, double* V);
void Q_rotate_Vec(double* vec, Quaternion q, double* vec_out);
Quaternion Q_normalise (Quaternion r);
Quaternion Q_konjugiert(Quaternion r);
double Complementary_Filter(double old_val, double new_val, double alpha_old);
double cos_degree_fast(double winkel);
double sin_degree_fast(double winkel);
position2D transform_position_2D(position2D p, double winkel_degree);
Quaternion Quat_from_Vectors(double* V_ref, double* V_mess);
Quaternion Q_normalise_fix_q0_zero_q3 (Quaternion r);
Quaternion Q_normalise_fix_q0(Quaternion r);
Quaternion Q_normalise_fix_q0_zero12(Quaternion r);
#endif /* MYMATH_H_ */
