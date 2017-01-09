// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011-2014 Nils Gageik

/*
 * myMath.c
 *
 *  Created on: 05.08.2011
 *      Author: gageik
 */

#include "QUAD_TWI_6DOF.h"


double euclidean_norm (double a, double b){

	double c;
	c = (a-b)*(a-b);
	return c;
}

double diff_norm (double a, double b){

	double c = a - b;
	c = betrag(c);
	return c;
}

short round_s (double zahl) {
// Rounds a double and converts to short

	short out;
	short sig = 1;

	if (zahl<0)
		sig = -1;

	zahl = zahl + 0.5f * sig;

	out = (short) zahl;

	return out;
}

// TODO: Überholen!!!
double betrag(double z){

	if (z < 0.0f)
		return -z;
	else
		return z;
}

int betrag_i(int a){

	if (a < 0)
		return -a;
	else
		return a;
}

double limit_max(double x, double max){
// Gibt Wert bis maximaler Grenze wieder

	int a = (int)x;
	int b = (int)max;

		if (a>b){
			return max;
		}
			return x;
}

double limit_min(double x, double min){
// Gibt Wert bis minimaler Grenze wieder

	int a = (int)x;
	int b = (int)min;

	if (a<b){
			return min;
	}
			return x;
}

double saturate(double x, double grenze){
	// magnitude of grenze limits x to both sides (+ and -)


	if (x > grenze){
		x = grenze;
	}

	if (betrag(x) > grenze){
		x = ((-1.0)*grenze);
	}

	return x;
}

double my_saturate(double x, double grenze){
	// magnitude of grenze limits x to both sides (+ and -)


	int a = (int)(x);
	int b = (int)grenze;
	int c = (int)(-grenze);


	if (a > b){
		x = grenze;
	}

	if (a < c){
		x = ((-1.0)*grenze);
	}

	return x;
}

double maximum(double a, double b){
// Gibt das Maximum aus zwei Werten wieder

	if (a>b)
		return a;
	else
		return b;
}

double minimum(double a, double b){
// Gibt das Minimum aus zwei Werten wieder

	if (a<b)
		return a;
	else
		return b;
}

unsigned int minimum_ui(unsigned int a, unsigned int b){
// Gibt das Minimum aus zwei Werten wieder

	if (a<b)
		return a;
	else
		return b;
}

double sign(double a){

	if (a>0)
		return 1.0f;
	else
		return -1.0f;
}

void Matrixmultiplikation (double * M1, double * M2, double *c){
	 // 3x3

	 int i,j,k;

	 for(i=0;i< 3;i++)
	  {
	   for(j=0;j< 3;j++)
	   {
	    c[3*i+j]=0;
	    for(k=0;k< 3;k++)
	    {
	     c[3*i+j] += M1[i+3*k] * M2[k+3*j];
	    }
	  }
	 }
}


void Transponiert2D(double Matrix[2][2], double R[2][2]){
	// Matrix Pointer to 2x2 double Array

	R[0][0] = Matrix[0][0];
	R[0][1] = Matrix[1][0];
	R[1][0] = Matrix[0][1];
	R[1][1] = Matrix[1][1];
}

void Matrixmultiplikation2D (double M1[2][2], double M2[2][2], double R[2][2]){
	// M1, M2, R Pointer to 2x2 double Array

	R[0][0] = M1[0][0]*M2[0][0]+M1[0][1]*M2[1][0];
	R[0][1] = M1[0][0]*M2[0][1]+M1[0][1]*M2[1][1];
	R[1][0] = M1[1][0]*M2[0][0]+M1[1][1]*M2[1][0];
	R[1][1] = M1[1][0]*M2[0][1]+M1[1][1]*M2[1][1];
}


void Matrixaddition2D(double M1[2][2], double M2[2][2], double R[2][2]){
	R[0][0] = M1[0][0]+M2[0][0];
	R[0][1] = M1[0][1]+M2[0][1];
	R[1][0] = M1[1][0]+M2[1][0];
	R[1][1] = M1[1][1]+M2[1][1];
}

void Vektoraddition2D(double* V1, double* V2, double* V){
	V[0] = V1[0]+V2[0];
	V[1] = V1[1]+V2[1];
}

void Vektoraddition3D(double* V1, double* V2, double* V){
	V[0] = V1[0]+V2[0];
	V[1] = V1[1]+V2[1];
	V[2] = V1[2]+V2[2];
}

void Skalar_mal_Matrix (double s, double R[2][2]){
	R[0][0] = R[0][0]*s;
	R[0][1] = R[0][1]*s;
	R[1][0] = R[1][0]*s;
	R[1][1] = R[1][1]*s;
}

void Skalar_mal_Vektor(double s, double* R){
	R[0] = R[0]*s;
	R[1] = R[1]*s;
}

void Skalar_mal_Vektor3D(double s, double* R){
	R[0] = R[0]*s;
	R[1] = R[1]*s;
	R[2] = R[2]*s;
}

void Exponential_Vektor_Fusion(double a, double b, double* R, double* S){
	// R = R * a + b * S;
	R[0] = R[0] * a + S[0] * b;
	R[1] = R[1] * a + S[1] * b;
	R[2] = R[2] * a + S[2] * b;
}

void Matrix_mal_Vektor_2D (double M[2][2], double* V, double * Vr){
	// M 2x2 double Array, V 1x2 Vector Array
	// V2 1x2 Vector contains results at the end
	Vr[0] = M[0][0]*V[0]+M[0][1]*V[1];
	Vr[1] = M[1][0]*V[0]+M[1][1]*V[1];
}

double Determinante_2D (double M[2][2]){
	// M 2x2 double Array
	double d;
	d = M[0][0]*M[1][1]-M[1][0]*M[0][1];

	return d;
}

void Matrix_Inverse (double M[2][2], double R[2][2]){
	// M,R 2x2 double Array
	// Computes Inverse of M and saves it to R
	double s = Determinante_2D(M);

	if (betrag(s) < 0.001)
		s = sign(s) * 0.001;

	R[0][0] = M[1][1]/s;
	R[0][1] = -M[0][1]/s;
	R[1][0] = -M[1][0]/s;
	R[1][1] = M[0][0]/s;
}

void fill_vector(double a, double b, double c, double* V){

	V[0] = a;
	V[1] = b;
	V[2] = c;
}

void cross_product (double* A, double* B, double* O) {
	O[0] = A[1]*B[2]-A[2]*B[1];
	O[1] = A[2]*B[0]-A[0]*B[2];
	O[2] = A[0]*B[1]-A[1]*B[0];
}

void normalize_vector (double* V){

	double b = sqrt(scalar_product(V,V));
	V[0] = V[0] / b;
	V[1] = V[1] / b;
	V[2] = V[2] / b;
}

double scalar_product(double* A, double* B){
	double o = A[0]*B[0]+A[1]*B[1]+A[2]*B[2];

	return o;
}

Quaternion Quat_from_Vectors(double* V_ref, double* V_mess){

	Quaternion q;

	double V[3];
	double s;

	normalize_vector(V_ref);
	normalize_vector(V_mess);

	cross_product(V_ref, V_mess, V);
	normalize_vector(V);
	s = acos(scalar_product(V_ref, V_mess));

	q.q0 = cos (s/2);
	q.q1 = V[0] * sin (s/2);
	q.q2 = V[1] * sin (s/2);
	q.q3 = V[2] * sin (s/2);

	q = Q_normalise(q);

	return q;
}

Quaternion Q_konjugiert(Quaternion r){

	Quaternion out;

	out.q0 = r.q0;
	out.q1 = -r.q1;
	out.q2 = -r.q2;
	out.q3 = -r.q3;

	return out;
}

Quaternion Q_normalise (Quaternion r){
	Quaternion out;

	double norm = sqrt(r.q0*r.q0 + r.q1*r.q1 + r.q2*r.q2 + r.q3*r.q3);
	out.q0 = r.q0 / norm;
	out.q1 = r.q1 / norm;
	out.q2 = r.q2 / norm;
	out.q3 = r.q3 / norm;

	return out;
}

Quaternion Q_normalise_fix_q0_zero_q3 (Quaternion r){
	// q0 is fixed, q3 is zero, normalise q1 & q2 relatively (CORRECTLY!)

	Quaternion out;
	out.q0 = r.q0;
	out.q3 = 0;					// No Compensation via Yaw

	double norm_faktor_n = sqrt(1.0 - r.q0*r.q0);
	double norm_betrag = sqrt(r.q2*r.q2 + r.q1*r.q1);

	out.q1 = (r.q1 / norm_betrag)*norm_faktor_n;
	out.q2 = (r.q2 / norm_betrag)*norm_faktor_n;

	return out;
}

Quaternion Q_normalise_fix_q0(Quaternion r){
	// q0 is fixed, normalise q1 & q2 & q3 relatively (CORRECTLY!)

	Quaternion out;

	out.q0 = r.q0;		// Keep q0 fix
	double norm_faktor_n = sqrt(1.0 - r.q0 * r.q0);
	double norm_betrag = sqrt(r.q1*r.q1 + r.q2*r.q2 + r.q3*r.q3);

	out.q1 = (r.q1 / norm_betrag)*norm_faktor_n;
	out.q2 = (r.q2 / norm_betrag)*norm_faktor_n;
	out.q3 = (r.q3 / norm_betrag)*norm_faktor_n;

	return out;
}


Quaternion Q_normalise_fix_q0_zero12(Quaternion r){
	// q0 is fixed, normalise q1 & q2 & q3 relatively (CORRECTLY!)

	Quaternion out;

	out.q0 = r.q0;		// Keep q0 fix
	double norm_faktor_n = sqrt(1.0 - r.q0 * r.q0);

	out.q1 = 0.0;
	out.q2 = 0.0;
	out.q3 = sign (r.q3) * norm_faktor_n;

	return out;
}


Quaternion Quatprodukt(Quaternion r, Quaternion dQuat){
		// Quat = r x dQuat
		Quaternion Quat;

		Quat.q0 = dQuat.q0*r.q0 - dQuat.q1*r.q1 - dQuat.q2*r.q2 - dQuat.q3 * r.q3;
		Quat.q1 = dQuat.q0*r.q1 + dQuat.q1*r.q0 - dQuat.q2*r.q3 + dQuat.q3 * r.q2;
		Quat.q2 = dQuat.q0*r.q2 + dQuat.q1*r.q3 + dQuat.q2*r.q0 - dQuat.q3 * r.q1;
		Quat.q3 = dQuat.q0*r.q3 - dQuat.q1*r.q2 + dQuat.q2*r.q1 + dQuat.q3 * r.q0;

		return Quat;
}

void Q_rotate_Vec(double* vec, Quaternion q, double* vec_out){
		// Quaternion q rotates Vector vec (3x1 Pointer)

		Quaternion v;
		v.q0 = 0.0f;
		v.q1 = vec[0];
		v.q2 = vec[1];
		v.q3 = vec[2];

		v = Quatprodukt(Q_konjugiert(q),v);
		v = Q_normalise (v);
		v = Quatprodukt(v,q);
		v = Q_normalise (v);

		vec_out[0] = v.q1;
		vec_out[1] = v.q2;
		vec_out[2] = v.q3;
}

double number_potent (double value, unsigned int exponent){

	double r = 1.0f;

	if (exponent == 0)
		return 1;

	for (;exponent > 0;exponent--)
		r = r * value;

	return r;
}

short myMedian(short a, short b, short alt){

	if (a==b)
		return a;

	if (abs(a-alt) < abs(b-alt))
		return a;


	return b;
}


double Complementary_Filter(double old_val, double new_val, double alpha_old){

	double cf = alpha_old * old_val + (1.0 - alpha_old) * new_val;

	return cf;
}

// Cosinus eines Winkel, mit Näherung für kleine Winkel

double cos_degree_fast(double winkel){

    // Bringe auf +/- 180 Grad
    if (winkel > 180)
        winkel = winkel - 360;

    if (winkel < -180)
        winkel = winkel + 360;

    double b = betrag(winkel);

    // Kleine Winkel gleich 1
    if (b <= 5.0f)
        return 1;

    if (b >= 175.0f)
        return -1;

    // Linearer Bereich: Kleine Winkel (von sin a) gleich a
    if ((b >= 85) && (b <= 95))
        return (sign(winkel) * (b - 90) * MAL_PI_DURCH_180);

    return cos(winkel * MAL_PI_DURCH_180);
}

// Sinus eines Winkel, mit Näherung für kleine Winkel
double sin_degree_fast(double winkel){
	return cos_degree_fast(winkel - 90);
}


position2D transform_position_2D(position2D p, double winkel_degree){
	position2D p_neu;

	double winkel_rad = winkel_degree * MAL_PI_DURCH_180;

	p_neu.x = p.x * cos(winkel_rad) + p.y * sin(winkel_rad);
	p_neu.y = p.y * cos(winkel_rad) - p.x * sin(winkel_rad);
	return p_neu;
}

