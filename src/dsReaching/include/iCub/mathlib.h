// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Micha Hersch, Eric Sauser EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#ifndef __MATHLIB_H__
#define __MATHLIB_H__


#include <math.h>
#include <float.h>
#include <iostream>
#include <fstream>
using namespace std;

#ifndef CORRECT_MATRIX_INDEX
#define CORRECT_MATRIX_INDEX
#endif

typedef float vec2_t[2];
typedef float vec3_t[3];
typedef float vec4_t[4];
typedef float matrix_t[16];

typedef vec3_t   CVector3_t;
typedef vec4_t   CVector4_t; // added M.Hersch
typedef vec4_t   CQuat_t;
typedef matrix_t CMatrix3_t;
typedef matrix_t CMatrix4_t;

#define pi 3.14159265358979323846
#define deg2rad (pi / 180.0)
#define rad2deg (180.0 / pi)
#define epsilon 1e-6

#define RND(x)  ((x)*((double)rand())/((double)(RAND_MAX+1.0)))

#define coutvec(v)  cout <<(v)[0]<<" "<<(v)[1]<<" "<<(v)[2]<<endl
#define coutvec4(v) cout <<(v)[0]<<" "<<(v)[1]<<" "<<(v)[2]<<" "<<(v)[3]<<endl
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))


#define sign(x) ( (x)>=0? 1:-1 )
#define abs(x) ((x)>=0?(x):(-(x)))


#define v_set(x,y,z,v) { (v)[0] = x; (v)[1] = y; (v)[2] = z; }
#define v_clear(v) { (v)[0] = (v)[1] = (v)[2] = 0; }
#define v_copy(a,b) { (b)[0] = (a)[0]; (b)[1] = (a)[1]; (b)[2] = (a)[2]; }
#define v_scale(a,b,c) { \
	(c)[0] = (a)[0] * (b); \
	(c)[1] = (a)[1] * (b); \
	(c)[2] = (a)[2] * (b); \
}
#define v_add(a,b,c) { \
	(c)[0] = (a)[0] + (b)[0]; \
	(c)[1] = (a)[1] + (b)[1]; \
	(c)[2] = (a)[2] + (b)[2]; \
}
#define v_sub(a,b,c) { \
	(c)[0] = (a)[0] - (b)[0]; \
	(c)[1] = (a)[1] - (b)[1]; \
	(c)[2] = (a)[2] - (b)[2]; \
}
#define v_dot(a,b) ((a)[0] * (b)[0] + (a)[1] * (b)[1] + (a)[2] * (b)[2])
#define v_cross(a,b,c){ \
        (c)[0] = (a)[1] * (b)[2] - (a)[2] * (b)[1]; \
        (c)[1] = (a)[2] * (b)[0] - (a)[0] * (b)[2]; \
        (c)[2] = (a)[0] * (b)[1] - (a)[1] * (b)[0]; \
}
// added by M.Hersch

#define v4_dot(a,b) ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2] + (a)[3]*(b)[3])
#define v4_set(x,y,z,u,v) { (v)[0] = x; (v)[1] = y; (v)[2] = z; (v)[3] = u; }

#define v4_add(a,b,c) { \
	(c)[0] = (a)[0] + (b)[0]; \
	(c)[1] = (a)[1] + (b)[1]; \
	(c)[2] = (a)[2] + (b)[2]; \
        (c)[3] = (a)[3] + (b)[3]; \
}

#define m4_add(a,b,c) m_add((a),(b),(c))
#define m4_sub(a,b,c) m_sub((a),(b),(c))

#ifndef CORRECT_MATRIX_INDEX  
#define m3_v_multiply(m,v,o) v_transform_normal2(v,m,o)
#else
#define m3_v_multiply(m,v,o) v_transform_normal(v,m,o)
#endif

#define m4_copy(m,o) (m_copy((m),(o)))


#ifndef M_MACROS
#define v4_sub(a,b,c) { \
	(c)[0] = (a)[0] - (b)[0]; \
	(c)[1] = (a)[1] - (b)[1]; \
	(c)[2] = (a)[2] - (b)[2]; \
        (c)[3] = (a)[3] - (b)[3]; \
}
#define v4_scale(a,b,c) { \
	(c)[0] = (a)[0] * (b); \
	(c)[1] = (a)[1] * (b); \
	(c)[2] = (a)[2] * (b); \
	(c)[3] = (a)[3] * (b); \
}

#define v4_add(a,b,c) { \
	(c)[0] = (a)[0] + (b)[0]; \
	(c)[1] = (a)[1] + (b)[1]; \
	(c)[2] = (a)[2] + (b)[2]; \
	(c)[3] = (a)[3] + (b)[3]; \
}

#define v4_copy(a,b) { (b)[0] = (a)[0]; (b)[1] = (a)[1]; (b)[2] = (a)[2]; \
(b)[3] = (a)[3]; }

#define v4_clear(a){(a)[0]=0;(a)[1]=0;(a)[2]=0;(a)[3]=0;}
#else

void v_cross(const float *v1,const float *v2,float *out);
void v4_scale(CVector4_t a, float b, CVector4_t c);
void v4_add(CVector4_t a, CVector4_t b, CVector4_t c);
void v4_sub(CVector4_t a, CVector4_t b, CVector4_t c);
void v4_copy(CVector4_t a, CVector4_t b);
#endif

float v_length(const float *v);
float v_normalize(const float *v,float *out);
void v_translate(const float *m,float *out);
void v_transform(const float *v,const float *m,float *out);
void v_transform_normal(const float *v,const float *m,float *out);
void v_transform_normal2(const float *v,const float *m,float *out);

// v4 vector operations (added by Micha Hersch)
float v4_length(const CVector4_t v);
float v4_squ_length(const CVector4_t v);
float v_squ_length(const CVector3_t v);

void inverse_diag3(CVector3_t v1,CVector3_t v2);
void inverse_diag4(CVector4_t v1,CVector4_t v2);

/**
 * @param m = 3 by 4 matrix (3 rows, 4, columns)
 * @param out = m*m' 
 */
void m3_4_square(CMatrix4_t m, CMatrix3_t out);
/**
 * @param m = 3 by 4 matrix (3 rows, 4, columns)
 * @param v = 4-vector
 * @param out = m*diag(v)*m' 
 */
void m3_4_weighted_square(CMatrix4_t m,CVector4_t v, CMatrix3_t out);
void m3_4_t_weighted_square(CMatrix4_t m,CVector3_t v, CMatrix4_t out);
void m3_add_diag(CMatrix3_t m, CVector3_t v, CMatrix3_t out);
void m4_add_diag(CMatrix4_t m, CVector4_t v, CMatrix4_t out);
void m3_4_v_multiply(CMatrix4_t m, CVector4_t v, CVector3_t out);
void m4_4_v_multiply(CMatrix4_t m, CVector4_t v, CVector4_t out);
void m4_3_v_multiply(CMatrix4_t m, CVector3_t v, CVector4_t out);//out =
void m3_diag_v_multiply(CVector3_t m, CVector3_t v, CVector3_t out);
void m3_4_t_v_multiply(CMatrix4_t m, CVector3_t v, CVector4_t out);
void m3_4_t_m_multiply(CMatrix4_t m1, CMatrix3_t m2, CVector4_t out);// out=m1'm2
void m4_diag_v_multiply(CVector4_t m, CVector4_t v, CVector4_t out);
void m4_minus(CMatrix4_t m, CMatrix4_t out);
void m_set_v3_column(CVector3_t v,int i,CMatrix4_t out);
void m_get_v3_column(CMatrix4_t m,int i,CVector3_t out);
void m_rescale(float k,CMatrix4_t m, CMatrix4_t out);
void m_sub(CMatrix4_t m1,CMatrix4_t m2,CMatrix4_t out);
void m_add(CMatrix4_t m1,CMatrix4_t m2,CMatrix4_t out);
void v_mult(const CVector3_t v1,const CVector3_t v2, CMatrix4_t m);

  // matrix
void m_copy(const float *m,float *out);
void m_identity(float *m);
void m_clear(CMatrix3_t m);//sets elmt 15 to 1, the rest to 0
void m_multiply(const float *m1,const float *m2,float *out); // out = m2*m1;ou m1*m2 if correct index
int m_inverse(const float *m,float *out);
void m_transpose(const float *m,float *out);
void m_transpose_rotation(const float *m,float *out);
//degree
void m_rotation_x(float angle,float *out);
void m_rotation_y(float angle,float *out);
void m_rotation_z(float angle,float *out);
void m_rotation_xyz(float phi, float theta, float psi, float *out);
void m_rotation_serial(float angle_1, float angle_2, float angle_3, float *out);
void m_rotation_v(float angle,float *v,float *out);
//radian
void m_rotation_x2(float angle,float *out);
void m_rotation_y2(float angle,float *out);
void m_rotation_z2(float angle,float *out);
void m_rotation_v2(float angle,float *v,float *out);
void m_rotation_xyz2(float phi, float theta, float psi, float *out);
void m_translate(const float *v,float *out);
void m_scale(const float *scale,float *out);
void m_look_at(const float *eye,const float *dir,const float *up,float *out);
void m_shadow(const float *plane,const float *light,float *out);

//added by micha
float m_rotation_axis(const CMatrix3_t m,CVector3_t out);//return rotation axis and angle

void q_set(const float *dir,float angle,float *out);
void q_copy(const float *q,float *out);
void q_slerp(const float *q1,const float *q2,float t,float *out);
//first q1 then q2
void q_multiply(const float *q1,const float *q2,float *out);
void q_to_matrix(const float *q,float *out);

void q_v_mult(const float *q, const CVector3_t in, CVector3_t out);
void q_clear(float *q);//{v_clear(q);q[3]=1.0f;}

/**
 * generates a normally distributed variable
 * @param mean mean of normal distribution
 * @param var variance of normal distribution
 * @return a sample
 */
float normalSample(float mean,float var);


// added by M.Hersch

ostream& operator << (ostream& out, const CVector3_t& v);
ostream& operator << (ostream& out, const CVector4_t& v);
ostream& operator << (ostream& out, const CMatrix3_t& m);
//ostream& operator << (ostream& out, const CMatrix4_t& m);
#endif /* __MATHLIB_H__ */
