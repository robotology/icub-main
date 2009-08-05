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
#include <memory.h>
#include "iCub/mathlib.h"




float v_length(const float *v) {
  return (float)(sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
}

float v_normalize(const float *v,float *out) {
  float length,ilength;
  length = (float)(sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
  if(length == 0) {
    v_clear(out);
    return 0;
  }
  ilength = 1.0f / length;
  v_scale(v,ilength,out);
  return length;
}





void v_translate(const float *m,float *out) {
  out[0] = m[12];
  out[1] = m[13];
  out[2] = m[14];
}

void v_transform(const float *v,const float *m,float *out) {
  float v1[3];
  v_copy(v,v1);
  out[0] = v1[0] * m[0] + v1[1] * m[4] + v1[2] * m[8] + m[12];
  out[1] = v1[0] * m[1] + v1[1] * m[5] + v1[2] * m[9] + m[13];
  out[2] = v1[0] * m[2] + v1[1] * m[6] + v1[2] * m[10] + m[14];
}

// multiplication when m is index 0 1 2 3//4 ...
void v_transform2(const float *v,const float *m,float *out) {
  float v1[3];
  v_copy(v,v1);
  out[0] = v1[0] * m[0] + v1[1] * m[1] + v1[2] * m[2] + m[3];
  out[1] = v1[0] * m[4] + v1[1] * m[5] + v1[2] * m[6] + m[7];
  out[2] = v1[0] * m[8] + v1[1] * m[9] + v1[2] * m[10] + m[11];
}

void v_transform_normal(const float *v,const float *m,float *out) {
  float v1[3];
  v_copy(v,v1);
  out[0] = v1[0] * m[0] + v1[1] * m[4] + v1[2] * m[8];
  out[1] = v1[0] * m[1] + v1[1] * m[5] + v1[2] * m[9];
  out[2] = v1[0] * m[2] + v1[1] * m[6] + v1[2] * m[10];
}

void v_transform_normal2(const float *v,const float *m,float *out) {
  float v1[3];
  v_copy(v,v1);
  out[0] = v1[0] * m[0] + v1[1] * m[1] + v1[2] * m[2];
  out[1] = v1[0] * m[4] + v1[1] * m[5] + v1[2] * m[6];
  out[2] = v1[0] * m[8] + v1[1] * m[9] + v1[2] * m[10];
}

void m_copy(const float *m,float *out) {
  memcpy(out,m,sizeof(float) * 16);
}

void m_identity(float *m) {
  m[0] = 1; m[4] = 0; m[8] = 0; m[12] = 0;
  m[1] = 0; m[5] = 1; m[9] = 0; m[13] = 0;
  m[2] = 0; m[6] = 0; m[10] = 1; m[14] = 0;
  m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
}

void m_multiply(const float *m1,const float *m2,float *out) {
  out[0] = m1[0]*m2[0] + m1[4]*m2[1] + m1[8]*m2[2] + m1[12]*m2[3];
  out[1] = m1[1]*m2[0] + m1[5]*m2[1] + m1[9]*m2[2] + m1[13]*m2[3];
  out[2] = m1[2]*m2[0] + m1[6]*m2[1] + m1[10]*m2[2] + m1[14]*m2[3];
  out[3] = m1[3]*m2[0] + m1[7]*m2[1] + m1[11]*m2[2] + m1[15]*m2[3];
  out[4] = m1[0]*m2[4] + m1[4]*m2[5] + m1[8]*m2[6] + m1[12]*m2[7];
  out[5] = m1[1]*m2[4] + m1[5]*m2[5] + m1[9]*m2[6] + m1[13]*m2[7];
  out[6] = m1[2]*m2[4] + m1[6]*m2[5] + m1[10]*m2[6] + m1[14]*m2[7];
  out[7] = m1[3]*m2[4] + m1[7]*m2[5] + m1[11]*m2[6] + m1[15]*m2[7];
  out[8] = m1[0]*m2[8] + m1[4]*m2[9] + m1[8]*m2[10] + m1[12]*m2[11];
  out[9] = m1[1]*m2[8] + m1[5]*m2[9] + m1[9]*m2[10] + m1[13]*m2[11];
  out[10] = m1[2]*m2[8] + m1[6]*m2[9] + m1[10]*m2[10] + m1[14]*m2[11];
  out[11] = m1[3]*m2[8] + m1[7]*m2[9] + m1[11]*m2[10] + m1[15]*m2[11];
  out[12] = m1[0]*m2[12] + m1[4]*m2[13] + m1[8]*m2[14] + m1[12]*m2[15];
  out[13] = m1[1]*m2[12] + m1[5]*m2[13] + m1[9]*m2[14] + m1[13]*m2[15];
  out[14] = m1[2]*m2[12] + m1[6]*m2[13] + m1[10]*m2[14] + m1[14]*m2[15];
  out[15] = m1[3]*m2[12] + m1[7]*m2[13] + m1[11]*m2[14] + m1[15]*m2[15];    
}

int m_inverse(const float *m,float *out) {
  float   det;
  det = m[0] * m[5] * m[10];
  det += m[4] * m[9] * m[2];
  det += m[8] * m[1] * m[6];
  det -= m[8] * m[5] * m[2];
  det -= m[4] * m[1] * m[10];
  det -= m[0] * m[9] * m[6];
  if(det * det < 1e-6) return -1;
  det = 1.0f / det;    
  out[0] =    (m[5] * m[10] - m[9] * m[6]) * det;
  out[1] =  - (m[1] * m[10] - m[9] * m[2]) * det;
  out[2] =    (m[1] * m[6] -  m[5] * m[2]) * det;
  out[3] = 0.0;
  out[4] =  - (m[4] * m[10] - m[8] * m[6]) * det;
  out[5] =    (m[0] * m[10] - m[8] * m[2]) * det;
  out[6] =  - (m[0] * m[6] -  m[4] * m[2]) * det;
  out[7] = 0.0;
  out[8] =    (m[4] * m[9] -  m[8] * m[5]) * det;
  out[9] =  - (m[0] * m[9] -  m[8] * m[1]) * det;
  out[10] =   (m[0] * m[5] -  m[4] * m[1]) * det;
  out[11] = 0.0;
  out[12] = - (m[12] * out[0] + m[13] * out[4] + m[14] * out[8]);
  out[13] = - (m[12] * out[1] + m[13] * out[5] + m[14] * out[9]);
  out[14] = - (m[12] * out[2] + m[13] * out[6] + m[14] * out[10]);
  out[15] = 1.0;
  return 0;
}

void m_transpose(const float *m,float *out) {
  out[0] = m[0]; out[4] = m[1]; out[8] = m[2]; out[12] = m[3];
  out[1] = m[4]; out[5] = m[5]; out[9] = m[6]; out[13] = m[7];
  out[2] = m[8]; out[6] = m[9]; out[10] = m[10]; out[14] = m[11];
  out[3] = m[12]; out[7] = m[13]; out[11] = m[14]; out[15] = m[15];
}

void m_transpose_rotation(const float *m,float *out) {
  out[0] = m[0]; out[4] = m[1]; out[8] = m[2]; out[12] = m[12];
  out[1] = m[4]; out[5] = m[5]; out[9] = m[6]; out[13] = m[13];
  out[2] = m[8]; out[6] = m[9]; out[10] = m[10]; out[14] = m[14];
  out[3] = m[3]; out[7] = m[7]; out[11] = m[11]; out[15] = m[15];
}

void m_rotation_x(float angle,float *out) {
  float rad = angle * ((float)deg2rad);
  float Cos = (float)cos(rad);
  float Sin = (float)sin(rad);
  out[0] = 1.0; out[4] = 0.0; out[8] = 0.0; out[12] = 0.0;
  out[1] = 0.0; out[5] = Cos; out[9] = -Sin; out[13] = 0.0;
  out[2] = 0.0; out[6] = Sin; out[10] = Cos; out[14] = 0.0;
  out[3] = 0.0; out[7] = 0.0; out[11] = 0.0; out[15] = 1.0;
}

void m_rotation_y(float angle,float *out) {
  float rad = angle * ((float)deg2rad);
  float Cos = (float)cos(rad);
  float Sin = (float)sin(rad);
  out[0] = Cos; out[4] = 0.0; out[8] = Sin; out[12] = 0.0;
  out[1] = 0.0; out[5] = 1.0; out[9] = 0.0; out[13] = 0.0;
  out[2] = -Sin; out[6] = 0.0; out[10] = Cos; out[14] = 0.0;
  out[3] = 0.0; out[7] = 0.0; out[11] = 0.0; out[15] = 1.0;
}

void m_rotation_z(float angle,float *out) {
  float rad = angle * ((float)deg2rad);
  float Cos = (float)cos(rad);
  float Sin = (float)sin(rad);
  out[0] = Cos; out[4] = -Sin; out[8] = 0.0; out[12] = 0.0;
  out[1] = Sin; out[5] =  Cos; out[9] = 0.0; out[13] = 0.0;
  out[2] = 0.0; out[6] = 0.0; out[10] = 1.0; out[14] = 0.0;
  out[3] = 0.0; out[7] = 0.0; out[11] = 0.0; out[15] = 1.0;
}

void m_rotation_v(float angle,float *v,float *out) {
  float rad = angle * ((float)deg2rad);
  float C = (float)cos(rad);
  float S = (float)sin(rad);
  float U = (1-C);
  out[0] = v[0]*v[0]*U +      C; out[4] = v[0]*v[1]*U - v[2]*S; out[8] = v[0]*v[2]*U + v[1]*S; out[12] = 0.0;
  out[1] = v[0]*v[1]*U + v[2]*S; out[5] = v[1]*v[1]*U +      C; out[9] = v[1]*v[2]*U - v[0]*S; out[13] = 0.0;
  out[2] = v[0]*v[2]*U - v[1]*S; out[6] = v[1]*v[2]*U + v[0]*S; out[10]= v[2]*v[2]*U +      C; out[14] = 0.0;
  out[3] = 0.0; out[7] = 0.0; out[11] = 0.0; out[15] = 1.0;
}

void m_rotation_x2(float angle,float *out) {
  float rad = angle;
  float Cos = (float)cos(rad);
  float Sin = (float)sin(rad);
  out[0] = 1.0; out[4] = 0.0; out[8] = 0.0; out[12] = 0.0;
  out[1] = 0.0; out[5] = Cos; out[9] = -Sin; out[13] = 0.0;
  out[2] = 0.0; out[6] = Sin; out[10] = Cos; out[14] = 0.0;
  out[3] = 0.0; out[7] = 0.0; out[11] = 0.0; out[15] = 1.0;
}

void m_rotation_y2(float angle,float *out) {
  float rad = angle;
  float Cos = (float)cos(rad);
  float Sin = (float)sin(rad); 
  out[0] = Cos; out[4] = 0.0; out[8] = Sin; out[12] = 0.0;
  out[1] = 0.0; out[5] = 1.0; out[9] = 0.0; out[13] = 0.0;
  out[2] = -Sin; out[6] = 0.0; out[10] = Cos; out[14] = 0.0;
  out[3] = 0.0; out[7] = 0.0; out[11] = 0.0; out[15] = 1.0;
}

void m_rotation_z2(float angle,float *out) {
  float rad = angle;
  float Cos = (float)cos(rad);
  float Sin = (float)sin(rad);
  out[0] = Cos; out[4] = -Sin; out[8] = 0.0; out[12] = 0.0;
  out[1] = Sin; out[5] = Cos; out[9] = 0.0; out[13] = 0.0;
  out[2] = 0.0; out[6] = 0.0; out[10] = 1.0; out[14] = 0.0;
  out[3] = 0.0; out[7] = 0.0; out[11] = 0.0; out[15] = 1.0;
}

void m_rotation_v2(float angle,float *v,float *out) {
  float rad = angle;
  float C = (float)cos(rad);
  float S = (float)sin(rad);
  float U = (1-C);
  out[0] = v[0]*v[0]*U +      C; out[4] = v[0]*v[1]*U - v[2]*S; out[8] = v[0]*v[2]*U + v[1]*S; out[12] = 0.0;
  out[1] = v[0]*v[1]*U + v[2]*S; out[5] = v[1]*v[1]*U +      C; out[9] = v[1]*v[2]*U - v[0]*S; out[13] = 0.0;
  out[2] = v[0]*v[2]*U - v[1]*S; out[6] = v[1]*v[2]*U + v[0]*S; out[10]= v[2]*v[2]*U +      C; out[14] = 0.0;
  out[3] = 0.0; out[7] = 0.0; out[11] = 0.0; out[15] = 1.0;
}

/*
  micha - phi rotation angle;
  theta: axe angle with horizontal plane Oxy
  psi: axe angle with sagital plane Oxz
*/

void m_rotation_xyz(float phi, float theta, float psi, float *out)
{
  phi   *= ((float)deg2rad);
  theta *= ((float)deg2rad);
  psi   *= ((float)deg2rad);

  out[0] =((float)(cos(theta)*cos(psi)));
  out[1] =((float)(cos(theta)*sin(psi)));
  out[2] =((float)(-sin(theta)));
  out[3] =0.0f;
  out[4] =((float)(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)));
  out[5] =((float)(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)));
  out[6] =((float)(sin(phi)*cos(theta)));
  out[7] =0.0f;
  out[8] =((float)(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)));
  out[9] =((float)(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)));
  out[10]=((float)(cos(phi)*cos(theta)));
  out[11]=0.0f;
  out[12]=0.0f;
  out[13]=0.0f;
  out[14]=0.0f;
  out[16]=1.0f;
}

/*
void m_rotation_xyz(float phi, float theta, float psi, float *out)
{
  phi *= deg2rad;
  theta *= deg2rad;
  psi *= deg2rad;

  out[0]=cos(theta)*cos(psi);                            out[4]=cos(theta)*sin(psi);                            out[8]=-sin(theta);          out[12]=0.0;
  out[1]=sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi); out[5]=sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi); out[9]=sin(phi)*cos(theta);  out[13]=0.0;
  out[2]=cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); out[6]=cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); out[10]=cos(phi)*cos(theta); out[14]=0.0;
  out[3]=0.0;                                            out[7]=0.0;                                            out[11]=0.0;                 out[15] = 1.0;
}
*/

void m_rotation_xyz2(float phi, float theta, float psi, float *out)
{
  out[0] =((float)(cos(theta)*cos(psi)));
  out[4] =((float)(cos(theta)*sin(psi)));
  out[8] =((float)(-sin(theta)));
  out[12]=0.0f;
  out[1] =((float)(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)));
  out[5] =((float)(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)));
  out[9] =((float)(sin(phi)*cos(theta)));
  out[13]=0.0f;
  out[2] =((float)(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)));
  out[6] =((float)(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)));
  out[10]=((float)(cos(phi)*cos(theta)));
  out[14]=0.0f;
  out[3] =0.0f;
  out[7] =0.0f; 
  out[11]=0.0f;
  out[15]=1.0f;
}

void m_rotation_serial(float angle_1, float angle_2, float angle_3, float *out)
{
  float Mtmp[16], Mtmp1[16], Mtmp2[16], Mtmp3[16];
  float vt1[]={0.0f,1.0f,0.0f};
  float vt2[]={1.0f,0.0f,0.0f};
  float vt3[]={0.0f,0.0f,1.0f};
  float v[3];


  //printf("l_sfe[deg]:%.2f\n",angle_1);
  //printf("l_saa[deg]:%.2f\n",angle_2);
  //printf("l_shr[deg]:%.2f\n\n",angle_3);

  m_rotation_v(angle_1, vt1, Mtmp1);
  v_transform(vt2, Mtmp1, v);
  m_rotation_v(angle_2, v, Mtmp2); 
  m_multiply(Mtmp2, Mtmp1, Mtmp);
  v_transform(vt3, Mtmp, v);
  m_rotation_v(angle_3, v, Mtmp3);

  m_multiply(Mtmp3, Mtmp, out);
}

void m_translate(const float *v,float *out) {
  out[0] = 1.0f; out[4] = 0.0f; out[8]  = 0.0f; out[12] = v[0];
  out[1] = 0.0f; out[5] = 1.0f; out[9]  = 0.0f; out[13] = v[1];
  out[2] = 0.0f; out[6] = 0.0f; out[10] = 1.0f; out[14] = v[2];
  out[3] = 0.0f; out[7] = 0.0f; out[11] = 0.0f; out[15] = 1.0f;
}

void m_scale(const float *scale,float *out) {
  out[0] = scale[0]; out[4] = 0.0; out[8] = 0.0; out[12] = 0.0;
  out[1] = 0.0; out[5] = scale[1]; out[9] = 0.0; out[13] = 0.0;
  out[2] = 0.0; out[6] = 0.0; out[10] = scale[2]; out[14] = 0.0;
  out[3] = 0.0; out[7] = 0.0; out[11] = 0.0; out[15] = 1.0;
}

void m_look_at(const float *eye,const float *dir,const float *up,float *out) {
  float x[3],y[3],z[3],ieye[3],m1[16],m2[16];
  v_sub(eye,dir,z);
  v_normalize(z,z);
  v_cross(up,z,x);
  v_cross(z,x,y);
  v_normalize(x,x);
  v_normalize(y,y);
  m1[0] = x[0]; m1[4] = x[1]; m1[8] = x[2]; m1[12] = 0.0;
  m1[1] = y[0]; m1[5] = y[1]; m1[9] = y[2]; m1[13] = 0.0;
  m1[2] = z[0]; m1[6] = z[1]; m1[10] = z[2]; m1[14] = 0.0;
  m1[3] = 0.0; m1[7] = 0.0; m1[11] = 0.0; m1[15] = 1.0;
  v_scale(eye,-1,ieye);
  m_translate(ieye,m2);
  m_multiply(m1,m2,out);
}

void m_shadow(const float *plane,const float *light,float *out) {
  float dot;
  dot = plane[0] * light[0] + plane[1] * light[1] +
    plane[2] * light[2] + plane[3] * light[3];
  out[0] = -light[0] * plane[0] + dot;
  out[1] = -light[1] * plane[0];
  out[2] = -light[2] * plane[0];
  out[3] = -light[3] * plane[0];
  out[4] = -light[0] * plane[1];
  out[5] = -light[1] * plane[1] + dot;
  out[6] = -light[2] * plane[1];
  out[7] = -light[3] * plane[1];
  out[8] = -light[0] * plane[2];
  out[9] = -light[1] * plane[2];
  out[10] = -light[2] * plane[2] + dot;
  out[11] = -light[3] * plane[2];
  out[12] = -light[0] * plane[3];
  out[13] = -light[1] * plane[3];
  out[14] = -light[2] * plane[3];
  out[15] = -light[3] * plane[3] + dot;
}

void q_set(const float *dir,float angle,float *out) {
  float sinangle,length,ilength;
  length = (float)sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
  if(length != 0.0) {
    ilength = 1.0f / length;
    sinangle = (float)sin(angle/2);
    out[0] = dir[0] * ilength * sinangle;
    out[1] = dir[1] * ilength * sinangle;
    out[2] = dir[2] * ilength * sinangle;
    out[3] = (float)cos(angle/2);
    return;
  }
  out[0] = out[1] = out[2] = 0.0;
  out[3] = 1.0;
}

void q_copy(const float *q,float *out) {
  out[0] = q[0];
  out[1] = q[1];
  out[2] = q[2];
  out[3] = q[3];
}

void q_slerp(const float *q1,const float *q2,float t,float *out) {
  float omega,cosomega,sinomega,k1,k2,q[4];
  cosomega = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
  if(cosomega < 0.0) {
    cosomega = -cosomega;
    q[0] = -q2[0];
    q[1] = -q2[1];
    q[2] = -q2[2];
    q[3] = -q2[3];
  } else {
    q[0] = q2[0];
    q[1] = q2[1];
    q[2] = q2[2];
    q[3] = q2[3];
  }
  if(1.0 - cosomega > 1e-6) {
    omega = (float)acos(cosomega);
    sinomega = (float)sin(omega);
    k1 = (float)(sin((1.0 - t) * omega) / sinomega);
    k2 = (float)(sin(t * omega) / sinomega);
  } else {
    k1 = 1.0f - t;
    k2 = t;
  }
  out[0] = q1[0] * k1 + q[0] * k2;
  out[1] = q1[1] * k1 + q[1] * k2;
  out[2] = q1[2] * k1 + q[2] * k2;
  out[3] = q1[3] * k1 + q[3] * k2;
}
//first q1 then q2
void q_multiply(const float *q1,const float *q2,float *out) {
  out[0] = q1[3] * q2[0] + q1[0] * q2[3] - q1[1] * q2[2] + q1[2] * q2[1];
  out[1] = q1[3] * q2[1] + q1[1] * q2[3] + q1[0] * q2[2] - q1[2] * q2[0];
  out[2] = q1[3] * q2[2] + q1[2] * q2[3] - q1[0] * q2[1] + q1[1] * q2[0];
  out[3] = q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2];
}


void q_v_mult(const float *q, const CVector3_t in, CVector3_t out){
  CVector3_t v1,v2,v3;
  float f1;
  f1 = q[3]*q[3];
  v_scale(in,f1,v1); //alpha^2*v
  v_cross(q,in,v2); //beta x v
  v_scale(v2,2*q[3],v3); // 2*alpha*(beta x v)
  v_add(v1,v3,v3); // alpha^2*v+2*alpha*(beta x v)
  f1 = v_dot(q,in); //beta'*v
  v_scale(q,f1,v1); //(beta'*v)*beta 
  v_add(v3,v1,v3);// alpha^2*v+2*alpha*(beta x v)+(beta'*v)*beta 
  v_cross(v2,q,v1); //(beta x v) x beta
  v_sub(v3,v1,out); 
}
// return rotation axis and angle
// implements the formulae 3.3.18 and 3.3.19 (p.73) of Altmann, 
// "Rotations, quaternions and double groups", 1986
float m_rotation_axis(const CMatrix3_t m,CVector3_t out){
  float angle,f;
  f =  0.5*(m[0]+m[5]+m[10]-1);
  if(abs(f)>1){
    return 0;
  }
  else{
    angle = acos(f);
    if(abs(angle)<epsilon){
      v_clear(out);
      return 0.0f;
    }
    else{
      out[0] = m[6]-m[9];
      out[1] = m[8]-m[2];
      out[2] = m[1]-m[4];
      f = 1/(2*sin(angle));
      v_scale(out,f,out);
      return angle;
    }
  }
}



void q_to_matrix(const float *q,float *out) {
  float x2,y2,z2,xx,yy,zz,xy,yz,xz,wx,wy,wz;
  x2 = q[0] + q[0];
  y2 = q[1] + q[1];
  z2 = q[2] + q[2];
  xx = q[0] * x2;
  yy = q[1] * y2;
  zz = q[2] * z2;
  xy = q[0] * y2;
  yz = q[1] * z2;
  xz = q[2] * x2;
  wx = q[3] * x2;
  wy = q[3] * y2;
  wz = q[3] * z2;
  out[0] = 1.0f - (yy + zz);
  out[1] = xy + wz;
  out[2] = xz - wy;
  out[3] = 0.0f;
  out[4] = xy - wz;
  out[5] = 1.0f - (xx + zz);
  out[6] = yz + wx;
  out[7] = 0.0f;
  out[8] = xz + wy;
  out[9] = yz - wx;
  out[10] = 1.0f - (xx + yy);
  out[11] = 0.0f;
  out[12] = 0.0f;
  out[13] = 0.0f;
  out[14] = 0.0f;
  out[15] = 1.0f;
}

void q_clear(CQuat_t q) {
  v_clear(q);
  q[3]=1.0f;
}

/* functions added by M. Hersch */
#ifdef M_MACROS
void v_cross(const float *v1,const float *v2,float *out) {
  out[0] = v1[1] * v2[2] - v1[2] * v2[1];
  out[1] = v1[2] * v2[0] - v1[0] * v2[2];
  out[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void v4_sub(CVector4_t a, CVector4_t b, CVector4_t c){
  c[0] = a[0] - b[0]; 
  c[1] = a[1] - b[1]; 
  c[2] = a[2] - b[2]; 
  c[3] = a[3] - b[3];
}

void v4_addv(CVector4_t a, CVector4_t b, CVector4_t c){
  c[0] = a[0] + b[0]; 
  c[1] = a[1] + b[1]; 
  c[2] = a[2] + b[2]; 
  c[3] = a[3] + b[3];
}

void v4_scale(CVector4_t a, float b, CVector4_t c){
  c[0] = a[0] * b;   c[1] = a[1] * b;   c[2] = a[2] * b; c[3] = a[3] * b;
}

void v4_copy(CVector4_t a, CVector4_t b){
  b[0] = a[0];  b[1] = a[1]; b[2] = a[2]; b[3] = a[3];
}

#endif


float v4_length(const CVector4_t v) {
  return (float)(sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]));
}

float v_squ_length(const CVector3_t v){
  return (float)(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float v4_squ_length(const CVector4_t v){
  return (float)(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
}
/* out = m*v*m' where m = 3x4 indexed in a transposed way (0 1 2 3// 5 ...), 
v is a diagonal matrix of size 4*/

// out = m'm ; m = 3 rows, 4 columns
void m3_4_square(CMatrix4_t m, CMatrix3_t out){
  //for correct matrix index
  out[0]= m[0]*m[0] + m[4]*m[4] + m[8]*m[8] + m[12]*m[12];
  out[1]= m[0]*m[1] + m[4]*m[5] + m[8]*m[9] + m[12]*m[13];
  out[2]= m[0]*m[2] + m[4]*m[6] + m[8]*m[10]+ m[12]*m[14];
  out[3]= 0.0f;
  out[4]= out[1];
  out[5]= m[1]*m[1] + m[5]*m[5] + m[9]*m[9] + m[13]*m[13];
  out[6]= m[1]*m[2] + m[5]*m[6] + m[9]*m[10]+ m[13]*m[14];
  out[7]= 0.0f;
  out[8]= out[2];
  out[9]= out[6];
  out[10]=m[2]*m[2] + m[6]*m[6] + m[10]*m[10]+ m[14]*m[14];
  out[11]=0.0f;
  out[12]=0.0f;
  out[13]=0.0f;
  out[14]=0.0f;
  out[15]=0.0f;    
}





void m3_4_weighted_square(CMatrix4_t m,CVector4_t v, CMatrix3_t out)
{
#ifndef CORRECT_MATRIX_INDEX
  out[0]= v[0]*m[0]*m[0] + v[1]*m[1]*m[1] + v[2]*m[2]*m[2] + v[3]*m[3]*m[3];
  out[1]= v[0]*m[0]*m[4] + v[1]*m[1]*m[5] + v[2]*m[2]*m[6] + v[3]*m[3]*m[7];
  out[2]= v[0]*m[0]*m[8] + v[1]*m[1]*m[9] + v[2]*m[2]*m[10]+ v[3]*m[3]*m[11];
  out[3]= 0.0f;
  out[4]= out[1];
  out[5]= v[0]*m[4]*m[4] + v[1]*m[5]*m[5] + v[2]*m[6]*m[6] + v[3]*m[7]*m[7];
  out[6]= v[0]*m[4]*m[8] + v[1]*m[5]*m[9] + v[2]*m[6]*m[10]+ v[3]*m[7]*m[11];
  out[7]= 0.0f;
  out[8]= out[2];
  out[9]= out[6];
  out[10]= v[0]*m[8]*m[8] + v[1]*m[9]*m[9]+ v[2]*m[10]*m[10]+ v[3]*m[11]*m[11];
  out[11]=0.0f;
  out[12]=0.0f;
  out[13]=0.0f;
  out[14]=0.0f;
  out[15]=0.0f;    
#else
  out[0]= v[0]*m[0]*m[0] + v[1]*m[4]*m[4] + v[2]*m[8]*m[8] + v[3]*m[12]*m[12];
  out[1]= v[0]*m[0]*m[1] + v[1]*m[4]*m[5] + v[2]*m[8]*m[9] + v[3]*m[12]*m[13];
  out[2]= v[0]*m[0]*m[2] + v[1]*m[4]*m[6] + v[2]*m[8]*m[10]+ v[3]*m[12]*m[14];
  out[3]= 0.0f;
  out[4]= out[1];
  out[5]= v[0]*m[1]*m[1] + v[1]*m[5]*m[5] + v[2]*m[9]*m[9] + v[3]*m[13]*m[13];
  out[6]= v[0]*m[1]*m[2] + v[1]*m[5]*m[6] + v[2]*m[9]*m[10]+ v[3]*m[13]*m[14];
  out[7]= 0.0f;
  out[8]= out[2];
  out[9]= out[6];
  out[10]= v[0]*m[2]*m[2] + v[1]*m[6]*m[6]+ v[2]*m[10]*m[10]+ v[3]*m[14]*m[14];
  out[11]=0.0f;
  out[12]=0.0f;
  out[13]=0.0f;
  out[14]=0.0f;
  out[15]=0.0f;    

#endif

}
void m3_4_t_weighted_square(CMatrix4_t m,CVector3_t v, CMatrix4_t out)
{
#ifndef CORRECT_MATRIX_INDEX
  out[0]=  v[0]*m[0]*m[0] + v[1]*m[4]*m[4] + v[2]*m[8]*m[8];
  out[1]=  v[0]*m[0]*m[1] + v[1]*m[4]*m[5] + v[2]*m[8]*m[9];
  out[2]=  v[0]*m[0]*m[2] + v[1]*m[4]*m[6] + v[2]*m[8]*m[10];
  out[3]=  v[0]*m[0]*m[3] + v[1]*m[4]*m[7] + v[2]*m[8]*m[11];
  out[4]= out[1];
  out[5]=  v[0]*m[1]*m[1] + v[1]*m[5]*m[5] + v[2]*m[9]*m[9];
  out[6]=  v[0]*m[1]*m[2] + v[1]*m[5]*m[6] + v[2]*m[9]*m[10];
  out[7]=  v[0]*m[1]*m[3] + v[1]*m[5]*m[7] + v[2]*m[9]*m[11];
  out[8]= out[2];
  out[9]= out[6];
  out[10]= v[0]*m[2]*m[2] + v[1]*m[6]*m[6] + v[2]*m[10]*m[10];
  out[11]= v[0]*m[2]*m[3] + v[1]*m[6]*m[7] + v[2]*m[10]*m[11];
  out[12]= out[3];
  out[13]= out[7];
  out[14]= out[11];
  out[15]= v[0]*m[3]*m[3] + v[1]*m[7]*m[7] + v[2]*m[11]*m[11];   
#else
  out[0]=  v[0]*m[0]*m[0] + v[1]*m[1]*m[1] + v[2]*m[2]*m[2];
  out[1]=  v[0]*m[0]*m[4] + v[1]*m[1]*m[5] + v[2]*m[2]*m[6];
  out[2]=  v[0]*m[0]*m[8] + v[1]*m[1]*m[9] + v[2]*m[2]*m[10];
  out[3]=  v[0]*m[0]*m[12]+ v[1]*m[1]*m[13]+ v[2]*m[2]*m[14];
  out[4]= out[1];
  out[5]=  v[0]*m[4]*m[4] + v[1]*m[5]*m[5] + v[2]*m[6]*m[6];
  out[6]=  v[0]*m[4]*m[8] + v[1]*m[5]*m[9] + v[2]*m[6]*m[10];
  out[7]=  v[0]*m[4]*m[12]+ v[1]*m[5]*m[13]+ v[2]*m[6]*m[14];
  out[8]= out[2];
  out[9]= out[6];
  out[10]= v[0]*m[8]*m[8] + v[1]*m[9]*m[9] + v[2]*m[10]*m[10];
  out[11]= v[0]*m[8]*m[12]+ v[1]*m[9]*m[13]+ v[2]*m[10]*m[14];
  out[12]= out[3];
  out[13]= out[7];
  out[14]= out[11];
  out[15]= v[0]*m[12]*m[12]+ v[1]*m[13]*m[13]+ v[2]*m[14]*m[14];   
#endif
}


void inverse_diag3(CVector3_t v, CVector3_t out)
{
  int i;
  for(i=0;i<3;i++){
    out[i]=1.0/v[i];
  }
}

void inverse_diag4(CVector4_t v, CVector4_t out)
{
  int i;
  for(i=0;i<4;i++){
    out[i]=1.0/v[i];
  }
}


void m3_add_diag(CMatrix3_t m, CVector3_t v, CMatrix3_t out)
{
  m_copy(m,out);
  out[0] += v[0];
  out[5] += v[1];
  out[10] += v[2];
}

void m4_add_diag(CMatrix4_t m, CVector4_t v, CMatrix4_t out)
{
  m4_copy(m,out);
  out[0] += v[0];
  out[5] += v[1];
  out[10]+= v[2];
  out[15]+= v[3];
}


/* out = m*v, where m = 3x4, v=4x1 */
void m3_4_v_multiply(CMatrix4_t m, CVector4_t v, CVector3_t out)
{
#ifndef CORRECT_MATRIX_INDEX
  out[0]= m[0]*v[0] + m[1]*v[1] + m[2]*v[2] + m[3]*v[3];
  out[1]= m[4]*v[0] + m[5]*v[1] + m[6]*v[2] + m[7]*v[3];
  out[2]= m[8]*v[0] + m[9]*v[1] + m[10]*v[2]+ m[11]*v[3];
#else
  out[0]= m[0]*v[0] + m[4]*v[1] + m[8]*v[2] + m[12]*v[3];
  out[1]= m[1]*v[0] + m[5]*v[1] + m[9]*v[2] + m[13]*v[3];
  out[2]= m[2]*v[0] + m[6]*v[1] + m[10]*v[2]+ m[14]*v[3];
#endif
}
/* out = m*v , where m is 4x3, v is 3x1 out is 4x1*/
void m4_3_v_multiply(CMatrix4_t m, CVector3_t v1, CVector4_t out){
  // correct matrix index
  out[0] = v1[0] * m[0] + v1[1] * m[4] + v1[2] * m[8];
  out[1] = v1[0] * m[1] + v1[1] * m[5] + v1[2] * m[9];
  out[2] = v1[0] * m[2] + v1[1] * m[6] + v1[2] * m[10];
  out[3] = v1[0] * m[3] + v1[1] * m[7] + v1[2] * m[11];
}


/* out = m*v, where m is 3x3 diagonal,v=3x1 */ 
void m3_diag_v_multiply(CVector3_t m, CVector3_t v, CVector3_t out)
{
  out[0]= m[0]*v[0];
  out[1]= m[1]*v[1];
  out[2]= m[2]*v[2];
}
/* out = m'*v, where m=3x4, v=3x1 */


void m3_4_t_v_multiply(CMatrix4_t m, CVector3_t v, CVector4_t out)
{
#ifndef CORRECT_MATRIX_INDEX
  out[0] = m[0]*v[0] + m[4]*v[1] + m[8]*v[2];
  out[1] = m[1]*v[0] + m[5]*v[1] + m[9]*v[2];
  out[2] = m[2]*v[0] + m[6]*v[1] + m[10]*v[2];
  out[3] = m[3]*v[0] + m[7]*v[1] + m[11]*v[2];
#else
  out[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
  out[1] = m[4]*v[0] + m[5]*v[1] + m[6]*v[2];
  out[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2];
  out[3] = m[12]*v[0] + m[13]*v[1] + m[14]*v[2];
#endif
}
//out = m1'*m2 where  m1 is 3x4, m2 is 3x3, out is 4x3
void m3_4_t_m_multiply(CMatrix4_t m1, CMatrix3_t m2, CMatrix4_t out){
  for(int i=0;i<3;i++){
    m3_4_t_v_multiply(m1,m2+4*i,out+4*i);
  }
}

void m4_4_v_multiply(CMatrix4_t m ,CVector4_t v1, CVector4_t out)
{
  out[0] = v1[0] * m[0] + v1[1] * m[4] + v1[2] * m[8] + v1[3] * m[12];
  out[1] = v1[0] * m[1] + v1[1] * m[5] + v1[2] * m[9] + v1[3] * m[13];
  out[2] = v1[0] * m[2] + v1[1] * m[6] + v1[2] * m[10]+ v1[3] * m[14];
  out[3] = v1[0] * m[3] + v1[1] * m[7] + v1[2] * m[11]+ v1[3] * m[15];
}


/* out = m*v, where m is 4x4 diagonal,v=4x1 */ 
void m4_diag_v_multiply(CVector4_t m, CVector4_t v, CVector4_t out)
{
  out[0]= m[0]*v[0];
  out[1]= m[1]*v[1];
  out[2]= m[2]*v[2];
  out[3]= m[3]*v[3];
}

void m4_minus(CMatrix4_t m, CMatrix4_t out)
{
  for (int i=0;i<16;i++){
    out[i] = -m[i];
  }
}

// indexed (0 1 2 3 // 4 5 ...)
void v_mult(const CVector3_t v1,const CVector3_t v2, CMatrix4_t m){
#ifndef CORRECT_MATRIX_INDEX
  for(int i=0;i<3;i++){
     for(int j=0;j<3;j++){
       m[i*4+j]=v1[i]*v2[j];
     }
     m[i*4+3] = v1[i];
  }
 v_copy(v2,&(m[12]));
#else
 for(int i=0;i<3;i++){
     for(int j=0;j<3;j++){
       m[i*4+j]=v2[i]*v1[j];
     }
     m[i*4+3] = v2[i];
  }
 v_copy(v1,&(m[12]));
#endif

  m[15] = 1;
}




void m_sub(CMatrix4_t m1,CMatrix4_t m2,CMatrix4_t out){
  for(int i=0;i<16;i++){
    out[i] = m1[i]-m2[i];
  }
}

void m_add(CMatrix4_t m1,CMatrix4_t m2,CMatrix4_t out){
  for(int i=0;i<16;i++){
    out[i] = m1[i]+m2[i];
  }
}


void m_rescale(float k,CMatrix4_t m, CMatrix4_t out){
   for(int i=0;i<16;i++){
    out[i] = k*m[i];
  }
}

void m_set_v3_column(CVector3_t v,int i, CMatrix3_t out)
{
    v_copy(v,out+4*i);    
}

void m_get_v3_column(CMatrix4_t m,int i, CVector3_t out)
{
    v_copy(m+4*i,out);    
}

void m_clear(CMatrix3_t m){
   for(int i=0;i<15;i++){
     m[i]=0;
   }
   m[15]=1;
}

// uses method described p.648 of Zwillinger
float normalSample(float mean, float var)
{
  float v1,v2,r,x;
  r=2;
  while(r>1 || r==0.0){
    v1 = rand()/((float)RAND_MAX);
    v2 = rand()/((float)RAND_MAX);
    r= v1*v1+v2*v2;
  }
  x=v1*sqrt(-2*log(r)/r);
  return x*sqrt(var)+mean;
}


ostream& operator<<(ostream& out, const CVector3_t& v)
{
  out << v[0] <<" "<<v[1] <<" "<<v[2];
  return out;
}

ostream& operator<<(ostream& out, const CVector4_t& v)
{
  out << v[0] <<" "<<v[1] <<" "<<v[2]<<" "<<v[3];
  return out;
}

// ostream& operator << (ostream& out, const CMatrix3_t& m)
// {
 
//   out << m[0] << '\t'<< m[1] << '\t'<< m[2] <<endl
//       << m[4] << '\t'<< m[5] << '\t'<< m[6] <<endl
//       << m[8] << '\t'<< m[9] << '\t'<< m[10] <<endl;
//   return out;
// }

ostream& operator<<(ostream& out, const CMatrix4_t& m)
{
#ifndef CORRECT_MATRIX_INDEX
  out << m[0] << '\t'<< m[1] << '\t'<< m[2]<< '\t'<< m[3] <<endl
      << m[4] << '\t'<< m[5] << '\t'<< m[6]<< '\t'<< m[7] <<endl
      << m[8] << '\t'<< m[9] << '\t'<< m[10]<< '\t'<< m[11] <<endl
      << m[12] << '\t'<< m[13] << '\t'<< m[14]<< '\t'<< m[15] <<endl;
#else
  out << m[0] << '\t'<< m[4] << '\t'<< m[8]<< '\t'<< m[12] <<endl
      << m[1] << '\t'<< m[5] << '\t'<< m[9]<< '\t'<< m[13] <<endl
      << m[2] << '\t'<< m[6] << '\t'<< m[10]<< '\t'<< m[14] <<endl
      << m[3] << '\t'<< m[7] << '\t'<< m[11]<< '\t'<< m[15] <<endl;
#endif
  return out;
}


