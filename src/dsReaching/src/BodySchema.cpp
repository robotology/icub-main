// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Micha Hersch, EPFL
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
#include "iCub/BodySchema.h"

#define MIN_EB 0.000000001

BodySchema::BodySchema(){
#ifdef ICUB
  l1 = 109.8;
  l2 = 151.46;
  v4_set(0,-pi/2,-pi/2,0,min_angle);
  v4_set(pi/2,70*deg2rad,54*deg2rad,106*pi/180,max_angle); // eb angle restricted to 110
#else
  l1 = 116;
  l2 = 101;
  v4_set(0,-pi/2,-pi/2,0,min_angle);
  v4_set(pi,pi/2,pi/6,115*pi/180,max_angle); // eb angle restricted to 110
#endif
}

BodySchema::~BodySchema(){}

void BodySchema::SetArmLength(float nl1,float nl2){
  l1 = nl1;
  l2 = nl2;
}

float BodySchema::GetUpperArmLength(){
  return l1;
}

float BodySchema::GetForeArmLength(){
  return l2;
}


void BodySchema::Angle2Cart(pArmConfig_t config, CVector3_t out){
  CVector4_t angle;
  v4_set(config->sfe,config->saa,config->shr,config->eb,angle);
  Angle2Cart(angle,out);
}
void BodySchema::Angle2Cart(CVector4_t angle, CVector3_t out){
 float c0,c1,c2,c3,s0,s1,s2,s3;

  c0=cos(angle[0]); s0=sin(angle[0]);
  c1=cos(angle[1]); s1=sin(angle[1]);
  c2=cos(angle[2]); s2=sin(angle[2]);
  c3=cos(angle[3]); s3=sin(angle[3]);

  out[0] = -(l1*c1+l2*(c1*c3 + s1*s2*s3));
  out[1] = -l1*c0*s1 + l2*(c0 * (c1*s2*s3- s1*c3) + s0*c2*s3);
  out[2] =  l1*s0*s1 + l2*(c0*c2*s3 - s0*(c1*s2*s3 - s1*c3));
}


void BodySchema::ElbowPosition(CVector4_t angle, CVector3_t out){
 float c0,c1,s0,s1;

  c0=cos(angle[0]); s0=sin(angle[0]);
  c1=cos(angle[1]); s1=sin(angle[1]);

  out[0] = -(l1*c1);
  out[1] = -l1*c0*s1;
  out[2] =  l1*s0*s1;
}

int BodySchema::InvKinematics (CVector3_t relTar, float alpha,float tol, CVector4_t out){
  RawInverseKinematics(relTar,alpha,out);
  if(CheckInverseKinematics(out,relTar,tol)){
    return AnglesInRange(out);
  }
  else{
    return 0;
  }
}

int BodySchema::CheckInverseKinematics(CVector4_t angles,CVector3_t target, float tol){
  CVector3_t cart,diff;
  BodySchema::Angle2Cart(angles,cart);
  v_sub(target,cart,diff);
  return v_squ_length(diff)<=tol*tol;
}

void BodySchema::RawInverseKinematics(CVector3_t relTar, float alpha,CVector4_t out){
  CVector3_t relTarNor,tmpc,a,tmp,u,v,tmp2,tmp3,e;
  float d,r,beta;
  float *sfe,*saa,*shr,*eb;
  //aliases
  sfe = out;
  saa = out+1;
  shr = out+2;
  eb  = out+3;

  d= v_length(relTar);

  (*eb) = acos((-d*d+l1*l1+l2*l2)/(2*l1*l2));
  beta = asin(l2/d*sin((*eb)));
  (*eb) = abs(pi - (*eb)); // because eb>0
  v_normalize(relTar,relTarNor);
  v_scale(relTarNor,l1*cos(beta),tmpc);
  r = l1*sin(beta);


  // not sure
  /* th = atan(-relTar[2]/relTar[0]);// y/x in flo. coordinates
  phi = asin(relTar[1]/d); // z in florents coordinates
  m_rotation_y2(th,rth);
  m_rotation_z2(phi,rphi);
  m_multiply(rphi,rth,tmp4);
  v_transform(af,tmp4);
  // hum hum
  */	     
  
  v_set(0,-1,0,a);
  // u=a-(a*n')*n;
  v_scale(relTarNor,v_dot(a,relTarNor),tmp);
  v_sub(a,tmp,tmp2);

  v_normalize(tmp2,u);
  v_cross(relTarNor,u,v);
  
  // e = c+r*(cos(al)*u+sin(al)*v);
  v_scale(v,sin(alpha),tmp);
  v_scale(u,cos(alpha),tmp2);
  v_add(tmp,tmp2,tmp3);
  v_scale(tmp3,r,tmp);
  v_add(tmpc,tmp,e);

  if (e[2]>0){ // saa > 0
    (*saa) = acos(-e[0]/l1);
  }
  else{ // saa < 0
    (*saa) = -acos(-e[0]/l1);
  }
  float ssaa = sin((*saa));

  if(abs(ssaa)<MIN_EB){
    (*sfe) = atan2(-e[2],e[1])-alpha/2; // to be checked
 //    if((*sfe)<0){
//       (*sfe) += pi;
//     }
    (*shr) = +alpha/2;  //care for continuity
  
  }
  else{
    (*sfe) = acos(-sign(e[2])*e[1]/(l1*ssaa)); // a examiner
   float seb = sin((*eb));
   if (abs(seb)<MIN_EB){
       (*shr) = +alpha;
   }
   else{    
       (*shr) = asin((-relTar[0]-l1*cos((*saa)) - l2*cos((*saa))*cos((*eb)))
		  /(l2*ssaa*seb));
   }
  }
}


void BodySchema::Jacobian(CVector4_t v, CMatrix4_t jac){
 float c0,c1,c2,c3,s0,s1,s2,s3;
  c0 = (float)cos(v[0]); s0 = (float)sin(v[0]);
  c1 = (float)cos(v[1]); s1 = (float)sin(v[1]);
  c2 = (float)cos(v[2]); s2 = (float)sin(v[2]);
  c3 = (float)cos(v[3]); s3 = (float)sin(v[3]);
  // correct matrix index
  jac[0] = 0;
  jac[4]=l1*s1-l2*(c1*s2*s3-s1*c3);
  jac[8]=-(l2*s1*c2*s3);
  jac[12]=-l2*(-c1*s3+s1*s2*c3)  ;
  jac[2]=l1*c0*s1-l2*(c0*(c1*s2*s3-s1*c3)+s0*c2*s3);
  jac[6]=(l1*c1+l2*(c1*c3+s1*s2*s3))*s0;
  jac[10]=-l2*(c0*s2+s0*c1*c2)*s3;
  jac[14]=l2*(c0*c2*c3-s0*(c1*s2*c3+s1*s3));
  jac[1]=l1*s0*s1+l2*(c0*c2*s3-s0*c1*s2*s3+s0*s1*c3);
  jac[5]=-(l1*c1+l2*(c1*c3+s1*s2*s3))*c0;
  jac[9]=l2*(c0*c1*c2-s0*s2)*s3;
  jac[13]=l2*(c0*(c1*s2*c3+s1*s3)+s0*c2*c3);
  jac[3]=0.0f;
  jac[7]=0.0f;
  jac[11]=0.0f;
  jac[15]=0.0f;
}

void BodySchema::IntermediateJacobian(int link, float len,CVector4_t v, CMatrix4_t jac){
  
  float c0,c1,c2,c3,s0,s1,s2,s3;
  float nl1,nl2;
  c0 = (float)cos(v[0]); s0 = (float)sin(v[0]);
  c1 = (float)cos(v[1]); s1 = (float)sin(v[1]);
  c2 =0;s2=0;c3=0;s3=0;
  switch (link){
  case 1:
    nl2 = 0;
    nl1 = len*l1;
    break;
  case 2:
    nl1 = l1;
    nl2 = len*l2;
    c2 = (float)cos(v[2]); s2 = (float)sin(v[2]);
    c3 = (float)cos(v[3]); s3 = (float)sin(v[3]);   
    break;
  default:
    cout <<"bad link number"<<endl;
    return;
  }
    
  // CORRECT_MATRIX_INDEX
  
  jac[0] = 0;
  jac[4]=nl1*s1-nl2*(c1*s2*s3-s1*c3);
  jac[8]=-(nl2*s1*c2*s3);
  jac[12]=-nl2*(-c1*s3+s1*s2*c3)  ;
  jac[2]=nl1*c0*s1-nl2*(c0*(c1*s2*s3-s1*c3)+s0*c2*s3);
  jac[6]=(nl1*c1+nl2*(c1*c3+s1*s2*s3))*s0;
  jac[10]=-nl2*(c0*s2+s0*c1*c2)*s3;
  jac[14]=nl2*(c0*c2*c3-s0*(c1*s2*c3+s1*s3));
  jac[1]=nl1*s0*s1+nl2*(c0*c2*s3-s0*c1*s2*s3+s0*s1*c3);
  jac[5]=-(nl1*c1+nl2*(c1*c3+s1*s2*s3))*c0;
  jac[9]=nl2*(c0*c1*c2-s0*s2)*s3;
  jac[13]=nl2*(c0*(c1*s2*c3+s1*s3)+s0*c2*c3);
  jac[3]=0.0f;
  jac[7]=0.0f;
  jac[11]=0.0f;
  jac[15]=1.0f;
}

// later one could do self-collision tests
int BodySchema::AcceptablePosition(CVector4_t angle){
  return AnglesInRange(angle);
}

void BodySchema::RandomAnglePos(CVector4_t angle){
 float rn;
  for (int i=0;i<4;i++){
    rn = abs(rand()/((float)(RAND_MAX+1)));
    angle[i] = min_angle[i]+(max_angle[i]-min_angle[i])*rn; 
  }
}


void BodySchema::RandomCartPos(CVector3_t cart){
  CVector4_t angle;
  RandomAnglePos(angle);
  Angle2Cart(angle,cart);
}

void  BodySchema::SetAnglesInRange(CVector4_t angle){
   for(int i=0;i<4;i++){ // checking joint limits
    angle[i] = max(min_angle[i],min(max_angle[i],angle[i]));
  }
}

int BodySchema::AnglesInRange(CVector4_t angle){
   for(int i=0;i<4;i++){ // checking joint limits
     if(angle[i] < min_angle[i] || angle[i] > max_angle[i])
       return 0;
   }
   return 1;
}


void BodySchema::GetAnglesLowerBound(CVector4_t lb){
  v4_copy(min_angle,lb);
}

void BodySchema::GetAnglesUpperBound(CVector4_t ub){
 v4_copy(max_angle,ub);
}


ostream& operator<<(ostream& out, const BodySchema& bs){
  out<<"body schema:";
  return out;
}
istream& operator>>(istream& in,  BodySchema& bs){
  char line[80];
  in.getline(line,80);
  return in;
}
