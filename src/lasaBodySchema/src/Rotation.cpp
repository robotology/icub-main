// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   micha.hersch@robotcub.org
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
#include "Rotation.h"


//#define RODRIGUES

Rotation::Rotation(){
  base_eps_b = eps_b =0.005;
  base_eps_a = eps_a =0.005;
  alpha = 1;
  norm = 0;
  v_clear(beta);
  v_clear(axis);
  axis_flag=0;
}

Rotation::~Rotation(){}

void Rotation::Copy(const Rotation& r){
  r.GetRotationParam(beta);
  alpha = GetAlpha();
  SetAxis();
}

int Rotation::GetRotationAxis(CVector3_t out) const
{
  if(AxisOk()){
    v_copy(axis,out);
    return 1;
  }
  else{
    int ret = SetAxis();
    if(ret){
      v_copy(axis,out);
    }
    return ret;
  }
}



float Rotation::GetRotationAngle() const
{
  float tmp;
  int si;
  tmp = v_length(beta);
  if(tmp>1){
    cout<<"error: beta is bigger than one"<<endl;
    return 0.0;
  }
  else{
    if(!AxisOk()){
      cout<<"error: axis direction is not defined"<<endl;
    }
    si = sign(v_dot(beta,axis));
    return si*2.0 *(float)asin(tmp);
  }
}

/**
 * @return 1 if new_beta is acceptable (norm<=1), 0 otherwise
 */
int Rotation::SetTransfo(CVector3_t new_beta){
  float sq_len = v_squ_length(new_beta);
  if(sq_len>1){
    return 0;
  }
  //  norm = sq_len;
  alpha = sqrt(1-sq_len);
  v_copy(new_beta,beta);
  axis_flag =0;
  return 1;
}

/**
 * sets the rotation according to the quaternion
 */
void Rotation::SetQuaternion(CQuat_t q){
  v_copy(q,beta);
  alpha = q[3];
  axis_flag =0;
}


void Rotation::TransformRotation(const CVector3_t beta_in, CVector3_t beta_out){
    CQuat_t q1,q2,q3;
    q_complete(beta,q1);
    q_complete(beta_in,q2);
    q_multiply(q2,q1,q3);
    v_copy(q3,beta_out);
}


/**
 * @brief updates the rotation angle in order to align as much as possible two
 * vectors. This the updating step of the CCD inverse kinematics algorithm.
 * The resulting rotation (around a fixed axis) will rotate <with> in order to align it with <tar>
 * @param tar target direction of alignment (in an external frame of ref)
 * @param with direction to align to the target (in a frame of ref attached to the
 * zero position of the rotation)
 */



float Rotation::AimingAngle(const CVector3_t tar,const CVector3_t with){
  CVector3_t vec;
  float f1,f2,f3,f4,angle,c;
  if(!AxisOk()){
    SetAxis();
  }
  v_cross(axis,with,vec);
  f1= v_dot(vec,tar);
  f2 = v_dot(with,axis);
  f3 = v_dot(tar,axis);
  f4 = v_dot(tar,with);
  f2 = f4-f2*f3;
  angle = atan2(f1,f2);
  c= cos(angle);
  if(c*f2+(f1*f1)/f2<0){ //checking it is a minimization
    angle = pi+angle;
  }
  return angle>pi?angle-2*pi:angle; //putting back to [-pi pi]
}

/**
 * @brief This function is the equivalent of AimAt for rotations 
 * @todo to test
 */
void Rotation::MinimizeRotation(Rotation& rot){
  CQuat_t b1;
  rot.GetQuaternion(b1);
  MinimizeRotation(b1);
}

/**
 * @todo finish testing - check that it is not the max that comes out
 */
void Rotation::MinimizeRotation(const CQuat_t q){
  float num,den,a1,angle;
  CVector3_t v,b1;//rotation param of rot
 if(!AxisOk()){
    SetAxis();
  }
  v_copy(q,b1);
  a1 = q[3];
  //  a1 =sqrt(1-v_dot(b1,b1));;
  num=-2*a1*v_dot(axis,b1);
  v_cross(b1,beta,v);//it should be axis instead of beta
  den = 2*a1*a1-1+v_squ_length(v);
  angle = atan2(num,den);
 
//  s= sin(angle/2);
//  c= sin(angle/2);
//   if(-2*(s/c)*num+(2*s/c-s/(c*c*c))*den < 0 && 0){ //checking it is a minimaization
//     angle = pi+angle;
//     cout<<"+";
//   }
//   cout<<" angle "<<angle*rad2deg<<endl;

  SetAngle(angle);
}

//#define FF

float Rotation::MinimizePositionAndRotationAngle(const CVector3_t tar,const CVector3_t with, const CQuat_t q,float k){
  float num_rot,den_rot,num_pos,den_pos,angle,num,den,c,s;
  CVector3_t vec;
  if(!AxisOk()){
    SetAxis();
  }
  //rotation



#ifndef FF
  v_cross(q,axis,vec);// v_cross(q,beta,vec);
  num_rot=0.5*q[3]*v_dot(axis,q);//-2
  den_rot = -0.25*(2*q[3]*q[3]-1+v_squ_length(vec));
  
  //position
  v_cross(axis,with,vec); 
  num_pos= -v_dot(vec,tar);
  den_pos =-v_dot(tar,with) +  v_dot(tar,axis) * v_dot(with,axis);
  
#else
  v_cross(q,axis,vec);// v_cross(q,beta,vec);
  num_rot=-1*q[3]*v_dot(axis,q);//-2
  den_rot = 0.5*(2*q[3]*q[3]-1+v_squ_length(vec));
  
  //position
  v_cross(axis,with,vec); 
  num_pos= v_dot(vec,tar);
  den_pos =v_dot(tar,with) -  v_dot(tar,axis) * v_dot(with,axis);

#endif


 
  // 

  num =k*num_pos+(1-k)*num_rot;
  den = k*den_pos+(1-k)*den_rot;
  

angle = atan2(num,den); 
 
  // check that it's a minimization
   s =sin(angle);
   c= cos(angle);
   //  if(c*den+(num*num)/den<0){
   
   if(-s*num-c*den<0){
#ifndef FF     
     angle += pi;
#endif
     //     cout<<"not ok ";
   }
   //   cout<<0.001*((int)(1000*(angle-GetAngle()))%int(2000*pi))<<" ";
  return angle>pi?angle-2*pi:angle; //putting back to [-pi pi]

}

void Rotation::SetRate(float rate){
  base_eps_b = eps_b = rate;
}

void Rotation::ScaleRate(float factor){
  eps_b = factor*base_eps_b;
}


void Rotation::Derivative(CMatrix3_t out)const{
  float f1;
  CMatrix3_t m1,m2;

  f1= v_dot(beta,beta);//beta'*beta
  m_identity(m1);//I
  m_rescale(1-2*f1,m1,m1);// (1-2beta'*beta)*I
  ComputeMatrixB(beta,m2);//beta|
  m_rescale(2*sqrt(1-f1),m2,m2); // 2*sqrt(1-beta'*beta)beta|
  m_sub(m1,m2,m2);// (1-2beta'*beta)*I- 2*sqrt(1-beta'*beta)beta|
  v_mult(beta,beta,m1);
  m_rescale(2,m1,m1);
  m_add(m2,m1,out);//(1-2beta'*beta)*I- 2*sqrt(1-beta'*beta)beta|+2beta*beta'
}


void Rotation::AngleDerivative(CVector3_t v, CVector3_t out){
  CVector3_t v1,v2,v3;
  float f;               // b = axis
  CheckAxis();
  v_scale(v,-2*norm*alpha,v1);//-4k*v
  f=(1-2*norm*norm);//(1-2*k^2)
  v_cross(axis,v,v2); //bxv
  v_scale(v2,f,v2); //2*(1-2*k^2)/sqrt(1-k^2)*(b x v)
  v_add(v1,v2,v3);;//-4k*v + (1-2*k^2)/sqrt(1-k^2)*(b x v)
  f=2*alpha*norm*v_dot(axis,v);//2*alpha*k*b'*v
  v_scale(axis,f,v1);
  v_add(v3,v1,out); //-4k*v+2*(1-2*k^2)/sqrt(1-k^2)*(b x v)+ 4*k*(b'*v)*b


}


void Rotation::NormDerivative(CVector3_t v, CVector3_t out){
  CVector3_t v1,v2,v3;
  float f;               // b = axis
  CheckAxis();
  if(alpha>epsilon){
    v_scale(v,-4*norm,v1);//-4k*v
    f= 2*(1-2*norm*norm)/alpha;//2*(1-2*k^2)/sqrt(1-k^2)
    v_cross(axis,v,v2); //bxv
    v_scale(v2,f,v2); //2*(1-2*k^2)/sqrt(1-k^2)*(b x v)
    v_add(v1,v2,v3);;//-4k*v + (1-2*k^2)/sqrt(1-k^2)*(b x v)
    f=4*norm*v_dot(axis,v);//4*k*b'*v
    v_scale(axis,f,v1);
    v_add(v3,v1,out); //-4k*v+2*(1-2*k^2)/sqrt(1-k^2)*(b x v)+ 4*k*(b'*v)*b
  }
  else{ //alpha is zero
     v_cross(axis,v,v2);
     v_scale(v2,-1,out);
  }
  //  coutvec(out);
}

/**
 *@brief dR/dtheta
 */

void Rotation::Jacobian(CMatrix3_t out)const{
  CMatrix3_t m1,m2;
  float f;
  CheckAxis();
  m_identity(out);
  ComputeMatrixB(axis,m1);
  f=-(1-2*norm*norm);// - I think because computeMatrixB yields axis|
  m_rescale(f,m1,m1);
  f=2*norm*alpha;
  m_rescale(f,out,m2);
  m_sub(m1,m2,m1);
  v_mult(axis,axis,m2);
  m_rescale(f,m2,m2);
  m_add(m1,m2,out);
}

/**
 * @todo check that it is f = -2*(1-2*norm*norm)/alpha and not f=-1*(..
 */
void Rotation::InverseNormDerivative(CVector3_t v, CVector3_t out){
  CVector3_t v1,v2,v3;
  float f;               // b = axis
  if(!AxisOk()){
    SetAxis();
  }
 if(alpha>epsilon){
    v_scale(v,-4*norm,v1);//-4k*v
    f= -2*(1-2*norm*norm)/alpha;//-2*(1-2*k^2)/sqrt(1-k^2) only the sine part is inverted
    v_cross(axis,v,v2); //bxv
    v_scale(v2,f,v2); //2*(1-2*k^2)/sqrt(1-k^2)*(b x v)
    v_add(v1,v2,v3);;//-4k*v + (1-2*k^2)/sqrt(1-k^2)*(b x v)
    f=4*norm*v_dot(axis,v);//4*k*b'*v
    v_scale(axis,f,v1);
    v_add(v3,v1,out); //-4k*v-(1-2*k^2)/sqrt(1-k^2)*(b x v)+ 4*k*(b'*v)*b
  }
  else{ //alpha is zero
     v_cross(axis,v,v2);
     v_scale(v2,-1,out);
  }
}


void Rotation::BetaDerivative(CVector3_t v, CMatrix3_t out)const{
  CMatrix3_t B,C,m1; 
  CVector3_t v1;
  ComputeMatrixC(v,C);
  ComputeMatrixB(v,B);
//   cout<<"C: "<<C<<endl;
//   cout<<"B: "<<B<<endl;
  v_cross(beta,v,v1);//beta x v
  v_mult(v1,beta,m1);//(beta x v)*beta'
  if(alpha>epsilon){
    m_rescale(-2.0/alpha,m1,m1);//2/alpha * (beta x v)*beta'
    m_rescale(2*alpha,B,B);// 2*alpha*B
    m_add(m1,B,m1);// 
    m_sub(m1,C,out); 
  }
  else{//only that component
    m_rescale(-1,m1,out);
  }
}




void Rotation::AxisDerivative(CVector3_t v, CMatrix3_t out){
  CMatrix3_t B,m1,m2;
  float f;
  if(!AxisOk()){
    SetAxis();
  }
  ComputeMatrixB(v,B);
  m_rescale(2*norm*alpha,B,B); //2*k*sqrt(1-k²)*B
  v_mult(axis,v,m1); //b*v'
  f=v_dot(axis,v); //b'*v
  m_identity(m2);   
  m_rescale(f,m2,m2);//b'*v*I
  m_add(m1,m2,m1);// b*v+b'v*I
  m_rescale(2*norm*norm,m1,m1);//2k²*( b*v'+b'v*I )
  m_add(B,m1,out); //2*k*sqrt(1-k²)*B + 2k²*( b*v'+b'v*I )
}

/**
 * derivative of the inverse transfo
 */
void Rotation::InverseAxisDerivative(CVector3_t v, CMatrix3_t out){
  CMatrix3_t B,m1,m2;
  float f;
  if(!AxisOk()){
    SetAxis();
  }
  ComputeMatrixB(v,B);
  m_rescale(-2*norm*alpha,B,B); //-2*k*sqrt(1-k²)*B
  v_mult(axis,v,m1); //b*v'
  f=v_dot(axis,v); //b'*v
  m_identity(m2);   
  m_rescale(f,m2,m2);//b'*v*I
  m_add(m1,m2,m1);// b*v+b'v*I
  m_rescale(2*norm*norm,m1,m1);//2k²*( b*v'+b'v*I )
  m_add(B,m1,out); //-2*k*sqrt(1-k²)*B + 2k²*( b*v'+b'v*I )
}


void Rotation::AngleAxisDerivative(CVector3_t v, CMatrix3_t out){
  CMatrix3_t B,m1,m2;
  float f;
  if(!AxisOk()){
    SetAxis();
  }
  ComputeMatrixB(v,B);
  m_rescale(1-2*norm*norm,B,B); //1-2*k²*B = (cos(theta)B
  v_mult(axis,v,m1); //b*v'
  f=v_dot(axis,v); //b'*v
  m_identity(m2);   
  m_rescale(f,m2,m2);//b'*v*I
  m_add(m1,m2,m1);// b*v+b'v*I
  m_rescale(2*alpha*norm,m1,m1);//2k*alpha*( b*v'+b'v*I ) = sin(theta)*(...)
  m_add(B,m1,out); //-2*k*sqrt(1-k²)*B + 2k²*( b*v'+b'v*I )
}

void Rotation::AddToAxis(CVector3_t daxis){
  float l,f;
  if(!AxisOk()){
    SetAxis();
  }
  l = v_normalize(daxis,daxis);
  f = min(l*eps_a,pi/180);
  v_scale(daxis,f,daxis);
  v_add(axis,daxis,axis);
  if(v_normalize(axis,axis)<epsilon){
    cout<<"axis undefined"<<endl;
  }
  v_scale(axis,norm,beta);
}


void Rotation::AddToNorm(float dnorm){
  //  cout<<" "<<dnorm;
  norm = norm+dnorm;

  while(abs(norm)>(1+1e-2)){
    dnorm *= 0.5;
    norm -=dnorm;
  }

  if(abs(norm)>1 && abs(norm)<(1+1e-2)){
    norm = norm-sign(norm)*2;
  }
  if(!AxisOk()){
    SetAxis();
  }

  v_scale(axis,norm,beta);
  alpha = sqrt(1-norm*norm);
}

void Rotation::Add(CVector3_t dbeta){
  float dbeta_len,beta_len,f;
  dbeta_len = v_normalize(dbeta,dbeta);
  f =  min(dbeta_len*eps_b,pi/180);
  v_scale(dbeta,f,dbeta);
  //  v_scale(dbeta,eps_b,dbeta);
  v_add(beta,dbeta,beta);//v_add
  cout<<"beta ";coutvec(beta);
  beta_len = v_length(beta);
  if(beta_len>1 && beta_len<(1+1e-2)){  //closing the manifold
    v_scale(beta,beta_len-2,beta);
    cout<<"cross"<<endl;
  }
  while(v_length(beta)>(1+1e-2)){
    v_scale(dbeta,0.5,dbeta);
    v_sub(beta,dbeta,beta);   
  }
  SetTransfo(beta);
}

void Rotation::Update(CVector3_t v, CVector3_t v_tr){
  // PD = -2/sqrt(1-p_beta'*p_beta) * cross(p_beta,x)*p_beta' +...
  //         2*sqrt(1-p_beta'*p_beta)*B-C;
  CVector3_t y,v1,dbeta;
  CMatrix3_t B,C,m1; 
  float beta_len,dbeta_len;
  //BETA_DERIVATIVE  
  ComputeMatrixC(v,C);
  ComputeMatrixB(v,B);
//   cout<<"C: "<<C<<endl;
//   cout<<"B: "<<B<<endl;
  v_cross(beta,v,v1);//beta x v
  v_mult(v1,beta,m1);//(beta x v)*beta'
  if(alpha>1e-7){
    m_rescale(-2.0/alpha,m1,m1);//2/alpha * (beta x v)*beta'
    m_rescale(2*alpha,B,B);// 2*alpha*B
    m_add(m1,B,m1);// 
    m_sub(m1,C,m1); 
  }
  //

  Transform(v,y);
  v_sub(v_tr,y,v1); // (y-RvR)
  v_transform_normal2(v1,m1,dbeta);

  // UPDATE
  // fixed increment size
  dbeta_len = v_length(dbeta);
  v_scale(dbeta,eps_b/dbeta_len,dbeta);
  //  v_scale(dbeta,eps_b,dbeta);
  v_add(beta,dbeta,beta);//v_add
  beta_len = v_length(beta);
  if(beta_len>1 && beta_len<(1+1e-2)){  //closing the manifold
    v_scale(beta,beta_len-2,beta);
    cout<<"cross"<<endl;
  }
  while(v_length(beta)>(1+1e-2)){
    v_scale(dbeta,0.5,dbeta);
    v_sub(beta,dbeta,beta);   
  }
//   while((v_length(beta)>(1+1e-7))){ //avoids beta>1
//     // cout<<(v_length(beta)>1+1e-10)<<endl;
//     v_scale(dbeta,0.5,dbeta);
//     v_sub(beta,dbeta,beta);//v_sub
//   }
  alpha = sqrt(1-v_squ_length(beta));
} 




#ifndef OLD_UPDATE_AXIS

/**
 * @brief updates the rotation axis of a rotation
 * @todo check compatiblity iwth the method in KinematicChain.cpp
 */

void Rotation::UpdateAxis(CVector3_t v, CVector3_t v_tr, float angle){
  CMatrix3_t B,m1,m2;
  CVector3_t v1,y,db;
  float f;//,d_phi,d_psi;



 SetAngle(angle);
  if(!AxisOk()){
    SetAxis();
  }


 // computing d/daxis of euclidean distance (y-RvR)²
 ComputeMatrixB(v,B);
 m_rescale(2*norm*alpha,B,B); //2*k*sqrt(1-k²)*B
 v_mult(axis,v,m1); //b*v'
 f=v_dot(axis,v); //b'*v
 m_identity(m2);   
 m_rescale(f,m2,m2);//b'*v*I
 m_add(m1,m2,m1);// b*v+b'v*I
 m_rescale(2*norm*norm,m1,m1);//2k²*( b*v'+b'v*I )
 m_add(B,m1,m1); //2*k*sqrt(1-k²)*B + 2k²*( b*v'+b'v*I )

 Transform(v,y);
//  cout<<"from " ;coutvec(v);
//  cout<<"to " ;coutvec(y);
//  cout<<"instead of " ;coutvec(v_tr);
  v_sub(v_tr,y,v1); // (y-RvR)
//  cout<<"diff "<<v1<<endl;
//  // v_normalize(v1,v1);//should not have an influence
  v_transform_normal2(v1,m1,db);
  f = v_length(db);
  f=f>pi/180?eps_a/f:eps_a;
  cout<<"norm "<<f<<endl;
  v_scale(db,f,db);
  v_add(axis,db,axis);

  f = v_normalize(axis,axis);
  axis_flag = 1;
  SetAngle(angle);
    
//  cout<<"db ";coutvec(db);
//  v_set(-sin(phi)*cos(psi), cos(phi)*cos(psi),0,v1);//db/dphi
//  v_set(-cos(phi)*sin(psi),-sin(phi)*sin(psi),cos(psi),v2);//db/dpsi
 
//  d_phi = v_dot(v1,db);
//  d_psi = v_dot(v2,db);
 
//  cout<<"d_phi "<<d_phi<<" d_psi "<<d_psi<<endl;
//  while((f=d_phi*d_phi+d_psi*d_psi)>eps_a){
//    d_phi /=2.0f;
//    d_psi /=2.0f;
//  }
 
//  phi += d_phi;
//  psi += d_psi;
//  v_set(cos(phi)*cos(psi),sin(phi)*cos(psi),sin(psi),axis);
}

#else

#ifndef OLD_UPDATE_AXIS2
void Rotation::UpdateAxis(CVector3_t v, CVector3_t v_tr, float angle){
  CVector3_t diff,v1,v2,c;
  CVector4_t q1;
  float f,rn,d,d1,d2,l; 
  //  CMatrix3_t A,B,C;
  cout<<"v ";coutvec(v);
  cout<<"v_tr ";coutvec(v_tr);
  v_sub(v_tr,v,diff);
  v_scale(diff,0.5,diff);
  v_add(v,diff,c); //center of circle
  if((l=v_normalize(diff,diff))){
    cout<<"l "<<l<<endl;
    if(angle<0){v_scale(diff,-1,diff);}
    rn = l/tan(angle/2); //radius of circle
    if(rn>epsilon){

//       f = v_dot(c,axis);
//       v_scale(axis,f,r);
//       v_sub(r,c,r);//(b'*c)b-c -(c minus its projection on present axis)
//       v_normalize(r,r);
//       v_cross(r,axis,n); //pre-image of diff
//       cout<<"n ";coutvec(n);
//       cout<<"diff ";coutvec(diff);

      d= v_normalize(c,v1); // local 2d base vector 1
      
      d1 = rn*rn/d;
      //d1 = (d*d-rn*rn)/d; // height of top of cerf-volant
      //d2 = rn*sqrt(d*d-rn*rn)/d; // heigth of rectangular triangle
      f = (d*d)-(rn*rn);
      d2 = f>=0?sqrt(f)*(rn/d):0;
      v_cross(v1,diff,v2);//local 2d base vector 2
//       cout<<"d  "<<d<<" d1 "<<d1<<" d2 "<<d2<<endl;
//       cout<<"v2 ";coutvec(v2);
     //  v_scale(v1,d1,v1);
//       v_sub(c,v1,c1);
//       v_scale(v2,d2,v1);
//       v_add(c1,v1,c);// rotation point on the axis
//         cout<<"rot point 1 ";coutvec(c);
//       if(v_length(c)<0.001*epsilon){//0  the rotation point
// 	v_copy(v2,c);
//       }
//        cout<<"rot point ";coutvec(c);
//       v_normalize(c,c);// new (desired) axis
//       cout<<"des axis ";coutvec(c);
      v_scale(v1,d1,v1);
      v_scale(v2,d2,v2);
      v_add(v1,v2,v1);
      v_cross(v1,diff,c);
      v_normalize(c,c);// new (desired) axis
      cout<<"des axis ";coutvec(c);
      v_cross(axis,c,v1);
      f = v_dot(axis,c);
      if(abs(f-1)>epsilon){
	f = acos(f);
	q_set(v1,eps_a*f,q1); // we only take a small rotation
	q_v_mult(q1,axis,v1); //rotating the axis
	v_normalize(v1,axis); // v_copy should theoretically be enough
	v_scale(axis,norm,beta);
      }
   //    m_set_v3_column(r,0,A);
//       m_set_v3_column(axis,1,A);
//       m_set_v3_column(n,2,A);

//       v_cross(diff,c,r); //can be taken somewhere else
// ;
//       m_set_v3_column(r,0,B);
//       m_set_v3_column(c,1,B);
//       m_set_v3_column(diff,2,B);

//       m_inverse(A,C);//A^-1
//       m_multiply(B,C,A);//R=B*Ai rotation matrix
      
//       f= m_rotation_axis(A,v1);
    }
  }
}




#else
/**
 * @brief updated the rotation axis. Does not use the rotation angle value.
 * only its sign.
 */
void Rotation::UpdateAxis(CVector3_t v, CVector3_t v_tr, float angle){
  CVector3_t dax,diff,c,r,n;
  float f;

  if(!AxisOk()){
    SetAxis();
  }
  v_sub(v_tr,v,diff);
  v_scale(diff,2,diff);
  v_add(v,diff,c); //center 
  if(v_normalize(diff,diff)){
    if(angle<0){v_scale(diff,-1,diff);}
    f = v_dot(c,axis);
    v_scale(axis,f,r);
    v_sub(r,c,r);//(b'*c)b-c 
    v_cross(axis,r,n);
    // v_cross(r,axis,n);
    v_normalize(n,n);
    v_cross(n,diff,c);//rotation axis for axis
    //   if(v_squ_length(c)>1e-6)
    v_copy(axis,n);//backup;
    v_scale(c,eps_a,beta);
    alpha = sqrt(1-v_dot(beta,beta));
    Rotation::Transform(n,axis);
//     v_cross(c,axis,dax);
//     v_scale(dax,eps_a,dax);
//     v_add(axis,dax,axis);
    v_normalize(axis,axis);
    v_scale(axis,norm,beta);
  }
}
#endif

#endif






int Rotation::SetAngle(float angle){
  if(angle>pi){
    return SetAngle(angle-2*pi)&& 0;
  }
  if(angle<=-pi){
    return SetAngle(angle+2*pi)&& 0;
  }
 //setting the rotation amplitude
  if(!AxisOk()){ //do this before modifying norm
   SetAxis();
 }

 norm = sin(angle/2.0f);
 alpha = cos(angle/2.0f);
 float n=norm;
// n = norm/v_length(axis);//axis should already be scaled 
//  assert(!isnan(n));
 v_scale(axis,n,beta);

 return 1;
}


int Rotation::SetRotationParam(CVector3_t nbeta){
  float f = v_squ_length(nbeta);
  if(f>1){
    return 0;
  }
  else{
    v_copy(nbeta,beta);  
    alpha = sqrt(1-f);
    axis_flag=0;
    return 1;
  }

}


int Rotation::SetRotationAxis(const CVector3_t a){
  if(v_normalize(a,axis)>epsilon){
    //    psi=asin(axis[2]);
    //phi=atan2(axis[1],axis[0]);
    v_scale(axis,norm,beta);
    axis_flag=1;
    return 1;
  }
  else{
    axis_flag=0;
    return 0;
  }
}

void Rotation::RandomAxis(){
  CVector3_t v;
  do{
    for(int i=0;i<3;i++){
      v[i] = sign(RND(1)-0.5)*RND(1);
    }
  } while(!SetRotationAxis(v));
}

void Rotation::ComputeMatrixC(const CVector3_t v, CMatrix3_t out)const{
  CMatrix3_t m1,m2;
  float f;
  f=v_dot(beta,v);
  v_mult(v,beta,m1);
  m_rescale(2.0,m1,m2);
  v_mult(beta,v,m1);
  m_sub(m2,m1,m1);
  m_identity(m2);
  m_rescale(f,m2,m2);
  m_sub(m1,m2,out);
  m_rescale(2,out,out); // new correct version
}


/**
 * constructs the matrix d/db (b x v)
 * @arg v the second factor of the product
 * @arg out the resulting matrix
 */  
void Rotation::ComputeMatrixB(const CVector3_t v, CMatrix3_t out)const{
  CVector3_t v1;
  //  cout<<"v: ";coutvec(v);
  v_set(0,-v[2],v[1],v1);
  m_set_v3_column(v1,0,out);

  v_set(v[2],0,-v[0],v1);
  m_set_v3_column(v1,1,out);

  v_set(-v[1],v[0],0,v1);
  m_set_v3_column(v1,2,out);
}



void Rotation::RotFromMatrix(const CMatrix3_t m){
  CVector3_t v;
  float angle = m_rotation_axis(m,v); //should be normalized already
  v_normalize(v,axis);
  norm = sin(angle/2);
  v_scale(axis,norm,beta);
  alpha = cos(angle/2);
  axis_flag=1;
}


//  p_alpha^2*x + 2*p_alpha*cross(p_beta,x) + ...
//         (p_beta'*x)*p_beta - cross(cross(p_beta,x),p_beta)+p_trans;
void Rotation::Transform(const CVector3_t in, CVector3_t out)const{
CVector3_t v1,v2,v3;
  float f1;
  f1 = alpha*alpha;
  v_scale(in,f1,v1); //alpha^2*v
  v_cross(beta,in,v2); //beta x v
  v_scale(v2,2*alpha,v3); // 2*alpha*(beta x v)
  v_add(v1,v3,v3); // alpha^2*v+2*alpha*(beta x v)
  f1 = v_dot(beta,in); //beta'*v
  v_scale(beta,f1,v1); //(beta'*v)*beta 
  v_add(v3,v1,v3);// alpha^2*v+2*alpha*(beta x v)+(beta'*v)*beta 
  v_cross(v2,beta,v1); //(beta x v) x beta
  v_sub(v3,v1,out);//alpha^2*v+2*alpha*(beta x v)+(beta'*v)*beta -(beta x v) x beta

}

// should be eqivalent to Derivative, also valid for homogeneous matrices
void Rotation::RotToMatrix(CMatrix3_t mat)const{
  m_identity(mat);
  Rotation::Transform(mat,mat);
  Rotation::Transform(mat+4, mat+4);
  Rotation::Transform(mat+8,mat+8);
}


void Rotation::InverseTransform(const CVector3_t in, CVector3_t out){
  v_scale(beta,-1,beta);     // inverting rotation sign
  Rotation::Transform(in,out);           // rotating back
  v_scale(beta,-1,beta);     // resetting rotation sign
}

Rotation& Rotation::operator*(const Rotation& r1){
  CQuat_t q1,q2,q3;
  r1.GetQuaternion(q1);
  GetQuaternion(q2);
  q_multiply(q1,q2,q3);
  SetQuaternion(q3);
  //  cout<<"q1 "<<q1[2]<<" q2 "<<q2[2]<<" q3 "<<q3[2]<<" beta "<<beta<<endl;
  return *this;
}



ostream& operator<<(ostream& out, const Rotation& rt)
{
  CVector3_t tmp;
  rt.GetRotationParam(tmp);
  out << rt.GetRotationAngle()<<" : "<<tmp[0]<<" "<<tmp[1]<<" "<<tmp[2];
  return out;
}


istream& operator>>(istream& in, Rotation& rt)
{
  CVector3_t b;
  float f,b1,b2,b3;
  char a1;
  //  char line[200];

//   in>>f>>a>>b[0]>>b[1]>>b[2]>>a>>t[0]>>t[1]>>t[2];
//   in.getline(line,200);
//   cout<<"hum "<<line<<endl;
//   sscanf(line,"%f : %f %f %f",&f,b,b+1,b+2);
//   cout<<"reading"<<endl;
//   coutvec(b);
  in>>f>>a1>>b1>>b2>>b3;
  v_set(b1,b2,b3,b);
  rt.SetTransfo(b);
  rt.SetAngle(f);
  return in;
}

//#define TEST_RIGID_TRANSFO
#ifdef TEST_RIGID_TRANSFO
// #include <iostream>
// #include <fstream>
int main(int argc, char *argv[]){
  CVector3_t v1,v2;
  ofstream mout("bf");
  Rotation rt,rt2;
  v_set(0.1,0.2,0.2,v1);
  v_set(0.5,0.4,0.3,v2);
  rt.SetTransfo(v1,v2);
  mout<<rt;
  mout.close();
  ifstream mmin("outrt.txt");
  mmin>>rt2;
  mmin.close();
  cout<<rt2;
  return 1;
}
#endif
  
  



