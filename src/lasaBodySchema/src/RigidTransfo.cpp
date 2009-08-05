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
#include "RigidTransfo.h"

//  RigidTransfo::RigidTransfo(): Rotation(), Translation(){
//    adaptive = 1;
//  }


#ifdef WRONG_UPDATE
//wrong
void RigidTransfo::Update(CVector3_t v, CVector3_t v_tr){
  CVector3_t v1,v2;
  Translation::InverseTransform(v_tr,v1);
  Rotation::Transform(v,v2);
  Translation::Update(v2,v_tr);
  Rotation::Update(v,v1);
}

void RigidTransfo::UpdateAxisAndTranslation(CVector3_t v, CVector3_t v_tr,
					    float angle){
  // CVector3_t v1,v2;
  //SetAngle(angle);
  // Translation::InverseTransform(v_tr,v1);
  // Rotation::Transform(v,v2);
  Translation::Update(v,v_tr);
  Rotation::UpdateAxis(v,v_tr,angle);
 
}


//probably wrong
void RigidTransfo::UpdateAxis(CVector3_t v, CVector3_t v_tr,float angle){
  CVector3_t v1;
  Translation::InverseTransform(v_tr,v1);
  Rotation::UpdateAxis(v,v1,angle);
}

void RigidTransfo::UpdateTranslation(CVector3_t v, CVector3_t v_tr){
  CVector3_t v1;  
  Rotation::Transform(v,v1);
  Translation::Update(v1,v_tr);
}

void RigidTransfo::UpdateTranslation(CVector3_t v, CVector3_t v_tr,float angle){
  SetAngle(angle);
  UpdateTranslation(v,v_tr);
}


void RigidTransfo::UpdateRotation(CVector3_t v, CVector3_t v_tr){
  CVector3_t v1;  
  Translation::InverseTransform(v_tr,v1);
  Rotation::Update(v,v1);
}

#else

float RigidTransfo::Update(CVector3_t v, CVector3_t v_tr){
  // PD = -2/sqrt(1-p_beta'*p_beta) * cross(p_beta,x)*p_beta' +...
  //         2*sqrt(1-p_beta'*p_beta)*B-C;
  CVector3_t y,v1,dbeta,dtrans;
  CMatrix3_t B,C,m1; 
  float beta_len,dbeta_len;
  
  ComputeMatrixC(v,C);
  ComputeMatrixB(v,B);
  //  cout<<"C: "<<C<<endl;
  //  cout<<"B: "<<B<<endl;
  v_cross(beta,v,v1);//beta x v
  v_mult(v1,beta,m1);//(beta x v)*beta'
  if(alpha>epsilon){
    m_rescale(-2.0/alpha,m1,m1);//2/alpha * (beta x v)*beta'
    m_rescale(2*alpha,B,B);// 2*alpha*B
    m_add(m1,B,m1);// 
    m_sub(m1,C,m1); 
  }
  // cout<<"m1: "<<m1<<endl;
  Transform(v,y);
  v_sub(v_tr,y,v1); // (y-RvR)
  //  cout<<"v1 ";coutvec(v1);
  v_transform_normal2(v1,m1,dbeta);
  // cout<<"db ";coutvec(dbeta);
  // fixed increment size
  dbeta_len = v_length(dbeta);
  if(abs(dbeta_len)>epsilon){
    v_scale(dbeta,eps_b/dbeta_len,dbeta);
  }
  //  v_scale(dbeta,eps_b,dbeta);
  v_add(beta,dbeta,beta);//v_add

 
  while(v_length(beta)>(1+1e-2)){
    v_scale(dbeta,0.5,dbeta);
    v_sub(beta,dbeta,beta);   
  }

  beta_len = v_length(beta);
 if(beta_len>1 && beta_len<(1+1e-2)){  //closing the manifold
    v_scale(beta,beta_len-2,beta);
       cout<<"cross"<<endl;
  }

//   while((v_length(beta)>(1+1e-7))){ //avoids beta>1
//     // cout<<(v_length(beta)>1+1e-10)<<endl;
//     v_scale(dbeta,0.5,dbeta);
//     v_sub(beta,dbeta,beta);//v_sub
//   }

 //for numerical reason 
  if(v_squ_length(beta)>1){
	  alpha=0;
  }else{
    alpha = sqrt(1-v_squ_length(beta));
  }  
  v_scale(v1,eps_tr,dtrans);
  v_add(trans,dtrans,trans); //
  return v_length(v1);
} 


#endif


int RigidTransfo::SetTransfo(CVector3_t new_beta, CVector3_t transl){
  float sq_len = v_squ_length(new_beta);
  if(sq_len>1){
    return 0;
  } 
  alpha = sqrt(1-sq_len);
  v_copy(new_beta,beta);
  v_copy(transl,trans);
  return 1;
}


void RigidTransfo::SetRate(float n_eps_rot,float n_eps_tr){
  Rotation::SetRate(n_eps_rot);
  Translation::SetRate(n_eps_tr);
}

void RigidTransfo::ScaleRate(float factor){
  Rotation::ScaleRate(factor);
  Translation::ScaleRate(factor);
}


void RigidTransfo::Transform(const CVector3_t in,CVector3_t out){
  CVector3_t v;
  Rotation::Transform(in,v);
  Translation::Transform(v,out);
 
}



void RigidTransfo::InverseTransform(const CVector3_t in,CVector3_t out){
  CVector3_t v;
  Translation::InverseTransform(in,v);
  Rotation::InverseTransform(v,out);
}




/**
 * not the inverse transformation, same order but reverse
 */
void RigidTransfo::ReverseTransform(const CVector3_t in,CVector3_t out){
  CVector3_t v;
  Rotation::InverseTransform(in,v);
  Translation::InverseTransform(v,out);

}


void RigidTransfo::Invert(){
  CVector3_t tr;
  Rotation::Invert();
  Rotation::Transform(trans,tr);
  v_scale(tr,-1,trans);
}

RigidTransfo& RigidTransfo::operator*(const RigidTransfo& rt1){
  CVector3_t v;
  //new translation
  Rotation::Transform(rt1.GetTranslation(),v);
  v_add(v,trans,trans);
  (Rotation&)*this = ((Rotation&)(*this))*(Rotation&)rt1;
  return *this;
}

// void RigidTransfo::InverseTransform(CVector3_t in,float angle;CVector3_t out){
//   SetAngle(angle);
//   InverseTransform(in,out);
// }
ostream& operator<<(ostream& out, const RigidTransfo& rt){
  out<<(const Rotation&)(rt)<<" ";
  out<<(const Translation&)(rt);
  return out;
}

istream& operator>>(istream& in, RigidTransfo& rt){
  char c;
  in>>(Rotation&)(rt)>>c;
  in>>(Translation&)(rt);
  return in;
} 

#ifdef  RIGID_TRANSFO_MAIN

//#define MATLAB_CHECK

void swap(CVector3_t a,CVector3_t b){
  CVector3_t c;
  v_copy(a,c);
  v_copy(b,a);
  v_copy(c,b);
}

int main (int argc, char *argv[]){
  CVector3_t axis,x1,x2,x3,y1,y2,y3,n1;
  int x;
  RigidTransfo rt1;
  RigidTransfo rt2;
  srand(time(NULL));
#ifdef MATLAB_CHECK
  float angle = pi/6;
  v_set(0.3,0.3,0.3,axis);
#else
  float angle = RND(2*pi)-pi;
  v_set(RND(2)-1,RND(2)-1,RND(2)-1,axis);
#endif
  v_normalize(axis,axis);
  v_scale(axis,sin(angle/2),axis);
  rt1.SetRotationParam(axis);
#ifdef MATLAB_CHECK
  v_set(3,4,5,axis);
#else
  v_set(RND(10),RND(10),RND(10),axis);
#endif
 rt1.SetTranslation(axis);
  //  v_set(10,20,-15,x1);
//   v_set(5,-10,10,x2);
//   v_set(-15,-10,5,x3);
  
  rt1.Transform(x1,y1);
  rt1.Transform(x2,y3);
  rt1.Transform(x3,y3);

  rt2.SetRate(0.01,0.1);
  
  for(int i=0;i<500;i++){
  v_set(RND(20)-10,RND(20)-10,RND(20)-10,x1);
#ifdef MATLAB_CHECK
  v_set(12, 5, 7,x1);
#endif
  rt1.Transform(x1,y1);
  v_set(normalSample(0,1),normalSample(0,1),normalSample(0,1),n1);
    v_add(y1,n1,y1);

  cout<<rt2<<" "; 
  cout<<rt1<<endl;
   rt2.Update(x1,y1);
#ifdef MATLAB_CHECK
   cout<<"y1 "<<y1<<endl;
#endif  
  


  }
  return 0;
}

#endif
