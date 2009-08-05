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
#include "KChainOrientationBodySchema.h"



void  KChainOrientationBodySchema::Jacobian(Matrix& jac){
  Matrix mat(joint_angle_dim,6);
  GetFullJacobian(mat.Array());
  mat.Transpose(jac); // different storing convention in mathlib.h
  //  jac.Print();
}

  




// /**
//  * @brief samples the workspace to find adequate weights
//  */
// void KChainOrientationBodySchema::GetWeights(cart_vec_t& cart_weights, joint_vec_t& joint_weights){
//   joint_vec_t angles;
//   CVector3_t kstack[MAX_LINKS];
//   CVector3_t pos;
//   int n =50; //number of samples
// #ifdef WITH_LAST_LINK
//   last_link->GetTranslation(pos);
//   int last = nb_joints;
// #else
//   v_clear(pos);
//   int last = nb_joints-1;  //assuming last rotation is irrelevant
// #endif 
//   cart_weights.Zero();
//   joint_weights.Zero();
//   cart_weights += 1;
//   for(int i=0;i<n;i++){
//     SetRandomAngle(angles);
//     ForwardKinematicsStack(pos,kstack);
//     for(int j=0;j<last;j++){ 
//       joint_weights[j] += v_length(kstack[j+1]);
//     }
//   }
//   joint_weights /=(float) n;
  
//   for(int j=0;j<last;j++){ 
//     joint_weights[j] = 1.f/joint_weights[j];
//   }
// }

int KChainOrientationBodySchema::InverseKinematics(const cart_vec_t& pos,joint_vec_t& angles){
  // int config[MAX_LINKS]; maybe later to try more sophistaicated search
  int done =ik_trials;
  joint_vec_t a;
  while(TryInverseKinematics(pos)>tol && done){
    // update config and done
    //   set angles
    SetRandomAngle(a);
    done--;
  }
  GetAngles(angles);
  return done;
}
    

float KChainOrientationBodySchema::TryInverseKinematics(const cart_vec_t position){
  int i,j;
  float cand,rdist=0,tdist=0,k=0.001; //rotation vs translation weight
  CVector3_t stack[MAX_LINKS];
  CVector3_t tar,newrod,rod,diff,pos;
  CQuat_t q1,q2,q3,q4,rot;
  
  // converting data format
  v_copy(position.GetArray(),pos);
  q_complete(position.GetArray()+3,rot);

  q_inv(rot,q1);
  for(i=0;i<nb_joints;i++){
    GetQuaternion(i,q2);
    q_multiply(q2,q1,q3);
    q_copy(q3,q1);
  } 
  for(j=0;j<50;j++){
#ifdef WITH_LAST_LINK
    last_link->GetTranslation(rod);
#else
    v_clear(rod);
#endif
    InverseKinematicsStack(pos,stack);
    for(i=nb_joints-1;i>=0;i--){
      GetInverseQuaternion(i,q2);
      q_multiply(q2,q1,q3); //q1*inv(Ri)
      if(IsInverted(i)==-1){
	v_copy(stack[i],tar);
	Translate(i,rod,newrod); //getting to joint
	cand = joints[i]->MinimizePositionAndRotationAngle(tar,newrod,q3,k);
	cand = AngleInClosestRange(i,cand);
	joints[i]->SetAngle(-cand);// todo to check if it is - 
	Rotate(i,newrod,rod);   // updating "rod"
	v_sub(tar,rod,diff);
      }
      else{
	Rotate(i,stack[i+1],tar); //rotating back
	cand = joints[i]->MinimizePositionAndRotationAngle(tar,rod,q3,k);
	cand = AngleInClosestRange(i,cand);
	joints[i]->SetAngle(cand);
	Rotate(i,rod,newrod);
	Translate(i,newrod,rod);
	v_sub(tar,newrod,diff);
      }
      GetQuaternion(i,q2);
      q_multiply(q3,q2,q1);
      q_multiply(q2,q3,q4);
      rdist = v_length(q4);//rotation distance, only the first 3 components 
      tdist = v_length(diff);//translation distance
      //     cout<<"rot "<<rdist<<" pos "<<tdist<<" prod: "<<(1-k)*rdist+k*tdist<<endl;
      if(tdist<tol && rdist<rot_tol){return rdist/rot_tol+tdist;}
    }
    q_multiply(rot,q1,q2);
    q_inv(rot,q3);
    q_multiply(q2,q3,q1);
  }
  return rdist/rot_tol + tdist;
}


#ifdef NO_JOINT_LIMITS

int KChainOrientationBodySchema::InverseKinematics(const cart_vec_t& pos,joint_vec_t& angles){
  float f;
  int it=0;
  CQuat_t q;
  v_copy(pos.GetArray()+3,q);
  q[3] = sqrtf(1-v_squ_length(q));
  f=InverseKinematicsCCD(pos.GetArray(),q); 
  if(f>tol){
    cout<<"can't find inverse kinematics"<<endl;
    return 0;
  }
  KinematicChain::GetAngles(angles.GetArray()); 
//   if(!AnglesInRange(angles)){
//     Matrix jac,pjac,mat(joint_angle_dim,joint_angle_dim);
//     joint_vec_t dangle,middle;
//     middle = (min_angle+max_angle)*0.5;  
//     do{ // gets back to allowable range
//       Jacobian(angles,jac);
//       jac.Inverse(pjac); //pseudo-inverse
//       if(!Matrix::IsInverseOk())
// 	{cout<<"bad matrix inversion in InverseKinematics"<<endl;}
//       mat.Identity();
//       mat -= pjac*jac;
//       dangle = mat*(middle-angles)*0.01;
//       angles += dangle;      
//       //     cout<<"ik "<<angles<<endl;
//       //mat.Print();
//     }while(!AnglesInRange(angles) && it++ <100);
//     if(it<=100){
//       cout<<"can't bring target configuration in range"<<endl;
//     }
//     else{
//       cout<<"in range"<<endl;
//     }
//     f=InverseKinPosCCD(pos.GetArray());
//   }
  return f<=tol;
}
#endif
