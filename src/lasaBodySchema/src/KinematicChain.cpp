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
#include "KinematicChain.h"


KinematicChain::KinematicChain(int n){
  int i;
  nb_joints = n;
#ifdef BLOCK_KC
  joints = joints_block;
  reverse = reverse_block;
#endif
  tol = 3; //inverse kin tolerance
  modality = UNSPECIFIED_MODALITY;
#ifdef WITH_LAST_LINK
  last_link = new Translation();
  own_last_link=1;
#endif
  for(i=0;i<n;i++){
    joints[i] = new RigidTransfo();
    reverse[i] = 1;
  }

  for(i=n;i<MAX_LINKS;i++){
    joints[i] = NULL;
    reverse[i] = 0;
  }
}


KinematicChain::~KinematicChain(){
    if(own_last_link){
        delete last_link;
    }

}


void KinematicChain::FreeChain(){
  cout<<"freeing kinematic chain"<<endl;
  for(int i=0;i<MAX_LINKS;i++){
    if(joints[i]) delete joints[i];
  }
#ifdef WITH_LAST_LINK 
  delete last_link;
#endif
}


// int KinematicChain::AddJoint(RigidTransfo *t,int inv){
//   reverse[nb_joints]=inv;
//   joints[nb_joints++]=t;
//   return nb_joints;
// }

int KinematicChain::SetAllLinks(CVector3_t *links){
  int ret =1;
  for(int i=0;i<=nb_joints;i++){
    ret*= SetLink(i,links[i]);
  } 
  return ret;
}

/**
 * @param i link index (assumed to be > 0)
 */
int KinematicChain::SetLink(int i, CVector3_t link){
  if(i==nb_joints){
#ifdef WITH_LAST_LINK
    last_link->SetTranslation(link);
#endif
  }
  else{
    if(i>nb_joints){
      return 0;
    }
    else{
      joints[i]->SetTranslation(link);
    }
  }
  return 1;
}


// int KinematicChain::GetLink(int i, CVector3_t link){
//  if(i>nb_joints){
//     return 0;
//   }
//  if(i==nb_joints){
//    return last_link->GetTranslation(link);
//  }
//  else{ 
//    return joints[i]->GetTranslation(link);
//  }
// }

int KinematicChain::SetRotationAxis(int i,CVector3_t axis){
  if(i>=nb_joints){
    return 0;
  }
  return joints[i]->SetRotationAxis(axis);
}

int KinematicChain::SetAllRotationAxes(CVector3_t *axes){
  int ret =1;
  for(int i=0;i<nb_joints;i++){
    ret*= SetRotationAxis(i,axes[i]);
  } 
  return ret;
}

void KinematicChain::ScaleRate(float factor){
 for(int i=0;i<nb_joints;i++){
   joints[i]->ScaleRate(factor);
 }
}


int KinematicChain::GetRotationAxis(int i,CVector3_t axis){
  if(i>=nb_joints){
    return 0;
  }
  return joints[i]->GetRotationAxis(axis);
}

float KinematicChain::GetAngle(int i){
  if(i>=0 && i< nb_joints){
    return joints[i]->GetRotationAngle();
  }
  else{
    cout<<"no such joint: "<<i<<endl;
    return 0.0;
  }
}

void KinematicChain::RandomAxes(){
  for(int i=0;i<nb_joints;i++){
    joints[i]->RandomAxis();
  }
}

int KinematicChain::Copy(const KinematicChain& kc){
  int i,n,ret;
  
  if(nb_joints>kc.GetNbJoints()){
    n = kc.GetNbJoints();
    ret =1;
  }
  else{
    if(nb_joints<kc.GetNbJoints()){
      n = nb_joints;
      ret =-1;
    }
    else{
      n = nb_joints;
      ret =0;
    }
  }
  for(i=0;i<n;i++){
    joints[i]->Copy(*(kc.GetTransfo(i)));
  }
#ifdef WITH_LAST_LINK
  last_link->SetTranslation(kc.GetLastLink()->GetTranslation());
#endif

  return ret;
}



void KinematicChain::ForwardKinematics(CVector3_t pos, CVector3_t local_pos)const{
    CVector3_t v1,v2;
  int i;
#ifdef WITH_LAST_LINK
  if(!local_pos){
      v_clear(v2); 
      last_link->Transform(v2,v1);
  }
  else{
      last_link->Transform(local_pos,v1);
  }
#else
  if(!local_pos){
      v_clear(v1);
  }
  else{
     v_copy(local_pos,v1);
  }
#endif 
 for(i=nb_joints-1;i>=0;i--){
    Transform(i,v1,v2);
    v_copy(v2,v1);
  }
  v_copy(v1,pos);
}


void KinematicChain::Transform(int i,CVector3_t in, float angle,CVector3_t out){
  joints[i]->SetAngle(angle);
  Transform(i,in,out);
}


void KinematicChain::SetAngles(float *angles){
  for(int i=0;i<nb_joints;i++){
      joints[i]->SetAngle(angles[i]);
  }
}


#ifdef DEPRECATED
void KinematicChain::InverseKinematics(CVector3_t pos){
  float tol2,d1,d2;
  int it=0;
  tol2= tol*tol;
  d2= 1000;
  do{
    //  d1=InverseKinematicsStep(pos);
    d1=RedundantInverseKinematicsStep(pos);
    if(it%10==0){
      if(abs(d1-d2)<0.001 && d1>tol){
// 	for(int i=0;i<nb_joints;i++){
// 	  joints[i]->AddToNorm(RND(0.2)-0.1);
// 	}
//	SetRandomAngles();
//	d2=1000;
      }
      d2=d1;
    }
    cout<<d1<<endl;
  }while(d1>tol && it++<200);
}
#endif

/**
 * @brief CCD inverse kinematics function for position constraints
 * @return distance to target position
 */
float KinematicChain::InverseKinPosCCD(CVector3_t pos){
  CVector3_t stack[MAX_LINKS];
  CVector3_t tar,newrod,rod,diff;
  float dist=-1.0f;
  for(int j=0;j<50;j++){
    InverseKinematicsStack(pos,stack);
#ifdef WITH_LAST_LINK
    last_link->GetTranslation(rod);
#else
    v_clear(rod);//modify for reaching with specific points
#endif
    for(int i=nb_joints-1;i>=0;i--){
      if(IsInverted(i)==-1){
	v_copy(stack[i],tar);
	Translate(i,rod,newrod); //getting to joint
	joints[i]->AimAt(tar,newrod);
	joints[i]->InvertRotationAngle();
	Rotate(i,newrod,rod);   // updating "rod"
	v_sub(tar,rod,diff);
      }
      else{
	Rotate(i,stack[i+1],tar); //rotating back
	joints[i]->AimAt(tar,rod);
	Rotate(i,rod,newrod);
	Translate(i,newrod,rod);
	v_sub(tar,newrod,diff);
      }
      dist=v_length(diff);
      if(dist<=tol){return dist;}
      //     cout<<v_length(diff)<<endl;;
    }
  }
  return dist;
}




void KinematicChain::InverseKinRotCCD(CQuat_t q){
  int i,j;
  CQuat_t q1,q2,q3,q4;
  
  q_inv(q,q1);
  for(i=0;i<nb_joints;i++){ //should be nb_joints-1 if not using last_link
    GetQuaternion(i,q2);
    q_multiply(q2,q1,q3);
    q_copy(q3,q1);
  } 
  for(j=0;j<20;j++){
    for(i=nb_joints-1;i>=0;i--){//should be nb_joints-2 if not using last_link
      GetInverseQuaternion(i,q2);
      q_multiply(q2,q1,q3); //q1*inv(Ri)
      joints[i]->MinimizeRotation(q3);
      if(IsInverted(i)==-1){
	joints[i]->InvertRotationAngle();
      }
      GetQuaternion(i,q2);
      q_multiply(q3,q2,q1);
      q_multiply(q2,q3,q4);
      cout<<"rot "<<v_length(q4)<<endl;
      //      if(v_length(diff)<tol){return;}
    }
    q_multiply(q,q1,q2);
    q_inv(q,q3);
    q_multiply(q2,q3,q1);
  }
}

float KinematicChain::InverseKinematicsCCD(CVector3_t pos, CQuat_t rot){
  int i,j;
  float rdist=0,tdist=0,k=0.001; //rotation vs translation weight
  CVector3_t stack[MAX_LINKS];
  CVector3_t tar,newrod,rod,diff;
  CQuat_t q1,q2,q3,q4;

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
	joints[i]->MinimizePositionAndRotation(tar,newrod,q3,k);
	joints[i]->InvertRotationAngle();
	Rotate(i,newrod,rod);   // updating "rod"
	v_sub(tar,rod,diff);
      }
      else{
	Rotate(i,stack[i+1],tar); //rotating back
	joints[i]->MinimizePositionAndRotation(tar,rod,q3,k);
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
      if(tdist<tol && rdist<0.05){return rdist/0.05+tdist;}
    }
    q_multiply(rot,q1,q2);
    q_inv(rot,q3);
    q_multiply(q2,q3,q1);
  }
  return rdist/0.05 + tdist;
}





/**
 * q1[i] contains all rotation up to i-1 i.e. q1*q2*q3*qi-1
 */
void KinematicChain::RotationStack(CQuat_t *q1){
  CQuat_t q;
  q_clear(q1[0]);//
  for(int i=0;i<nb_joints;i++){
    joints[i]->GetQuaternion(q);
    if(reverse[i]==-1){
      v_scale(q,-1,q); //inverting only the first 3 comp
    }
    q_multiply(q,q1[i],q1[i+1]);
  }
}


/**
 * v1[i] contains the value after it was transformed by joint i,i.e in
 * the frame of ref of joint i-1
 */
void KinematicChain::ForwardKinematicsStack(CVector3_t pos,CVector3_t *v1){
  v_copy(pos,v1[nb_joints]);
  for(int i=nb_joints;i>0;i--){
    Transform(i-1,v1[i],v1[i-1]);
 }
}
/**
 * ikstack[i] contains the values after it was inverse transformed by joints up to i-1,
 * i.e. in the frame of ref of joint i-1
 */
void KinematicChain::InverseKinematicsStack(CVector3_t pos, CVector3_t *ikstack){
  v_copy(pos,ikstack[0]);
  for(int i=0;i<nb_joints;i++){
    InverseTransform(i,ikstack[i],ikstack[i+1]);
  }
}

void KinematicChain::GlobalTransfo(RigidTransfo& rt){
  CVector3_t transl;
  GlobalTranslation(transl);
  GlobalRotation(rt);
  rt.SetTranslation(transl);
}

void KinematicChain::GlobalRotation(Rotation& r){
  int i;
  r.Identity();
  for(i=0;i<nb_joints;i++){
    if(reverse[i] == -1){
      joints[i]->Invert();
      r = r*((Rotation&)(*(joints[i])));
      joints[i]->Invert();   
    }
    else{
      r = r*((Rotation&)(*(joints[i])));
    }
  }
}

/**
 *@brief checks for a collision between the two extremal links of the chain.
 * Does not include the last link
*/
float KinematicChain::CollisionDetection(float& l1, float& l2){
  float distance,d=0.0f;
  int last = nb_joints-1;
  RigidTransfo rt;
  Rotation rot;
  CVector3_t v,v1,v2,p1,p2;
  v_clear(v);
  for(int i=1;i<last;i++){
    d += v_squ_length(joints[i]->GetTranslation());
  }
  if(d<epsilon){
    return -1;
  }
  GlobalTransfo(rt);
  GlobalRotation(rot);
  v_clear(p1);
  rt.Transform(p1,p2);
  if(IsInverted(last)==1){
    v_scale(GetTransfo(last)->GetTranslation(),-1,v);
  }
  else{
    GetTransfo(last)->GetTranslation(v);
  }
  rot.Transform(v,v2);
  //  cout<<"transfo "<<v<<" "<<v2<<" "<<v_length(v)<<" "<<v_length(v2)<<endl;
  
  if(IsInverted(0)==-1){
    v_scale(GetTransfo(0)->GetTranslation(),-1,v1);
  }
  else{
    GetTransfo(0)->GetTranslation(v1);
  }
//   cout<<"input"<<endl<<p1<<" "<<v1<<endl;
//   cout<<p2<<" "<<v2<<endl;
//   cout<<*this<<endl;
  if(segments_nearest_points(p1,v1,p2,v2,&l1,&l2,&distance)){
    return distance;  
  }
  else{
    return -1;
  }
}



#ifdef DEPRECATED
/**
 * reverse transformations are discarded
 */ 
float KinematicChain::RedundantInverseKinematicsStep(CVector3_t target){
  int i,j;
  CVector3_t kinstack[MAX_LINKS];
  CVector3_t ikstack[MAX_LINKS];
  CVector3_t v1[MAX_LINKS];
  CVector3_t v2[MAX_LINKS];
  CVector3_t diff[MAX_LINKS];
  float delta_norm[MAX_LINKS];
  float sum=0;
  CVector3_t pos,tmp;
  CQuat_t q,q1,q2;
  Rotation rot;
 
  v_clear(pos);
  ForwardKinematicsStack(pos,kinstack);
  InverseKinematicsStack(target,ikstack);
  for(i=0;i<nb_joints;i++){
      joints[i]->InverseNormDerivative(ikstack[i],v1[i]);
      joints[i]->NormDerivative(kinstack[i+1],v2[i]);  
      v_sub(kinstack[i],ikstack[i],diff[i]);
      delta_norm[i]=0;
  }
  v_sub(kinstack[nb_joints],ikstack[nb_joints],diff[nb_joints]);

  for(j=0;j<=nb_joints;j++){

  // i in inverse side
    q_clear(q2);
    delta_norm[j-1] += v_dot(diff[j],v1[j-1]);//i = j-1
    for(i=j-2;i>=0;i--){
      joints[i+1]->GetQuaternion(q1);
      q_multiply(q2,q1,q);
      q_copy(q,q2);
 //      ComposedRotation(i+1,j-1,q);     
      v_scale(q,-1,q);
      rot.SetQuaternion(q);
      rot.Transform(v1[i],tmp);
      delta_norm[i] += v_dot(diff[j],tmp);
      //      cout<<i<<" + "<<v_dot(diff[j],tmp)<<endl;
    }

    // i in forward side
    if(j<nb_joints){
      delta_norm[j] -= v_dot(diff[j],v2[j]); //i=j
      cout<<j<<" - "<<v_dot(diff[j],v2[j])<<endl;
      q_clear(q2);
      for(i=j+1;i<nb_joints;i++){
	joints[i-1]->GetQuaternion(q1);
	q_multiply(q1,q2,q);
	q_copy(q,q2);
	// ComposedRotation(j,i-1,q);
	rot.SetQuaternion(q);
	rot.Transform(v2[i],tmp);
	delta_norm[i] -= v_dot(diff[j],tmp);
	cout<<i<<" - "<<v_dot(diff[j],tmp)<<endl;
      }
    }
  }

  for(j=0;j<nb_joints;j++){
    sum+=delta_norm[j]*delta_norm[j];
  }
  sum = 1/(sqrt(sum)*100);
  for(j=0;j<nb_joints;j++){
    joints[j]->AddToNorm(delta_norm[j]*sum);
  }
  cout<<*this<<endl;
  return v_length(diff[0]);
}


float KinematicChain::InverseKinematicsStep(CVector3_t pos){
  
  CVector3_t v1[MAX_LINKS];
  CVector3_t v,delta,v2;
  CQuat_t q;
  CQuat_t q1[MAX_LINKS];
  Rotation rot;
  int i;
  float dangles[MAX_LINKS];
  float sum,ret_val;
  v_clear(q1[0]);
  q_set(q1[0],0,q1[0]);
  for(i=0;i<nb_joints;i++){
    joints[i]->GetQuaternion(q);
    if(reverse[i]==-1){
      v_scale(q,-1,q); //inverting only the first 3 comp
    }
 q_multiply(q,q1[i],q1[i+1]);
  }

    // forward kinematics
#ifdef WITH_LAST_LINK   
  v_clear(v1[nb_joints]);
  last_link->Transform(v1[nb_joints],v1[nb_joints-1]);
#else
  v_clear(v1[nb_joints-1]);
#endif
  for(i=nb_joints-1;i>0;i--){
    Transform(i,v1[i],v1[i-1]);
 }
    Transform(0,v1[0],v);
  v_sub(pos,v,v); //(y-R(0)
  ret_val = v_length(v);
  //   cout<<v_length(v)<<" :";

  //  cout<<"diff ";coutvec(v);
  sum=0;
  for(i=0;i<nb_joints;i++){

    rot.SetQuaternion(q1[i]);
#ifdef OLD_KIN
    rot.Derivative(m1);
     v_transform_normal2(v,m1,delta);
#endif
    //updating angle
     if(joints[i]->IsAdaptive()){
       joints[i]->NormDerivative(v1[i],v2);
       if(reverse[i]==-1){
	 joints[i]->InverseNormDerivative(v1[i],v2);
       }
       else{
	 joints[i]->NormDerivative(v1[i],v2);
       }
     }
     else{
       v_clear(v2);
     }
#ifndef OLD_KIN
    rot.Transform(v2,delta);
#endif
//    coutvec(v);
//     coutvec(delta);
    //   cout<<"dot "<<v_dot(delta,v);
    dangles[i] = v_dot(delta,v);
   
    sum+=dangles[i]*dangles[i];
    // cout
    //    cout<<" "<<sum; 
  }
  if(sum<epsilon){return ret_val;}
  sum = 1/(sqrt(sum)*500);
  for(i=0;i<nb_joints;i++){
    //  cout<<";"<<dangles[i];
    joints[i]->AddToNorm(dangles[i]*sum); //to be checked
  }
  return ret_val;
}

#endif

//jac is assumed to be a homogeneous matrix 
// the last joint is assumed to be irrelevant for the position
void  KinematicChain::GetJacobian(float *jac){
  CQuat_t q[MAX_LINKS];
  CVector3_t v1[MAX_LINKS];
  CVector3_t v2,der; 
  float f;
  Rotation rot;
#ifdef WITH_LAST_LINK
  last_link->GetTranslation(v2);
#else
  v_clear(v2);//modify for reaching with specific point
#endif
  RotationStack(q);
  ForwardKinematicsStack(v2,v1);
#ifdef WITH_LAST_LINK
  for(int i=0;i<nb_joints;i++)//{ below
#else
  for(int i=0;i<nb_joints-1;i++)//{ below ;  assuming the last rotation is irrelevant for the position
#endif    
    { 
   rot.SetQuaternion(q[i]);
    if(reverse[i]==-1){
      joints[i]->InverseNormDerivative(v1[i+1],v2);
    }
    else{
      joints[i]->NormDerivative(v1[i+1],v2);
    }
    rot.Transform(v2,der);//d/dsin(theta/2);
    f=0.5*joints[i]->GetAlpha();
    v_scale(der,f,jac+4*i);
    jac[4*i+3]=0;
  }
}

//position and rotation jacobian
void  KinematicChain::GetFullJacobian(float *jac){
  CQuat_t q[MAX_LINKS];
  CVector3_t v1[MAX_LINKS];
  CVector3_t v2,der; 
  CQuat_t q1,q2,q3,q4;
  float f;
  Rotation rot;

#ifdef WITH_LAST_LINK
  last_link->GetTranslation(v2);
#else
  v_clear(v2);
#endif
  q_clear(q1);
  RotationStack(q);
  ForwardKinematicsStack(v2,v1);
  for(int i=nb_joints-1;i>=0;i--){
    rot.SetQuaternion(q[i]);
    joints[i]->NormDerivative(v1[i],v2);
    if(reverse[i]==-1){
      joints[i]->InverseNormDerivative(v1[i+1],v2);
      joints[i]->InverseQuaternionNormDerivative(q3);
    }
    else{
      joints[i]->NormDerivative(v1[i+1],v2);
      joints[i]->QuaternionNormDerivative(q3);
    }
    rot.Transform(v2,der);//d/dsin(theta/2);
    f=0.5*joints[i]->GetAlpha();
    v_scale(der,f,jac+6*i);
    q_multiply(q1,q3,q4);
    q_multiply(q4,q[i],q3);//q3 = q[i]* q2'* q1 where '*': quat. mult
    v_scale(q3,f,jac+6*i+3);//we take only the 3 first components
    GetQuaternion(i,q2);  
    q_multiply(q1,q2,q3); //for the next iteration
    q_copy(q3,q1);
 }
}

void KinematicChain::InitKinematicStack(const CVector3_t local_pos, CVector3_t out)const{
#ifdef WITH_LAST_LINK
  CVector3_t v;
  if(local_pos){   
    last_link->GetTranslation(v);
    v_add(v,local_pos,out);
  }
  else{
    last_link->GetTranslation(out);
  }
#else
  if(local_pos){
    v_copy(local_pos,out);
  }
  else{
    v_clear(out);
  }
#endif
}

#define OPTIMISED_ROTATION_UPDATE
#ifdef OPTIMISED_ROTATION_UPDATE
//only rotations are considered for now
float KinematicChain::UpdateWithFullJacobian(float *angle_pos,
                                             float *angle_disp, RigidTransfo *rt){

     CVector3_t delta_a[MAX_LINKS];
     CVector3_t v,vd,qd,qdiff,qds;
     CMatrix4_t m1,m2;
     CMatrix4_t rot_acc[MAX_LINKS];
     //  Rotation rot;
     float jac[MAX_LINKS*6];
     CQuat_t q1,q2,q3,q4,q7,q8;
     int i,j;
     float ret_val;//return value
     float sum_rot;
     SetAngles(angle_pos);
     GetFullJacobian(jac);
     v_clear(vd);//accumulator for v_dot
     v_clear(qd);//accumulator for q_dot
     for(i=0;i<nb_joints;i++){
        v_scale(jac+i*6+3,angle_disp[i],v);
        v_add(v,qd,qd);
     }
     if(v_length(qd)<epsilon){return 0;}//no movement
     rt->GetRotationParam(qds);//q_dot_star
     v_sub(qd,qds,qdiff);  
          ret_val = v_length(qdiff)/(v_length(qds));
     sum_rot=0;
    

     q_clear(q7); 
     q_clear(q8);
    
     for(j=0;j<nb_joints;j++){ //q8 is initialized with the gloabl rotation
         GetQuaternion(j,q2);
         q_multiply(q2,q8,q3);
         q_copy(q3,q8);
     }

     for(j=0;j<nb_joints;j++){
         m_clear(rot_acc[j]);
         if(j>0){
             GetQuaternion(j-1,q2);
             q_multiply(q7,q2,q3);
             q_copy(q3,q7);
         }

         GetInverseQuaternion(j,q2);
         q_multiply(q8,q2,q3);
         q_copy(q3,q8);

         q_clear(q1);         //initialization for i<j 
         q_copy(q7,q4);       //initialization for i<j 
         for(i=0;i<nb_joints;i++){ //k
             if(i<j){
                 QuaternionAxisDerivative(j,m1);
                 if(i>0){
                     GetQuaternion(i-1,q2);
                     q_multiply(q2,q1,q3);
                     q_copy(q3,q1);
                 }
                 GetInverseQuaternion(i,q2);
                 q_multiply(q4,q2,q3);
                 q_copy(q3,q4);
               
                 QuaternionAngleDerivative(i,q2);
                 q_multiply(q4,q2,q3);
                 q_multiply(q3,q1,q2); //before the matrix
                 q_copy(q8,q3);//after the matrix
             }
             if(i==j){   // i==j
                 QuaternionAxisAngleDerivative(j,m1);
                 q_copy(q8,q3);        // after the matrix
                 q_copy(q7,q2);        //before the matrix         
                 q_clear(q1);   //initialization for i>j 
                 q_copy(q8,q4); //initialization for i>j 
             }
             //i>j
             if(i>j){
                 QuaternionAxisDerivative(j,m1);

                 if(i>j+2){
                     GetQuaternion(i-1,q2);
                     q_multiply(q2,q1,q3);
                     q_copy(q3,q1);
                 }
                 
                 GetInverseQuaternion(i,q2);
                 q_multiply(q4,q2,q3);
                 q_copy(q3,q4);

                 QuaternionAngleDerivative(i,q3);
                 q_multiply(q4,q3,q2);
                 q_multiply(q2,q1,q3); //after the matrix
                 q_copy(q7,q2);// befire the matrix
             }
              
   //compute matrices            
             q_multiply(m1,q2,q4);
             q_copy(q4,m1);
             q_multiply(m1+4,q2,q4);
             q_copy(q4,m1+4);
             q_multiply(m1+8,q2,q4);
             q_copy(q4,m1+8);// last column of m1 is zero
                  
             q_multiply(q3,m1,m2);
             q_multiply(q3,m1+4,m2+4);
             q_multiply(q3,m1+8,m2+8);// last column of m1 is zero
             m_rescale(angle_disp[i],m2,m1);
             //accumulating sum_{i} d/daj(d/dthi(q)*thi
             // a index is horizontal, q index is vertical
             m_add(m1,rot_acc[j],rot_acc[j]);
         }
         v_transform_normal2(qdiff,rot_acc[j],delta_a[j]);
         v_scale(delta_a[j],-1,delta_a[j]); //gradient descent
         //cout<<"delta a "<<delta_a[j]<<endl;
         sum_rot += v_squ_length(delta_a[j]);
     }
     sum_rot = 0.5/sqrtf(sum_rot);
     for(j=0;j<nb_joints;j++){
         v_scale(delta_a[j],sum_rot,delta_a[j]);
         joints[j]->AddToAxis(delta_a[j]);
     }
     return ret_val;
}

#else

//only rotations are considered for now
float KinematicChain::UpdateWithFullJacobian(float *angle_pos,
                                             float *angle_disp, RigidTransfo *rt){

     CVector3_t delta_a[MAX_LINKS];
     CVector3_t v,vd,qd,qdiff,qds;
     CMatrix4_t m1,m2;
     CMatrix4_t rot_acc[MAX_LINKS];
     //  Rotation rot;
     float jac[MAX_LINKS*6];
     CQuat_t q1,q2,q3;
     int i,j,k;
     float ret_val;//return value
     float sum_rot;
     SetAngles(angle_pos);
     GetFullJacobian(jac);
     v_clear(vd);//accumulator for v_dot
     v_clear(qd);//accumulator for q_dot
     for(i=0;i<nb_joints;i++){
    //     v_scale(jac+i*6,angle_disp[i],v);
//         v_add(v,vd,vd);
        v_scale(jac+i*6+3,angle_disp[i],v);
        v_add(v,qd,qd);
     }
     if(v_length(qd)<epsilon){return 0;}//no movement
     //   q_complete(qd,q1);
    //  cout<<"angle "<<angle_disp[0]<<endl;
//      cout<<"having  ";coutvec(q1);
//     rt->GetQuaternion(q2);  cout<<"instead ";coutvec(q2);    
     //    rt->GetInverseQuaternion(q2);
     //    q_multiply(q1,q2,q4);
     //    coutvec4(q4);
     //     ret_val = 2*acos(q4[3]); // q4 contains q*^{-1}q
   
     //     ret_val = 2*acos(q4[3]); 
     // ret_val = v_length(q4);
     rt->GetRotationParam(qds);//q_dot_star
     v_sub(qd,qds,qdiff);  
          ret_val = v_length(qdiff)/(v_length(qds));
     // ret_val = v_dot(qd,qds)/(v_length(qd)*v_length(qds));

     
     sum_rot=0;
     for(j=0;j<nb_joints;j++){
         m_clear(rot_acc[j]);
         for(i=0;i<nb_joints;i++){
             //        rt->GetInverseQuaternion(q1);  // first rotation in chain is q*^{-1}
             q_clear(q1);
             for(k=0;k<nb_joints;k++){
                 // k==j
                 if(k==j){
                     if(k==i){
                         QuaternionAxisAngleDerivative(j,m1);
                         //        cout<<m1<<endl;
                     }
                     else{
                         QuaternionAxisDerivative(j,m1);
                     }
                     q_multiply(m1,q1,q3);
                     q_copy(q3,m1);
                     q_multiply(m1+4,q1,q3);
                     q_copy(q3,m1+4);
                     q_multiply(m1+8,q1,q3);
                     q_copy(q3,m1+8);// last column of m1 is zero
                     q_clear(q1);
                 }
                 else{
                     //k==i
                     if(k==i){
                         QuaternionAngleDerivative(i,q2);
                         q_multiply(q2,q1,q3);
                         q_copy(q3,q1);
                     }
                     else{ //k!=i && k!=j
                         GetQuaternion(k,q2);
                         q_multiply(q2,q1,q3);
                         q_copy(q3,q1);
                     }
                 }
             }
             q_multiply(q1,m1,m2);
             q_multiply(q1,m1+4,m2+4);
             q_multiply(q1,m1+8,m2+8);// last column of m1 is zero
             m_rescale(angle_disp[i],m2,m1);
             //accumulating sum_{i} d/daj(d/dthi(q)*thi
             // a index is horizontal, q index is vertical
             m_add(m1,rot_acc[j],rot_acc[j]);
         }
         v_transform_normal2(qdiff,rot_acc[j],delta_a[j]);
         v_scale(delta_a[j],-1,delta_a[j]); //gradient descent
         //cout<<"delta a "<<delta_a[j]<<endl;
         sum_rot += v_squ_length(delta_a[j]);
     }
     sum_rot = 0.5/sqrtf(sum_rot);
     for(j=0;j<nb_joints;j++){
         v_scale(delta_a[j],sum_rot,delta_a[j]);
         joints[j]->AddToAxis(delta_a[j]);
     }
     return ret_val;
}

#endif

// #define COSINE_MIN
// #define WITH_CORRECTION
float KinematicChain::UpdateWithJacobian(float *angle_pos, float *angle_disp, CVector3_t visual_disp, 
					 float threshold, CVector3_t local_pos){
  CVector3_t v1[MAX_LINKS],vv[MAX_LINKS], delta_rot[MAX_LINKS], delta_tr[MAX_LINKS];
  CQuat_t q1[MAX_LINKS];
  CMatrix3_t jstack[MAX_LINKS];
  CVector3_t v,v2,dpos;
  CQuat_t q2,q3,q4,q5,q6;
  int i,k;
  Rotation rot;
  CMatrix3_t m0,m1,m2,m3,m4,m6,m7;
  float f,sum_rot=0,sum_tr=0,ret_val;
  SetAngles(angle_pos);
  InitKinematicStack(local_pos,v2);
  ForwardKinematicsStack(v2,v1);
  RotationStack(q1);
  q_clear(q6);

  // computing jacobian
  v_clear(dpos);
  for(i=0;i<nb_joints;i++){
    rot.SetQuaternion(q1[i]);
    joints[i]->AngleDerivative(v1[i+1],v2);
    //   cout<<v1[i+1]<<endl;
    rot.Transform(v2,v);
    v_scale(v,angle_disp[i],v2);
    v_add(dpos,v2,dpos);
}

#ifdef COSINE_MIN
  float norm_prop = v_length(dpos);
  float norm_vis  = v_length(visual_disp);
  if(norm_vis> epsilon && norm_prop >epsilon){
    ret_val = v_dot(dpos,visual_disp)/(norm_prop*norm_vis);
  }
  else{
    cout<<norm_prop<<" - "<<norm_vis<<endl;
    return -2.0;
  }

#ifdef WITH_CORRECTION
  // computing correction term for minimizing the angle (and not the dot product)
  CMatrix3_t m_corr;
  v_mult(dpos,dpos,m2);
  f= 1.f/(norm_prop*norm_prop);
  m_rescale(f,m2,m1);
  m_identity(m2);
  m_sub(m2,m1,m2);
  f=1.f/norm_prop;
  m_rescale(f,m2,m_corr);
#else
  m_identity(m_corr);
#endif


#else
  CVector3_t diff;
   v_sub(visual_disp,dpos,diff);
   ret_val = v_length(diff);
  if(isnan(ret_val)){return -2.0;}
#endif


  // pre-computing products (could be put in the loop below)
  for(k=0;k<nb_joints;k++){
    joints[k]->Jacobian(jstack[k]);
    v_transform_normal2(v1[k+1],jstack[k],vv[k]);
  //   cout<<k<<" vv "<<vv[k]<<"  "<< v1[k+1]<<endl;
//     cout<<jstack[k]<<endl;
  }
  for(i=0;i<nb_joints;i++){
    m_clear(m4);//accumulator for dT/dli
    m_clear(m7);//accumulator for dT/dai
    q_clear(q2);
    q_clear(q3);
    joints[i]->AxisDerivative(v1[i+1],m0);

    for(k=0;k<i;k++){
      q_multiply(q2,q3,q4); //q2 = joints[k-1]
      q_copy(q4,q3);//outer rotations      
      joints[k]->GetQuaternion(q2);
      q_inv(q2,q5);// joints[k]
      q_multiply(q6,q5,q4);
      q_copy(q4,q6);//inner rotations
      rot.SetQuaternion(q6);
      rot.RotToMatrix(m6);//inner rotations
      rot.SetQuaternion(q3);
      rot.RotToMatrix(m3);//outer rotations 

      // translation update accumulator      
      m_identity(m2);
      joints[k]->NormDerivative(m6+0,m2+0);
      joints[k]->NormDerivative(m6+4,m2+4);
      joints[k]->NormDerivative(m6+8,m2+8);
 
      m3_multiply(m3,m2,m1);
      f =angle_disp[k]*0.5*joints[k]->GetAlpha();//converting to d/dtheta 
      m_rescale(f,m1,m2);
      m_add(m4,m2,m4);

      //rotation update accumulator
      m3_multiply(m6,m0,m2);
      m3_multiply(jstack[k],m2,m1);
      m3_multiply(m3,m1,m2);
      m_add(m7,m2,m7);
     }

		       
    // k=i
    q_multiply(q2,q3,q4); //q2 = joints[i-1]
    q_copy(q4,q3);      
    rot.SetQuaternion(q3);
    rot.RotToMatrix(m3);//outer rotation

    joints[i]->AngleAxisDerivative(v1[i+1],m2);
    m_rescale(angle_disp[i],m2,m1);
    m_add(m7,m1,m7);

    q_clear(q6);
    joints[i]->GetQuaternion(q2);
    for(k=i+1;k<nb_joints;k++){
      q_multiply(q2,q6,q4); //q2 = joint[k-1]
      q_copy(q4,q6);
      joints[k]->GetQuaternion(q2);
      rot.SetQuaternion(q6);
      rot.Transform(vv[k],v2);
      joints[i]->AxisDerivative(v2,m1);
      m3_multiply(m3,m1,m2);
      m_rescale(angle_disp[k],m2,m1);
      m_add(m7,m1,m7);
      if(isnan(m1[0])){
	  cout<<m1<<endl;
	  cout<<m3<<endl;
	  cout<<vv[k]<<endl;
	  cout<<i<<" "<<k<<endl;
	  return -2.0;
      }
    }
#ifdef COSINE_MIN
    m3_multiply(m_corr,m7,m1);
    v_transform_normal2(visual_disp,m1,delta_rot[i]);
    m3_multiply(m_corr,m4,m1);
    v_transform_normal2(visual_disp,m1,delta_tr[i]);
#else
     v_transform_normal2(diff,m7,delta_rot[i]);
     v_transform_normal2(diff,m4,delta_tr[i]);
     //     cout<<delta_rot[i]<<endl;
#endif
    sum_rot += v_squ_length(delta_rot[i]);  
    sum_tr += v_squ_length(delta_tr[i]);   
  }



  if(sum_rot>epsilon){//not necessary (already done in AddToAxis);
    sum_rot = 1/sqrt(sum_rot);
  }
  if(sum_tr>epsilon){
    sum_tr = 30/sqrt(sum_tr);//was 50
    //   sum_tr =0;
  }
  for(i=0;i<nb_joints;i++){
    v_scale(delta_tr[i],sum_tr,delta_tr[i]);
    v_scale(delta_rot[i],sum_rot,delta_rot[i]);// not necessary (already done in AddToAxis);
    
    joints[i]->Translation::Add(delta_tr[i]); 
    joints[i]->Rotation::AddToAxis(delta_rot[i]);
  }
  if(v_squ_length(v1[0])>300){
    //    ScaleChain(300);
  }
  return ret_val;
}

#ifndef OLD_UPDATE
float KinematicChain::Update(float *angle, CVector3_t vision, float threshold, CVector3_t local_pos){
  CVector3_t v1[MAX_LINKS], delta[MAX_LINKS], delta_tr[MAX_LINKS];
  CVector3_t v,v2;
  //  CQuat_t q; unused
  CQuat_t q1[MAX_LINKS];
  CMatrix3_t m2,m3;
  Rotation rot;
  int i;
  float sum=0.0f,sum_tr=0.0f,ret_val;
  
  SetAngles(angle);
  InitKinematicStack(local_pos,v2);
  ForwardKinematicsStack(v2,v1);
  v_sub(vision,v1[0],v); //(y-R(0))
  ret_val = v_length(v);//return value
  if(ret_val > threshold){
      return -ret_val;
  }
  // stop learning if ok
  //  if (ret_val<1){return ret_val;}


  // computing the composed of all rotations
  RotationStack(q1);
  
  // starting updating process
  for(i=0;i<nb_joints;i++){
    //    for(i=0;i<2;i++){
    if(!joints[i]->IsAdaptive()){
      v_clear(delta[i]);
      v_clear(delta_tr[i]);
    }
    else{
      if(reverse[i]==-1){
	rot.SetQuaternion(q1[i+1]);//including Ri in rotation
      }
      else{
	rot.SetQuaternion(q1[i]);
      }
       
      //	 cout<<"rot "<<i<<" "<<rot<<endl;
      /*------------- updating translation -----------------*/
      m_identity(m2);
      // computing rotation matrix (should be equivalent to rot.RotToMatrix(m3))
      rot.Transform((m2+0),(m3+0));
      rot.Transform((m2+4),(m3+4));
      rot.Transform((m2+8),(m3+8));
      v_transform_normal2(v,m3,delta_tr[i]);

      //joints[i]->Translation::Add(delta_tr[i]);
      sum_tr+= v_squ_length(delta_tr[i]);

      /*-------------   updating rotation -----------------*/
      if(reverse[i]==-1){
	joints[i]->Translation::InverseTransform(v1[i+1],v2);//including transl. in transfo
	joints[i]->InverseAxisDerivative(v2,m2);
	rot.SetQuaternion(q1[i]); //exluding Ri from rotation
      }
      else{
	joints[i]->AxisDerivative(v1[i+1],m2);
      }
      rot.Transform((m2+0),(m3+0));
      rot.Transform((m2+4),(m3+4));
      rot.Transform((m2+8),(m3+8));

      v_transform_normal2(v,m3,delta[i]);
      //     cout<<"addedl "<<delta[i]<<endl;
      //      cout<<"mat"<<endl<<m2<<endl;
      //    joints[i]->Rotation::AddToAxis(delta[i]);

      sum+=v_squ_length(delta[i]);

      if(reverse[i] == -1){
	//       v_scale(delta[i],-1,delta[i]);
	v_scale(delta_tr[i],-1,delta_tr[i]);
      }

    }
  }
#ifdef WITH_LAST_LINK
  // updating last link
  rot.SetQuaternion(q1[nb_joints]);
  //     rot.Transform((CVector3_t)(m2+0), (CVector3_t)(m3+0));
  m_identity(m2);
  rot.Transform((m2+0),(m3+0));
  rot.Transform((m2+4),(m3+4));
  rot.Transform((m2+8),(m3+8));
  v_transform_normal2(v,m3,delta_tr[nb_joints]);
  //  last_link->Add(delta_tr[nb_joints]);
  sum_tr+=v_squ_length(delta_tr[nb_joints]);
#endif
  if(sum>epsilon){
    sum = 1/sqrt(sum);
    //       sum=0;
  }
  if(sum_tr>epsilon){
    sum_tr = 30/sqrt(sum_tr);//was 50
  }
  for(i=0;i<nb_joints;i++){
    v_scale(delta_tr[i],sum_tr,delta_tr[i]);
    v_scale(delta[i],sum,delta[i]);
    joints[i]->Translation::Add(delta_tr[i]);
    joints[i]->Rotation::AddToAxis(delta[i]);
    //      cout<<"added "<<delta[i]<<endl;     
  }
#ifdef WITH_LAST_LINK
  v_scale(delta_tr[nb_joints],sum_tr,delta_tr[nb_joints]);
  last_link->Add(delta_tr[nb_joints]);
#endif
  return ret_val;
}

#else
float KinematicChain::Update(float *angle, CVector3_t vision, float threshold, CVector3_t local_pos){
  CVector3_t v1[MAX_LINKS], delta[MAX_LINKS], delta_tr[MAX_LINKS];
  CVector3_t v,v2;
  CQuat_t q;
  CQuat_t q1[MAX_LINKS];
  CMatrix3_t m2,m3;
  Rotation rot;
  int i,last;
  float sum=0.0f,sum_tr=0.0f,ret_val;
  

  // computing the composed of all rotations (should be replaced by RotationStack and adapted)
  v_clear(q1[0]);
  q_set(q1[0],0,q1[0]);
  for(i=0;i<nb_joints;i++){
    joints[i]->SetAngle(angle[i]);
    joints[i]->GetQuaternion(q);
    if(reverse[i]==-1){
      v_scale(q,-1,q); //inverting only the first 3 comp
    }
    q_multiply(q,q1[i],q1[i+1]);//q1[i] contains all rotations up to i-1
  }

  // forward kinematics (should be replaced by forward kinematics stack and adapted)
#ifdef WITH_LAST_LINK
  last = nb_joints;
#else
  last =nb_joints-1;
#endif 
  if(local_pos==NULL){ // zero by default
    v_clear(v1[last]);
  }
  else{
    v_copy(local_pos,v1[last]);
  }
#ifdef LAST_LINK
  last_link->Transform(v1[nb_joints],v1[nb_joints-1]);
#endif
  for(i=nb_joints-1;i>0;i--){

    Transform(i,v1[i],angle[i],v1[i-1]);//v1[i] contains all transfo down to T(i+1)angles are supposed to be already set
    //    cout<<"v1 "<<i-1<<" "<<v1[i-1]<<endl;
  }
  Transform(0,v1[0],v);
  // cout<<*(joints[0])<<endl;
  //   cout<<"v "<<v<<endl;

  //difference between real and current transfo
  v_sub(vision,v,v); //(y-R(0)
  ret_val = v_length(v);//return value
  // stop learning if ok
  //  if (ret_val<1){return ret_val;}
  for(i=0;i<nb_joints;i++){
    //    for(i=0;i<2;i++){
    if(!joints[i]->IsAdaptive()){
      v_clear(delta[i]);
      v_clear(delta_tr[i]);
    }
    else{
      if(reverse[i]==-1){
	rot.SetQuaternion(q1[i+1]);//including Ri in rotation
      }
      else{
	rot.SetQuaternion(q1[i]);
      }
       
      //	 cout<<"rot "<<i<<" "<<rot<<endl;
      /*------------- updating translation -----------------*/
      m_identity(m2);
      // computing rotation matrix (should be equivalent to rot.RotToMatrix(m3))
      rot.Transform((m2+0),(m3+0));
      rot.Transform((m2+4),(m3+4));
      rot.Transform((m2+8),(m3+8));
      v_transform_normal2(v,m3,delta_tr[i]);

      //joints[i]->Translation::Add(delta_tr[i]);
      sum_tr+= v_squ_length(delta_tr[i]);

      /*-------------   updating rotation -----------------*/
      if(reverse[i]==-1){
	joints[i]->Translation::InverseTransform(v1[i],v2);//including transl. in transfo
	joints[i]->InverseAxisDerivative(v2,m2);
	rot.SetQuaternion(q1[i]); //exluding Ri from rotation
      }
      else{
	joints[i]->AxisDerivative(v1[i],m2);
      }
      rot.Transform((m2+0),(m3+0));
      rot.Transform((m2+4),(m3+4));
      rot.Transform((m2+8),(m3+8));

      v_transform_normal2(v,m3,delta[i]);
      //     cout<<"addedl "<<delta[i]<<endl;
      //      cout<<"mat"<<endl<<m2<<endl;
      //    joints[i]->Rotation::AddToAxis(delta[i]);

      sum+=v_squ_length(delta[i]);

      if(reverse[i] == -1){
	//       v_scale(delta[i],-1,delta[i]);
	v_scale(delta_tr[i],-1,delta_tr[i]);
      }

    }
  }
#ifdef WITH_LAST_LINK
  // updating last link
  rot.SetQuaternion(q1[nb_joints]);
  //     rot.Transform((CVector3_t)(m2+0), (CVector3_t)(m3+0));
  m_identity(m2);
  rot.Transform((m2+0),(m3+0));
  rot.Transform((m2+4),(m3+4));
  rot.Transform((m2+8),(m3+8));
  v_transform_normal2(v,m3,delta_tr[nb_joints]);
  //  last_link->Add(delta_tr[nb_joints]);
  sum_tr+=v_squ_length(delta_tr[nb_joints]);
#endif
  if(sum>epsilon){
    sum = 1/sqrt(sum);
    //       sum=0;
  }
  if(sum_tr>epsilon){
    sum_tr = 30/sqrt(sum_tr);//was 50
  }
  for(i=0;i<nb_joints;i++){
    v_scale(delta_tr[i],sum_tr,delta_tr[i]);
    v_scale(delta[i],sum,delta[i]);
    joints[i]->Translation::Add(delta_tr[i]);
    joints[i]->Rotation::AddToAxis(delta[i]);
    //      cout<<"added "<<delta[i]<<endl;     
  }
#ifdef WITH_LAST_LINK
  v_scale(delta_tr[nb_joints],sum_tr,delta_tr[nb_joints]);
  last_link->Add(delta_tr[nb_joints]);
#endif
  return ret_val;
}

#endif

/**
 * @brief updates the kinematic chain based on touch information.
 * We assume that the links touch when their distance is below a threshold. 
 * But the sensors have to depts or direction. 
 * @param angle the array of angles of the chain
 * @param k0 the touching point on the first segment
 * @param k1 the touching point on the last segment
 * @return the distance between the two touching points in the current body schema 
 * @todo test it
 */
float KinematicChain::UpdateTouch(float *angle, float k0, float k1){

  CVector3_t kinstack[MAX_LINKS], delta[MAX_LINKS], delta_tr[MAX_LINKS];
  CVector3_t v,v2,touch_pos0,touch_pos1;
  //  CQuat_t q;
  CQuat_t q1[MAX_LINKS];
  CMatrix3_t m2,m3;
  //  RigidTransfo **joints_bk=NULL;
  //   int * reverse_bk;
  Rotation rot;
  int i;
  float sum=0.0f,sum_tr=0.0f,ret_val;
  int shifted =0;
  joints[nb_joints-1]->GetTranslation(touch_pos1);
  v_scale(touch_pos1,k1,touch_pos1);
  joints[0]->GetTranslation(touch_pos0);
  v_scale(touch_pos0,k0,touch_pos0);

  SetAngles(angle);
  nb_joints--; //discarding the last rotation
  
   if(IsInverted(0)==-1){
     // joints_bk = joints;   
     //   joints = &(joints[1]); //discarding the first rotation (shifting)
     // reverse_bk = reverse;
     //  reverse = &(reverse[1]);  
     joints++;
     reverse++;
     nb_joints--;
     shifted=1;
  }
 


  // forward kinematics
  ForwardKinematicsStack(touch_pos1,kinstack);
 
  //difference between real and current transfo
  v_sub(touch_pos0,kinstack[0],v); //(y-R(0)
 
  ret_val = v_length(v);//return value
  // stop learning if ok
  if (ret_val<10){
  // computing the composed of all rotations
  RotationStack(q1);
  for(i=0;i<nb_joints;i++){
    //    for(i=0;i<2;i++){
    if(!joints[i]->IsAdaptive()){
      v_clear(delta[i]);
      v_clear(delta_tr[i]);
    }
    else{
      if(reverse[i]==-1){
	rot.SetQuaternion(q1[i+1]);//including Ri in rotation
      }
      else{
	rot.SetQuaternion(q1[i]);
      }
       
      /*------------- updating translation -----------------*/
  
      // computing rotation matrix (should be equivalent to rot.InverseTransform(v,delta_tr))
      rot.RotToMatrix(m3);
      v_transform_normal2(v,m3,delta_tr[i]);
    
      if(reverse[i] == -1){ // to be checked
	v_scale(delta_tr[i],-1,delta_tr[i]);
      }    
      sum_tr+= v_squ_length(delta_tr[i]);

      /*-------------   updating rotation -----------------*/

      if(reverse[i]==-1){
	joints[i]->Translation::InverseTransform(kinstack[i+1],v2);//including transl. in transfo
	joints[i]->InverseAxisDerivative(v2,m2);
	rot.SetQuaternion(q1[i]); //exluding Ri from rotation
      }
      else{
	joints[i]->AxisDerivative(kinstack[i+1],m2);
      }

      rot.Transform((m2+0),(m3+0));
      rot.Transform((m2+4),(m3+4));
      rot.Transform((m2+8),(m3+8));
      v_transform_normal2(v,m3,delta[i]);

      sum+=v_squ_length(delta[i]);
    }
  }
  if(sum>epsilon){
    sum = 1/sqrt(sum);
  }
  if(sum_tr>epsilon){
    sum_tr = 30/sqrt(sum_tr);//was 50
  }
  for(i=0;i<nb_joints;i++){
    v_scale(delta_tr[i],sum_tr,delta_tr[i]);
    v_scale(delta[i],sum,delta[i]);
    joints[i]->Translation::Add(delta_tr[i]);
    joints[i]->Rotation::AddToAxis(delta[i]);
  }
  }
  nb_joints++;
  if(shifted){ //shifting back
    // joints = joints_bk;
    //    reverse = reverse_bk;
    joints--;
    reverse--;
    nb_joints++;
  }
  return ret_val;
}

float KinematicChain::ScaleChain(float length){
  float f,len = 0;
  CVector3_t v;
  int i;
  for(i=0;i<nb_joints;i++){
    joints[i]->GetTranslation(v);
    len+= v_squ_length(v);
  }
  len = sqrtf(len);
  f = length/len;
  for(i=0;i<nb_joints;i++){
    joints[i]->Translation::Scale(f);
  }
  return len;
}

int KinematicChain::GetLink(int i, CVector3_t link){
#ifdef WITH_LAST_LINK
  if(i==nb_joints){
    last_link->GetTranslation(link);
    return 1;
  }
#endif
  if(i>=0 && i<nb_joints){
    joints[i]->GetTranslation(link);
    return 1;
  }
  return 0;
}


void KinematicChain::LinkRef(float *angle,int n,CVector3_t in,CVector3_t out){
  // to implement
}


void KinematicChain::PrintAxes(ostream& out){
  CVector3_t v;
  //    cout<<"printing"<<endl;
  // out<<endl;
  for(int i=0;i<nb_joints;i++){
    joints[i]->GetRotationAxis(v);
    out<<v<<" ";
  }
  out<<endl;
}

float *KinematicChain::Serialize(float *data, int offset){
  int j=0;
  int n = GetNbJoints();
  data[j++] = (float)(n-offset);//number of DOFS sent
  for(int i=offset;i<n;i++){
    v_copy(joints[i]->GetRotationAxis(),data+j);
    j+=3;
    v_copy(joints[i]->GetTranslation(),data+j); 
    j+=3;
    data[j++] = (float) reverse[i];
  }
#ifdef WITH_LAST_LINK
  v_copy(last_link->GetTranslation(),data+j); 
#endif
  return data;
}

/**
 * \brief reads a kinematic chain from a float array
 * \param offset joint with which to start in this chain (for partial kinematic chain updating) 
 */
int KinematicChain::Deserialize(const float *data, int offset){
  int j =0;
  int n = (int) data[j++];
  int last = n+offset;
  if(last>nb_joints){
    cout<<"warning not enough joints in Deserialize"<<endl;
    last = nb_joints;
  }
  for(int i=offset;i<last;i++){
    joints[i]->SetRotationAxis(data+j);
    j+=3;
    joints[i]->SetTranslation(data+j);
    j+=3;
    reverse[i] = sign(data[j]);
    j++;
  }
#ifdef WITH_LAST_LINK
  last_link->SetTranslation(data+j);
#endif
  return last-offset;
}


int KinematicChain::Deserialize(const float *data, int from, int to){
    int k,i=0;
    int n = (int) data[i];
    if(to>n-1){
        cout<<"warning: not enough data in Deserialize"<<endl;
        to = n-1;
    }    
    if(to-from+1>nb_joints){
        cout<<"warning: not enough joints in Deserialize"<<endl;
        to = nb_joints-1;
    }
    for(int j=from;j<=to;j++){
        k=j*7+1;
        joints[i]->SetRotationAxis(data+k);
        k+=3;
        joints[i]->SetTranslation(data+k);
        k+=3;
        reverse[i] = sign(data[k]);
        i++;
    }
#ifdef WITH_LAST_LINK
    k=to*7+2;
    last_link->SetTranslation(data+k);
#endif
  return to-from+1;
}



ostream& operator<<(ostream& out, const KinematicChain& kc)
{
  int n = kc.GetNbJoints();
  for(int i=0;i<n;i++){
    out<<*(kc.GetTransfo(i));
#ifdef KC_INVERTED
    out<<" "<<kc.IsInverted(i);
#endif
    out<<" - ";
  }
#ifdef WITH_LAST_LINK
  out<<*(kc.GetLastLink());
#endif
  out<<endl;
  return out;
}

istream& operator>>(istream& in, KinematicChain& kc)
{
  int n,m,i;
  RigidTransfo *j;
  CVector3_t v,w;
  char c1,c2;
  float angle;
  
  SkipComments(in);
  m=kc.GetNbJoints();
  in>>n; //reading the number of joints
  SkipComments(in);
  for(i=0;i<m;i++){ 
      //   in>>*(kc.GetTransfo(i));
      in>>angle>>c1>>v[0]>>v[1]>>v[2]>>c2>>w[0]>>w[1]>>w[2];
      kc.GetTransfo(i)->SetRotationAxis(v);
      kc.GetTransfo(i)->SetTranslation(w);
      kc.GetTransfo(i)->SetAngle(angle);
      SkipComments(in);
  }
  for(i=m;i<n;i++){ 
    j = new RigidTransfo();
    //    in>>(*j);
      in>>angle>>c1>>v[0]>>v[1]>>v[2]>>c2>>w[0]>>w[1]>>w[2];
      j->SetRotationAxis(v);
      j->SetTranslation(w);
      j->SetAngle(angle);
    kc.AddJoint(j,1);
    SkipComments(in);
  }
#ifdef WITH_LAST_LINK
    in>>v[0]>>v[1]>>v[2];
    if(!in.fail() ){
      kc.SetLastLink(v);
    }
#endif
  for(i=m;i>n;i--){ 
    kc.DeleteJoint();
  }
  return in;
}



