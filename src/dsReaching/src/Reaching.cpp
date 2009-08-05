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
#include "iCub/Reaching.h"



Reaching::Reaching(){
  float l1,l2;
  body = new BodySchema();
  l1 = body->GetUpperArmLength();
  l2 = body->GetForeArmLength();
  v_clear(shoulder_abs_pos);//not in body schema for now
  incr = pi/100;
  tol = 10;
  zero_vel_angle = 0.000001;

  configManifold.clear();

  alpha = 0.04; //0.08
  beta =0.01; // 0.02
  v4_set(0,0,0,0,v_angle);
  v_set(0,0,0,v_cart);
  v4_set(0,0,0,0,des_v_angle);
  v_set(0,0,0,des_v_cart);
  //  v4_set(0,0,0,0,weight_angle);
  v4_set(0.1/(l1+l2),0.1/(l1+l2),0.1/l2,0.1/l2,weight_angle);

  v_set(1,1,1,weight_cart);

  out_str = NULL;


#ifdef OBSTACLE_AVOIDANCE
  env= NULL;
  obstacle_rad = 40;
  nu = 0.1;
#endif


}

Reaching::~Reaching(){
  if (out_str) out_str->close();
  if (body) delete body;
}



void Reaching::SetBodySchema(BodySchema *b){
  if(body) delete body;
  body = b;
}
BodySchema* Reaching::GetBodySchema(){
  return body;
}



/************************************************
 * method SetArmLength
 *
 * sets the arm length.
 * Caution!!! This method also update the weights
 ***********************************************/


void Reaching::SetArmLength(float nl1,float nl2){
  CVector3_t wc;
  CVector4_t wa;
  body->SetArmLength(nl1,nl2);
  v4_set(1/(nl1+nl2),1/(nl1+nl2),1/nl2,1/nl2,wa);
  v_set(1,1,1,wc);
  SetWeights(wc,wa);
}


/***********************************************
 * method SetShoulderPosition
 *
 * sets the shoulder position with respoect to an 
 * absolute referential (the same one as SetTarget).
 * this referential is oriented like Eric's (x:left,
 * y: up, z:front)
 *************************************************/


void Reaching::SetShoulderPosition(CVector3_t pos){
  v_copy(pos,shoulder_abs_pos);
}

void Reaching::GetWeights(CVector3_t w_cart, CVector4_t w_angle)const{
#ifdef DYN_WEIGHTS
  v4_copy(base_weight_angle,w_angle);
  v_copy(base_weight_cart,w_cart);
#else
  v4_copy(weight_angle,w_angle);
  v_copy(weight_cart,w_cart);
#endif
}
void Reaching::SetWeights(CVector3_t w_cart, CVector4_t w_angle){
  v_copy(w_cart,weight_cart);
  v4_copy(w_angle,weight_angle);
#ifdef DYN_WEIGHTS
  v_copy(w_cart,base_weight_cart);
  v4_copy(w_angle,base_weight_angle);
#endif    
}





void Reaching::DecrementAngleWeights(float factor){
#ifdef DYN_WEIGHTS
  v4_scale(base_weight_angle,factor,base_weight_angle);
#else
  v4_scale(weight_angle,factor,weight_angle);
#endif
}

void Reaching::IncrementAngleWeights(float factor){
  float f = factor;
  if (f<0) return;
#ifdef DYN_WEIGHTS
  v4_scale(base_weight_angle,f,base_weight_angle);
#else
  v4_scale(weight_angle,f,weight_angle);
#endif
}

void Reaching::IncrementCartWeights(float factor){
  if (factor<0) return;
#ifdef DYN_WEIGHTS
  v_scale(base_weight_cart,factor,base_weight_cart);
#else
  v_scale(weight_cart,factor,weight_cart);
#endif
}

/**
 * gets the target in a global frame of ref
 */

void Reaching::GetTarget(CVector3_t tar) const {
  Local2AbsRef(target,tar);
}


/**
 * setting the target relatively to shoulder position
 * @param new_target target in global referential
 * @return returns 1 if target is reachable
 * @return -1 if target is pointable, 0 otherwise
 */
int Reaching::SetTarget(CVector3_t new_target){
  CVector3_t local_tar;
  Abs2LocalRef(new_target,local_tar);
  return SetLocalTarget(local_tar);
}


/**
 * setting the target relatively to shoulder position
 * @param new_target target in local referential
 * @return returns 1 if target is reachable
 * @return -1 if target is pointable, 0 otherwise
 */

//correct
int Reaching::SetLocalTarget(CVector3_t new_target)
{
  int res,pointing=1;
  float d,s;
  CVector3_t virt_target;
  float l1,l2;
  l1 = body->GetUpperArmLength();
  l2 = body->GetForeArmLength();

   v_copy(new_target,target);

  d= v_length(target);
  // pointing if target is too far

  if (d>l1+l2){  //body schema should do that
    s=(l1+l2-20)/d; 
    v_scale(target,s,target);
    pointing =-1;
  } 
  //get virtual target 

  body->VirtualTarget(target,virt_target);
  res = ComputeConfigManifold(virt_target); 

  res *= pointing;
  if (res==1){
#ifndef SILENT 
    cout << "reachable target ";coutvec(target);
#endif
  }
  else{
    if(res == -1){
#ifndef SILENT
      cout << "pointable target ";coutvec(target);
#endif
    }
    else{
#ifndef SILENT 
      cout << "unreachable target ";coutvec(target);
#endif
    }
    
  }
  return res;
}

int Reaching::UpdateTarget(){
  if(GetVisualMode()){
    return SetTarget(target);
  }
  else{
    return 0;
  }
}

float Reaching::UpdateLocalTarget(CVector3_t new_target){
  CVector3_t diff,v1,v2;
  CVector4_t v3,v4,v5;
  CMatrix3_t m1,m2;
  CMatrix4_t jac,m3,m4;

  if(new_target){
    v_sub(new_target,target,diff);
    if(v_length(diff)>tol){
      SetLocalTarget(new_target);
      return -1.0f;
    }
    else{
      //updating cartesian target
      v_copy(new_target,target);
    }
  }
  Angle2Cart(tar_angle,v1);
  v_sub(target,v1,v2); // xTn - K(thTo) a verifier
  Jacobian(tar_angle,jac);
  m3_4_square(jac,m1);//JJ'
  m_inverse(m1,m2); //(JJ')^{-1}
  m3_4_t_m_multiply(jac,m2,m3);//J'(JJ')^{-1}
  m4_3_v_multiply(m3,v2,v3);// J'(JJ')^{-1}(xTn - K(thTo)) check
  m_multiply(m3,jac,m4);// J'(JJ')^{-1}J  check
  m_identity(m3);
  m4_sub(m4,m3,jac);//(J'(JJ')^{-1}J -I)
  v4_sub(tar_angle,pos_angle,v4);
  m4_4_v_multiply(jac,v4,v5);//(J'(JJ')^{-1}J -I)(tar_angle-pos_angle)
  v4_add(v5,v3,v4);//(J'(JJ')^{-1}J -I)(tar_angle-pos_angle)+J'(JJ')^{-1}(xTn - K(thTo))
  //maybe smoother
  //  v4_scale(v4,0.5,v4);
  v4_add(tar_angle,v4,v3);
  //  coutvec4(v3);
  if(AcceptablePosition(v3)){
    v4_add(tar_angle,v4,tar_angle);
  }
  //checking precision
  Angle2Cart(tar_angle,v1);
  v_sub(target,v1,v2);
  return v_length(v2); 
}

int Reaching::SetArmConfigurationTarget(CVector4_t new_target)
{
  if(AcceptablePosition(new_target)){
    ArmConfig_t config;
    CVector2ArmConfig(new_target,&config);
    configManifold.clear();
    configManifold.push_back(config);
    Angle2Cart(new_target,target);
    return 1;
  }
  else{
#ifndef SILENT
    cout << "unreachable position: ";
    coutvec4(new_target);
#endif
    return 0;
  }
}




/**********************************************
// method ComputeConfigManifold
//
// Computes all the joint angle positions that
// correpond to a target location. If the target
// is too far, computes the positions that point
// to the target. A sampling is performed, whose
// coarsness in defined by <incr>.

@arg cart_goal goal in local FoF.
// returns:
// 1 if the target is reachable
// 0 otherwise
************************************************/




int Reaching::ComputeConfigManifold(CVector3_t cart_goal)
{  
  float alpha;
  ArmConfig_t tmpconfig;
#ifdef BODY_SCHEMA
  CVector4_t angles;
#endif

  configManifold.clear();

  for (alpha=-pi;alpha<pi;alpha+=incr){
#ifndef BODY_SCHEMA
    if(InvKinematics(cart_goal,alpha,tol,&tmpconfig)){
    configManifold.push_back(tmpconfig);
    }
#else
    if(body->InvKinematics(cart_goal,alpha,tol,angles)){
      tmpconfig.FromVector(angles);
      configManifold.push_back(tmpconfig);
    }
#endif
  }
  return (configManifold.size()>0);
}  


//int Reaching::DumpConfigManifold(char *filename)

/******************************
// method Hoap2Rob
//
// Converts joint angles from Hoap deg referential to 
// Robota rad referential
******************************/


void Reaching::Hoap2Rob(pArmConfig_t conf)
{
  conf->sfe = -conf->sfe*deg2rad+pi/2;
  conf->saa = conf->saa*deg2rad+pi/2;
  conf->shr = conf->shr*deg2rad;
  conf->eb = -conf->eb*deg2rad;
}
void Reaching::Hoap2Rob(CVector4_t conf)
{
  conf[0] = -conf[0]*deg2rad+pi/2;
  conf[1] = conf[1]*deg2rad+pi/2;
  conf[2] = conf[2]*deg2rad;
  conf[3] = -conf[3]*deg2rad;
}


/******************************
// method Rob2Hoap
//
// Converts joint angles from  Robota rad referential
// to Hoap deg referential

******************************/


void Reaching::Rob2Hoap(pArmConfig_t conf)
{
  conf->sfe = (-conf->sfe+pi/2) *rad2deg;
  conf->saa = (conf->saa-pi/2)*rad2deg;
  conf->shr = conf->shr*rad2deg;
  conf->eb = -conf->eb*rad2deg;
}

void Reaching::Rob2Hoap(CVector4_t conf)
{
  conf[0] = (-conf[0]+pi/2) *rad2deg;
  conf[1] = (conf[1]-pi/2)*rad2deg;
  conf[2] = conf[2]*rad2deg;
  conf[3] = -conf[3]*rad2deg;
}


void Reaching::Rob2Icub(CVector4_t angles){
  angles[0] *= -rad2deg;   //sfe stays the same
  angles[1] = (-angles[1]+pi/2)*rad2deg;//saa
  angles[2] = (-angles[2]+pi/2)*rad2deg;//shr
  angles[3] = angles[3]*rad2deg; //eb
}

void Reaching::Icub2Rob(CVector4_t angles){
  angles[0] *= -deg2rad;   //sfe stays the same
  angles[1] = -angles[1]*deg2rad+pi/2;//saa
  angles[2] = -angles[2]*deg2rad+pi/2;//shr
  angles[3] = angles[3]*deg2rad; //eb
}

/***********************************
// method Angle2Cart
//
// Performs the forward kinematic function
// Input in Robota rad referential
// Output in Eric's referential
//
************************************/



void Reaching::Angle2Cart(pArmConfig_t config, CVector3_t out){
  CVector4_t angle;
  v4_set(config->sfe,config->saa,config->shr,config->eb,angle);
  body->Angle2Cart(angle,out);
}


void Reaching::Angle2Cart(CVector4_t angle, CVector3_t out)
{
  body->Angle2Cart(angle,out);
}



void Reaching::Angle2CartAbs(CVector4_t angle, CVector3_t out)
{
  CVector3_t local;
  Angle2Cart(angle,local);
  Local2AbsRef(local,out); 
}

void Reaching::ElbowPosition(CVector4_t angle, CVector3_t out)
{
  body->ElbowPosition(angle,out);
}


/*-------------------------------------------------
 * method GetAngle
 *------------------------------------------------*/
/**
 * gives the current angular position
 *
 * @param angle the vector in which to put the current angular position
 *
 ***************************************************/
void Reaching::GetAngle(CVector4_t angle) const {
  v4_copy(pos_angle,angle);
}
void Reaching::GetTargetAngle(CVector4_t angle) const {
  v4_copy(tar_angle,angle);
}
/*-------------------------------------------------
 * method GetAngle
 *------------------------------------------------*/
/**
 * gives the current angular position
 *
 * @return the current angular position
 *
 ***************************************************/
// CVector4_t Reaching::GetAngle(){
//   return pos_angle;
// }

/*-------------------------------------------------
 * method GetAbsHandPosition
 *------------------------------------------------*/
/**
 * gives the current cartesian position in a global ref.
 *
 * @param out the vector in which to put the current angular position
 *
 ***************************************************/
void Reaching::GetAbsHandPosition(CVector3_t out) const {
  Local2AbsRef(pos_cart,out);
}



void Reaching::GetLocalHandPosition(CVector3_t out) const {
  v_copy(pos_cart,out);
}


/*****************************************
// method InvKinematics
//
// Inverse kinematics method based on florent's
// method. If the target is too far, the method will return
// a position that points to the target (will reach a point on
// the shoulder-target axis
//
// args:
// relTar : target (in cartesian coordinates , eric's referential, origin at shoulder)
// alpha : angle with the vertical (parameter of the solution curve)
// out: where the solution is put
//
// returns:
// 1. if the target is reachable or pointable while keeping in the joint angle bounds
// 0. Otherwise.

// Note: this method is not the cleanest, could be implemented using
// quaternions (cf Hestenes)

****************************************/


int  Reaching::InvKinematics (CVector3_t relTar, float alpha, float toler,  pArmConfig_t out){
  CVector3_t v;
  int ret;
  ret=body->InvKinematics(relTar,alpha,toler,v);
  out->FromVector(v);
  return ret;
}


/*******************************************
// method FindNearestPosition
//
// Finds the nearest joint angle position to <pos1> out of
// the set of positions that are in the configManifold
// Closness is measured using the euclidean distance.
// 
// args:
// @param pos1 the distance to which the candidates are evaluated
   @param out where the result is put 
//
// @return the squared euclidean distance to target
//
// Note: One could have a distance measure which takes into account
// the physical world (energy, work, ...)
// This is a brute force method (trying all possibilities). One could
// think of something smarter if one has an inverse kinematics function
********************************************/


float Reaching::FindNearestPosition(pArmConfig_t pos1,pArmConfig_t out)
{
  ArmConfigList_t::iterator it;
  pArmConfig_t min;
  float mindist,dist;

  //  cout<<"manifold size: "<< configManifold.size()<<endl;
  mindist = AngleDistance(pos1,&(configManifold[0]));
  min = &(configManifold[0]);
  for(it=configManifold.begin();it!=configManifold.end();it++){
    dist = AngleDistance(pos1,&(*it)); 
    if(dist<mindist){
      mindist = dist;
      min = &(*it);
    }
  }
  out->sfe = min->sfe;
  out->saa = min->saa;
  out->shr = min->shr;
  out->eb = min->eb;
  return mindist;
}


/*************************************
// method AngleDistance
//
// return squared euclidean distance between two
// joint angle configurations
***************************************/

float Reaching::AngleDistance(pArmConfig_t pos1,pArmConfig_t pos2)
{
  float sum = 0.0;

  sum += (pos1->sfe - pos2->sfe)*(pos1->sfe - pos2->sfe);
  sum += (pos1->saa - pos2->saa)*(pos1->saa - pos2->saa);
#ifndef UPPER_ARM_PRIORITY
  sum += (pos1->shr - pos2->shr)*(pos1->shr - pos2->shr);
  sum += (pos1->eb  - pos2->eb) * (pos1->eb  - pos2->eb);
#endif
  return sum;
}


float Reaching::AngleDistance(CVector4_t pos1,CVector4_t pos2)
{
  float sum = 0.0;
  int ind, i;
#ifdef UPPER_ARM_PRIORITY
  ind = 2;
#else
  ind = 4;
#endif
  for(i=0;i<ind;i++){
    sum += (pos1[i]-pos2[i])*(pos1[i]-pos2[i]);
  }
  return sum;
}


int Reaching::ArmConfig2CVector(pArmConfig_t arm,CVector4_t out){
  if(arm){
  out[0] = arm->sfe;
  out[1] = arm->saa;
  out[2] = arm->shr;
  out[3] = arm->eb;
  return 1;
  }
  else return 0;
}


int Reaching::CVector2ArmConfig(CVector4_t v, pArmConfig_t arm){
  if(arm){
   arm->sfe = v[0];
   arm->saa= v[1];
   arm->shr= v[2];
   arm->eb= v[3];
  return 1;
  }
  else return 0;
}


/***************************************************
 * method Jacobian
 *
 * valid for eric's referential and robota rad ref. 
 * args:
 * --- 
 * @param v the angle position at which to compute
 * the Jacobian matrix
 * @param jac  a 4x4 matrix filled up by the method
 * the last row contains zeros and 1 on the diagonal.
 * 
 * indices are 0 1 2 3 4 \\ 5 6 7 8 \\ ...    
***********************************************/
void Reaching::Jacobian(CVector4_t v, CMatrix4_t jac){
  body->Jacobian(v,jac);
}



/***************************************************
 * method IntermediateJacobian
 *
 * computes the Jacobian for a point on the arm.
 * valid for eric's referential and robota rad ref. 
 * args:
 * --- 
 * @param link 1 for the upper arm, 2 for the foremarm
 * @param len length of that link
 * @param v the angle position at which to compute
 * the Jacobian matrix
 * @param jac  a 4x4 matrix filled up by the method
 * the last row contains zeros and 1 on the diagonal.
 *     
***********************************************/


void Reaching::IntermediateJacobian(int link, float len,CVector4_t v, CMatrix4_t jac){
  body->IntermediateJacobian(link,len,v,jac);
}


/**************************************************
 * method ReachingStep
 *
 * Performs a reaching step, using the Grossberg's VITE algorithm
 * on the cartesian and joint angle space. Unification is enforced
 * by constraining the coherence of joint angles and cartesian positions
 * (by use of lagrange multipliers)
 * 
 ***************************************************/


#ifdef TRACE_COSTS
float
#else
void 
#endif

Reaching::ReachingStep(){

  ArmConfig_t angletarget;
  CVector4_t tmp1,tmp3; // temporary variables
  CVector3_t tmp13;
  
  float dist;
  if (configManifold.empty()){ // no movement
    v4_copy(pos_angle,tar_angle);
    v_copy(pos_cart,target);
  }
  else{
    dist = UpdateLocalTarget();
    if(dist>tol){
      dist = FindNearestPosition(&position,&angletarget); 
      ArmConfig2CVector(&angletarget,tar_angle);
    }
  }

  // vite in angle space and cart space
  Vite4d(tar_angle,pos_angle,v_angle,des_angle,des_v_angle);
  Vite3d(target,pos_cart,v_cart,des_cart,des_v_cart);


#ifdef OBSTACLE_AVOIDANCE
  // find the closest obstacle
  float ro,point;
  float gamma;
  CMatrix4_t ijac;
  v4_clear(des_a_angle_obs);
  if(env){
    for (i=1;i<3;i++){
      for(int j=0;j<env->CountManipulableObjects();j++){
	LinkDistanceToObject(i,env->GetObject(j),&ro,&point,tmp13);
	//      coutvec(tmp13);
	IntermediateJacobian(i,point,pos_angle,ijac);
	m3_4_t_v_multiply(ijac,tmp13,tmp1);
	// eq. 18 of Khatib,icra'85
	if(ro<=obstacle_rad){
	  gamma = nu *(1/ro -1/obstacle_rad)/(ro*ro); //(ro*ro) 
	  //test
	  //   gamma *= -v4_dot(tmp1,v_angle)/50;
	  //   gamma = max(0.0,gamma);
	}
	else{
	  gamma =0;
	}
	v4_scale(tmp1,gamma,tmp1);
	v4_add(des_a_angle_obs,tmp1,des_a_angle_obs);
      }
    }
    v4_add(des_a_angle_obs,des_v_angle,des_v_angle);
    v4_add(pos_angle,des_v_angle,des_angle);
  }
#endif

  SetAnglesInRange(des_angle);
  // in case we are too close to workspace boundaries
  v4_sub(des_angle,pos_angle,des_v_angle);
  v_sub(des_cart,pos_cart,des_v_cart);     

  UpdateWeights();
  // coherence enforcement
  ProjectVector(des_v_cart,des_v_angle,tmp3);

  v4_add(tmp3,pos_angle,tmp1);    

  SetAnglesInRange(tmp1);
  Angle2Cart(tmp1,tmp13); // pos_cart  -- to modify for visual servo

    v4_sub(tmp1,pos_angle,v_angle);
    v_sub(tmp13,pos_cart,v_cart);

    v_copy(tmp13,pos_cart);
    v4_copy(tmp1,pos_angle);

#ifdef TRACE_COSTS
    float costs;
    costs = GetCosts(des_angle,des_cart);
#endif


#ifdef TRACE_COSTS
    return costs;
#endif
}


void Reaching::ProjectVector(CVector3_t v3,CVector4_t v4, CVector4_t out){
  CMatrix4_t jac; 
  CVector4_t v41,v42;
  CVector3_t v31,v32;
  CMatrix3_t m31,m32;
  Jacobian(pos_angle,jac); //J   -- to modify for visual servo
  m3_4_v_multiply(jac,v4,v31); //J*v4
  v_sub(v3,v31,v32);//v3-J*v4
  m3_4_weighted_square(jac,weight_angle,m31);//JWaJ'
  m3_add_diag(m31,weight_cart,m32); //JWaJ'+Wx
  m_inverse(m32, m31); //(JWaJ'+Wx)^-1
  m3_v_multiply(m31,v32,v31);//(JWaJ'+Wx)^-1*(v3-J*v4)
  m3_4_t_v_multiply(jac,v31,v41);//J'(JWaJ'+Wx)^-1*(v3-J*v4)
  m4_diag_v_multiply(weight_angle,v41,v42);//WaJ'(JWaJ'+Wx)^-1*(v3-J*v4)
  v4_add(v4,v42,out);//v4+WaJ'(JWaJ'+Wx)^-1*(v3-J*v4)
}


void Reaching::SetAnglesInRange(CVector4_t angle){
  body->SetAnglesInRange(angle);
}


void Reaching:: UpdateWeights(){   // -- to modify for visual servo

  CVector4_t min_angle, max_angle;
  body->GetAnglesLowerBound(min_angle);
  body->GetAnglesUpperBound(max_angle);


#ifdef DYN_WEIGHTS
#ifdef SAI_WEIGHT_MODULATION
  float sai = SpuriousAttractorIndex();
#endif
  for(int i=0;i<4;i++){

    weight_angle[i] = 0.5*base_weight_angle[i]*//
      (cos((pos_angle[i] - min_angle[i])*2*pi/
	   (max_angle[i]-min_angle[i])+pi)+1);


#ifdef DIST_WEIGHT_MODULATION
 weight_angle[i] *= (1.0-abs(tar_angle[i]-pos_angle[i])/(max_angle[i]-min_angle[i]));
#endif
#ifdef SAI_WEIGHT_MODULATION
    weight_angle[i] *= min(2,sai);
#endif
  }
#ifdef END_CART_WEIGHT
  CVector3_t tmp;
  v_sub(target,pos_cart,tmp);    
  float cart_dist = max(v_length(tmp)/150,1e-10);
  for(int i=0;i<4;i++){
    weight_angle[i] /= cart_dist;
  }
  cout<<"dist "<< cart_dist*150<<" weights ";coutvec4(weight_angle);
#endif

#endif

}


/*-----------------------------------------------------------------
 * method Vite3d
 *----------------------------------------------------------------*/
/**
 * performs the vite dynamical system in 3d
 * @param target the target
 * @param pos the position at time t
 * @param speed the speed at time t
 * @param new_pos the position at time t+1 (output of the method)
 * @param new_speed the speed at time t+1 (output of the method
*/
void Reaching::Vite3d(CVector3_t target,CVector3_t pos,CVector3_t speed,
		      CVector3_t new_pos, CVector3_t new_speed) {
	CVector3_t tmp1,tmp2;
	//vite in cart space
	// v update
	v_sub(target,pos,tmp1);
	v_scale(tmp1,beta,tmp2);
	v_sub(tmp2,speed,tmp1);
  	v_scale(tmp1,alpha,tmp2);
	v_add(speed,tmp2,new_speed); // desired velocity
	// p update  
	v_add(pos,new_speed,new_pos);
}


/*-----------------------------------------------------------------
 * method Vite4d
 *----------------------------------------------------------------*/
/**
 * performs the vite dynamical system in 4d
 * @param target the target
 * @param pos the position at time t
 * @param speed the speed at time t
 * @param new_pos the position at time t+1 (output of the method)
 * @param new_speed the speed at time t+1 (output of the method
*/
void Reaching::Vite4d(CVector4_t target,CVector4_t pos,CVector4_t speed,
		      CVector4_t new_pos, CVector4_t new_speed) {
	CVector4_t tmp1,tmp2;
	//vite in joint space
	// speed update
	v4_sub(target,pos,tmp1);
	v4_scale(tmp1,beta,tmp2);
	v4_sub(tmp2,speed,tmp1);
  	v4_scale(tmp1,alpha,tmp2);
	v4_add(speed,tmp2,new_speed); 
	// position update  
	v4_add(pos,new_speed,new_pos);
}




/**************************************************
 * method SpuriousAttractorIndex
 *
 * @return a number indicating how far we are from a
 * spurious attractor
 * 
 ****************************************************/
float Reaching::SpuriousAttractorIndex()
{
  CVector4_t da,tmp14,tmp24;
  CVector3_t dx;
  CMatrix4_t jac; //jacobian matrix
  CVector3_t wxm1,tmp13;
  float dalength;

  v_sub(target,pos_cart,dx);
  v4_sub(tar_angle,pos_angle,da);
 
 
  Jacobian(pos_angle,jac);
  inverse_diag3(weight_cart,wxm1);
  m3_diag_v_multiply(wxm1,dx,tmp13);
  m3_4_t_v_multiply(jac,tmp13,tmp14);
  m4_diag_v_multiply(weight_angle,tmp14,tmp24);
  v4_add(da,tmp24,tmp14);
  
  dalength = v4_length(da);
  if (dalength < 0.00001){
    return 2;
    }
  else{
    return (v4_length(tmp14))/dalength;
  }
}

#ifndef LIGHT_VERSION
// returns hoap angles in rad
void Reaching::RetrieveJointAngles(pHoap2_JointAngles angles)
{
  CVector4_t v1,v2;
   v4_copy(pos_angle,v1);
  Rob2Hoap(v1);
  v4_scale(v1,deg2rad,v2);
  angles->R_SFE = v2[0];
  angles->R_SAA = v2[1];
  angles->R_SHR = v2[2];
  angles->R_EB =  v2[3];
}


// angles given in Hoap deg referential
int Reaching::SetActualHoapPosition(pHoap2_JointAngles angles){
  CVector4_t v;
  v[0] = angles->R_SFE;
  v[1] = angles->R_SAA;
  v[2] = angles->R_SHR;
  v[3] = angles->R_EB;
  Hoap2Rob(v);
  return SetActualRobPosition(v);
}
#endif


#ifdef TRACE_COSTS

// works for homogeneous weights only. not clean
float Reaching::GetCosts(CVector4_t des_angle,CVector3_t des_pos){
  CVector4_t v1,v2;
  CVector3_t w1,w2;
  float s1,s2;
  v4_sub(des_angle,pos_angle,v1);
  //m4_diag_v_multiply(weight_angle,v1,v2);
  if (weight_angle[0] == 0){
    s1 = v4_dot(v1,v1);
    return s1;
  }
  else{
    v4_scale(v1,1/weight_angle[0],v2);
    s1 = v4_dot(v1,v2);
  }
  v_sub(des_cart,pos_cart,w1);
  
  if (weight_cart[0] == 0){
    s2 = v_dot(w1,w1);
    return s2;
  }
  else{
     v4_scale(w1,1/weight_cart[0],w2);
     s2 = v_dot(w1,w2);
  }

  //  m3_diag_v_multiply(weight_cart,w1,w2);
 
  return (s1+s2)/(1/weight_cart[0]+1/weight_angle[0]);
}

#endif


/*------------------------------------------------
 * method SetActualRobPosition
 *-----------------------------------------------*/
 /**
 * @param angles the new position in rad Robota frame of reference 
 * @return 1 if the new position is within the workspace boundaries
 * and 0 otherwise (in that case nothing happens)
 * 
***************************************************/

int Reaching::SetActualRobPosition(CVector4_t angles){
  if(AcceptablePosition(angles)){
    v4_copy(angles,pos_angle);
    v4_set(0,0,0,0,v_angle);
    v_set(0,0,0,v_cart);
    Angle2Cart(pos_angle,pos_cart);
    CVector2ArmConfig(pos_angle,&position);
    v_clear(tar_angle);
    return 1;
  }
  else{
    return 0;
  }
} 


/*------------------------------------------------
 * method SetActualRobPosAndSpeed
 *-----------------------------------------------*/
 /**
 * @param position the new position in rad Robota frame of reference 
 * @param speed the new joint angle speed 
 * @return 1 if the new position is within the workspace boundaries
 * and 0 otherwise (in that case nothing happens)
 * 
***************************************************/
int Reaching::SetActualRobPosAndSpeed(CVector4_t position, CVector4_t speed){
  CMatrix4_t jac;
  if(SetActualRobPosition(position)){
    v4_copy(speed,v_angle);
    Jacobian(pos_angle,jac);
    m3_4_v_multiply(jac,speed,v_cart);
   return 1;
  }
  else{
    return 0;
  }
}

/*------------------------------------------------
 * method SetRobAcceleration
 *-----------------------------------------------*/
 /**
 * @param acc the new acceleration in rad Robota frame of reference 
 * @return 1 if the new position is within the workspace boundaries
 * and 0 otherwise (in that case the position is put on the boundaries)
 * 
***************************************************/
int Reaching::SetRobAcceleration(CVector4_t acc){
  CVector3_t tmpc;
  CVector4_t tmpa;
  int ret =1;
  float tmp;
  CVector4_t min_angle, max_angle;
  body->GetAnglesLowerBound(min_angle);
  body->GetAnglesUpperBound(max_angle);



  v4_copy(pos_angle,tmpa);
  v4_add(v_angle,acc,v_angle);
  v4_add(pos_angle,v_angle,pos_angle);
  for (int i=0;i<4;i++){
    tmp = min(max_angle[i],max(min_angle[i],pos_angle[i]));
    if(tmp !=pos_angle[i]){
      ret = 0;
      pos_angle[i] = tmp;
    }
  }
  if (!ret){
    v4_sub(pos_angle,tmpa,v_angle);
  }
  v_copy(pos_cart,tmpc);
  Angle2Cart(pos_angle,pos_cart);
  v_sub(pos_cart,tmpc,v_cart);
  return ret;
}




/******************************************************************
// method SaveConfig
//
// writes the arm configuration and target location in
// a file. Format: "x y z: th1 th2 th3 th4" (x,y,z = target location)
// args: <fname> the filename where to write the arm configuration
// and target location (in shoulder-centered, erics referential)
// 
// 
// returns: 1 if sucessful writing, 0 otherwise
******************************************************************/

int Reaching::SaveConfig(string *fname)
{
  if (!out_str){
    out_str = new ofstream(fname->c_str());
    if (out_str->fail()) {
      free(out_str);
      out_str = NULL;
      return 0;
    }
  }
  *out_str << target[0] <<" "<<target[1] <<" "<< target[2] << ":"<< *this<<endl;
#ifdef BODY_SCHEMA
  *out_str << *body;
#endif
  return 1;
}



int Reaching::RecordTraj_ACT(CVector4_t initpos, CVector4_t targ, string& fname)
{
  ofstream out(fname.c_str());
  if(!out.fail()){
    if(SetArmConfigurationTarget(targ)){
      return StartRecordTraj(initpos,out);
    }
    return -2; //unvalid target position
  }
  return -3; //unvalid filename
}


int Reaching::RecordTraj(CVector4_t initpos, CVector3_t targ, string& fname)
{
  ofstream out(fname.c_str());
   if(!out.fail()){
    if(SetTarget(targ)){
      return StartRecordTraj(initpos,out);
    }
    return -2; //unvalid target position
  }
  return -3; //unvalid filename
}
      


/******************************************
 *
 * method StartRecordTraj
 *
 * returns:
 * ------- 
 * 0 if target reached
 * 1 if target not reached
 * -1 if initial position not valid
 ******************************************/


int Reaching::StartRecordTraj(CVector4_t initpos,ofstream& out){
  if(SetActualRobPosition(initpos)){
    out << *this << endl;
    for (int i=0;i<MAX_IT && !TargetReached();i++){
      ReachingStep();
      out << *this << endl;
    }
    out.close();
    if(TargetReached()){
      return 0; // everything ok;
    }
    else{
      return 1;
    }
  } 
  return -1; //unvalid init position
}

int Reaching::StartRecordTraj(CVector4_t initpos){
  if(SetActualRobPosition(initpos)){
    return Reach();
  }
  else{
    return -1; //unvalid init position
  }
} 

int Reaching::Reach(){
  for (int i=0;i<MAX_IT && !TargetReached();i++){
    ReachingStep();
  }
  if(TargetReached()){
    return 0; // everything ok;
  }
  else{
    return 1;
  }
} 



/**********************************************
// method AcceptablePosition
//
// returns: 
// 1 if <angle> is within workspace boundaries
// 0 otherwise. 
***********************************************/

// angles must be given in rad robota referential
int Reaching::AcceptablePosition(CVector4_t angle){
  return body->AcceptablePosition(angle);
}

/*************************************************
 * method TargetReached
 * 
 * Target is considered to be reached if within a distance of <tol>
 * from end-effector and the arm is not moving anymore
 *
 * returns: 1 if target is reached
 *          0 otherwise
 *************************************************/

inline int Reaching::TargetReached(){
  CVector3_t dist;
  float d;
  v_sub(target,pos_cart,dist);
  d = v_length(dist);
  return d<tol && HasStopped();
}

inline int Reaching::HasStopped(){
  return v4_length(v_angle)<zero_vel_angle;
}


/**********************************************************************

Some useful methods to perform tests and simulations

***********************************************************************/
void Reaching::RandomAnglePos(CVector4_t angle){
  body->RandomAnglePos(angle);
}


/**
 * output in shoulder referential
 */

void Reaching::RandomCartPos(CVector3_t cart){
  body->RandomCartPos(cart);
}


void Reaching::RandomCartAbsPos(CVector3_t cart){
  CVector3_t cart_l; //local ref
  RandomCartPos(cart_l);
  Local2AbsRef(cart_l,cart);
}

void Reaching::Local2AbsRef(const CVector3_t cart_l,CVector3_t cart) const{
  v_add(cart_l,shoulder_abs_pos,cart);
}


void Reaching::Abs2LocalRef(const CVector3_t cart, CVector3_t out) const {
  v_sub(cart,shoulder_abs_pos,out);
}

int Reaching::GetVisualMode(){return body->GetVisualMode();}



#ifdef MANY_JUMPS

int Reaching::PerformTrajectories(string *infname, string *outfname){

    ifstream istr(infname->c_str());

  if(istr.fail()){
    cout <<"can't open file "<<infname->c_str()<<endl;
    return 0;
  }
  else{
    int timeOfJump, tar_ok,tar_spec;
    string fname(*outfname);
    string line;
    char nc[80];
    CVector4_t pos, new_tar;
    int n=0;

    sprintf(nc,"%d",n);
    fname = *outfname  + nc +".txt"; // appending the traj number to the filename
    ofstream out(fname.c_str());
    getline(istr,line);
    if(4 != sscanf(line.c_str(),"%f %f %f %f \n", &(pos[0]),&(pos[1]),&(pos[2]),&(pos[3]))){
#ifndef SILENT
	cout << "line " << n << " not decripted"<<endl;
#endif
    }
    else{
      SetActualRobPosition(pos);
    }
    out << *this << endl; 
    while(!istr.eof()){
      getline(istr,line);
      tar_spec = sscanf(line.c_str(),"%d: %f %f %f %f\n", 
			     &timeOfJump, &(new_tar[0]),&(new_tar[1]),&(new_tar[2]), &(new_tar[3]));
      if ((tar_spec !=5) && (tar_spec != 4)){
#ifndef SILENT
	//cout << "line " << n << " not decripted"<<endl;
#endif
      }
      else{
	if (tar_spec ==4){                        // a 3d cartesian end-effector is specified 	
	  tar_ok = SetTarget(new_tar);            // only the first 3 values
	}
	else{
	  tar_ok = SetArmConfigurationTarget(new_tar); // a 4d arm configuration is specified
	  //	  coutvec4(new_tar);
	}
	if(tar_ok){
	  for (int i=0;i<timeOfJump && !TargetReached();i++){
	    ReachingStep();
#ifdef LEARNING
	    LearningLocal();
#endif
	    out << *this << endl;
	  }
#ifdef LEARNING
	  LearningGlobal();
#endif
	}
      } 
    }
    out.close();
    if(TargetReached()){
      return 1;
    }
  }
  return 0;
}
#endif




#ifdef TARGET_JUMP

int Reaching::PerformTrajectories(string *infname, string *outfname){
  int n =1;
  
  ifstream istr(infname->c_str());

  if(istr.fail()){
    cout <<"can't open file "<<infname->c_str()<<endl;
    return 0;
  }
  else{
    int timeOfJump;
    string fname(*outfname);
    string line;
    char nc[80];
    CVector3_t tar, new_tar;
    CVector4_t pos;
    while(!istr.eof()){
      sprintf(nc,"%d",n);
      fname = *outfname  + nc +".txt";
      getline(istr,line);
      if(11 != sscanf(line.c_str(),"%f %f %f:%f %f %f %f:%d:%f %f %f ",&(tar[0]),&(tar[1]),&(tar[2]),
		     &(pos[0]),&(pos[1]),&(pos[2]),&(pos[3]), &timeOfJump, &(new_tar[0]),&(new_tar[1]),&(new_tar[2]))){
#ifndef SILENT
	cout << "line " << n << " not decripted"<<endl;
#endif
      }
      else{
	RecordTraj(pos,tar,fname,timeOfJump,new_tar);
      }
      n++;
    }
  }
  return n;
}

#else
#ifndef MANY_JUMPS


// returns the  number of successful reaching movements
int Reaching::PerformTrajectories(string *infname, string *outfname,int max_traj){
  int n =1;
  int cnt_reached = 0; //counts the number of successful movement
  ifstream istr(infname->c_str());

  if(istr.fail()){
    cout <<"can't open file "<<infname->c_str()<<endl;
    return 0;
  }
  else{
    string fname(*outfname);
    string line;
    char nc[80];
    CVector3_t tar;
    CVector4_t pos;
    if(max_traj<0){max_traj = INT_MAX;}
    while(!istr.eof() && n<=max_traj){
      sprintf(nc,"%d",n);
      fname = *outfname  + nc +".txt";
      getline(istr,line);
      if(7 != sscanf(line.c_str(),"%f %f %f : %f %f %f %f ",&(tar[0]),&(tar[1]),&(tar[2]),
		     &(pos[0]),&(pos[1]),&(pos[2]),&(pos[3]))){
#ifndef SILENT
	cout << "line " << n << " not decripted"<<endl;
#endif
      }
      else{
	if(!RecordTraj(pos,tar,fname)){
	  cnt_reached++;
	}
      }
      n++;
    }
  }
  return cnt_reached;
}

#endif
#endif

/*--------------------------------------------
 * method PerformPerturbedTrajectory
 *-------------------------------------------*/
/**
 * @param infname the file name specifying the initial position, the
 *  target location, and the external perturbation
 * @param outfname the file name where to write the resulting trajectory
 * @return 1 if the target is reached, 0 if the target is not reached, a
 * negative number in case of error
 ************************************************************************/ 

int Reaching::PerformPerturbedTrajectory(string *infname, string *outfname){
  string line;
  int i,perturbationTime,n=1;
  CVector3_t tar;
  CVector4_t pos,acc;
  ifstream istr(infname->c_str());
  string fname(*outfname);

  if(istr.fail()){
    cout <<"can't open file "<<infname->c_str()<<endl;
    return 0;
  }
  else{
    getline(istr,line);
    if(7 != sscanf(line.c_str(),"%f %f %f : %f %f %f %f ",&(tar[0]),&(tar[1]),&(tar[2]),
		     &(pos[0]),&(pos[1]),&(pos[2]),&(pos[3]))){
#ifndef SILENT
      cout << "line " << n << " not decripted"<<endl;
#endif
      return -3;
    }
    else{
      if(SetTarget(tar)){
	if(SetActualRobPosition(pos)){
	  ofstream out(fname.c_str());
	  while(!istr.eof()){ 
	    n++;
	    getline(istr,line);
	    if(5!= sscanf(line.c_str(),"%d : %f %f %f %f",&perturbationTime, 
			  &(acc[0]),&(acc[1]), &(acc[2]),&(acc[3]))){
#ifndef SILENT
	      cout << "line " << n << " not decripted"<<endl;
#endif
	    }
	    else{
	      for(i=1;i<perturbationTime;i++){
		ReachingStep();
		out << *this << endl;
	      }
	      SetRobAcceleration(acc);
	      out << *this << endl;
	    }
	  }
	  for(i=0;i<MAX_IT && !TargetReached();i++){
	    ReachingStep();
	    out << *this << endl;
	  }
	  out.close();
	  return TargetReached();
	}
	else{
	  return -1;
	}
      }
      else{
	return -2;
      }
    }
  }
}



#ifdef OBSTACLE_AVOIDANCE
int Reaching::LinkDistanceToObject(int link, EnvironmentObject *obj, float *dist, 
			 float *point, CVector3_t contact_vector){
  CVector3_t objPos;
  CVector3_t lowerLinkExtr;
  CVector3_t upperLinkExtr;
  CVector3_t v1,v2,linkv,u,vertexPos,v_tmp,tmp3;
  CMatrix4_t ref,tmpmat,tmpmat2;
  CVector3_t s1,s2;
  int i,j,k,dir1,dir2,pindex,min_i;
  float alldists[12]; //closest distance to each edge
  float allpoints[24]; // closest pair of points on each edge
  float min_dist;
  int s3[3];

  for (i=0;i<12;i++){
    alldists[i]=FLT_MAX;
  }

  switch(link){
  case 1:
    v_clear(upperLinkExtr);
    ElbowPosition(pos_angle,lowerLinkExtr);
    break;
  case 2:
    ElbowPosition(pos_angle,upperLinkExtr);
    v_copy(pos_cart,lowerLinkExtr);
    break;
  default:
    cout<<"unvalid link number"<< endl;
  }
  if(!obj){
    return 0;
  }

    Abs2LocalRef(obj->GetPosition(), objPos);
  v_sub(lowerLinkExtr,objPos,v1);
  v_sub(upperLinkExtr,objPos,v2);

  m_copy(obj->solid->m_ref.m_orient,ref); //stupid to do it each time
  v_clear(s3);
  
  //checking when two parallel sides must me checked
  for(i=0;i<3;i++){
    s1[i] = v_dot(v1, &ref[i*4]);
    s2[i] = v_dot(v2, &ref[i*4]);
    if (s1[i]*s2[i]>=0){
      s3[i] = sign(s1[i]+s2[i]);
    }
  }
  //    cout << "s3: ";coutvec(s3);
  m_copy(ref,tmpmat);
  for(i=0;i<3;i++){
    if(s3[i]){
      v_sub(lowerLinkExtr,upperLinkExtr,linkv);
      v_scale(obj->solid->m_size,-0.5,u);
      u[i] *=-s3[i];
      v_add(objPos,u,vertexPos);
      //      cout<<"vPos "<<i<<": ";coutvec(vertexPos);
      v_scale(&(ref[i*4]),s3[i],&(tmpmat[i*4]));
      // cout<<"norm "<<i<<": ";coutvec((tmpmat+i*4));
      m_inverse(tmpmat,tmpmat2);
      if(v_dot(&(tmpmat[i*4]),linkv)<0){
	v_sub(lowerLinkExtr,vertexPos,v_tmp);
	*point = 1;
      }
      else{
	v_sub(upperLinkExtr,vertexPos,v_tmp);
	*point = 0;
      }

      v_transform_normal(v_tmp,tmpmat2,tmp3); 

      if(tmp3[(i+1)%3]>=0 && tmp3[(i+2)%3]>=0 // the link points to the rectangle
	 && tmp3[(i+1)%3]<=obj->solid->m_size[(i+1)%3] && 
	 tmp3[(i+2)%3]<=obj->solid->m_size[(i+2)%3]){
	if(tmp3[i]<0){
	  return 0; // there is a collision
	}
	else{
	  *dist = tmp3[i];
	  v_copy(&(tmpmat[i*4]),contact_vector);
	  v_scale(contact_vector,*dist,contact_vector);
	  return 1;
	}
      }
    }
  }
  // the link does not point to a rectangle -> look for the edges
  v_scale(obj->solid->m_size,-0.5,u);
  for(i=0;i<3;i++){// each kind of edge
      dir1 = s3[(i+1)%3]; 
      dir2 = s3[(i+2)%3];
    for(j=0;j<2;j++){ 
      if(dir1 == 0 || dir1==2*j-1){ //edges of this face must be computed
	for(k=0;k<2;k++){
	  if(dir2 == 0 || dir2==2*k-1){ //edges of this face must be computed
	    v_copy(u,v_tmp);
	    v_tmp[(i+1)%3]*=-(2*j-1);
	    v_tmp[(i+2)%3]*=-(2*k-1);
	    v_add(objPos,v_tmp,vertexPos);
	    v_scale(&(ref[4*i]),obj->solid->m_size[i],v1); // edge with the right length
	    pindex = 4*i+2*j+k;
	    FindSegmentsNearestPoints(vertexPos,v1,upperLinkExtr,linkv,&(allpoints[2*pindex]),&(allpoints[2*pindex+1]),&(alldists[pindex]));
	  }
	}
      }
    }
  }
  //looking for the min
  min_dist = alldists[0];
  min_i = 0;
  for(i=1;i<12;i++){
    if(alldists[i] < min_dist){
      min_dist = alldists[i];
      min_i = i;
    }
  }
  // returning the min distance
  *dist = min_dist;
  *point = allpoints[2*min_i+1];
  v_scale(linkv,*point,v1);
  v_add(upperLinkExtr,v1,v2); // nearest point on link
  k = min_i%2; //retrieving the right edge
  j = ((min_i-k)%4)/2;
  i = min_i/4; 
  //  cout<<"ijk: "<<i<<" "<<j<<" "<<k<<endl;
  v_copy(u,v_tmp);
  v_tmp[(i+1)%3]*=-(2*j-1);
  v_tmp[(i+2)%3]*=-(2*k-1); 
  v_add(objPos,v_tmp,vertexPos); // starting vertex of the edge
  v_scale(&(ref[3*1]),allpoints[2*min_i]*(obj->solid->m_size[min_i]),v1);
  v_add(vertexPos,v1,v1); // nearest point on solid
  v_sub(v2,v1,contact_vector);
  return 1;
}

//nearest points between two segments given by pi+veci, results given in ratio of vec [0 1]
int Reaching::FindSegmentsNearestPoints(CVector3_t p1,CVector3_t vec1,CVector3_t p2,CVector3_t vec2, float *nearest_point1, float *nearest_point2, float *dist){
  CMatrix3_t mat;
  CMatrix3_t invmat;
  CVector3_t vec3,tmp,tmp1,tmp2,k,v2;
  int i; 
  m_identity(mat);
  v_cross(vec1,vec2,vec3);// check if vec1 and vec2 are not //
  if(v_squ_length(vec3)<0.00001){
    return 0;
  }

  v_scale(vec2,-1,v2);
  m_set_v3_column(vec1,0,mat);
  m_set_v3_column(v2,1,mat);
  m_set_v3_column(vec3,2,mat);
  m_inverse(mat,invmat);
  v_sub(p2,p1,tmp);
  v_transform_normal(tmp,invmat,k);
  for(i=0;i<2;i++){
    k[i] = max(min(1,k[i]),0);
  }
  v_scale(vec1,k[0],tmp);
  v_add(p1,tmp,tmp1);
  v_scale(vec2,k[1],tmp);
  v_add(p2,tmp,tmp2);
  v_sub(tmp2,tmp1,tmp);
  *dist=v_length(tmp);
  *nearest_point1 = k[0];
  *nearest_point2 = k[1];
  return 1;
}

  
      

	    
#endif
#ifdef LEARNING
  float Reaching::LearningLocal(){return 0.0;}
  float Reaching::LearningGlobal(){return 0.0;}
#endif

/********************************************
// operator <<
//
// overriding the << operator for the ostream class
// used for cout << this
**********************************************/

ostream& operator<<(ostream& out, const Reaching& reach)
{
  CVector4_t p_angle;
  CVector3_t p_cart;
  reach.GetAngle(p_angle);
  reach.GetAbsHandPosition(p_cart);
  out<<p_angle<<" "<<p_cart;
#ifdef PRINT_WEIGHTS
  out << " "<< reach.weight_angle[0]<<" "<< reach.weight_angle[1]<<" "<< 
    reach.weight_angle[2]<<" "<< reach.weight_angle[3]<<" "<< 
    reach.weight_cart[0]<<" "<< reach.weight_cart[1]<<" "<<
    reach.weight_cart[2];
#endif
#ifdef PRINT_TARGET
reach.GetTargetAngle(p_angle);  
 out<<" "<<p_angle;
#endif
#ifdef PRINT_SAI 
  float index = ((Reaching)reach).SpuriousAttractorIndex();
  out <<" "<< index;
#endif
  return out;
}
 
istream& operator>>(istream& in, Reaching& r){
  CVector3_t tar;
  CVector4_t pos;
  char c;
  in>>tar[0]>>tar[1]>>tar[2]>>c>>pos[0]>>pos[1]>>pos[2]>>pos[3];
  if(!r.SetActualRobPosition(pos)){
    cout<<"initial position out of range"<<endl;
  }
  r.SetLocalTarget(tar);    
  return in;
}



ostream& operator<<(ostream& out, const ArmConfigList_t manifold){
  //  int i,n;
  ArmConfigList_t::const_iterator it;
  CVector4_t angles;
  for(it=manifold.begin();it!=manifold.end();it++){
    it->ToVector(angles);
    out<<angles<<endl;
  }
  return out;
}

