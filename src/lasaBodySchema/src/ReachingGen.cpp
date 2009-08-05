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
#include "ReachingGen.h"

//#define SILENT



//#define REACHING_MAIN


#ifdef REACHING_MAIN
#include "KChainBodySchema.h"

int main(int argc,char *argv[]){
    Reaching reach;
    BodySchema *body = new KChainBodySchema();
    Matrix jac(cartesian_dim,joint_angle_dim);
    joint_vec_t angle,angle1;
    cart_vec_t pos;
    srand(time(NULL));

    body->Load(argv[1]);
    //  body->Print(cout);
    reach.SetBodySchema(body);

    for(int j=1;j<2;j++){
        body->SetRandomAngle(angle);
        //   body->SetAnglesInRange(angle1);
        //   body->SetPosition(angle1);
        //  angle.Print();
        body->Angle2Cart(pos);
        if(!reach.SetLocalTarget(pos)){
            cout<<"can't reach"<<endl ;
        }
        //  cout<<pos<<endl<<endl;
        body->SetRandomAngle(angle1);
        //   body->SetAnglesInRange(angle1);
        //   body->SetPosition(angle1);
        reach.MatchToBodySchema(); 
        // while(!reach.TargetReached()){
        for(int i=0;i<2;i++){
            reach.ReachingStep();
            reach.GetPosition(pos,angle);
            if(reach.TargetReached()){break;}
            if(i==999){
                cout<<endl<<"failed"<<endl;
                reach.Print();
                //    for(int k=0;k<300;k++){
                // 	reach.ReachingStep();
                // 	reach.GetPosition(pos,angle);
                // 	cout<<pos<<" "<<angle<<endl;
                //       }
            }
        }
    }
    //  cout<<pos<<"  "<<angle<<endl;


    //  cout<<(KinematicChain)(*body)<<endl;

    //  ifstream file(argv[1]);
    // file>>r;
    // cout<<r<<endl;
    cout<<"done"<<endl;
    return 1;
}
  
#endif




Reaching::Reaching(){
    alpha = 0.002; //0.08
    beta =0.0005; // 0.02
    zero_vel_angle = 1e-4;
    out_str = NULL;
    body =NULL;
    tol = 100;
    pure_joint_ctl=0;
#ifdef OBSTACLE_AVOIDANCE
    env= NULL;
    obstacle_rad = 40;
    nu = 0.1;
#endif

}

Reaching::~Reaching(){
    if (out_str) out_str->close();
}

void Reaching::SetBodySchema(BodySchema *b){
    body = b;
    cout<<" dim "<<base_weight_cart.Size()<<" "<<endl;
    body->GetWeights(base_weight_cart, base_weight_angle);
    weight_cart = base_weight_cart;
}

BodySchema* Reaching::GetBodySchema(){
    return body;
}

void Reaching::MatchToBodySchema(){
    body->GetAngles(pos_angle);
    body->Angle2Cart(pos_cart);
}


void Reaching::RandomTarget(cart_vec_t& pos,int strict){
    joint_vec_t angles;
    body->GetRandomAngle(angles);
    body->Angle2Cart(angles,pos);
    body->SetRandomAngle(angles);
    SetLocalTarget(pos,strict);
}

void Reaching::SetStill(){
    tar_angle = pos_angle;
    target = pos_cart;
}


void Reaching::SetTargetAngle(joint_vec_t& angles){
    body->SetAnglesInRange(angles);
    tar_angle = angles;
    body->Angle2Cart(tar_angle,target);
}
/**
 * setting the target relatively to shoulder position
 * @param new_target target in local referential
 * @return returns 1 if target is reachable
 * @return -1 if target is pointable, 0 otherwise
 */

int Reaching::SetLocalTarget(cart_vec_t& new_target,int strict)
{
    int res;
    joint_vec_t tangle;
    body->SetPosition(pos_angle);// trial 
    res = body->InverseKinematics(new_target, tangle);
    if(res){
        target = new_target;
#ifndef SILENT 
        cout << "reachable target ";new_target.Print();
#endif
    }
    else{
#ifndef SILENT 
        cout << "unreachable target ";new_target.Print();
#endif
        if(strict){
            // ignoring unreachable target 
            //  tar_angle = pos_angle;
            //  target = pos_cart;
            return 0;
        } 
        else{
            body->Angle2Cart(target);
        }
    }
    tar_angle = tangle;
    return res;
}




/**
   @brief Performs a reaching step, using the Grossberg's VITE algorithm
   * on the cartesian and joint angle space. Unification is enforced
   * by constraining the coherence of joint angles and cartesian positions
   * (by use of lagrange multipliers)
   * 
   */


void Reaching::ReachingStep(float dt){
    if(dt<EPSILON) return;//no point of doing anything
    //  cout<<"DT "<<dt<<endl;
    joint_vec_t tmp1,tmp3; // temporary variables
    cart_vec_t tmp13;
    float dist=0;
  
    // dt = dt/10; 
    //dist = UpdateLocalTarget();
    if(dist>tol || isnan(dist)){
        SetLocalTarget(target);
        cout<<dist<<" "<<tol<<endl;
    }
    // vite in angle space and cart space
    alpha*=dt;beta*=dt;
    ViteAngle(tar_angle,pos_angle,v_angle,des_angle,des_v_angle);
    ViteCart(target,pos_cart,v_cart,des_cart,des_v_cart);
    alpha /= dt; beta/=dt;

    //  cout<<"vite done"<<endl; 
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
 
    body->SetAnglesInRange(des_angle);
    des_v_angle = des_angle - pos_angle;
    des_v_cart = des_cart - pos_cart;
    if(pure_joint_ctl){
        tmp1 = des_angle;
    }
    else{
        UpdateWeights();
        // coherence enforcement
        ProjectVector(des_v_cart,des_v_angle,tmp3);
        tmp1 = pos_angle+tmp3;
    }
    body->SetAnglesInRange(tmp1);
    body->Angle2Cart(tmp1,tmp13);
    v_angle = tmp1 - pos_angle;
    v_cart = tmp13 - pos_cart;
    pos_cart = tmp13;
    pos_angle = tmp1;
    pos_angle.Print();
    pos_cart.Print();
}


void Reaching::ProjectVector(cart_vec_t& vcart,joint_vec_t& vangle, joint_vec_t& out){
    Matrix jac, 
        jact, m31,m32(cartesian_dim,cartesian_dim,false),
        wa(joint_angle_dim,joint_angle_dim,false); 
    joint_vec_t v41,v42;
    cart_vec_t v31,v32;
    body->Jacobian(pos_angle,jac); //J
    jact = jac.Transpose();
    wa.Diag(weight_angle);
    v31= vcart-jac*vangle;
    m31 = jac*wa*jact;
    m31 += m32.Diag(weight_cart);
    m32 = m31.Inverse();
    if(!Matrix::IsInverseOk()){cout<<"bad inversion"<<endl;}
    out = vangle+wa*jact*m32*v31;
}




/**
 * performs the vite dynamical system in 3d
 * @param target the target
 * @param pos the position at time t
 * @param speed the speed at time t
 * @param new_pos the position at time t+1 (output of the method)
 * @param new_speed the speed at time t+1 (output of the method
 */
void Reaching::ViteCart(cart_vec_t& target,cart_vec_t& pos,cart_vec_t& speed,
                        cart_vec_t& new_pos, cart_vec_t& new_speed) {
    //	cart_vec_t tmp1,tmp2;
	//vite in cart space
	// v update
    new_speed  = speed + ((target-pos)*beta - speed)*alpha;
    new_pos = pos+new_speed;
    //  cout<<"new cart speed "<<speed<<endl;
}


/**
 * performs the vite dynamical system in 4d
 * @param target the target
 * @param pos the position at time t
 * @param speed the speed at time t
 * @param new_pos the position at time t+1 (output of the method)
 * @param new_speed the speed at time t+1 (output of the method
 */
void Reaching::ViteAngle(joint_vec_t& target,joint_vec_t& pos,joint_vec_t& speed,
                         joint_vec_t& new_pos, joint_vec_t& new_speed) {
	//vite in joint space
	// v update
  	new_speed  = speed + ((target-pos)*beta - speed)*alpha;
	new_pos = pos+new_speed;
}

void Reaching::UpdateWeights(){
    joint_vec_t& min_angle = body->GetAnglesLowerBound();
    joint_vec_t& max_angle = body->GetAnglesUpperBound();
    float range;
    for(int i=0;i<joint_angle_dim;i++){
        range = max_angle[i]-min_angle[i];
        if(range>EPSILON){
            weight_angle[i] = 0.5*base_weight_angle[i]*//
                (cos((pos_angle[i] - min_angle[i])*2*PIf/range
                     +PIf)+1);
            weight_angle[i]*=0.1;   
        }
        else{
            weight_angle[i] = 1.f/EPSILON;
        }
    }
//     if(pure_joint_ctl){
//         weight_angle *= 0.0f;
//     }
}



float Reaching::UpdateLocalTarget(cart_vec_t *new_target){
    cart_vec_t diff,v1,v2;
    joint_vec_t v3,v4,v5;
    Matrix m1,m2;
    Matrix jac,jacT,m3,m4;
    float f;
    if(new_target){
        diff = *new_target-target;
        if(diff.Norm2()>tol*tol){
            return SetLocalTarget(*new_target);
            //      return -1.0f;
        }
        else{
            //updating cartesian target
            target=*new_target;
        }
    }
    body->Angle2Cart(tar_angle,v1);
    v2=target-v1;
    body->Jacobian(tar_angle,jac);
     jac.Inverse(m3);//todo : find the weighted version

    if(!Matrix::IsInverseOk()){cout<<"bad pseudo-inverse in Reaching::UpdateLocalTarget"<<endl;}
    v3 = m3*v2*0.1;
    m4 = m3*jac;
    m3.Resize(tar_angle.Size(),tar_angle.Size());
    m3.Identity();
    v5=(m4-m3)*(tar_angle-pos_angle);
    if((f=v5.Norm2())>0.01*0.01){
        v5*= 0.001/sqrtf(f);
    }
    v4 = v5+v3; 
    v3 = tar_angle+v4;
 
    if(body->AnglesInRange(v3)){
        tar_angle=v3;
    }
    //checking precision
    body->Angle2Cart(tar_angle,v1);
    return body->CartesianDistance(target,v1); 
}




#ifdef NOT_YET




void Reaching::SetIncrement(float new_incr){
  incr=new_incr;
}

/***********************************************
 * method SetShoulderPosition
 *
 * sets the shoulder position with respoect to an 
 * absolute referential (the same one as SetTarget).
 * this referential is oriented like Eric's (x:left,
 * y: up, z:front)
 *************************************************/


void Reaching::SetShoulderPosition(cart_vec_t& pos){
  v_copy(pos,shoulder_abs_pos);
}

void Reaching::GetWeights(cart_vec_t& w_cart, joint_vec_t& w_angle)const{
#ifdef DYN_WEIGHTS
  v4_copy(base_weight_angle,w_angle);
  v_copy(base_weight_cart,w_cart);
#else
  v4_copy(weight_angle,w_angle);
  v_copy(weight_cart,w_cart);
#endif
}
void Reaching::SetWeights(cart_vec_t& w_cart, joint_vec_t& w_angle){
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

void Reaching::GetTarget(cart_vec_t& tar) const {
  Local2AbsRef(target,tar);
}

/**
 * setting the target relatively to shoulder position
 * @param new_target target in global referential
 * @param strict if 0, pointing is allowed, else not 
 * @return returns 1 if target is reachable
 * @return -1 if target is pointable, 0 otherwise
 */
int Reaching::SetTarget(cart_vec_t& new_target,int strict){
  cart_vec_t local_tar;
  Abs2LocalRef(new_target,local_tar);
  return SetLocalTarget(local_tar,strict);
}



int Reaching::UpdateTarget(){
  if(GetVisualMode()){
    return SetTarget(target);
  }
  else{
    return 0;
  }
}



int Reaching::SetArmConfigurationTarget(joint_vec_t& new_target)
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




int Reaching::ComputeConfigManifold(cart_vec_t& cart_goal)
{  
  float alpha;
  ArmConfig_t tmpconfig;
#ifdef BODY_SCHEMA
  joint_vec_t& angles;
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
void Reaching::Hoap2Rob(joint_vec_t& conf)
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

void Reaching::Rob2Hoap(joint_vec_t& conf)
{
  conf[0] = (-conf[0]+pi/2) *rad2deg;
  conf[1] = (conf[1]-pi/2)*rad2deg;
  conf[2] = conf[2]*rad2deg;
  conf[3] = -conf[3]*rad2deg;
}


void Reaching::Rob2Icub(joint_vec_t& angles){
  angles[0] *= -rad2deg;   //sfe stays the same
  angles[1] = (-angles[1]+pi/2)*rad2deg;//saa
  angles[2] = (-angles[2]+pi/2)*rad2deg;//shr
  angles[3] = angles[3]*rad2deg; //eb
}

void Reaching::Icub2Rob(joint_vec_t& angles){
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



void Reaching::Angle2Cart(pArmConfig_t config, cart_vec_t& out){
  joint_vec_t& angle;
  v4_set(config->sfe,config->saa,config->shr,config->eb,angle);
  body->Angle2Cart(angle,out);
}



void Reaching::Angle2Cart(joint_vec_t& angle, cart_vec_t& out)
{
  body->Angle2Cart(angle,out);
}



void Reaching::Angle2CartAbs(joint_vec_t& angle, cart_vec_t& out)
{
  cart_vec_t& local;
  Angle2Cart(angle,local);
  Local2AbsRef(local,out); 
}

void Reaching::ElbowPosition(joint_vec_t& angle, cart_vec_t& out)
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
void Reaching::GetAngle(joint_vec_t& angle) const {
  v4_copy(pos_angle,angle);
}
void Reaching::GetTargetAngle(joint_vec_t& angle) const {
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
// joint_vec_t& Reaching::GetAngle(){
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
void Reaching::GetAbsHandPosition(cart_vec_t& out) const {
  Local2AbsRef(pos_cart,out);
}



void Reaching::GetLocalHandPosition(cart_vec_t& out) const {
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




int  Reaching::InvKinematics (cart_vec_t& relTar, float alpha, float toler,  pArmConfig_t out){
  cart_vec_t& v;
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

/* for later
float Reaching::FindNearestPosition(joint_vec_t& pos1,joint_vec_t& out)
{
  ArmConfigList_t::iterator it;
  pArmConfig_t min;
  float mindist,dist;

  mindist = AngleDistance(pos1,&(configManifold[0]));
  min = &(configManifold[0]);
  for(it=configManifold.begin();it!=configManifold.end();it++){
    dist = AngleDistance(pos1,&(*it)); 
    if(dist<mindist){
      mindist = dist;
      min = &(*it);
    }
  }
  out.sfe = min->sfe;
  out.saa = min->saa;
  out.shr = min->shr;
  out.eb = min->eb;
  return mindist;
}
*/
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


float Reaching::AngleDistance(joint_vec_t& pos1,joint_vec_t& pos2)
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

/* for later
float Reaching::AngleDistance(joint_vec_t& pos1,joint_vec_t& pos2)
{
  joint_vec_t&  diff;
  v4_sub(pos1,pos2,diff);
  return v4_squ_length(diff);
}
*/




int Reaching::ArmConfig2CVector(pArmConfig_t arm,joint_vec_t& out){
  if(arm){
  out[0] = arm->sfe;
  out[1] = arm->saa;
  out[2] = arm->shr;
  out[3] = arm->eb;
  return 1;
  }
  else return 0;
}


int Reaching::CVector2ArmConfig(joint_vec_t& v, pArmConfig_t arm){
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
void Reaching::Jacobian(joint_vec_t& v, CMatrix4_t jac){
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


void Reaching::IntermediateJacobian(int link, float len,joint_vec_t& v, CMatrix4_t jac){
  body->IntermediateJacobian(link,len,v,jac);
}


void Reaching::SetAnglesInRange(joint_vec_t& angle){
  body->SetAnglesInRange(angle);
}


void Reaching:: UpdateWeights(){   // -- to modify for visual servo
#ifdef BODY_SCHEMA
  joint_vec_t& min_angle, max_angle;
  body->GetAnglesLowerBound(min_angle);
  body->GetAnglesUpperBound(max_angle);
#endif

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
  cart_vec_t& tmp;
  v_sub(target,pos_cart,tmp);    
  float cart_dist = max(v_length(tmp)/150,1e-10);
  for(int i=0;i<4;i++){
    weight_angle[i] /= cart_dist;
  }
  cout<<"dist "<< cart_dist*150<<" weights ";coutvec4(weight_angle);
#endif


#endif

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
  joint_vec_t& da,tmp14,tmp24;
  cart_vec_t& dx;
  CMatrix4_t jac; //jacobian matrix
  cart_vec_t& wxm1,tmp13;
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
  joint_vec_t& v1,v2;
  //  Hoap2_JointAngles *angles;
  // angles =(Hoap2_JointAngles *)an;
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
  joint_vec_t& v;
  v[0] = angles->R_SFE;
  v[1] = angles->R_SAA;
  v[2] = angles->R_SHR;
  v[3] = angles->R_EB;
//ArmConfig2CVector(angles,v);
  Hoap2Rob(v);
  return SetActualRobPosition(v);
}
#endif


#ifdef TRACE_COSTS

// works for homogeneous weights only. not clean
float Reaching::GetCosts(joint_vec_t& des_angle,cart_vec_t& des_pos){
  joint_vec_t& v1,v2;
  cart_vec_t& w1,w2;
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

int Reaching::SetActualRobPosition(joint_vec_t& angles){
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
int Reaching::SetActualRobPosAndSpeed(joint_vec_t& position, joint_vec_t& speed){
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
int Reaching::SetRobAcceleration(joint_vec_t& acc){
  cart_vec_t& tmpc;
  joint_vec_t& tmpa;
  int ret =1;
  float tmp;
#ifdef BODY_SCHEMA
  joint_vec_t& min_angle, max_angle;
  body->GetAnglesLowerBound(min_angle);
  body->GetAnglesUpperBound(max_angle);
#endif


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



// int Reaching::InitOutputStream(string& filename, ofstream **ostr){
//   *ostr = new ofstream(filename);
//   if (*ostr->fail()){
//     return 0;
//   }
//   else{
//     return 1;
//   }
// }


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



int Reaching::RecordTraj_ACT(joint_vec_t& initpos, joint_vec_t& targ, string& fname)
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


int Reaching::RecordTraj(joint_vec_t& initpos, cart_vec_t& targ, string& fname)
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


int Reaching::StartRecordTraj(joint_vec_t& initpos,ofstream& out){
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

int Reaching::StartRecordTraj(joint_vec_t& initpos){
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
#ifndef BODY_SCHEMA
int Reaching::AcceptablePosition(joint_vec_t& angle){
  int i;
  for(i=0;i<4;i++){
    //if (max(min_angle[i], min(max_angle[i],angle[i])) != angle[i]){
    if(min_angle[i]>=angle[i] || max_angle[i]<=angle[i]){
#ifndef SILENT
      //      cout << "angle " <<i <<" "<<angle[i]<<" beyond limits"<<endl;
#endif
      return 0;
    }
  }
  return 1;
}
#else
int Reaching::AcceptablePosition(joint_vec_t& angle){
  return body->AcceptablePosition(angle);
}
#endif
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
  cart_vec_t& dist;
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
#ifndef BODY_SCHEMA
void Reaching::RandomAnglePos(joint_vec_t& angle){
  float rn;
  for (int i=0;i<4;i++){
    rn = abs(rand()/((float)(RAND_MAX+1)));
    angle[i] = min_angle[i]+(max_angle[i]-min_angle[i])*rn; 
  }
}
#else
void Reaching::RandomAnglePos(joint_vec_t& angle){
  body->RandomAnglePos(angle);
}
#endif

/**
 * output in shoulder referential
 */

#ifndef BODY_SCHEMA
void Reaching::RandomCartPos(cart_vec_t& cart){
  joint_vec_t& angle;
  RandomAnglePos(angle);
  Angle2Cart(angle,cart);
}
#else
void Reaching::RandomCartPos(cart_vec_t& cart){
  body->RandomCartPos(cart);
}
#endif


void Reaching::RandomCartAbsPos(cart_vec_t& cart){
  cart_vec_t& cart_l; //local ref
  RandomCartPos(cart_l);
  Local2AbsRef(cart_l,cart);
}

void Reaching::Local2AbsRef(const cart_vec_t& cart_l,cart_vec_t& cart) const{
  v_add(cart_l,shoulder_abs_pos,cart);
}


void Reaching::Abs2LocalRef(const cart_vec_t& cart, cart_vec_t& out) const {
  v_sub(cart,shoulder_abs_pos,out);
  //  cout <<"r reach"<<endl;
}
#ifndef BODY_SCHEMA
int Reaching::GetVisualMode(){return 0;}
#else
int Reaching::GetVisualMode(){return body->GetVisualMode();}
#endif

// // target is already specified
// void Reaching::VelocityConstrainedMovement(char *filename){
  
//   int t;
//   vector<> vel;
//   ReadVelocityProfile(filename,&vel);
//   len = vel.size()+delay;
//   for(t=0;t<len;t++){
//     ReachingStep();
    



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
    joint_vec_t& pos, new_tar;
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
    cart_vec_t& tar, new_tar;
    joint_vec_t& pos;
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
    cart_vec_t& tar;
    joint_vec_t& pos;
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
  cart_vec_t& tar;
  joint_vec_t& pos,acc;
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


#ifdef WITH_ENVIRONMENT

int Reaching::SetEnvironment(Environment *e){
  if(!env){
    env = e;
    return 1;
  }
  else{
    return 0;
  }
}

#endif

#ifdef OBSTACLE_AVOIDANCE
int Reaching::LinkDistanceToObject(int link, EnvironmentObject *obj, float *dist, 
			 float *point, cart_vec_t& contact_vector){
  cart_vec_t& objPos;
  cart_vec_t& lowerLinkExtr;
  cart_vec_t& upperLinkExtr;
  cart_vec_t& v1,v2,linkv,u,vertexPos,v_tmp,tmp3;
  CMatrix4_t ref,tmpmat,tmpmat2;
  cart_vec_t& s1,s2;
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
  //  v_sub(obj->solid->m_position, shoulder_abs_pos,objPos);
    Abs2LocalRef(obj->GetPosition(), objPos);
  //  cout<<"ule: ";coutvec(upperLinkExtr);
  //  cout<<"lle: ";coutvec(lowerLinkExtr);
    //  coutvec(objPos);
  v_sub(lowerLinkExtr,objPos,v1);
  v_sub(upperLinkExtr,objPos,v2);
  //m_transpose(obj->solid->m_ref.m_orient,ref); //stupid to do it each time
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
      //      cout<<"linkv ";coutvec(linkv);
      // cout <<"pt "<<*point<<endl;
	
      
// #ifdef OLD_AV
//       v_copy(linkv,(cart_vec_t&)&tmpmat[i*4]);
//       m_inverse(tmpmat,tmpmat2);
//       v_sub(upperLinkExtr,vertexPos,tmp2);
      // tmp3 should contain the intersection coordinates in
      // the referential defined by the edges of the surface 
      v_transform_normal(v_tmp,tmpmat2,tmp3); 
      // cout<<"tmp3 ";coutvec(tmp3);
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
	//     cout<<"i:"<<i<<" j:"<<j<<" k:"<<k<<"dist "<<alldists[pindex]<<endl;
// 	    cout<<"v1:";coutvec(v1);
// 	    cout<<"ref:";coutvec((&(ref[4*i])));
// 	    cout<<"vP:";coutvec(vertexPos);
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
int Reaching::FindSegmentsNearestPoints(cart_vec_t& p1,cart_vec_t& vec1,cart_vec_t& p2,cart_vec_t& vec2, float *nearest_point1, float *nearest_point2, float *dist){
  CMatrix3_t mat;
  CMatrix3_t invmat;
  cart_vec_t& vec3,tmp,tmp1,tmp2,k,v2;
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


//   out << reach.pos_angle[0]<<" "<<reach.pos_angle[1]<<" "<<reach.pos_angle[2]
//       <<" "<<reach.pos_angle[3] << " "<<reach.pos_cart[0]<<" "
//       <<reach.pos_cart[1]<<" "<<reach.pos_cart[2];
  joint_vec_t& p_angle;
  cart_vec_t& p_cart;
  reach.GetAngle(p_angle);
  reach.GetAbsHandPosition(p_cart);
    
//   out << p_angle[0]<<" "<<p_angle[1]<<" "<<p_angle[2]
//       <<" "<<p_angle[3] << " "<<p_cart[0]<<" "
//       <<p_cart[1]<<" "<<p_cart[2];
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
  cart_vec_t& tar;
  joint_vec_t& pos;
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
  joint_vec_t& angles;
  for(it=manifold.begin();it!=manifold.end();it++){
    it->ToVector(angles);
    out<<angles<<endl;
  }
  return out;
}

#endif //NOT_YET


