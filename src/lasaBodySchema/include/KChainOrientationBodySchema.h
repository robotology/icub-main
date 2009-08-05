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
#ifndef __K_CHAIN_ORIENTATION_BODY_SCHEMA_H__
#define __K_CHAIN_ORIENTATION_BODY_SCHEMA_H__

#include "KChainBodySchema.h"

/**
 *  This class is a wrapper for KinematicChain, so that it becomes a BodySchema
 */
class KChainOrientationBodySchema : public KChainBodySchema{

 protected:
  float rot_tol;


 public:
 KChainOrientationBodySchema():KChainBodySchema(){rot_tol=0.05;};
    KChainOrientationBodySchema(int teles):KChainBodySchema(teles){rot_tol=0.05;};
  virtual ~KChainOrientationBodySchema(){};
  //  KChainOrientationBodySchema(char *filename):BodySchema(),KinematicChain(){BodySchema::Load(filename);};
  //  KChainBodySchema(const KinematicChain& chain);
  void Angle2Cart(joint_vec_t& angles,cart_vec_t& pos)
  {KinematicChain::SetAngles(angles.GetArray());Angle2Cart(pos);}
  void Angle2Cart(cart_vec_t& pos)
  {Rotation r;ForwardKinematics(pos.GetArray());GlobalRotation(r);r.GetRotationParam(pos.GetArray()+3);}
  int InverseKinematics(const cart_vec_t& pos,joint_vec_t& angles);
  /* int InverseKinematics(const cart_vec_t& pos,joint_vec_t& start_angle,joint_vec_t& angles) */
/*   {SetAngles(start_angle.GetArray()); return InverseKinematics(pos,angles);} */
 /*  void SetRandomAngle(joint_vec_t& angles) */
/*   {GetRandomAngle(angles);SetAngles(angles.GetArray());} */
  void Jacobian(Matrix& jac);//jac is assumed to of the right dimension
  // void Jacobian(joint_vec_t& angles, Matrix& jac){SetAngles(angles.GetArray());Jacobian(jac);}
  //  void Load(istream& in){in>>*((KinematicChain *)this);};
  // void Print(ostream& out)const{out<<"P "<<*((KinematicChain *)this);}
  // void GetWeights(cart_vec_t& cart_weights, joint_vec_t& joint_weights);
  // void SetPosition(joint_vec_t& angles){SetAngles(angles.GetArray());};
  // void GetAngles(joint_vec_t& angles){KinematicChain::GetAngles(angles.GetArray());};
  void AddVirtualRotationAxis(){};

/* virtual float UpdateTransfo(joint_vec_t& proprio, cart_vec_t& vision) */
/*     {return UpdateToolTransfo(proprio,vision);} */
/*   virtual float UpdateToolTransfo(joint_vec_t& proprio, cart_vec_t& vision) */
/*     {return Update(proprio,vision);}//maybe update a single transfo */
/*   virtual void ShowTransformation(){cout<<(KinematicChain )(*this)<<endl;} */

 protected:
  float TryInverseKinematics(const cart_vec_t);

};






#endif
