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
#ifndef __BODY_SCHEMA_H__
#define __BODY_SCHEMA_H__

#include "iCub/mathlib.h"


typedef struct {
  float sfe;
  float saa;
  float shr;
  float eb;
  void FromVector(CVector4_t v){sfe=v[0];saa=v[1];shr=v[2];eb=v[3];}
  void ToVector(CVector4_t v)const {v[0]=sfe;v[1]=saa;v[2]=shr;v[3]=eb;}
} ArmConfig_t, *pArmConfig_t;


class BodySchema
{
 protected:
  float l1; // Upper arm length
  float l2; // Forearm length
  //CVector3_t shoulder_abs_pos;    // shoulder position
 
  /**
   * joint angle lower limits
   */
  CVector4_t min_angle;
 /**
   *joint angle upper limits
   */
  CVector4_t max_angle;

 public:

  BodySchema();
  virtual ~BodySchema();
  virtual void SetArmLength(float nl1,float nl2);
  virtual float GetUpperArmLength();
  virtual float GetForeArmLength();
  
  //  void SetShoulderPosition(CVector3_t pos);
  virtual void Angle2Cart(pArmConfig_t config, CVector3_t out);
  virtual void Angle2Cart(CVector4_t angle, CVector3_t out);
/*   virtual void Angle2CartAbs(CVector4_t angle, CVector3_t out); */
/*   virtual void Angle2CartAbsProp(CVector4_t angle, CVector3_t out); */
  //void Angle2CartAbs(CVector4_t angle, CVector3_t out);
  virtual void ElbowPosition(CVector4_t angle, CVector3_t out);
  virtual void VirtualTarget(CVector3_t tar, CVector3_t out){v_copy(tar,out);}
#ifdef OLD_BODY_SCHEMA
  virtual int InvKinematics (CVector3_t relTar, float alpha, float tol, pArmConfig_t out);
#else
  virtual int InvKinematics (CVector3_t relTar, float alpha, float tol, CVector4_t out);
  void RawInverseKinematics(CVector3_t relTar, float alpha,CVector4_t out);
  int CheckInverseKinematics(CVector4_t angles,CVector3_t target, float tol);
#endif
  virtual void Jacobian(CVector4_t v, CMatrix4_t jac);
  virtual void IntermediateJacobian(int link, float len,CVector4_t v, CMatrix4_t jac);
  virtual int AcceptablePosition(CVector4_t angle);
  virtual void RandomAnglePos(CVector4_t angle);
  virtual void RandomCartPos(CVector3_t cart);
  virtual void SetAnglesInRange(CVector4_t angle);
  virtual int  AnglesInRange(CVector4_t angle);
  virtual void GetAnglesLowerBound(CVector4_t lb);
  virtual void GetAnglesUpperBound(CVector4_t ub);

  virtual void Load(char *filename){};
  //empty shells for adaptation

  virtual float UpdateTransfo(){return 0.0;};
  virtual float UpdateTransfo(CVector4_t proprio, CVector3_t vision){return 0.0;} 
  virtual float UpdateCalibTransfo(){return 0.0;}
  virtual void SetVisualPos(CVector3_t vision){};//empty shell
  virtual void SetProprioceptivePos(CVector4_t propri){}; //empty shell
  virtual int GetVisualMode(){return 0;}
  virtual void ShowTransformation(){}

  
};


ostream& operator<<(ostream& out, const BodySchema& bs);
istream& operator>>(istream& in, BodySchema& bs);
#endif
