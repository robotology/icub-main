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
#ifndef __RIGID_TRANSFO_H__
#define __RIGID_TRANSFO_H__

#include "mathlib.h"
#include "Translation.h"
#include "Rotation.h"




/**
 * This class represents a rigid transformation using the Euler parametrization
 * for rotation. See Hestenes, "New Foundations for Classical Mechanics",
 * pp 280 for the mathematical formalism 
 */
class RigidTransfo : public Rotation, public Translation{
   
 protected:
  int adaptive;

 public:
   RigidTransfo(){adaptive=1;};
/*   virtual ~RigidTransfo(); */


   float Update(CVector3_t v, CVector3_t v_tr);

#ifdef WRONG_UPDATE
   void UpdateRotation(CVector3_t v, CVector3_t v_tr);

   void UpdateAxis(CVector3_t v, CVector3_t v_tr,float angle);

   void UpdateTranslation(CVector3_t v, CVector3_t v_tr);

   void UpdateTranslation(CVector3_t v, CVector3_t v_tr,float angle);

   void UpdateAxisAndTranslation(CVector3_t v, CVector3_t v_tr,float angle);
#endif


  int SetTransfo(CVector3_t beta, CVector3_t transl);
 

 /**
  * @brief sets the learnig rate sizes
  * @param new_eps rotation step size
  * @param new_eps_tr translation step size
  */
  virtual void SetRate(float new_eps,float new_eps_tr );

  /** 
   * @briefscale the learning rate sizes 
   * @param factor the scaling factor (for both rotation and
   * translation learning ratea)
   */
  virtual void ScaleRate(float factor);

  /**
   * @brief Performs the rigid tranformation
   * @param in the vector to be transformed
   * @param out the resulting (transformed) vector
   */
  void Transform(const CVector3_t in, CVector3_t out);


  void Transform(const CVector3_t in, float angle, CVector3_t out)
  {SetAngle(angle);Transform(in,out);};
  /**
   * @brief Performs the inverse rigid tranformation
   * @param in the vector to be transformed
   * @param out the resulting (transformed) vector
   */

   void InverseTransform(const CVector3_t in,CVector3_t out);
   void ReverseTransform(const CVector3_t in,CVector3_t out);

   void InverseTransform(CVector3_t in, float angle,CVector3_t out)
   {SetAngle(angle);InverseTransform(in,out);};

   void Invert();
   void NoLearning(){adaptive=0;};
   int IsAdaptive()const{return adaptive;}
   void Copy(const RigidTransfo& r){Rotation::Copy(r);Translation::Copy(r);adaptive=r.IsAdaptive();}
   RigidTransfo& operator*(const RigidTransfo& rt1);
   void Identity(){v_clear(beta);alpha=1;v_clear(trans);}
};

/**
 * @brief output streaming operator
 */
ostream& operator<<(ostream& out, const RigidTransfo& rt);

/**
 * @brief input streaming operator
 */
istream& operator>>(istream& in, RigidTransfo& rt);
#endif
