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
#ifndef __TRANSLATION_H__
#define __TRANSLATION_H__
#include "mathlib.h"


/**
 *  a standard translation. 
 */
class Translation {
protected:
  CVector3_t trans;     /**< the translation vector */
  float eps_tr;         /**< the learning rate */
  float base_eps_tr;    /**< the "base" learning rate */

public:
  Translation();
  virtual ~Translation(){};
  void SetRate(float e){base_eps_tr=eps_tr=e;};
  void ScaleRate(float f){eps_tr=f>0?f*base_eps_tr:eps_tr;};
  void SetTranslation(const CVector3_t t){v_copy(t,trans);};
  void GetTranslation(CVector3_t tr)const {v_copy(trans,tr)};
  void GetInverseTranslation(CVector3_t tr)const {v_scale(trans,-1,tr)};
  float const * GetTranslation()const{return trans;};

  void Transform(const CVector3_t in, CVector3_t out){v_add(in,trans,out);};
  void InverseTransform(const CVector3_t in, CVector3_t out){v_sub(in,trans,out);};
  
  /**
   * @brief updates the translation given an initial and transformed vector
   * @param v a 3d point
   * @param v_tr the transformed point after an unknown translation
   */
  void Update(CVector3_t v, CVector3_t v_tr);
  void Add(CVector3_t dtrans);
  void Invert(){v_scale(trans,-1,trans);}
  void Copy(const Translation& t){t.GetTranslation(trans);};
  void Scale(float f){v_scale(trans,f,trans);}

};

ostream&  operator<<(ostream& out, const Translation& t);
istream& operator>>(istream& in,Translation& t);

#endif
