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
#include "Translation.h"


Translation::Translation(){
  v_clear(trans);
  // SetRate(0.0001);
  SetRate(0.003);
}


void Translation::Update(CVector3_t v, CVector3_t v_tr){
  CVector3_t y,v1;
  Transform(v,y);
  v_sub(v_tr,y,v1);
  v_scale(v1,eps_tr,v1);
  v_add(trans,v1,trans);
}

void Translation::Add(CVector3_t dtrans){
  CVector3_t v;
  v_scale(dtrans,eps_tr,v);
  v_add(trans,v,trans);
}




ostream&  operator<<(ostream& out, const Translation& t){
  CVector3_t v;
  t.GetTranslation(v);
  out<<v[0]<<" "<<v[1]<<" "<<v[2];
  return out;
}

istream& operator>>(istream& in,Translation& t){
  CVector3_t v;
  in>>v[0]>>v[1]>>v[2];
  t.SetTranslation(v);
  return in;
}
