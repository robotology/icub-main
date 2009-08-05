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
#include "Geometry.h"


CRef::CRef(){
  Identity();
}
CRef::~CRef(){}

void CRef::Update(){
  CMatrix3_t tmp;
  m_translate(m_origin,tmp);
#ifndef KINEMATIC_TREE_VIEW
  m_multiply(tmp,m_orient,m_result);
#else
  m_multiply(m_orient,tmp,m_result);
#endif
  if(m_absRef!=NULL){
  	m_absRef->Update();
    m_multiply(*m_absRef->GetAbsRef(),m_result,m_absResult);
    m_multiply(*m_absRef->GetAbsRef(),tmp,m_absResultPos);
  }else{
    m_copy(m_result,m_absResult);
    m_copy(tmp,m_absResultPos);
  }
}

void CRef::Identity(){
  m_absRef  = NULL;
  v_set(0,0,0,m_origin);  
  m_identity(m_orient);
  m_identity(m_result);
  m_identity(m_absResult);
}

CMatrix4_t *CRef::GetRef(){
  return &m_result;
}

CMatrix4_t *CRef::GetAbsRef(){
  return &m_absResult;
}

CMatrix4_t *CRef::GetAbsRefPos(){
  return &m_absResultPos;
}

void CRef::Copy(CRef_t *src){
  m_absRef = src->m_absRef;
  v_copy(src->m_origin,m_origin);
  m_copy(src->m_orient,m_orient);
  Update();
}

