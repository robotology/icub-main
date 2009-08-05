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
#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <vector>
using namespace std;

#include "mathlib.h"

//#define KINEMATIC_TREE_VIEW

/*typedef struct{
    vec3_t  v;
} CVector3_ext_t;
  */
union CVector3_ext_t{
  vec3_t  v;
  vec3_t m_Vector;
};

typedef vector<CVector3_ext_t> CVector3_List_t;


class CRef;
typedef CRef CRef_t, *pCRef_t;

class CRef
{
public:
  // Up referential
	pCRef_t		    m_absRef;

  // Self origin
  CVector3_t    m_origin;
  // Self orientation
  CMatrix4_t    m_orient;

  // Self Referential
  CMatrix4_t    m_result;

  // Absolute referential
  CMatrix4_t    m_absResult;

  // Absolute referential without orientation
  CMatrix4_t    m_absResultPos;
public:
  CRef();
  virtual ~CRef();

  virtual void  Update();
  
  void          Identity();
  CMatrix4_t*   GetRef();
  CMatrix4_t*   GetAbsRef();
  CMatrix4_t*   GetAbsRefPos();

  CVector3_t*   GetOrigin();
  void          SetOrigin(float x, float y, float z);
  void          SetOrigin(CVector3_t origin);

  CMatrix4_t*   GetOrient();
  void          SetOrient(CMatrix4_t orient);

  void Copy(CRef_t *src);
};

/*

class JointDOF
{
public:
  float       m_Value;
  float       m_dValue;
  float       m_Cost;
  pCRef_t     m_Ref;
  CVector3_t  m_Axis;

public:
  JointDOF(){
    m_Value   = 0.0f;
    m_dValue  = 0.0f;
    m_Cost    = 0.0f;
    m_Ref     = NULL;

    v_set(0,0,0,m_Axis);
  }

  void              Apply(){
    CMatrix_t RotMat,tmp;
    m_rotation_v(m_dValue,m_Axis,RotMat);
    m_copy(m_Ref->m_orient,tmp);  
    m_multiply(tmp,RotMat,m_Ref.m_orient);
  }
  virtual float     UpdateValue(){
    m_Value = 0.0f;
    return m_Value;
  }
  virtual float     UpdateCost(){
    m_Cost  = 0.0f;
    return m_Cost;
  }

  void              AddValue(float dStep){
    m_dValue = dStep;
    Apply();
  }

};
*/
#endif
