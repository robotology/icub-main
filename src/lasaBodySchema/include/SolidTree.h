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
#ifndef __SOLIDTREE_H__
#define __SOLIDTREE_H__


#include "TreeParser.h"
#include "Geometry.h"
#include "mathlib.h"

//#ifndef PLANNER_VERSION
#ifndef LIGHT_VERSION
#include "GLTools.h"
#include <GL/glut.h>
#endif

#include <vector>
#include <string>
using namespace std;

class CMesh;
typedef CMesh *pCMesh;

class CSolid
{
public:
  CRef_t         m_ref;

  CVector3_t     m_position;

  CVector3_t     m_size;
  CVector3_t     m_color;

  string         m_name;

  pCMesh          m_mesh;
public:
  CSolid();
  virtual ~CSolid();
  virtual void Update();
  virtual void AddMesh(pCMesh mesh);
};
typedef CSolid CSolid_t, *pCSolid_t;



class CSolidTree;
typedef CSolidTree CSolidTree_t, *pCSolidTree_t;

typedef vector<pCSolidTree_t> CSolidTreeList_t;


class CSolidTree : public CSolid
{
public:
  CSolidTreeList_t m_next;

public:
  CSolidTree();
  //  CsolidTree(CSolidTree *tree); // to be checked
  CSolidTree(pTree config);
  virtual ~CSolidTree();
  void AddSolid(pCSolidTree_t newSolid);

  virtual void Update();
  virtual void AddMesh(pCMesh mesh);
  pCSolidTree_t Find(string name);
};


// Triangles
typedef int Triangle[3];
typedef struct{
  Triangle m_Triangle;
} CTriangle_ext_t;
typedef vector<CTriangle_ext_t> CTriangle_List_t;

class CMesh;
typedef CMesh *pCMesh;

// SkinMesh
class SkinMesh
{
public:
  vector<int>   m_Index;
  vector<float> m_Weight;
  CRef_t        m_Ref;
  string        m_Name;

  pCRef_t       m_ExtRef;

  pCMesh        m_Mesh;

public:
  SkinMesh(pCMesh mesh);
  SkinMesh(pCMesh mesh, pTree config);

  void Render();
  void SetExtRef(pCRef_t ref);
}; 
typedef SkinMesh *pSkinMesh;
typedef vector<pSkinMesh> SkinMesh_List;


// Mesh
class CMesh
{
public:
  CVector3_List_t   m_Vertices;  
  CVector3_List_t   m_Normals;  
  CTriangle_List_t  m_Triangles;

  SkinMesh_List     m_SkinMeshes;

  CVector3_List_t   m_SkinVertices;
  CVector3_List_t   m_SkinNormals;  

  bool              m_DrawWireFrame;
  bool              m_DrawSolid;


public:
  CMesh();
  CMesh(pTree config);
#ifndef LIGHT_VERSION
  void Render();
#endif
};


#ifndef LIGHT_VERSION
class CSolidRenderer
{
  pCSolid_t m_solid;

public:
  bool      m_drawOrient;
  bool      m_drawRef;

 public:
  CSolidRenderer(pCSolid_t newSolid);
  virtual ~CSolidRenderer();
  void RenderBegin();
  void RenderEnd(); 
  virtual void Render();
  void DrawRef();
  void DrawOrient();
};




class CSolidTreeRenderer;
typedef CSolidTreeRenderer CSolidTreeRenderer_t, *pCSolidTreeRenderer_t;

typedef vector<pCSolidTreeRenderer_t> CSolidTreeRendererList_t;

class CSolidTreeRenderer : public CSolidRenderer
{
  CSolidTreeRendererList_t m_next;

 public:
  CSolidTreeRenderer(pCSolidTree_t newSolidTree);
  virtual ~CSolidTreeRenderer();
  virtual void Render();
};

#endif
#endif
