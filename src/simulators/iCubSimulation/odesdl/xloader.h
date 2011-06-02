// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff
* This code was written starting from tutorial code by Kosei Demura
* email:   vadim.tikhanoff@iit.it
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


/**
 * \file xloader.h
 * \brief Header for the 3D x loader
 * \date 2007
 * \note Released under GNU GPL v2.0
 **/

#ifndef XLOADER_H
#define XLOADER_H
#include <ode/ode.h>

/*typedef struct dTRIMESH {
  float   *Vertices;
  int     VertexCount;
  int     *Indices;
  int     IndexCount;
} dTriMesh;

void dLoadMeshFromX( const char* filename, dTriMesh* trimesh );
*/

struct dxTriMeshX {
    float   *Vertices;
    int     VertexCount;
    int     *Indices;
    int     IndexCount;
    float     *MeshCoord;
    int     MeshCoordCount;
    float   *NormCoord;
    int     NormCount;
};

typedef struct dxTriMeshX *dTriMeshX;

dTriMeshX dLoadMeshFromX(const char* FileName);

void dTriMeshXDestroy(dTriMeshX TriMesh);


#endif
