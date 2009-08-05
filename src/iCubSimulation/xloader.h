/**
 * \file xloader.h
 * \brief Header for the 3D x loader
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/

#include <ode/ode.h>

typedef struct dTRIMESH {
  float   *Vertices;
  int     VertexCount;
  int     *Indices;
  int     IndexCount;
} dTriMesh;

void dLoadMeshFromX( const char* filename, dTriMesh* trimesh );
