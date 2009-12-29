#ifndef GL3DOBJECT_H_
#define GL3DOBJECT_H_

#include "GLTools.h"
#include "MathLib/MathLib.h"
using namespace MathLib;

#define GL3DOBJ_MAX_TRIANGLES   32768
#define GL3DOBJ_MAX_VERTICES    32768

class GL3DObject
{
public:
  static  int   objCount;
  static  float *verData;
  static  int   *triData;

  
public:
  int nbVertices;
  int nbTriangles;
  
  int   *triangles;
  float *vertices;
  float *normals;

  bool  invNormals;
    
  int   callListId;
  
public:  
  GL3DObject();
  ~GL3DObject();
  
  void Free();
  
  int   LoadFromObjFile(const char *filename, bool invNormals=false);
  void  CalcNormals();
  void  Render();
  int   BuildDisplayList();
  void  AddOffset(Vector3& offset);
  void  Transform(Matrix3& trans);
};


#endif /*GL3DOBJECT_H_*/
