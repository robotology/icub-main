#ifndef __GLCAMERA_H__
#define __GLCAMERA_H__
#ifdef WIN32
#include <windows.h>
#endif

#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>
#include <string>
using namespace std;

#include "MathLib/MathLib.h"
using namespace MathLib;

class GLCamera;
typedef GLCamera *pGLCamera;
typedef vector<pGLCamera> GLCamera_List;

class GLCamera
{
public:
  enum CameraMode{FreeMove,Centered};
public:
  CameraMode m_Mode;
  
  Vector3 m_position;
  Matrix3 m_orient;

  Ref     m_ref;
  Ref     m_holdRef;

  bool    m_hold;

  int     m_X;
  int     m_Y;
  int     m_Width;
  int     m_Height;

  float   m_Near;
  float   m_Far;
  
  float   m_ViewAngle;
  
  float   m_PrincipalX;
  float   m_PrincipalY;
  float   m_ImWidth;
  float   m_ImHeight;

  float   m_FocalX;
  float   m_FocalY;

public:
  GLCamera();
  
  void Clear();

  void SetViewport(int w, int h);
  void SetViewport(int x, int y, int w, int h);

  void SetProjection(float viewAngle, float near, float far);
  void SetProjection(float im_width,
                     float im_height,
                     float principal_x,
		                 float principal_y,
		                 float focal_x,
		                 float focal_y,
		                 float near, float far);

  void SetOrientation(const Matrix3 & orient);
  void SetPosition(const Vector3 & pos);

  void Apply(bool setIdentity=true);

  void Hold();
  void Accept();

  void Move(float dx, float dy, float dz);
/*
 *   void Move(float dx, float dy, float dz, float ay, float ax,float az=0.0f);

  void Save(string fname);
  
  void Load(string fname);
  */
};


#endif
