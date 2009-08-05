#ifndef VECTOR3_H
#define VECTOR3_H

#include <math.h>
#include "TVector.h"

class Vector3 : public TVector<3>
{
  friend class Referential;
  friend class Matrix3;
  friend class Matrix4;
  
public:
  static const Vector3 ZERO;
  static const Vector3 EX;
  static const Vector3 EY;
  static const Vector3 EZ;

public:
  inline Vector3():TVector<3>(){};
  inline Vector3(const Vector3 &vector):TVector<3>(vector){};
  inline Vector3(const TVector<3> vector):TVector<3>(vector){};
  inline Vector3(const Vector &vector):TVector<3>(vector){};
  inline Vector3(const float _[3]):TVector<3>(_){};
  inline Vector3(float x, float y, float z):TVector<3>(){
    ///Set(x,y,z);
    _[0] = x; _[1] = y; _[2] = z;
  }   
  inline ~Vector3(){};
 
  inline float& x() {return _[0];}
  inline float& y() {return _[1];}
  inline float& z() {return _[2];} 

  inline float cx() {return _[0];}
  inline float cy() {return _[1];}
  inline float cz() {return _[2];} 

  
/*  inline Vector3& Set(float x, float y, float z)
  {
    _[0] = x; _[1] = y; _[2] = z;
    return *this;
  }
  */
  /*
  inline Vector3 operator - () const
  {
    Vector3 result;
    for (unsigned int i = 0; i < 3; i++)
      result._[i] = -_[i];
    return result;
  }
  */
  
  inline Vector3 Cross(const Vector3 &vector)
  {
    Vector3 result;
    return Cross(vector,result);
  }

  inline Vector3& Cross(const Vector3 &vector, Vector3& result)
  {
    result._[0]  = _[1] * vector._[2] - _[2] * vector._[1];
    result._[1]  = _[2] * vector._[0] - _[0] * vector._[2];
    result._[2]  = _[0] * vector._[1] - _[1] * vector._[0];
    return result;        
  }
};

//typedef Vector3 Vec3;

#endif
