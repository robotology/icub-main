#ifndef VECTOR3_H
#define VECTOR3_H

#include "MathLibCommon.h"



#include <math.h>
#include "TVector.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

/**
 * \class Vector3
 * 
 * \ingroup MathLib
 * 
 * \brief An implementation of the template TVector class
 */
class Vector3 : public TVector<3>
{
  /*
  friend class Referential;
  friend class Matrix3;
  friend class Matrix4;
  */
public:
  /// Zero vector
  static const Vector3 ZERO;
  /// Vector 1 0 0
  static const Vector3 EX;
  /// Vector 0 1 0
  static const Vector3 EY;
  /// Vector 0 0 1
  static const Vector3 EZ;

public:
  /// Empty constructor
  inline Vector3():TVector<3>(){};
  /// Copy constructor
  inline Vector3(const Vector3 &vector):TVector<3>(vector){};
  /// Copy constructor
  inline Vector3(const TVector<3> vector):TVector<3>(vector){};
  /*
  /// Copy constructor
  inline Vector3(const Vector &vector):TVector<3>(vector){};
  */
  /// Data-based constructor
  inline Vector3(const REALTYPE _[3]):TVector<3>(_){};
  /// Data-based constructor
  inline Vector3(REALTYPE x, REALTYPE y, REALTYPE z):TVector<3>(){
    Set(x,y,z);
  }   
  inline ~Vector3(){};
 
  /// Access to the first element (can be modified)
  inline REALTYPE& x() {return _[0];}
  /// Access to the second element (can be modified)
  inline REALTYPE& y() {return _[1];}
  /// Access to the third element (can be modified)
  inline REALTYPE& z() {return _[2];} 

  /// Access to the first element (constant)
  inline REALTYPE cx() const {return _[0];} 
  /// Access to the second element (constant)
  inline REALTYPE cy() const {return _[1];}
  /// Access to the thrid element (constant)
  inline REALTYPE cz() const {return _[2];} 

  inline Vector3& Set(REALTYPE x, REALTYPE y, REALTYPE z){
    _[0] = x; _[1] = y; _[2] = z;
    return *this;  
  }

  inline Vector3& Set(const Vector3 &vector){
    TVector<3>::Set(vector);
    return *this;  
  }

  inline Vector3& Set(const Vector &vector){
    TVector<3>::Set(vector);
    return *this;  
  }

  /// Cross product 
  inline Vector3 Cross(const Vector3 &vector) const
  {
    Vector3 result;
    return Cross(vector,result);
  }

  /// Cross product 
  inline Vector3& Cross(const Vector3 &vector, Vector3& result) const
  {
    result._[0]  = _[1] * vector._[2] - _[2] * vector._[1];
    result._[1]  = _[2] * vector._[0] - _[0] * vector._[2];
    result._[2]  = _[0] * vector._[1] - _[1] * vector._[0];
    return result;        
  }
};

typedef Vector3 Vec3;

#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif
