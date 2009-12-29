#ifndef QUATERNION_H
#define QUATERNION_H

#include "MathLibCommon.h"

#include <math.h>
#include "Vector3.h"
#include "Matrix3.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

class Quaternion
{
  friend class Vector3;
protected:
    union{
        REALTYPE _[4];
        struct {
            REALTYPE x, y, z, w;
        }; 
    };

public:
    static const Quaternion ZERO;
    static const Quaternion IDENTITY;


    inline Quaternion(){
        Zero();
    }

    inline Quaternion(const REALTYPE _[4]){
        for (int i = 0; i < 4; i++)
            this->_[i] = _[i];
    }

    inline Quaternion(const Quaternion &quaternion){
        for (int i = 0; i < 4; i++)
          _[i] = quaternion._[i];
    }
    inline Quaternion(REALTYPE w, REALTYPE x, REALTYPE y, REALTYPE z)
        : x(x), y(y), z(z), w(w) {}

    inline ~Quaternion(){}
  
    inline Quaternion& Zero(){
        for (int i = 0; i < 4; i++)
            this->_[i] = R_ZERO;
        return *this;           
    }
    inline Quaternion& Identity(){
        w = R_ONE;
        x = y = z = R_ZERO;
        return *this;           
    }

    inline Quaternion& Set(REALTYPE nw, REALTYPE nx, REALTYPE ny, REALTYPE nz){
        x = nx;
        y = ny;
        z = nz;
        w = nw;
        return *this;
    }
    
    REALTYPE X(){ return x;}
    REALTYPE Y(){ return y;}
    REALTYPE Z(){ return z;}
    REALTYPE W(){ return w;}

    inline Quaternion Minus() const {
        Quaternion result;
        return Minus(result);
    }

    inline Quaternion& Minus(Quaternion &result) const {
        for (int i = 0; i < 4; i++)
            result._[i] = -_[i];
        return result;
    }

    inline Quaternion& SMinus(){
        for (int i = 0; i < 4; i++)
            _[i] = -_[i];
        return *this;
    }

    inline Quaternion Conjugate() const{
        Quaternion result;
        return Conjugate(result);
    }
  
    inline Quaternion& Conjugate(Quaternion &result) const{
        result.x = -x;
        result.y = -y;
        result.z = -z;
        result.w =  w;
        return result;
    }
    inline Quaternion& SConjugate(){
        x = -x;
        y = -y;
        z = -z;        
    }

    Quaternion& Rotation(const Vector3& axisAngle);
    Quaternion& Rotation(const Vector3& axis, REALTYPE angle);
    Quaternion& RotationX(REALTYPE angle);
    Quaternion& RotationY(REALTYPE angle);
    Quaternion& RotationZ(REALTYPE angle);  
    Quaternion& Rotation(const Matrix3& rotation);


    inline REALTYPE Dot(const Quaternion &quaternion) const{
        return x*quaternion.x + y*quaternion.y + z*quaternion.z + w*quaternion.w;
    }
    Quaternion& SetNearest(Quaternion &quaternion){
        if(Dot(quaternion)<R_ZERO)
            quaternion.SMinus();
        return quaternion;        
    }
    inline REALTYPE Norm(){
        return sqrt(x*x+y*y+z*z+w*w);    
    }
    inline REALTYPE Norm2(){
        return x*x+y*y+z*z+w*w;    
    }
    
    inline Quaternion& Normalize(){
        REALTYPE norm = Norm();
        if(norm<1e-6){
            Identity();
        }else{
            norm = R_ONE/norm;
            for (int i = 0; i < 4; i++)
                _[i] *= norm;                
        }
        return *this;    
    }
  
    inline Quaternion operator - () const {
        Quaternion result;
        for (int i = 0; i < 4; i++)
            result._[i] = -_[i];
        return result;
    }

    inline Quaternion& operator = (const Quaternion &quaternion){
      for (int i = 0; i < 4; i++)
          _[i] = quaternion._[i];
      return *this;
    }

    inline Quaternion& operator *= (const Quaternion &quaternion)
    {
        Quaternion result;
        Mult(quaternion,result);
        *this = result;
        return *this;   
    }

    inline Quaternion operator * (const Quaternion &quaternion) const{
        Quaternion result;
        return Mult(quaternion,result);
    }

    inline Quaternion& Mult(const Quaternion &quaternion, Quaternion &result) const{
        result.x = w * quaternion.x + x * quaternion.w + y * quaternion.z - z * quaternion.y;
        result.y = w * quaternion.y + y * quaternion.w + z * quaternion.x - x * quaternion.z;
        result.z = w * quaternion.z + z * quaternion.w + x * quaternion.y - y * quaternion.x;
        result.w = w * quaternion.w - x * quaternion.x - y * quaternion.y - z * quaternion.z;
        return result;
    }


    Quaternion& Slerp(const Quaternion &target, REALTYPE t, Quaternion &result) const;

};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif










/*
    inline Vector3 Rotate(const Vector3 &vector) const;

    inline Quaternion& operator += (const Quaternion &quaternion)
    {
        for (int i = 0; i < 4; i++)
            _[i] += quaternion._[i];
        return *this;   
    }

    inline Quaternion& operator -= (const Quaternion &quaternion)
    {
        for (int i = 0; i < 4; i++)
            _[i] -= quaternion._[i];
        return *this;   
    }

    inline Quaternion operator + (const Quaternion &quaternion) const
    {
        Quaternion result;
        return Add(quaternion,result);
    }

  inline Quaternion& Add(const Quaternion &quaternion, Quaternion &result) const
  {
    for (int i = 0; i < 4; i++)
      result._[i] = _[i] + quaternion._[i];
    return result;    
  }

    inline Quaternion operator - (const Quaternion &quaternion) const
    {
    Quaternion result;
    return Sub(quaternion,result);
  }

  inline Quaternion& Sub(const Quaternion &quaternion, Quaternion &result) const
  {
        for (int i = 0; i < 4; i++)
            result._[i] = _[i] - quaternion._[i];
        return result;
    }

  inline Quaternion& operator += (REALTYPE scalar)
  {
    w += scalar;
    return *this;
  }

  inline Quaternion& operator -= (REALTYPE scalar)
  {
    w -= scalar;
    return *this;
  }

  inline Quaternion& operator *= (REALTYPE scalar)
  {
    w *= scalar;
    return *this;
  }

  inline Quaternion&  operator /= (REALTYPE scalar)
  {
    w /= scalar;
    return *this;
  }





  inline Quaternion operator + (REALTYPE scalar) const
  {
    Quaternion result;
    return Add(scalar,result);
  }

  inline Quaternion& Add(REALTYPE scalar, Quaternion &result) const
  {
    result = *this;
    result.w += scalar;
    return result;
  }

  inline Quaternion operator - (REALTYPE scalar) const
  {
    Quaternion result;
    return Sub(scalar,result);
  }

  inline Quaternion& Sub(REALTYPE scalar, Quaternion &result) const
  {
    result = *this;
    result.w -= scalar;
    return result;
  }

  inline Quaternion operator * (REALTYPE scalar) const
  {
    Quaternion result;
    return Mult(scalar,result);
  }

  inline Quaternion& Mult(REALTYPE scalar, Quaternion &result) const
  {
    result = *this;
    result.w *= scalar;
    return result;
  }

  inline Quaternion operator / (REALTYPE scalar) const
  {
    Quaternion result;
    return Div(scalar,result);
  }

  inline Quaternion& Div(REALTYPE scalar, Quaternion &result) const
  {
    result = *this;
    result.w /= scalar;
    return result;
  }

    inline Quaternion Inverse() const {
        Quaternion result;
        return Inverse(result);
    }

    inline Quaternion& Inverse(Quaternion &result) const{
        Conjugate(result);
        result /= Norm2();
    return result;
  }

*/
