#ifndef MATRIX3_H
#define MATRIX3_H

#include "MathLibCommon.h"

#include <math.h>
#include "TMatrix.h"
#include "Vector3.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

/**
 * \class Matrix3
 * 
 * \ingroup MathLib
 * 
 * \brief An implementation of the template TMatrix class
 * 
 * This template square matrix class can be used for doing various matrix manipulation.
 * This should be combined with the TVector class for doing almost anything
 * you ever dreamt of. 
 */

class Matrix3 : public TMatrix<3>
{
  //friend class Referential;
  //friend class Matrix4;
public:
  /// A constant zero matrix
  static const Matrix3 ZERO;
  /// A constant identity matrix
  static const Matrix3 IDENTITY;

  /// Empty constructor
  inline Matrix3():TMatrix<3>(){};
  /// Copy constructor
  inline Matrix3(const Matrix3 &matrix):TMatrix<3>(matrix){};
  /// Copy constructor
  inline Matrix3(const TMatrix<3> &matrix):TMatrix<3>(matrix){};
  /*
  /// Copy constructor
  //inline Matrix3(const Matrix & matrix):TMatrix<3>(matrix){};
  */
  /// Constructor with data pointer
  inline Matrix3(const REALTYPE _[3][3]):TMatrix<3>(_){};
  /// Constructor with values
  inline Matrix3(REALTYPE _00, REALTYPE _01, REALTYPE _02,
                 REALTYPE _10, REALTYPE _11, REALTYPE _12,
                 REALTYPE _20, REALTYPE _21, REALTYPE _22):TMatrix<3>(){
    Set(_00,_01,_02,
        _10,_11,_12,
        _20,_21,_22);
  }

  inline virtual ~Matrix3(){}


  inline Matrix3& Set(REALTYPE _00, REALTYPE _01, REALTYPE _02,
                      REALTYPE _10, REALTYPE _11, REALTYPE _12,
                      REALTYPE _20, REALTYPE _21, REALTYPE _22){
    _[0][0] = _00; _[0][1] = _01; _[0][2] = _02;  
    _[1][0] = _10; _[1][1] = _11; _[1][2] = _12;  
    _[2][0] = _20; _[2][1] = _21; _[2][2] = _22;        
    return *this;
  }

  inline Matrix3& Set(const Matrix3& matrix){
    TMatrix<3>::Set(matrix);
    return *this;
  }

  inline Matrix3& Set(const Matrix& matrix){
    TMatrix<3>::Set(matrix);
    return *this;
  }


  /// Assign each column
	inline Matrix3& SetColumns(const Vector3 &vector0, const Vector3 &vector1, const Vector3 &vector2)
	{
    SetColumn(vector0,0);
    SetColumn(vector1,1);
    SetColumn(vector2,2);
    return *this;
	}

  /// Assign each row
	inline Matrix3& SetRows(const Vector3 &vector0, const Vector3 &vector1, const Vector3 &vector2)
	{
    SetRow(vector0,0);
    SetRow(vector1,1);
    SetRow(vector2,2);
    return *this;
	}

  /// Create a skew symmetric matix from a vector 
  inline Matrix3& SkewSymmetric(Vector3 vector){    
    _[0][0] = R_ZERO;      _[1][1] = R_ZERO;      _[2][2] = R_ZERO;
    _[1][0] = vector._[2]; _[2][1] = vector._[0]; _[0][2] = vector._[1];
    _[0][1] =-vector._[2]; _[1][2] =-vector._[0]; _[2][0] =-vector._[1];
    return *this;
  }

  
  /// Create a rotation matrix around axis X with a given angle 
  Matrix3& RotationX(REALTYPE angleX);

  /// Create a rotation matrix around axis Y with a given angle 
  Matrix3& RotationY(REALTYPE angleY);

  /// Create a rotation matrix around axis Z with a given angle 
  Matrix3& RotationZ(REALTYPE angleZ);

  /// Create a rotation matrix around axes X,Y,Z with given angles 
  Matrix3& RotationYXZ(REALTYPE angleX, REALTYPE angleY, REALTYPE angleZ);

  /// Create a rotation matrix around an arbitrary axis with a given angle 
  Matrix3& RotationV(REALTYPE angle, const Vector3 &axis);
  
  /// Create a rotation matrix around an arbitrary axis with a given angle 
  inline Matrix3& RotationV(const Vector3 &axis){
    RotationV(axis.Norm(),axis);
    return *this;    
  }

  /// Get the rotation axis of a rotation matrix (arbitrary norm)
  inline Vector3  GetRotationAxis(){
    Vector3 result;
    return GetRotationAxis(result);
  }
  /// Get the rotation axis of a rotation matrix (arbitrary norm)
  inline Vector3& GetRotationAxis(Vector3 &result){
    result._[0] =  _[2][1]-_[1][2];
    result._[1] =  _[0][2]-_[2][0];
    result._[2] =  _[1][0]-_[0][1];
    return result;
  }
  
  /// Get the rotation axis of a rotation matrix (the norm equals the rotation angle)
  inline Vector3  GetExactRotationAxis() const{
    Vector3 result;
    return GetExactRotationAxis(result);
  }

  /// Get the rotation axis of a rotation matrix (the norm equals the rotation angle)
  inline Vector3& GetExactRotationAxis(Vector3 &result) const{
    GetNormRotationAxis(result);
    result *= GetRotationAngle();
    return result;
  }

  /// Get the rotation axis of a rotation matrix (the norm equals 1)
  inline Vector3& GetNormRotationAxis(Vector3 &result) const{
    result._[0] =  _[2][1]-_[1][2];
    result._[1] =  _[0][2]-_[2][0];
    result._[2] =  _[1][0]-_[0][1];
    REALTYPE norm = result.Norm();
    if(norm>EPSILON)
      result*=(1.0f/norm);
    else
      result.Zero();
    return result;
  }

  /// Get the rotation angle of a rotation matrix
  inline REALTYPE GetRotationAngle() const{
    REALTYPE res = (_[0][0]+_[1][1]+_[2][2]-1.0f)/2.0f;
    if(res>1.0f) return 0.0f;
    else if (res<-1.0f) return PI;
    else return acosf(res);
  }


  /// Return a matrix where the amount of rotation has been scaled
  inline Matrix3& RotationScale(REALTYPE scale){
    Matrix3 result;
    return RotationScale(scale,result);      
  }
  
  /// Return a matrix where the amount of rotation has been scaled
  inline Matrix3& RotationScale(REALTYPE scale, Matrix3 & result){
    Vector3 v;
    GetNormRotationAxis(v);
    result.RotationV(GetRotationAngle()*scale,v);
    return result;
  }

  /// Return a rotation matrix in between src and dst. Scale is in [0,1]
  inline Matrix3& RotationScale(const Matrix3& src, const Matrix3& dst, REALTYPE scale){
    
    if(scale<R_ZERO) scale = R_ZERO;
    if(scale>R_ONE)  scale = R_ONE;
    
    Matrix3 tmpM;    
    src.Transpose(tmpM);
    tmpM.Mult(dst,*this);
    
    Vector3 tmpV;
    GetExactRotationAxis(tmpV);    
    tmpV *= scale;
    
    tmpM.RotationV(tmpV);
    
    src.Mult(tmpM,*this);
    return *this;
  }
  
  /// Transpose
  Matrix3 Transpose() const 
  {
    Matrix3 result;
    return Transpose(result);  
  }
  
  /// Transpose
  Matrix3 Transpose(Matrix3& result) const
  {
    for (unsigned int j = 0; j < 3; j++){
      for (unsigned int i = 0; i < 3; i++)
        result._[i][j] = _[j][i];
    }
    return result;
  }
  
  /// Normalize the matrix (Gram-Schmidt) with the given primary axis (x=0,y=1,z=2)
  Matrix3& Normalize(int mainAxe = 2)
  {
    if((mainAxe<0)||(mainAxe>2)) mainAxe=2;
    
    Vector3 v0(_[0][0],_[1][0],_[2][0]);
    Vector3 v1(_[0][1],_[1][1],_[2][1]);
    Vector3 v2(_[0][2],_[1][2],_[2][2]);
    switch(mainAxe){
    case 0:
      
      v0.Normalize();
      v1-=v0 * v0.Dot(v1);
      v1.Normalize();
      v2 = v0.Cross(v1);
  
      break;
    case 1:
      v1.Normalize();
      v2-=v1 * v1.Dot(v2);
      v2.Normalize();
      v0 = v1.Cross(v2);
      break;
    case 2:
      v2.Normalize();
      v1-=v2 * v2.Dot(v1);
      v1.Normalize();
      v0 = v1.Cross(v2);
      break;
    }
    SetColumn(v0,0);  
    SetColumn(v1,1);  
    SetColumn(v2,2);
    return *this;
  }  
  
  /// Normalize the matrix (Gram-Schmidt) according to the given order of axis (x=0,y=1,z=2)
  Matrix3& Normalize(int firAxe, int secAxe, int trdAxe)
  {
    
    Vector3 v0(_[0][0],_[1][0],_[2][0]);
    Vector3 v1(_[0][1],_[1][1],_[2][1]);
    Vector3 v2(_[0][2],_[1][2],_[2][2]);

    Vector3 w0;
    Vector3 w1;
    Vector3 w2;

    switch(firAxe){case 0: w0 = v0; break; case 1: w0 = v1; break; case 2: w0 = v2; break;}
    switch(secAxe){case 0: w1 = v0; break; case 1: w1 = v1; break; case 2: w1 = v2; break;}
    switch(trdAxe){case 0: w2 = v0; break; case 1: w2 = v1; break; case 2: w2 = v2; break;}

    w0.Normalize();
    w1-=w0 * w0.Dot(w1);
    w1.Normalize();
    w2 = w0.Cross(w1);
  
    switch(firAxe){case 0: v0 = w0; break; case 1: v0 = w1; break; case 2: v0 = w2; break;}
    switch(secAxe){case 0: v1 = w0; break; case 1: v1 = w1; break; case 2: v1 = w2; break;}
    switch(trdAxe){case 0: v2 = w0; break; case 1: v2 = w1; break; case 2: v2 = w2; break;}
    
    SetColumn(v0,0);  
    SetColumn(v1,1);  
    SetColumn(v2,2);
    return *this;
  }
  
  Matrix3& SetCross(const Vector3& vector)
  {
    _[0][0] =            0; _[0][1] = -vector._[2]; _[0][2] =  vector._[1];  
    _[1][0] =  vector._[2]; _[1][1] =            0; _[1][2] = -vector._[0];
    _[2][0] = -vector._[1]; _[2][1] =  vector._[0]; _[2][2] =            0;
    return *this;        
  }
  
  Matrix3& EulerRotation(int axis0, int axis1, int axis2, const Vector3& angles);  
  
  Vector3& GivensRotationPlane(REALTYPE a, REALTYPE b, Vector3 & result, int path=0);
  Vector3& GetEulerAnglesGeneric(int i, int neg, int alt, int rev, Vector3& result, int path=0);

  Vector3& GetEulerAnglesXZX(bool rev, Vector3& result, int path=0){return GetEulerAnglesGeneric(0,0,0,(rev?1:0),result,path);} 
  Vector3& GetEulerAnglesYXY(bool rev, Vector3& result, int path=0){return GetEulerAnglesGeneric(1,0,0,(rev?1:0),result,path);}
  Vector3& GetEulerAnglesZYZ(bool rev, Vector3& result, int path=0){return GetEulerAnglesGeneric(2,0,0,(rev?1:0),result,path);} 

  Vector3& GetEulerAnglesXZY(bool rev, Vector3& result, int path=0){
    if(rev){return GetEulerAnglesGeneric(1,1,1,(rev?1:0),result,path);}
    else{   return GetEulerAnglesGeneric(0,0,1,(rev?1:0),result,path);}
  } 
  Vector3& GetEulerAnglesYXZ(bool rev, Vector3& result, int path=0){
    if(rev){return GetEulerAnglesGeneric(2,1,1,(rev?1:0),result,path);}
    else{   return GetEulerAnglesGeneric(1,0,1,(rev?1:0),result,path);}
  } 
  Vector3& GetEulerAnglesZYX(bool rev, Vector3& result, int path=0){ 
    if(rev){return GetEulerAnglesGeneric(0,1,1,(rev?1:0),result,path);}
    else{   return GetEulerAnglesGeneric(2,0,1,(rev?1:0),result,path);}
  }  
  Vector3& GetEulerAnglesXYX(bool rev, Vector3& result, int path=0){return GetEulerAnglesGeneric(0,1,0,(rev?1:0),result,path);} 
  Vector3& GetEulerAnglesYZY(bool rev, Vector3& result, int path=0){return GetEulerAnglesGeneric(1,1,0,(rev?1:0),result,path);} 
  Vector3& GetEulerAnglesZXZ(bool rev, Vector3& result, int path=0){return GetEulerAnglesGeneric(2,1,0,(rev?1:0),result,path);} 
 
  Vector3& GetEulerAnglesXYZ(bool rev, Vector3& result, int path=0){
    if(rev){return GetEulerAnglesGeneric(2,0,1,(rev?1:0),result,path);}
    else{   return GetEulerAnglesGeneric(0,1,1,(rev?1:0),result,path);}
  } 
  Vector3& GetEulerAnglesYZX(bool rev, Vector3& result, int path=0){
    if(rev){return GetEulerAnglesGeneric(0,0,1,(rev?1:0),result,path);}
    else{   return GetEulerAnglesGeneric(1,1,1,(rev?1:0),result,path);}
  } 
  Vector3& GetEulerAnglesZXY(bool rev, Vector3& result, int path=0){
    if(rev){return GetEulerAnglesGeneric(1,0,1,(rev?1:0),result,path);}
    else{   return GetEulerAnglesGeneric(2,1,1,(rev?1:0),result,path);}    
  } 

  Vector3& GetEulerAngles(int r1, int r2, int r3, int rev, Vector3& result, int path=0){
    switch(r1){
    case 0:
      switch(r2){
      case 1:
        switch(r3){
        case 0: return GetEulerAnglesXYX(rev, result,path); break;
        case 2: return GetEulerAnglesXYZ(rev, result,path); break;
        }
        break;
      case 2:
        switch(r3){
        case 0: return GetEulerAnglesXZX(rev, result,path); break;
        case 1: return GetEulerAnglesXZY(rev, result,path); break;
        }
        break;
      }
      break;
    case 1:
      switch(r2){
      case 0:
        switch(r3){
        case 1: return GetEulerAnglesYXY(rev, result,path); break;
        case 2: return GetEulerAnglesYXZ(rev, result,path); break;
        }
        break;
      case 2:
        switch(r3){
        case 0: return GetEulerAnglesYZX(rev, result,path); break;
        case 1: return GetEulerAnglesYZY(rev, result,path); break;
        }
        break;
      }
      break;
    case 2:
      switch(r2){
      case 0:
        switch(r3){
        case 1: return GetEulerAnglesZXY(rev, result,path); break;
        case 2: return GetEulerAnglesZXZ(rev, result,path); break;
        }
        break;
      case 1:
        switch(r3){
        case 0: return GetEulerAnglesZYX(rev, result,path); break;
        case 2: return GetEulerAnglesZYZ(rev, result,path); break;
        }
        break;
      }
      break;
    }
    cout << "GET EULER ANGLES ERROR (bad indices)"<<endl;
    return result;
  } 

};

typedef Matrix3 Mat3;

#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif
