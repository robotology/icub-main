#ifndef MATRIX3_H
#define MATRIX3_H

#include <math.h>
#include "TMatrix.h"
#include "Vector3.h"

class Matrix3 : public TMatrix<3>
{
  friend class Referential;
  friend class Matrix4;
public:
	static const Matrix3 ZERO;
	static const Matrix3 IDENTITY;

  inline Matrix3():TMatrix<3>(){};
  inline Matrix3(const Matrix3 &matrix):TMatrix<3>(matrix){};
  inline Matrix3(const TMatrix<3> &matrix):TMatrix<3>(matrix){};
  inline Matrix3(const Matrix & matrix):TMatrix<3>(matrix){};

  inline Matrix3(const float _[3][3]):TMatrix<3>(_){};
  inline Matrix3(float _00, float _01, float _02,
                 float _10, float _11, float _12,
                 float _20, float _21, float _22):TMatrix<3>(){
    _[0][0] = _00; _[0][1] = _01; _[0][2] = _02;  
    _[1][0] = _10; _[1][1] = _11; _[1][2] = _12;  
    _[2][0] = _20; _[2][1] = _21; _[2][2] = _22;  
  }

  inline virtual ~Matrix3(){}


  
  /*
  inline Matrix3& operator = (const TMatrix<3> &matrix)
  {    
    for (unsigned int j = 0; j < 3; j++){
      for (unsigned int i = 0; i < 3; i++)
        _[j][i] = matrix._[j][i];
    }
    return *this;    
  }*/ 
  


	inline Matrix3& SetColumns(const Vector3 &vector0, const Vector3 &vector1, const Vector3 &vector2)
	{
    SetColumn(vector0,0);
    SetColumn(vector1,1);
    SetColumn(vector2,2);
    return *this;
	}

	inline Matrix3& SetRows(const Vector3 &vector0, const Vector3 &vector1, const Vector3 &vector2)
	{
    SetRow(vector0,0);
    SetRow(vector1,1);
    SetRow(vector2,2);
    return *this;
	}
  

  Matrix3& RotationX(float angleX);
  Matrix3& RotationY(float angleY);
  Matrix3& RotationZ(float angleZ);
  /*
  Matrix3& RotationXY(float angleX, float angleY);
  Matrix3& RotationXZ(float angleX, float angleZ);
  Matrix3& RotationYX(float angleY, float angleX);
  Matrix3& RotationYZ(float angleY, float angleZ);
  Matrix3& RotationZX(float angleZ, float angleX);
  Matrix3& RotationZY(float angleZ, float angleY);
  */
  Matrix3& RotationYXZ(float angleX, float angleY, float angleZ);
  Matrix3& RotationV(float angle, const Vector3 &axis);
  
  inline Vector3  GetRotationAxis(){
    Vector3 result;
    return GetRotationAxis(result);
  }
  inline Vector3& GetRotationAxis(Vector3 &result){
    result._[0] =  _[2][1]-_[1][2];
    result._[1] =  _[0][2]-_[2][0];
    result._[2] =  _[1][0]-_[0][1];
    return result;
  }
  
  inline Vector3  GetExactRotationAxis(){
    Vector3 result;
    return GetExactRotationAxis(result);
  }
  inline Vector3& GetExactRotationAxis(Vector3 &result){
    GetNormRotationAxis(result);
    result *= GetRotationAngle();
    return result;
  }

  inline Vector3& GetNormRotationAxis(Vector3 &result){
    result._[0] =  _[2][1]-_[1][2];
    result._[1] =  _[0][2]-_[2][0];
    result._[2] =  _[1][0]-_[0][1];
    float norm = result.Norm();
    if(norm>EPSILON)
      result*=(1.0f/norm);
    else
      result.Zero();
    return result;
  }

  inline float GetRotationAngle(){
    float res = (_[0][0]+_[1][1]+_[2][2]-1.0f)/2.0f;
    if(res>1.0f) return 0.0f;
    else if (res<-1.0f) return PIf;
    else return acosf(res);
  }

  inline Matrix3& RotationV(const Vector3 &axis){
    RotationV(axis.Norm(),axis);
    return *this;    
  }


  inline Matrix3& RotationScale(float scale){
    Matrix3 result;
    return RotationScale(scale,result);      
  }
  
  inline Matrix3& RotationScale(float scale, Matrix3 & result){
    Vector3 v;
    GetNormRotationAxis(v);
    result.RotationV(GetRotationAngle()*scale,v);
    return result;
  }

  Matrix3 Transpose() const 
  {
    Matrix3 result;
    return Transpose(result);  
  }
  Matrix3 Transpose(Matrix3& result) const
  {
    for (unsigned int j = 0; j < 3; j++){
      for (unsigned int i = 0; i < 3; i++)
        result._[i][j] = _[j][i];
    }
    return result;
  }
  
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
};

typedef Matrix3 Mat3;

#endif
