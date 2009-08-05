#ifndef MATRIX4_H
#define MATRIX4_H

#include <math.h>
#include "TMatrix.h"
#include "Vector3.h"
#include "Matrix3.h"

class Matrix4 :public TMatrix<4>
{
  friend class Referential;
public:
	static const Matrix4 ZERO;
	static const Matrix4 IDENTITY;

  inline Matrix4():TMatrix<4>(){};
  inline Matrix4(const Matrix4 &matrix):TMatrix<4>(matrix){};
  inline Matrix4(const TMatrix<4> &matrix):TMatrix<4>(matrix){};
  inline Matrix4(const Matrix & matrix):TMatrix<4>(matrix){};
  
  inline Matrix4(const float _[4][4]):TMatrix<4>(_){};
  inline Matrix4(float _00, float _01, float _02, float _03,
                 float _10, float _11, float _12, float _13,
                 float _20, float _21, float _22, float _23,
                 float _30, float _31, float _32, float _33):TMatrix<4>(){
    _[0][0] = _00; _[0][1] = _01; _[0][2] = _02; _[0][3] = _03; 
    _[1][0] = _10; _[1][1] = _11; _[1][2] = _12; _[1][3] = _13;  
    _[2][0] = _20; _[2][1] = _21; _[2][2] = _22; _[2][3] = _23;  
    _[3][0] = _30; _[3][1] = _31; _[3][2] = _32; _[3][3] = _33;  
  }
  inline Matrix4(const Matrix3 &matrix):TMatrix<4>(){
    Vector3 v;
    Transformation(matrix, v);
  }

  inline virtual ~Matrix4(){}


  inline Vector3 GetTranslation()
  {
    Vector3 result;
    return GetTranslation(result);
  }

  inline Vector3& GetTranslation(Vector3 &result)
  {
    result._[0] = _[0][3];
    result._[1] = _[1][3];
    result._[2] = _[2][3];
    return result;
  }

  inline Matrix4& SetTranslation(Vector3 &trans)
  {
    _[0][3] = trans._[0];
    _[1][3] = trans._[1];
    _[2][3] = trans._[2];
    return *this;
  }
  
  inline Matrix3 GetOrientation()
  {
    Matrix3 result;
    return GetOrientation(result);
  }

  inline Matrix3& GetOrientation(Matrix3 &result)
  {
    result._[0][0] = _[0][0]; result._[0][1] = _[0][1]; result._[0][2] = _[0][2];
    result._[1][0] = _[1][0]; result._[1][1] = _[1][1]; result._[1][2] = _[1][2];
    result._[2][0] = _[2][0]; result._[2][1] = _[2][1]; result._[2][2] = _[2][2];
    return result;
  }

  inline Matrix4& SetOrientation(Matrix3 &orient)
  {
    _[0][0] = orient._[0][0]; _[0][1] = orient._[0][1]; _[0][2] = orient._[0][2];
    _[1][0] = orient._[1][0]; _[1][1] = orient._[1][1]; _[1][2] = orient._[1][2];
    _[2][0] = orient._[2][0]; _[2][1] = orient._[2][1]; _[2][2] = orient._[2][2];
    return *this;
  }

  inline Matrix4& Transformation(const Matrix3 &rotation, const Vector3 &translation)
  {
    for (unsigned int j = 0; j < 3; j++){
      for (unsigned int i = 0; i < 3; i++)
        _[j][i] = rotation._[j][i];
      _[j][3] = translation._[j];
      _[3][j] = 0.0f;
    }
    _[3][3] = 1.0f;    
    return *this;
  }

  inline Matrix4& InverseTransformation(const Matrix3 &rotation, const Vector3 &translation)
  {
    for (int j = 0; j < 3; j++){
      _[j][3] = 0.0f;
      for (int i = 0; i < 3; i++){
        _[j][i]  = rotation._[i][j];
        _[j][3] -= rotation._[i][j] * translation._[i];
      }
      _[3][j] = 0.0f;
    }
    _[3][3] = 1.0f;
    return *this;
  }

  inline Matrix4& InverseTransformation() const
  {
    Matrix4 result;
    return InverseTransformation(result);   
  }

  inline Matrix4& InverseTransformation(Matrix4 &result) const
  {
    for (int j = 0; j < 3; j++){
      result._[j][3] = 0.0f;
      for (int i = 0; i < 3; i++){
        result._[j][i]  = _[i][j];
        result._[j][3] -= _[i][j] * _[i][3];
      }
      result._[3][j] = 0.0f;
    }
    result._[3][3] = 1.0f;    
    return result;    
  }
  
  
  /*
  inline Vector3 operator * (const Vector3 &vector) const
  {
    Vector3 result;
    return Mult(vector,result);
  }
  */
  inline Vector3 Transform(const Vector3 &vector) const
  {
    Vector3 result;
    return Transform(vector,result);
  }

  inline Vector3& Transform(const Vector3 &vector, Vector3 & result) const
  {
    for (unsigned int j = 0; j < 3; j++)
    {
      result._[j] = 0.0f;
      for (unsigned int i = 0; i < 3; i++)
        result._[j] += _[j][i] * vector._[i];
      result._[j] += _[j][3];  
    }
    return result;
  } 

  inline Matrix4& Transform(const Matrix4 &matrix, Matrix4 & result) const
  {
    result.Identity();
    for(unsigned int k = 0; k< 4; k++){          
      for (unsigned int j = 0; j < 3; j++)
      {
        result._[j][k] = 0.0f;
        for (unsigned int i = 0; i < 3; i++)
          result._[j][k] += _[j][i] * matrix._[i][k];
        result._[j][k] += _[j][3];  
      }
    }
    return result;
  } 
     
};

typedef Matrix4 Mat4;

#endif
