#ifndef SPATIAL_MATRIX_H
#define SPATIAL_MATRIX_H

#include "Matrix3.h"
#include "SpatialVector.h"
/**
 * \class SpatialMatrix
 * 
 * \ingroup MathLib
 * 
 * \brief An implementation of the template TMatrix class
 * 
 * This template square matrix class can be used for doing various matrix manipulation.
 * This should be combined with the TVector class for doing almost anything
 * you ever dreamt of. 
 */

class SpatialMatrix
{
public:
  Matrix3 m00;
  Matrix3 m01;
  Matrix3 m10;
  Matrix3 m11;
  
public:
  /// Empty constructor
  inline SpatialMatrix(){};
  /// Copy constructor
  inline SpatialMatrix(const SpatialMatrix &matrix){
    Set(matrix);
  };  
  /// Copy Constructor 
  inline SpatialMatrix(const Matrix3& _00,const Matrix3& _01,const Matrix3& _10,const Matrix3& _11){
    Set(_00,_01,_10,_11);
  };
  /// Constructor with values
  inline SpatialMatrix(REALTYPE _00, REALTYPE _01, REALTYPE _02, REALTYPE _03, REALTYPE _04, REALTYPE _05,
                       REALTYPE _10, REALTYPE _11, REALTYPE _12, REALTYPE _13, REALTYPE _14, REALTYPE _15,
                       REALTYPE _20, REALTYPE _21, REALTYPE _22, REALTYPE _23, REALTYPE _24, REALTYPE _25,
                       REALTYPE _30, REALTYPE _31, REALTYPE _32, REALTYPE _33, REALTYPE _34, REALTYPE _35,
                       REALTYPE _40, REALTYPE _41, REALTYPE _42, REALTYPE _43, REALTYPE _44, REALTYPE _45,
                       REALTYPE _50, REALTYPE _51, REALTYPE _52, REALTYPE _53, REALTYPE _54, REALTYPE _55){
    Set(_00,_01,_02,_03,_05,_05,
        _10,_11,_12,_13,_15,_15,
        _20,_21,_22,_23,_25,_25,
        _30,_31,_32,_33,_35,_35,
        _40,_41,_42,_43,_45,_45,
        _50,_51,_52,_53,_55,_55);
  }

  /// Destructor
  inline virtual ~SpatialMatrix(){}

  inline SpatialMatrix& Set(const SpatialMatrix& matrix){
    m00.Set(matrix.m00);  
    m01.Set(matrix.m01);  
    m10.Set(matrix.m10);  
    m11.Set(matrix.m11);
    return *this;  
  }

  inline SpatialMatrix& Set(REALTYPE _00, REALTYPE _01, REALTYPE _02, REALTYPE _03, REALTYPE _04, REALTYPE _05,
                            REALTYPE _10, REALTYPE _11, REALTYPE _12, REALTYPE _13, REALTYPE _14, REALTYPE _15,
                            REALTYPE _20, REALTYPE _21, REALTYPE _22, REALTYPE _23, REALTYPE _24, REALTYPE _25,
                            REALTYPE _30, REALTYPE _31, REALTYPE _32, REALTYPE _33, REALTYPE _34, REALTYPE _35,
                            REALTYPE _40, REALTYPE _41, REALTYPE _42, REALTYPE _43, REALTYPE _44, REALTYPE _45,
                            REALTYPE _50, REALTYPE _51, REALTYPE _52, REALTYPE _53, REALTYPE _54, REALTYPE _55){
    m00.Set(_00,_01,_02,
            _10,_11,_12,
            _20,_21,_22);
    m01.Set(_03,_04,_05,
            _13,_14,_15,
            _23,_24,_25);
    m10.Set(_30,_31,_32,
            _40,_41,_42,
            _50,_51,_52);
    m11.Set(_33,_34,_35,
            _43,_44,_45,
            _53,_54,_55);
    return *this;
  }
  
  inline SpatialMatrix& Set(const Matrix3& _00,const Matrix3& _01,const Matrix3& _10,const Matrix3& _11){
    m00.Set(_00);
    m01.Set(_01);
    m10.Set(_10);
    m11.Set(_11);
    return *this;
  }


  /// Get the sub matrix of size 3 given a block coordinate
  inline Matrix3& GetSubMatrix3(unsigned int blockRow, unsigned int blockCol){
    if(blockRow==0){
      if(blockCol==0){
        return m00;
      }else if(blockCol==1){
        return m01;
      }
    }else if(blockRow==1){
      if(blockCol==0){
        return m10;
      }else if(blockCol==1){
        return m11;
      }      
    }
  }
  
  inline Matrix3& GetSubMatrix3(unsigned int blockRow, unsigned int blockCol, Matrix3 &result) const
  {
    if(blockRow==0){
      if(blockCol==0){
        result.Set(m00);
      }else if(blockCol==1){
        result.Set(m01);
      }
    }else if(blockRow==1){
      if(blockCol==0){
        result.Set(m10);
      }else if(blockCol==1){
        result.Set(m11);
      }      
    }
    return result;
  }

  /// Set the sub matrix of size 3 given a block coordinate
  inline SpatialMatrix SetSubMatrix3(unsigned int blockRow, unsigned int blockCol, const Matrix3 &matrix)
  {
    if(blockRow==0){
      if(blockCol==0){
        m00.Set(matrix);
      }else if(blockCol==1){
        m01.Set(matrix);
      }
    }else if(blockRow==1){
      if(blockCol==0){
        m10.Set(matrix);
      }else if(blockCol==1){
        m11.Set(matrix);
      }      
    }
    return *this;
  }

  /// Assignment operator
  inline SpatialMatrix& operator = (const SpatialMatrix &matrix)
  {
    return Set(matrix);    
  }  

  /// Inverse operator
  inline SpatialMatrix operator - () const
  {
    SpatialMatrix result;
    return Minus(result);
  }

  /// Inverse operator
  inline SpatialMatrix& Minus(SpatialMatrix& result) const
  {
    m00.Minus(result.m00);
    m01.Minus(result.m01);
    m10.Minus(result.m10);
    m11.Minus(result.m11);
    return result;
  }
  
  
  /// Assignment data-wise operations
  inline SpatialMatrix& operator += (const SpatialMatrix &matrix)
  {
    return SAdd(matrix);
  }

  inline SpatialMatrix& operator -= (const SpatialMatrix &matrix)
  {
    return SSub(matrix);
  }

  inline SpatialMatrix& operator ^= (const SpatialMatrix &matrix)
  {
    return SPMult(matrix);
  }

  inline SpatialMatrix& operator /= (const SpatialMatrix &matrix)
  {
    return SPDiv(matrix);
  }  

  inline SpatialMatrix& operator *= (const SpatialMatrix &matrix)
  {
    return SMult(matrix);
  }

  inline SpatialMatrix& SAdd(const SpatialMatrix &matrix)
  {
    m00.SAdd(matrix.m00);
    m01.SAdd(matrix.m01);
    m10.SAdd(matrix.m10);
    m11.SAdd(matrix.m11);
    return *this;
  }

  inline SpatialMatrix& SSub(const SpatialMatrix &matrix)
  {
    m00.SSub(matrix.m00);
    m01.SSub(matrix.m01);
    m10.SSub(matrix.m10);
    m11.SSub(matrix.m11);
    return *this;
  }

  inline SpatialMatrix& SPMult(const SpatialMatrix &matrix)
  {
    m00.SPMult(matrix.m00);
    m01.SPMult(matrix.m01);
    m10.SPMult(matrix.m10);
    m11.SPMult(matrix.m11);
    return *this;
  }

  inline SpatialMatrix& SPDiv(const SpatialMatrix &matrix)
  {
    m00.SPDiv(matrix.m00);
    m01.SPDiv(matrix.m01);
    m10.SPDiv(matrix.m10);
    m11.SPDiv(matrix.m11);
    return *this;
  }  

  inline SpatialMatrix& SMult(const SpatialMatrix &matrix)
  {
    SpatialMatrix copy(*this);
    copy.Mult(matrix,*this);
    return *this;
  }







  inline SpatialMatrix& operator += (REALTYPE scalar)
  {
    return SAdd(scalar);
  }

  inline SpatialMatrix& operator -= (REALTYPE scalar)
  {
    return SSub(scalar);
  }

  inline SpatialMatrix& operator *= (REALTYPE scalar)
  {
    return SMult(scalar);
  }

  inline SpatialMatrix& operator /= (REALTYPE scalar)
  {
    return SDiv(scalar);
  }

  inline SpatialMatrix& SAdd(REALTYPE scalar)
  {
    m00.SAdd(scalar);
    m01.SAdd(scalar);
    m10.SAdd(scalar);
    m11.SAdd(scalar);
    return *this;
  }

  inline SpatialMatrix& SSub(REALTYPE scalar)
  {
    m00.SSub(scalar);
    m01.SSub(scalar);
    m10.SSub(scalar);
    m11.SSub(scalar);
    return *this;
  }

  inline SpatialMatrix& SMult(REALTYPE scalar)
  {
    m00.SMult(scalar);
    m01.SMult(scalar);
    m10.SMult(scalar);
    m11.SMult(scalar);
    return *this;
  }

  inline SpatialMatrix& SDiv(REALTYPE scalar)
  {
    scalar = R_ONE / scalar;
    m00.SMult(scalar);
    m01.SMult(scalar);
    m10.SMult(scalar);
    m11.SMult(scalar);
    return *this;
  }
  
  
  inline SpatialMatrix operator + (const SpatialMatrix &matrix) const
  {
    SpatialMatrix result;
    return Add(matrix,result);
  }

  inline SpatialMatrix operator - (const SpatialMatrix &matrix) const
  {
    SpatialMatrix result;
    return Sub(matrix,result);
  }

  inline SpatialMatrix operator ^ (const SpatialMatrix &matrix) const
  {
    SpatialMatrix result;
    return PMult(matrix,result);
  }

  inline SpatialMatrix operator / (const SpatialMatrix &matrix) const
  {
    SpatialMatrix result;
    return PDiv(matrix,result);
  }

  inline SpatialMatrix operator * (const SpatialMatrix &matrix) const
  {
    SpatialMatrix result;
    return Mult(matrix,result);
  }

  inline SpatialMatrix Add(const SpatialMatrix &matrix) const
  {
    SpatialMatrix result;
    return Add(matrix,result);
  }

  inline SpatialMatrix Sub(const SpatialMatrix &matrix) const
  {
    SpatialMatrix result;
    return Sub(matrix,result);
  }

  inline SpatialMatrix PMult(const SpatialMatrix &matrix) const
  {
    SpatialMatrix result;
    return PMult(matrix,result);
  }

  inline SpatialMatrix PDiv(const SpatialMatrix &matrix) const
  {
    SpatialMatrix result;
    return PDiv(matrix,result);
  }

  inline SpatialMatrix Mult(const SpatialMatrix &matrix) const
  {
    SpatialMatrix result;
    return Mult(matrix,result);
  }

  inline SpatialMatrix& Add(const SpatialMatrix &matrix, SpatialMatrix & result) const
  {
    m00.Add(matrix.m00,result.m00);
    m01.Add(matrix.m01,result.m01);
    m10.Add(matrix.m10,result.m10);
    m11.Add(matrix.m11,result.m11);
    return result;
  }

  inline SpatialMatrix& Sub(const SpatialMatrix &matrix, SpatialMatrix & result) const
  {
    m00.Sub(matrix.m00,result.m00);
    m01.Sub(matrix.m01,result.m01);
    m10.Sub(matrix.m10,result.m10);
    m11.Sub(matrix.m11,result.m11);
    return result;
  }

  inline SpatialMatrix& PMult(const SpatialMatrix &matrix, SpatialMatrix & result) const
  {
    m00.PMult(matrix.m00,result.m00);
    m01.PMult(matrix.m01,result.m01);
    m10.PMult(matrix.m10,result.m10);
    m11.PMult(matrix.m11,result.m11);
    return result;
  }

  inline SpatialMatrix& PDiv(const SpatialMatrix &matrix, SpatialMatrix & result) const
  {
    m00.PDiv(matrix.m00,result.m00);
    m01.PDiv(matrix.m01,result.m01);
    m10.PDiv(matrix.m10,result.m10);
    m11.PDiv(matrix.m11,result.m11);
    return result;
  }

  inline SpatialMatrix& Mult(const SpatialMatrix &matrix, SpatialMatrix & result) const
  {
    Matrix3 tmp;
    m00.Mult(matrix.m00,result.m00);
    result.m00.SAdd(m01.Mult(matrix.m10,tmp));
    m00.Mult(matrix.m10,result.m01);
    result.m01.SAdd(m01.Mult(matrix.m11,tmp));
    m10.Mult(matrix.m00,result.m10);
    result.m10.SAdd(m11.Mult(matrix.m10,tmp));
    m10.Mult(matrix.m01,result.m11);
    result.m11.SAdd(m11.Mult(matrix.m11,tmp));
    return result;
  }











  inline SpatialMatrix operator + (REALTYPE scalar) const
  {
    SpatialMatrix result;
    return Add(scalar,result);
  }

  inline SpatialMatrix operator - (REALTYPE scalar) const
  {
    SpatialMatrix result;
    return Sub(scalar,result);
  }

  inline SpatialMatrix operator * (REALTYPE scalar) const
  {
    SpatialMatrix result;
    return Mult(scalar,result);
  }

  inline SpatialMatrix operator / (REALTYPE scalar) const
  {
    SpatialMatrix result;
    return Div(scalar,result);
  }

  inline SpatialMatrix Add(REALTYPE scalar) const
  {
    SpatialMatrix result;
    return Add(scalar,result);
  }

  inline SpatialMatrix Sub(REALTYPE scalar) const
  {
    SpatialMatrix result;
    return Sub(scalar,result);
  }

  inline SpatialMatrix Mult(REALTYPE scalar) const
  {
    SpatialMatrix result;
    return Mult(scalar,result);
  }

  inline SpatialMatrix Div(REALTYPE scalar) const
  {
    SpatialMatrix result;
    return Div(scalar,result);
  }


  inline SpatialMatrix& Add(REALTYPE scalar, SpatialMatrix & result) const
  {
    m00.Add(scalar,result.m00);
    m01.Add(scalar,result.m01);
    m10.Add(scalar,result.m10);
    m11.Add(scalar,result.m11);
    return result;
  }

  inline SpatialMatrix& Sub(REALTYPE scalar, SpatialMatrix & result) const
  {
    m00.Sub(scalar,result.m00);
    m01.Sub(scalar,result.m01);
    m10.Sub(scalar,result.m10);
    m11.Sub(scalar,result.m11);
    return result;
  }

  inline SpatialMatrix& Mult(REALTYPE scalar, SpatialMatrix & result) const
  {
    m00.Mult(scalar,result.m00);
    m01.Mult(scalar,result.m01);
    m10.Mult(scalar,result.m10);
    m11.Mult(scalar,result.m11);
    return result;
  }

  inline SpatialMatrix& Div(REALTYPE scalar, SpatialMatrix & result) const
  {
    scalar = R_ONE / scalar;
    m00.Mult(scalar,result.m00);
    m01.Mult(scalar,result.m01);
    m10.Mult(scalar,result.m10);
    m11.Mult(scalar,result.m11);
    return result;
  }



  inline SpatialVector operator * (const SpatialVector &vector) const
  {
    SpatialVector result;
    return Mult(vector,result);
  }

  inline SpatialVector Mult(const SpatialVector &vector) const
  {
    SpatialVector result;
    return Mult(vector,result);
  }

  inline SpatialVector& Mult(const SpatialVector &vector, SpatialVector &result) const
  {
    Vector3 tmp;
    m00.Mult(result.mAngular,result.mAngular);
    result.mAngular.SAdd(m01.Mult(vector.mLinear,tmp));
    m10.Mult(result.mAngular,result.mLinear);
    result.mLinear.SAdd(m11.Mult(vector.mLinear,tmp));
    return result;
  }  


  /// Tests equality of two matrices
  inline bool operator == (const SpatialMatrix& matrix) const
  {
    return ((m00==matrix.m00)&&(m01==matrix.m01)&&(m10==matrix.m10)&&(m11==matrix.m11));
  }

  /// tests inequality of two vectors
  inline bool operator != (const SpatialMatrix& matrix) const
  {
    return !(*this ==  matrix);
  }

  

  /// Self transpose
  inline SpatialMatrix& STranspose()
  {
    SpatialMatrix copy(*this);
    return copy.Transpose(*this);
  }

  /// Returns the transpose
  inline SpatialMatrix Transpose() const
  {
    SpatialMatrix result;
    return Transpose(result);
  }

  /// Returns the transpose in the result
  inline SpatialMatrix& Transpose(SpatialMatrix& result) const
  {
    m00.Transpose(result.m00);
    m01.Transpose(result.m10);
    m10.Transpose(result.m01);
    m11.Transpose(result.m11);
    return result;
  }


};


#endif
