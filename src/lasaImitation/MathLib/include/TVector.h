#ifndef __TVECTOR_H__
#define __TVECTOR_H__

#include "MathLibCommon.h"
#define  USE_T_EXTENSIONS
#include <math.h>
#include <iostream>

#include "Vector.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

template<unsigned int ROW> class TMatrix;

/**
 * \class TVector
 * 
 * \ingroup MathLib
 * 
 * \brief The basic template vector class. See class Vector for function definition
 * 
 * This template vector class can be used for doing various vector manipulation.
 * This should be combined with the Matrix class for doing almost anything
 * you ever dreamt of.
 */
template<unsigned int ROW> class TVector
{
public:
  /*
  friend class Vector;
  friend class Vector3;
  friend class TMatrix<ROW>;
  friend class Matrix3;
  friend class Matrix4;
  */

public:
  static const TVector<ROW>   ZERO;
  static const unsigned int   row   = ROW;
  static       REALTYPE          undef;
   

public:
  REALTYPE _[ROW];

  /// The empty constructor
  inline TVector()
  {
    Zero();
  }

  /// Copy constructor
  inline TVector(const TVector<ROW> &vector)
  {
    Set(vector);
  }
  
  /// Constructor with data pointer to be copied
  inline TVector(const REALTYPE *_)
  {
    Set(_);
  }

  /*
  /// A copy constructor of a Vector object 
  inline TVector(const Vector & vector)
  {
    Set(vector);
  }
  */
  
  
  
  /// Sets data using data pointer
  inline TVector<ROW>& Set(const REALTYPE *_)
  {
    for (unsigned int i = 0; i < ROW; i++)
      this->_[i] = _[i];
    return *this;
  }

  /// Makes a copy of the TVector argument
  inline TVector<ROW>& Set(const TVector<ROW> &vector)
  {        
    for (unsigned int i = 0; i < ROW; i++)
      _[i] = vector._[i];
    return *this;    
  }   
  
  /// Makes a copy of the Vector argument
  inline TVector<ROW>& Set(const Vector &vector)
  {        
    const unsigned int k = (ROW<=vector.row?ROW:vector.row);    
    for (unsigned int i = 0; i < k; i++)
      _[i] = vector._[i];
    for (unsigned int i = k; i < ROW; i++)
      _[i] = R_ZERO;
    return *this;
  }   

  /// Gets the data array
  inline REALTYPE *GetArray(){
    return _;
  }  
  
  /// Sets all values to zero
  inline void Zero()
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] = 0;    
  }
  
  /// Gets a reference to the row element
  inline REALTYPE& operator[] (const unsigned int row)
  {
    if(row<ROW)
      return _[row];
    return undef; 
  }

  /// Gets a reference to the row element
  inline REALTYPE& operator() (const unsigned int row)
  {
    if(row<ROW)
      return _[row];
    return undef; 
  }

  /// Gets the value of the row element
  inline REALTYPE Get(const unsigned int row) const 
  {
    if(row<ROW)
      return _[row];
    return undef; 
  }
  
  /// Assigment operator
  inline TVector<ROW>& operator = (const TVector<ROW> &vector)
  {        
    return Set(vector);    
  }
     
  /// Inverse operator
  inline TVector<ROW> operator - () const
  {
    TVector<ROW> result;
    return Minus(result);
  }

  /// Inverse operator
  inline TVector<ROW>& Minus(TVector<ROW>& result) const
  {
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = -_[i];
    return result;
  }

  /// Self Inversion
  inline TVector<ROW>& SMinus() 
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] = -_[i];
    return *this;
  }


  /// Assigment data-wise operations 
  inline TVector<ROW>& operator += (const TVector<ROW> &vector)
  {
    return SAdd(vector);
  }

  inline TVector<ROW>& operator -= (const TVector<ROW> &vector)
  {
    return SSub(vector);
  }

  inline TVector<ROW>& operator *= (const TVector<ROW> &vector)
  {
    return SMult(vector);
  }

  inline TVector<ROW>& operator /= (const TVector<ROW> &vector)
  {
    return SDiv(vector);
  }
 
  /// Assigment data-wise operations 
  inline TVector<ROW>& SAdd(const TVector<ROW> &vector)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] += vector._[i];
    return *this;
  }

  inline TVector<ROW>& SSub(const TVector<ROW> &vector)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] -= vector._[i];
    return *this;
  }

  inline TVector<ROW>& SMult(const TVector<ROW> &vector)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] *= vector._[i];
    return *this;
  }

  inline TVector<ROW>& SDiv(const TVector<ROW> &vector)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] /= vector._[i];
    return *this;
  }

  /// Assigment operations
  inline TVector<ROW>& operator += (REALTYPE scalar)
  {
    return SAdd(scalar);
  }

  inline TVector<ROW>& operator -= (REALTYPE scalar)
  {
    return SSub(scalar);
  }

  inline TVector<ROW>& operator *= (REALTYPE scalar)
  {
    return SMult(scalar);
  }

  inline TVector<ROW>& operator /= (REALTYPE scalar)
  {
    return SDiv(scalar);
  }

  /// Assigment and operations
  inline TVector<ROW>& SAdd(REALTYPE scalar)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] += scalar;
    return *this;
  }

  inline TVector<ROW>& SSub(REALTYPE scalar)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] -= scalar;
    return *this;
  }

  inline TVector<ROW>& SMult(REALTYPE scalar)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] *= scalar;
    return *this;
  }

  inline TVector<ROW>& SDiv(REALTYPE scalar)
  {
    scalar = R_ONE / scalar;
    for (unsigned int i = 0; i < ROW; i++)
      _[i] *= scalar;
    return *this;
  }



  /// Vector data-wise operators
  inline TVector<ROW> operator + (const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return Add(vector,result);
  }

  inline TVector<ROW> operator - (const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return Sub(vector,result); 
  }

  inline TVector<ROW> operator ^ (const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return Mult(vector,result);
  }

  inline TVector<ROW> operator / (const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return Div(vector,result); 
  }

  inline TVector<ROW> Add(const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return Add(vector,result);
  }

  inline TVector<ROW> Sub(const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return Sub(vector,result); 
  }

  inline TVector<ROW> Mult(const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return Mult(vector,result);
  }

  inline TVector<ROW> Div(const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return Div(vector,result); 
  }

  /// Vector data-wise operations (faster than using operators)
  inline TVector<ROW>& Add(const TVector<ROW> &vector, TVector<ROW> &result) const
  {
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] + vector._[i];
    return result;
  }

  inline TVector<ROW>& Sub(const TVector<ROW> &vector, TVector<ROW> &result) const
  {
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] - vector._[i];
    return result; 
  }

  inline TVector<ROW>& Mult(const TVector<ROW> &vector, TVector<ROW> &result) const
  {
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] * vector._[i];
    return result;
  }

  inline TVector<ROW>& Div(const TVector<ROW> &vector, TVector<ROW> &result) const
  {
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] / vector._[i];
    return result; 
  }


  /// Scalar operations using operators
  inline TVector<ROW> operator + (REALTYPE scalar) const
  {
    TVector<ROW> result;
    return Add(scalar,result);
  }

  inline TVector<ROW> operator - (REALTYPE scalar) const
  {
    TVector<ROW> result;
    return Sub(scalar,result);
  }

  inline TVector<ROW> operator * (REALTYPE scalar) const
  {
    TVector<ROW> result;
    return Mult(scalar,result);
  }

  inline TVector<ROW> operator / (REALTYPE scalar) const
  {
    TVector<ROW> result;
    return Div(scalar,result);
  }

  inline TVector<ROW> Add(REALTYPE scalar) const
  {
    TVector<ROW> result;
    return Add(scalar,result);
  }

  inline TVector<ROW> Sub(REALTYPE scalar) const
  {
    TVector<ROW> result;
    return Sub(scalar,result);
  }

  inline TVector<ROW> Mult(REALTYPE scalar) const
  {
    TVector<ROW> result;
    return Mult(scalar,result);
  }

  inline TVector<ROW> Div(REALTYPE scalar) const
  {
    TVector<ROW> result;
    return Div(scalar,result);
  }

  /// Scalar operations with result as a parameter (faster than pure operators)
  inline TVector<ROW>& Add(REALTYPE scalar, TVector<ROW>& result) const
  {
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] + scalar;
    return result;
  }

  inline TVector<ROW>& Sub(REALTYPE scalar, TVector<ROW>& result) const
  {
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] - scalar;
    return result;
  }

  inline TVector<ROW>& Mult(REALTYPE scalar, TVector<ROW>& result) const
  {
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] * scalar;
    return result;
  }

  inline TVector<ROW>& Div(REALTYPE scalar, TVector<ROW>& result) const
  {
    scalar = R_ONE/scalar;
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] * scalar;
    return result;
  }

  inline TMatrix<ROW> & Mult(TVector<ROW> & vector, TMatrix<ROW> & result){
    for (unsigned int j = 0; j < ROW; j++){
      for (unsigned int i = 0; i < ROW; i++){
        result._[i][j] = _[i] * vector._[j];
      }      
    }      
    return result;
  }


  /// Tests equality of two vectors
  inline bool operator == (const TVector<ROW>& vector) const
  {
    for (unsigned int i = 0; i < ROW; i++)
      if(_[i] != vector._[i]) return false;
    return true;
  }

  /// tests inequality of two vectors
  inline bool operator != (const TVector<ROW>& vector) const
  {
    return !(*this ==  vector);
  }

  /// Returns the norm
  inline REALTYPE Norm() const 
  {
    return sqrtf(Norm2());
  }

  /// Returns the squared norm
  inline REALTYPE Norm2() const 
  {
    REALTYPE result = R_ZERO;
    for (unsigned int i = 0; i < ROW; i++)
      result += _[i]*_[i];
    return result;
  }

  /// Normalize the data to 1
  inline void Normalize()
  {
    REALTYPE scalar = R_ONE / Norm();
    (*this)*=scalar;
  }
  
  /// Performs the dot product
  inline REALTYPE Dot(const TVector<ROW> &vector) const
  {
    REALTYPE result = R_ZERO;
    for (unsigned int i = 0; i < ROW; i++)
      result += _[i]*vector._[i];
    return result;     
  }

  /// Truncs the data between minVal and maxVal
  inline TVector<ROW> Trunc(const REALTYPE minVal, const REALTYPE maxVal) 
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] = TRUNC(_[i],minVal,maxVal);
    return *this;     
  }

  /// Truncs each data between each minVal and maxVal vectors
  inline TVector<ROW> Trunc(const TVector<ROW> &minVal, const TVector<ROW> &maxVal) 
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] = TRUNC(_[i],minVal._[i],maxVal._[i]);
    return *this;     
  }
  
  /// Prints out the vector to stdout 
  void Print() const
  {
    std::cout << "Vector" <<ROW<<std::endl;;
    for (unsigned int i = 0; i < ROW; i++)
      std::cout << _[i] <<" ";
    std::cout << std::endl;    
  }
};

template<unsigned int ROW> const TVector<ROW> TVector<ROW>::ZERO;

template<unsigned int ROW> REALTYPE TVector<ROW>::undef = R_ZERO;


#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif
