#ifndef SPATIAL_VECTOR_H
#define SPATIAL_VECTOR_H

#include <math.h>
#include "Vector3.h"


/**
 * \class SpatialVector
 * 
 * \ingroup MathLib
 * 
 * \brief An implementation of the Spatial Vector notation
 */
class SpatialVector
{
  //  friend class Matrix3;
public:
  Vector3 mAngular;
  Vector3 mLinear;
  
public:
  /// Empty constructor
  inline SpatialVector(){}
  /// Copy constructor
  inline SpatialVector(const SpatialVector &vector){
    Set(vector);
  }
  /// Copy constructor
  inline SpatialVector(const Vector3 &angular, const Vector3 &linear){
    SetAngularComponent(angular);
    SetLinearComponent(linear);
  }
  /// Data-based constructor
  inline SpatialVector(REALTYPE wx, REALTYPE wy, REALTYPE wz, REALTYPE x, REALTYPE y, REALTYPE z){
    SetAngularComponent(wx,wy,wz);
    SetLinearComponent(x,y,z);
  }   
  inline ~SpatialVector(){};
 
 
  inline SpatialVector& Zero(){
    mAngular.Zero();
    mLinear.Zero();
    return *this;
  }
 

  /// Access to the first element (can be modified)
  inline REALTYPE& wx() {return mAngular.x();}
  /// Access to the second element (can be modified)
  inline REALTYPE& wy() {return mAngular.y();}
  /// Access to the third element (can be modified)
  inline REALTYPE& wz() {return mAngular.z();} 
  /// Access to the fourth element (can be modified)
  inline REALTYPE& x() {return mLinear.x();}
  /// Access to the fifth element (can be modified)
  inline REALTYPE& y() {return mLinear.y();}
  /// Access to the sixth element (can be modified)
  inline REALTYPE& z() {return mLinear.z();} 

  /// Access to the first element (constant)
  inline REALTYPE cwx() const {return mAngular.cx();}
  /// Access to the second element (constant)
  inline REALTYPE cwy() const {return mAngular.cy();}
  /// Access to the thrid element (constant)
  inline REALTYPE cwz() const {return mAngular.cz();} 
  /// Access to the fourth element (constant)
  inline REALTYPE cx() const {return mLinear.cx();}
  /// Access to the fifth element (constant)
  inline REALTYPE cy() const {return mLinear.cy();}
  /// Access to the sixth element (constant)
  inline REALTYPE cz() const {return mLinear.cz();} 


  inline SpatialVector& Set(const SpatialVector &vector){
    mAngular.Set(vector.mAngular);
    mLinear.Set(vector.mLinear);
    return *this;
  }


  inline Vector3& GetLinearComponent()
  {
    return mLinear;
  }

  inline Vector3& GetAngularComponent()
  {
    return mAngular;
  }

  inline Vector3& GetLinearComponent(Vector3 & result) const
  {
    result.Set(mLinear);
    return result;
  }

  inline Vector3& GetAngularComponent(Vector3 & result) const
  {
    result.Set(mAngular);
    return result;
  }

  inline SpatialVector& SetLinearComponent(const Vector3 & vector)
  {
    mLinear.Set(vector);
    return *this;
  }

  inline SpatialVector& SetAngularComponent(const Vector3 & vector)
  {
    mAngular.Set(vector);
    return *this;
  }

  inline SpatialVector& SetLinearComponent(REALTYPE x, REALTYPE y, REALTYPE z)
  {
    mLinear.Set(x,y,z);
    return *this;
  }

  inline SpatialVector& SetAngularComponent(REALTYPE wx, REALTYPE wy, REALTYPE wz)
  {
    mAngular.Set(wx,wy,wz);
    return *this;
  }

  
  /// Assigment operator
  inline SpatialVector& operator = (const SpatialVector &vector)
  {        
    return Set(vector);    
  }
     
  /// Inverse operator
  inline SpatialVector operator - () const
  {
    SpatialVector result;
    return Minus(result);
  }

  /// Inverse operator
  inline SpatialVector& Minus(SpatialVector& result) const
  {
    mAngular.Minus(result.mAngular);
    mLinear.Minus(result.mLinear);
    return result;
  }


  /// Assigment data-wise operations 
  inline SpatialVector& operator += (const SpatialVector &vector)
  {
    return SAdd(vector);
  }

  inline SpatialVector& operator -= (const SpatialVector &vector)
  {
    return SSub(vector);
  }

  inline SpatialVector& operator *= (const SpatialVector &vector)
  {
    return SMult(vector);
  }

  inline SpatialVector& operator /= (const SpatialVector &vector)
  {
    return SDiv(vector);
  }

  /// Assigment data-wise operations 
  inline SpatialVector& SAdd(const SpatialVector &vector)
  {
    mAngular.SAdd(vector.mAngular);
    mLinear.SAdd(vector.mLinear);
    return *this;
  }

  inline SpatialVector& SSub(const SpatialVector &vector)
  {
    mAngular.SSub(vector.mAngular);
    mLinear.SSub(vector.mLinear);
    return *this;
  }

  inline SpatialVector& SMult(const SpatialVector &vector)
  {
    mAngular.SMult(vector.mAngular);
    mLinear.SMult(vector.mLinear);
    return *this;
  }

  inline SpatialVector& SDiv(const SpatialVector &vector)
  {
    mAngular.SDiv(vector.mAngular);
    mLinear.SDiv(vector.mLinear);
    return *this;
  }
  
  /// Assigment operations
  inline SpatialVector& operator += (REALTYPE scalar)
  {
    return SAdd(scalar);
  }

  inline SpatialVector& operator -= (REALTYPE scalar)
  {
    return SSub(scalar);
  }

  inline SpatialVector& operator *= (REALTYPE scalar)
  {
    return SMult(scalar);
  }

  inline SpatialVector& operator /= (REALTYPE scalar)
  {
    return SDiv(scalar);
  }

  /// Assigment and operations
  inline SpatialVector& SAdd(REALTYPE scalar)
  {
    mAngular.SAdd(scalar);
    mLinear.SAdd(scalar);
    return *this;
  }

  inline SpatialVector& SSub(REALTYPE scalar)
  {
    mAngular.SSub(scalar);
    mLinear.SSub(scalar);
    return *this;
  }

  inline SpatialVector& SMult(REALTYPE scalar)
  {
    mAngular.SMult(scalar);
    mLinear.SMult(scalar);
    return *this;
  }

  inline SpatialVector& SDiv(REALTYPE scalar)
  {
    scalar = R_ONE / scalar;
    mAngular.SMult(scalar);
    mLinear.SMult(scalar);
    return *this;
  }

  /// Vector data-wise operators
  inline SpatialVector operator + (const SpatialVector &vector) const
  {
    SpatialVector result;
    return Add(vector,result);
  }

  inline SpatialVector operator - (const SpatialVector &vector) const
  {
    SpatialVector result;
    return Sub(vector,result); 
  }

  inline SpatialVector operator ^ (const SpatialVector &vector) const
  {
    SpatialVector result;
    return Mult(vector,result);
  }

  inline SpatialVector operator / (const SpatialVector &vector) const
  {
    SpatialVector result;
    return Div(vector,result); 
  }

  inline SpatialVector Add(const SpatialVector &vector) const
  {
    SpatialVector result;
    return Add(vector,result);
  }

  inline SpatialVector Sub(const SpatialVector &vector) const
  {
    SpatialVector result;
    return Sub(vector,result); 
  }

  inline SpatialVector Mult(const SpatialVector &vector) const
  {
    SpatialVector result;
    return Mult(vector,result);
  }

  inline SpatialVector Div(const SpatialVector &vector) const
  {
    SpatialVector result;
    return Div(vector,result); 
  }
  
  /// Vector data-wise operations (faster than using operators)
  inline SpatialVector& Add(const SpatialVector &vector, SpatialVector &result) const
  {
    mAngular.Add(vector.mAngular,result.mAngular);
    mLinear.Add(vector.mLinear,result.mLinear);
    return result;
  }

  inline SpatialVector& Sub(const SpatialVector &vector, SpatialVector &result) const
  {
    mAngular.Sub(vector.mAngular,result.mAngular);
    mLinear.Sub(vector.mLinear,result.mLinear);
    return result; 
  }

  inline SpatialVector& Mult(const SpatialVector &vector, SpatialVector &result) const
  {
    mAngular.Mult(vector.mAngular,result.mAngular);
    mLinear.Mult(vector.mLinear,result.mLinear);
    return result;
  }

  inline SpatialVector& Div(const SpatialVector &vector, SpatialVector &result) const
  {
    mAngular.Div(vector.mAngular,result.mAngular);
    mLinear.Div(vector.mLinear,result.mLinear);
    return result; 
  }
  
  /// Scalar operations using operators
  inline SpatialVector operator + (REALTYPE scalar) const
  {
    SpatialVector result;
    return Add(scalar,result);
  }

  inline SpatialVector operator - (REALTYPE scalar) const
  {
    SpatialVector result;
    return Sub(scalar,result);
  }

  inline SpatialVector operator * (REALTYPE scalar) const
  {
    SpatialVector result;
    return Mult(scalar,result);
  }

  inline SpatialVector operator / (REALTYPE scalar) const
  {
    SpatialVector result;
    return Div(scalar,result);
  }

  inline SpatialVector Add(REALTYPE scalar) const
  {
    SpatialVector result;
    return Add(scalar,result);
  }

  inline SpatialVector Sub(REALTYPE scalar) const
  {
    SpatialVector result;
    return Sub(scalar,result);
  }

  inline SpatialVector Mult(REALTYPE scalar) const
  {
    SpatialVector result;
    return Mult(scalar,result);
  }

  inline SpatialVector Div(REALTYPE scalar) const
  {
    SpatialVector result;
    return Div(scalar,result);
  }

  /// Scalar operations with result as a parameter (faster than pure operators)
  inline SpatialVector& Add(REALTYPE scalar, SpatialVector& result) const
  {
    mAngular.Add(scalar,result.mAngular);
    mLinear.Add(scalar,result.mLinear);
    return result;
  }

  inline SpatialVector& Sub(REALTYPE scalar, SpatialVector& result) const
  {
    mAngular.Sub(scalar,result.mAngular);
    mLinear.Sub(scalar,result.mLinear);
    return result;
  }

  inline SpatialVector& Mult(REALTYPE scalar, SpatialVector& result) const
  {
    mAngular.Mult(scalar,result.mAngular);
    mLinear.Mult(scalar,result.mLinear);
    return result;
  }

  inline SpatialVector& Div(REALTYPE scalar, SpatialVector& result) const
  {
    scalar = R_ONE/scalar;
    mAngular.Mult(scalar,result.mAngular);
    mLinear.Mult(scalar,result.mLinear);
    return result;
  }

  /// Tests equality of two vectors
  inline bool operator == (const SpatialVector& vector) const
  {
    return ((mLinear==vector.mLinear)&&(mAngular==vector.mAngular));
  }

  /// tests inequality of two vectors
  inline bool operator != (const SpatialVector& vector) const
  {
    return !(*this ==  vector);
  }
  
    /// Performs the dot product
  inline REALTYPE Dot(const SpatialVector &vector) const
  {
    REALTYPE result = mAngular.Dot(vector.mAngular);
    result += mLinear.Dot(vector.mLinear);
    return result;     
  }

  /// Truncs the data between minVal and maxVal
  inline SpatialVector Trunc(const REALTYPE minVal, const REALTYPE maxVal) 
  {
    mAngular.Trunc(minVal,maxVal);
    mLinear.Trunc(minVal,maxVal);
    return *this;     
  }

  /// Truncs each data between each minVal and maxVal vectors
  inline SpatialVector Trunc(const SpatialVector &minVal, const SpatialVector &maxVal) 
  {
    mAngular.Trunc(minVal.mAngular,maxVal.mAngular);
    mLinear.Trunc(minVal.mLinear,maxVal.mLinear);
    return *this;     
  }
  
  inline TVector<6>& ToTVector6(TVector<6> & result){

    for(int i=0;i<3;i++){
      result._[i]   = mAngular._[i];
      result._[i+3] = mLinear._[i];
    }
    return result;
  }

  
  
  /// Prints out the vector to stdout 
  void Print() const
  {
    std::cout << "SpatialVector" <<std::endl;;
    for (unsigned int i = 0; i < 3; i++)
      std::cout << mAngular._[i] <<" ";
    for (unsigned int i = 0; i < 3; i++)
      std::cout << mLinear._[i] <<" ";
    std::cout << std::endl;    
  }
  
};


#endif
