#ifndef VECTOR_H
#define VECTOR_H

#include "MathLibCommon.h"


#include "Macros.h"

#include <math.h>
#include <iostream>
#include <vector>
using namespace std;

#ifndef NULL
#define NULL 0
#endif

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

#ifdef  USE_T_EXTENSIONS
template<unsigned int ROW> class TVector;
#endif

typedef vector<unsigned int> IndicesVector;

/** 
 * \defgroup MathLib MathLib
 * 
 * \brief A mathematical library for doing various things such as 
 * matrix manipulation 
 */


/**
 * \class Vector
 * 
 * \ingroup MathLib
 * 
 * \brief The basic vector class
 * 
 * This vector class can be used for doing various vector manipulation.
 * This should be combined with the Matrix class for doing almost anything
 * you ever dreamt of.
 */

class Matrix;

class Vector
{
  friend class Matrix;
#ifdef  USE_T_EXTENSIONS  
  template<unsigned int ROW> friend class TVector;  
#endif
  
protected:
  static  REALTYPE undef;           ///< The default value returned when a vector out-of-bound is reached
   
          unsigned int   row;       ///< The size of the vector
	        REALTYPE         *_;      ///< The data array. 

public:

  /// Create an empty vector
	inline Vector() {
    row = 0;
    _   = NULL;
  }
  
  /// The destructor
  inline virtual ~Vector(){
    Release(); 
  }

  /// The copy constructor
  inline Vector(const Vector &vector)
  {
    row = 0;
    _   = NULL;
    Resize(vector.row,false);
    for (unsigned int i = 0; i < row; i++)
      _[i] = vector._[i];
  }

  /**
   * \brief Create a sized vector
   * \param size  The size
   * \param clear Tell if the vector sould be set to 0
   */  
  inline Vector(unsigned int size, bool clear = true)
  {
    row = 0;
    _   = NULL;
    Resize(size,false);
    if(clear)
      Zero();
  }
  
  /**
   * \brief Create a vector by copying a memory area
   * \param _     The array of REALTYPE to be copied
   * \param size  The size
   */  
	inline Vector(const REALTYPE _[], unsigned int size)
	{
    row       = 0;
    this->_   = NULL;
    Resize(size,false);
		for (unsigned int i = 0; i < row; i++)
			this->_[i] = _[i];
	}
   
#ifdef  USE_T_EXTENSIONS
  /// Copy Contructor of a template Vector (see TVector)  
  template<unsigned int ROW> inline Vector(const TVector<ROW> &vector)
  {
    row = 0;
    _   = NULL;
    Resize(ROW,false);
    for (unsigned int i = 0; i < row; i++)
      _[i] = vector._[i];        
  }
#endif     
  
  /// An assignment function
  inline Vector& Set(const Vector &vector){
    return (*this)=vector;  
  }

  /// Set all values to 0
  inline Vector& Zero()
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] = R_ZERO;
    return *this;    
  }

  /// Set all values to 1
  inline Vector& One()
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] = R_ONE;
    return *this;    
  }

  /// Set random values uniformly distributed between 0 and 1
  inline Vector& Random(){
    for (unsigned int j = 0; j < row; j++)
      _[j] = RND(R_ONE);   
    return *this;    
  }
    
  /// Get the size of the vector
  inline unsigned int Size() const{
    return row;
  }
  
  /// Get the data pointer
  inline REALTYPE *GetArray() const{
    return _;
  }

  /**
   * \brief Get a vector value
   * \param row   The row number (starting from 0)
   * \return The corresponding value
   */  
  inline REALTYPE At(const unsigned int row) const
  {
    if(row<this->row)
      return _[row];
    return undef; 
  }
  
  /**
   * \brief Get the reference to a vector value
   * \param row   The row number (starting from 0)
   * \return The corresponding value
   */  
  inline REALTYPE& operator[] (const unsigned int row)
  {
    if(row<this->row)
      return _[row];
    return undef; 
  }
  
  /**
   * \brief Get the reference to a vector value
   * \param row   The row number (starting from 0)
   * \return The corresponding value
   */  
  inline REALTYPE& operator() (const unsigned int row)
  {
    if(row<this->row)
      return _[row];
    return undef; 
  }
  
  
  /**
   * \brief The - operator
   * \return A new vector 
   */  
	inline Vector operator - () const
	{
		Vector result(row,false);
		for (unsigned int i = 0; i < row; i++)
			result._[i] = -_[i];
    return result;
	}
    
  inline Vector& operator = (const Vector &vector)
  {
    Resize(vector.row,false);
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      _[i] = vector._[i];
    for (unsigned int i = k; i < row; i++)
      _[i] = 0;
    return *this;    
  }
  
	inline Vector& operator += (const Vector &vector)
	{
    const unsigned int k = (row<=vector.row?row:vector.row);
		for (unsigned int i = 0; i < k; i++)
			_[i] += vector._[i];
    return *this;
	}

	inline Vector& operator -= (const Vector &vector)
	{
    const unsigned int k = (row<=vector.row?row:vector.row);
		for (unsigned int i = 0; i < k; i++)
			_[i] -= vector._[i];
    return *this;
	}
  
  /// Element-wise multiplication
  inline Vector& operator ^= (const Vector &vector)
  {
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      _[i] *= vector._[i];
    return *this;
  }

  /// Element-wise division
  inline Vector& operator /= (const Vector &vector)
  {
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      _[i] /= vector._[i];
    return *this;
  }

  inline Vector& operator += (REALTYPE scalar)
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] += scalar;
    return *this;
  }

  inline Vector& operator -= (REALTYPE scalar)
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] -= scalar;
    return *this;
  }

	inline Vector& operator *= (REALTYPE scalar)
	{
		for (unsigned int i = 0; i < row; i++)
			_[i] *= scalar;
    return *this;
	}

	inline Vector& operator /= (REALTYPE scalar)
	{
		scalar = R_ONE / scalar;
		for (unsigned int i = 0; i < row; i++)
			_[i] *= scalar;
    return *this;
	}

  inline Vector operator + (const Vector &vector) const
  {
    Vector result(row,false);
    return Add(vector,result);    
  }
  

  inline Vector operator - (const Vector &vector) const
  {
    Vector result(row,false);
    return Sub(vector,result);    
  }

  /// Element-wise multiplication  
  inline Vector operator ^ (const Vector &vector) const
  {
    Vector result(row,false);
    return PMult(vector,result);    
  }

  /// Element-wise division
  inline Vector operator / (const Vector &vector) const
  {
    Vector result(row,false);
    return PDiv(vector,result);    
  }

  inline REALTYPE operator * (const Vector &vector) const
  { 
    return this->Dot(vector);  
  }
  
  inline Vector operator + (REALTYPE scalar) const
  {
    Vector result(row,false);
    return Add(scalar,result);    
  }

  inline Vector operator - (REALTYPE scalar) const
  {
    Vector result(row,false);
    return Sub(scalar,result);    
  }

  inline Vector operator * (REALTYPE scalar) const
  {
    Vector result(row,false);
    return Mult(scalar,result);    
  }

  inline Vector operator / (REALTYPE scalar) const
  {
    Vector result(row,false);
    return Div(scalar,result);    
  }
  
  inline bool operator == (const Vector& vector) const
  {
    if(row!=vector.row) return false;
    for (unsigned int i = 0; i < row; i++)
      if(_[i] != vector._[i]) return false;
    return true;
  }

  inline bool operator != (const Vector& vector) const
  {
    return !(*this ==  vector);
  }

  /**
   * \brief Sum two vector in a faster way than using the + operator
   * \param vector The second vector to be summed up
   * \param result The result
   * \return The result vector 
   */  
  inline Vector& Add(const Vector &vector, Vector& result) const
  {   
    result.Resize(row,false);
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      result._[i] = _[i] + vector._[i];
    for (unsigned int i = k; i < row; i++)
      result._[i] = _[i];      
    return result;
  }

  /**
   * \brief Substract two vector in a faster way than using the + operator
   * \param vector The substracting vector
   * \param result The result
   * \return The result vector 
   */  
  inline Vector& Sub(const Vector &vector, Vector& result) const
  {
    result.Resize(row,false);
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      result._[i] = _[i] - vector._[i];
    for (unsigned int i = k; i < row; i++)
      result._[i] = _[i];      
    return result; 
  }
  
  /// Element-wise multiplication
  inline Vector& PMult(const Vector &vector, Vector& result) const
  {   
    result.Resize(row,false);
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      result._[i] = _[i] * vector._[i];
    for (unsigned int i = k; i < row; i++)
      result._[i] = _[i];      
    return result;
  }

  /// Element-wise division
  inline Vector& PDiv(const Vector &vector, Vector& result) const
  {
    result.Resize(row,false);
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      result._[i] = _[i] / vector._[i];
    for (unsigned int i = k; i < row; i++)
      result._[i] = _[i];      
    return result; 
  }

  /// Scalar summation
  inline Vector& Add(REALTYPE scalar, Vector& result) const
  {
    result.Resize(row,false);
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] + scalar;
    return result;
  }

  
  /// Scalar substraction
  inline Vector& Sub(REALTYPE scalar, Vector& result) const
  {
    result.Resize(row,false);
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] - scalar;
    return result;
  }

  
  /// Scalar multiplication
  inline Vector& Mult(REALTYPE scalar, Vector& result) const
  {
    result.Resize(row,false);
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] * scalar;
    return result;
  }

  /// Scalar division
  inline Vector& Div(REALTYPE scalar, Vector& result) const
  {
    result.Resize(row,false);
    scalar = R_ONE/scalar;
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] * scalar;
    return result;
  }


  Matrix& MultTranspose(const Vector & vec, Matrix& result);

  /// Sum up all elements of the vector
  inline REALTYPE Sum() const 
  {
    REALTYPE result = R_ZERO;
    for (unsigned int i = 0; i < row; i++)
      result += _[i];
    return result;
  }

  /// The norm of the vector
  inline REALTYPE Norm() const 
  {
    #ifdef MATHLIB_USE_DOUBLE_AS_REAL    
      return sqrt(Norm2());
    #else
      return sqrtf(Norm2());
    #endif    
  }

  /// The squared norm of the vector
  inline REALTYPE Norm2() const 
  {
    REALTYPE result = R_ZERO;
    for (unsigned int i = 0; i < row; i++)
      result += _[i]*_[i];
    return result;
  }

  /// Normalize the vector to 1
	inline void Normalize()
	{
		REALTYPE scalar = R_ONE / Norm();
    (*this)*=scalar;
	}
  
  /// The distance between two vectors  
  inline REALTYPE Distance(const Vector &vector) const
  {
    return (*this-vector).Norm();
  }
  
  /// The squared distance between two vectors  
  inline REALTYPE Distance2(const Vector &vector) const
  {
    return (*this-vector).Norm2();  
  }

  /// The dot product with another vector    
  inline REALTYPE Dot(const Vector &vector) const
  {
    const unsigned int k = (row<=vector.row?row:vector.row);
    REALTYPE result = R_ZERO;
    for (unsigned int i = 0; i < k; i++)
      result += _[i]*vector._[i];
    return result;     
  }

  /// The maximum value of the vector    
  inline REALTYPE Max(){
    if(row==0)
      return R_ZERO;
      
    REALTYPE res=_[0];
    for(unsigned int i=1;i<row;i++){
      if(_[i]>res) res = _[i];  
    }
    return res;
  }

  /// The minimum value of the vector    
  inline REALTYPE Min(){
    if(row==0)
      return R_ZERO;
      
    REALTYPE res=_[0];
    for(unsigned int i=1;i<row;i++){
      if(_[i]<res) res = _[i];  
    }
    return res;
  }

  /// The index of the maximum value of the vector    
  inline int MaxId(){
    if(row==0)
      return -1;
      
    REALTYPE mx  = _[0];
    int   res = 0;
    for(unsigned int i=1;i<row;i++){
      if(_[i]>mx){ mx = _[i]; res = i;}  
    }
    return res;
  }

  /// The index of the minimum value of the vector    
  inline int MinId(){
    if(row==0)
      return -1;
      
    REALTYPE mx  = _[0];
    int   res = 0;
    for(unsigned int i=1;i<row;i++){
      if(_[i]<mx){ mx = _[i]; res = i;}  
    }
    return res;
  }

  /// The maximum value of two vector    
  inline Vector& Max(const Vector& v0,const Vector& v1){
    const unsigned int kmin = (v0.row<v1.row?v0.row:v1.row);      
    const unsigned int kmax = (v0.row<v1.row?v1.row:v0.row);
    Resize(kmax,false);
    for(unsigned int i=0;i<kmin;i++){
      _[i] = (v0._[i]>v1._[i]?v0._[i]:v1._[i]);  
    }
    if(kmin<kmax){
      if(v0.row<v1.row){
        for(unsigned int i=kmin;i<kmax;i++){
          _[i] = v1._[i];  
        }        
      }else{
        for(unsigned int i=kmin;i<kmax;i++){
          _[i] = v0._[i];          
        }
      }
    }
    return *this;
  }

  /// The minimum value of two vector    
  inline Vector& Min(const Vector& v0,const Vector& v1){
    const unsigned int kmin = (v0.row<v1.row?v0.row:v1.row);      
    const unsigned int kmax = (v0.row<v1.row?v1.row:v0.row);
    Resize(kmax,false);
    for(unsigned int i=0;i<kmin;i++){
      _[i] = (v0._[i]<v1._[i]?v0._[i]:v1._[i]);  
    }
    if(kmin<kmax){
      if(v0.row<v1.row){
        for(unsigned int i=kmin;i<kmax;i++){
          _[i] = v1._[i];  
        }        
      }else{
        for(unsigned int i=kmin;i<kmax;i++){
          _[i] = v0._[i];          
        }
      }
    }
    return *this;
  }

  /// Return the absolute value of the vector
  inline Vector Abs(){
    Vector result(row);
    return Abs(result);
  }

  /// Set the result to the absolute value of the vector
  inline Vector& Abs(Vector &result) const{
    result.Resize(row,false);
    for(unsigned int i=0;i<row;i++){
      result._[i] = fabs(_[i]);
    }
    return result;
  }
  
  inline Vector& Sort(IndicesVector * indices=NULL){
    if(indices){
        indices->resize(row);
        for(int i=0;i<row;i++) indices->at(i)=i;    
    }
    REALTYPE cmax;
    int maxId;
    for(int i=0;i<row-1;i++){
      cmax  = _[i];
      maxId = i;
      for(int j=i+1;j<row;j++){
        if(cmax<_[j]){
          cmax = _[j];
          maxId = j;  
        }            
      }
      if(maxId!=i){
        REALTYPE tmp    = _[i];
        _[i]            = _[maxId];
        _[maxId]        = tmp;
        if(indices){
            indices->at(i)     = maxId;
            indices->at(maxId) = i;
        }    
      }     
    }  
    return *this;
  }

  inline Vector& AbsSort(IndicesVector * indices=NULL){
    if(indices){
        indices->resize(row);
        for(int i=0;i<row;i++) indices->at(i)=i;    
    }
    REALTYPE cmax;
    int maxId;
    for(int i=0;i<row-1;i++){
      cmax  = fabs(_[i]);
      maxId = i;
      for(int j=i+1;j<row;j++){
        if(cmax<fabs(_[j])){
          cmax = fabs(_[j]);
          maxId = j;  
        }            
      }
      if(maxId!=i){
        REALTYPE tmp    = _[i];
        _[i]            = _[maxId];
        _[maxId]        = tmp;
        if(indices){
            indices->at(i)     = maxId;
            indices->at(maxId) = i;
        }    
      }     
    }  
    return *this;
  }
  
  /**
   * \brief Set the value of another vector into the current vector
   * \param startPos The index starting from which the data of the passed vector will be copied
   * \param vector   The input vector
   */  
  inline Vector& SetSubVector(unsigned int startPos, const Vector &vector)
  {
    if(startPos<row){
      const unsigned int k = (row-startPos<=vector.row?row-startPos:vector.row);
      for (unsigned int i = 0; i < k; i++){
        _[startPos+i] = vector._[i];  
      }
    }
    return *this;   
  }

  /**
   * \brief Get a vector containing some of the vector values
   * \param startPos The starting index
   * \param len      The length of data to be copied
   */  
  inline Vector GetSubVector(unsigned int startPos, unsigned int len)
  {
    Vector result(len,false);
    return GetSubVector(startPos,len,result);
  }
  

  /**
   * \brief Get a vector containing some of the vector values
   * \param startPos The starting index
   * \param len      The length of data to be copied
   * \param result   The output vector 
   */  
  inline Vector& GetSubVector(unsigned int startPos, unsigned int len, Vector &result)
  {
    result.Resize(len,false);    
    if(startPos<row){
      const unsigned int k = (row-startPos<=len?row-startPos:len);
      for (unsigned int i = 0; i < k; i++){
        result[i] = _[startPos+i]; 
      }
      for (unsigned int i = k; i < len; i++){
        result[i] = R_ZERO;
      }
      
    }else{
      result.Zero();  
    }
    return result;   
  }

  inline Vector& GetSubVector(const Vector& ids, Vector &result) const
  {
    int s = ids.Size();
    IndicesVector id; 
    for(int i=0;i<s;i++) id.push_back(int(ROUND(ids.At(i))));
    return GetSubVector(id,result);
  }

  /**
   * \brief Get a vector containing the vector values a given indices
   * \param ids      A vecotr containing the indices
   * \param result   The output vector 
   */  
  inline Vector& GetSubVector(const IndicesVector& ids, Vector &result) const
  {
    const unsigned int k=ids.size();
    result.Resize(k);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g = ids[i];
      if(g<row){
        result._[i] = _[g];
      }else{
        result._[i] = R_ZERO;
      }
    }
    return result;     
  }

  /**
   * \brief Set at given indices a given vector values 
   * \param ids      The indices
   * \param result   The vector 
   */  
  inline Vector& SetSubVector(const IndicesVector& ids, const Vector &source)
  {
    const unsigned int j=ids.size();
    const unsigned int k= (source.row<j?source.row:j);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g = ids[i];
      if(g<row){
        _[g] = source._[i];
      }
    }
    return *this;     
  }

  inline Vector& InsertSubVector(unsigned int start,  
                                 const Vector& vector, 
                                 unsigned int vectorStart, unsigned int vectorLength){

    if(vectorStart >= vector.row) return *this;    
    if(start       >= row)        return *this;

    if(vectorStart+vectorLength > vector.row) vectorLength = vector.row-vectorStart;
    
    if(start+vectorLength > row) vectorLength = row-start;
     
    unsigned int rowOffset       = start;
    unsigned int vectorRowOffset = vectorStart;
    for(unsigned int j=0;j<vectorLength;j++){
      _[rowOffset] = vector._[vectorRowOffset];
      rowOffset++;
      vectorRowOffset++;
    }
    return *this;
  } 

  /// Shift the value to the right (rightmost value is set to the left) 
  inline Vector& ShiftRight(){
    if(row>1){
      REALTYPE zero = _[row-1];
      for(unsigned int i=row-1;i>0;i--){
        _[i] = _[i-1];
      }
      _[0] = zero;
    }
    return *this;  
  }

  /// Shift the value toi the left (leftmost value is set to the right) 
  inline Vector& ShiftLeft(){
    if(row>1){
      REALTYPE zero = _[0];
      for(unsigned int i=0;i<row-1;i++){
        _[i] = _[i+1];
      }
      _[row-1] = zero;
    }
    return *this;  
  }

  inline Vector& Trunc(const Vector& min, const Vector& max){
    unsigned int k = MIN(min.row,max.row);
    k = MIN(row,k);
    for(unsigned int i=0;i<k;i++){
      _[i] = TRUNC(_[i],min._[i],max._[i]);  
    }
    for(unsigned int i=k;k<row;k++){
      _[i] = R_ZERO;  
    }
    return *this;
  }

  inline Vector& Trunc(const int min, const int max){
    for(unsigned int i=0;i<row;i++){
      _[i] = TRUNC(_[i],min,max);  
    }
    return *this;
  }

  /// Print the vector to stdout
  void Print() const
  {
    std::cout << "Vector " <<row<<std::endl;;
    for (unsigned int i = 0; i < row; i++)
      std::cout << _[i] <<" ";
    std::cout << std::endl;
  }
  
  int PrintToString(char * str, int maxsize=0){
    int cIndex = 0;
    str[0] = 0;
    if(maxsize<=0){
      for (unsigned int i = 0; i < row; i++){        
        cIndex += sprintf(str+cIndex,"%1.12f ",_[i]);
      }
    }else{
      for (unsigned int i = 0; i < row; i++){                
        int nb = snprintf(str+cIndex,maxsize-cIndex,"%1.12f ",_[i]);        
        if(nb>=maxsize-cIndex) break;
        cIndex += nb;
      }
    }
    return cIndex;
  }
  

protected:
    inline void Release(){
    if(_!=NULL) delete [] _; 
    row = 0;
    _   = NULL;
  }  
public:  
  /**
   * \brief Resize the vector
   * \param size     The new size
   * \param copy     Should keep the original data or just resize. 
   */  
  inline virtual void Resize(unsigned int size, bool copy = true){
    if(row!=size){
      if(size){
        REALTYPE *arr = new REALTYPE[size];
        if(copy){
          unsigned int m = (row<size?row:size);
          for(unsigned int i=0; i<m; i++)
            arr[i] = _[i];
          for(unsigned int i=m; i<size; i++)
            arr[i] = R_ZERO;
        }
        if(_!=NULL) delete [] _; 
        _   = arr;
        row = size;        
      }else{
        Release();
      }
    }
  }
};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif
