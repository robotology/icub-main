// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Eric Sauser, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   micha.hersch@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
#ifndef __VECTOR_H__
#define __VECTOR_H__

#include "Macros.h"
#include <math.h>
#include <iostream>


#ifndef NULL
#define NULL 0
#endif

#ifdef  USE_T_EXTENSIONS
template<unsigned int ROW> class TVector;
#endif

class Vector
{
  friend class Matrix;
#ifdef  USE_T_EXTENSIONS  
  template<unsigned int ROW> friend class TVector;  
#endif
  
protected:
  static  float undef;
   
  unsigned int   row;
  float         *_;

public:

  
	inline Vector() {
    row = 0;
    _   = NULL;
   }
  
  inline virtual ~Vector(){
    Release(); 
  }

  inline Vector(const Vector &vector)
  {
    row = 0;
    _   = NULL;
    Resize(vector.row,false);
    for (unsigned int i = 0; i < row; i++)
      _[i] = vector._[i];
  }

  inline Vector(unsigned int size, bool clear = true)
  {
    row = 0;
    _   = NULL;
    Resize(size,false);
    if(clear)
      Zero();
  }
  
  inline Vector(const float _[], unsigned int size)
  {
    row       = 0;
    this->_   = NULL;
    Resize(size,false);
    for (unsigned int i = 0; i < row; i++)
      this->_[i] = _[i];
  }
   
#ifdef  USE_T_EXTENSIONS
  template<unsigned int ROW> inline Vector(const TVector<ROW> &vector)
  {
    row = 0;
    _   = NULL;
    Resize(ROW,false);
    for (unsigned int i = 0; i < row; i++)
      _[i] = vector._[i];        
  }
#endif     
   
  inline Vector& Zero()
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] = 0.0f;
    return *this;    
  }

  inline Vector& One()
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] = 1.0f;
    return *this;    
  }

  inline Vector& Random(){
    for (unsigned int j = 0; j < row; j++)
      _[j] =((float)rand())/((float)(RAND_MAX+1.0));    
    return *this;    
  }
    
  inline unsigned int Size() const{
    return row;
  }
  
  inline float *GetArray() const{
    return _;
  }
  
  inline float& operator[] (const unsigned int row)
  {
    if(row<this->row)
      return _[row];
    return undef; 
  }
  
  inline float& operator() (const unsigned int row)
  {
    if(row<this->row)
      return _[row];
    return undef; 
  }
  
  inline Vector operator - () const
  {
    Vector result(row,false);
    for (unsigned int i = 0; i < row; i++)
      result._[i] = -_[i];
    return result;
  }
  
  inline Vector& Set(const Vector &vector){
    return (*this)=vector;  
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

  inline Vector& operator ^= (const Vector &vector)
  {
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      _[i] *= vector._[i];
    return *this;
  }

  inline Vector& operator /= (const Vector &vector)
  {
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      _[i] /= vector._[i];
    return *this;
  }

  inline Vector& operator += (float scalar)
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] += scalar;
    return *this;
  }

  inline Vector& operator -= (float scalar)
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] -= scalar;
    return *this;
  }

	inline Vector& operator *= (float scalar)
	{
		for (unsigned int i = 0; i < row; i++)
			_[i] *= scalar;
    return *this;
	}

	inline Vector& operator /= (float scalar)
	{
		scalar = 1.0f / scalar;
		for (unsigned int i = 0; i < row; i++)
			_[i] *= scalar;
    return *this;
	}

  inline Vector operator + (const Vector &vector) const
  {
    Vector result(row,false);
    return Add(vector,result);    
  }
  
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

  inline Vector operator - (const Vector &vector) const
  {
    Vector result(row,false);
    return Sub(vector,result);    
  }
  
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

  inline Vector operator ^ (const Vector &vector) const
  {
    Vector result(row,false);
    return PMult(vector,result);    
  }
  
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

  inline Vector operator / (const Vector &vector) const
  {
    Vector result(row,false);
    return PDiv(vector,result);    
  }
  
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

  inline float operator * (const Vector &vector) const
  { 
    return this->Dot(vector);  
  }
  
  inline Vector operator + (float scalar) const
  {
    Vector result(row,false);
    return Add(scalar,result);    
  }
  
  inline Vector& Add(float scalar, Vector& result) const
  {
    result.Resize(row,false);
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] + scalar;
    return result;
  }

  inline Vector operator - (float scalar) const
  {
    Vector result(row,false);
    return Sub(scalar,result);    
  }
  
  inline Vector& Sub(float scalar, Vector& result) const
  {
    result.Resize(row,false);
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] - scalar;
    return result;
  }

  inline Vector operator * (float scalar) const
  {
    Vector result(row,false);
    return Mult(scalar,result);    
  }
  
  inline Vector& Mult(float scalar, Vector& result) const
  {
    result.Resize(row,false);
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] * scalar;
    return result;
  }

  inline Vector operator / (float scalar) const
  {
    Vector result(row,false);
    return Div(scalar,result);    
  }
  
  inline Vector& Div(float scalar, Vector& result) const
  {
    result.Resize(row,false);
    scalar = 1.0f/scalar;
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] * scalar;
    return result;
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


  inline float Sum() const 
  {
    float result = 0.0f;
    for (unsigned int i = 0; i < row; i++)
      result += _[i];
    return result;
  }

  inline float Norm() const 
  {
    return sqrtf(Norm2());
  }

  inline float Norm2() const 
  {
    float result = 0.0f;
    for (unsigned int i = 0; i < row; i++)
      result += _[i]*_[i];
    return result;
  }

	inline void Normalize()
	{
		float scalar = 1.0f / Norm();
    (*this)*=scalar;
	}
  
  inline float Distance(const Vector &vector) const
  {
    return (*this-vector).Norm();
  }
  
  inline float Distance2(const Vector &vector) const
  {
    return (*this-vector).Norm2();  
  }

  inline float Dot(const Vector &vector) const
  {
    const unsigned int k = (row<=vector.row?row:vector.row);
    float result = 0.0f;
    for (unsigned int i = 0; i < k; i++)
      result += _[i]*vector._[i];
    return result;     
  }

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

  inline Vector GetSubVector(unsigned int startPos, unsigned int len)
  {
    Vector result(len,false);
    return GetSubVector(startPos,len,result);
  }
  

  inline Vector& GetSubVector(unsigned int startPos, unsigned int len, Vector &result)
  {
    result.Resize(len,false);    
    if(startPos<row){
      const unsigned int k = (row-startPos<=len?row-startPos:len);
      for (unsigned int i = 0; i < k; i++){
        result[i] = _[startPos+i]; 
      }
      for (unsigned int i = k; i < len; i++){
        result[i] = 0.0f;
      }
      
    }else{
      result.Zero();  
    }
    return result;   
  }

  inline float Max(){
    if(row==0)
      return 0.0f;
      
    float res=_[0];
    for(unsigned int i=1;i<row;i++){
      if(_[i]>res) res = _[i];  
    }
    return res;
  }

  inline int MaxId(){
    if(row==0)
      return -1;
      
    float mx  = _[0];
    int   res = 0;
    for(unsigned int i=1;i<row;i++){
      if(_[i]>mx){ mx = _[i]; res = i;}  
    }
    return res;
  }

  inline Vector Abs(){
    Vector result(row);
    return Abs(result);
  }

  inline Vector& Abs(Vector &result) const{
    result.Resize(row,false);
    for(unsigned int i=0;i<row;i++){
      result._[i] = fabs(_[i]);
    }
    return result;
  }

  inline Vector& GetSubVector(const Vector &ids, Vector &result) const
  {
    const unsigned int k=ids.Size();
    result.Resize(k);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g = (unsigned int)(fabs(ROUND(ids._[i])));
      if(g<row){
        result._[i] = _[g];
      }else{
        result._[i] = 0.0f;
      }
    }
    return result;     
  }


  void Print() const
  {
    std::cout << "Vector " <<row<<std::endl;;
    for (unsigned int i = 0; i < row; i++)
      std::cout << _[i] <<" ";
    std::cout << std::endl;
  }
  
inline std::ostream& StreamOut(std::ostream& out) const
{
   for(unsigned int i=0;i<row;i++){
     out<< _[i]<<" ";
   }
   return out;
 }

protected:
    inline void Release(){
    if(_!=NULL) delete [] _; 
    row = 0;
    _   = NULL;
  }  
public:  
  inline virtual void Resize(unsigned int size, bool copy = true){
    if(row!=size){
      if(size){
        float *arr = new float[size];
        if(copy){
          unsigned int m = (row<size?row:size);
          for(unsigned int i=0; i<m; i++)
            arr[i] = _[i];
          for(unsigned int i=m; i<size; i++)
            arr[i] = 0.0f;
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


inline std::ostream& operator<<(std::ostream& out, const Vector& v){
  return v.StreamOut(out);
}
#endif
