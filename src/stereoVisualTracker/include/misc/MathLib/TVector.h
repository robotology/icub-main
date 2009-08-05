#ifndef __TVECTOR_H__
#define __TVECTOR_H__
#define  USE_T_EXTENSIONS
#include <math.h>
#include <iostream>

#include "Vector.h"

template<unsigned int ROW> class TMatrix;

template<unsigned int ROW> class TVector
{
public:
  friend class Vector;
  friend class Vector3;
  friend class TMatrix<ROW>;
  friend class Matrix3;
  friend class Matrix4;

protected:
  static const TVector<ROW>   ZERO;
	static const unsigned int   row   = ROW;
  static       float          undef;
   

public:
	float _[ROW];

	inline TVector()
  {
    Zero();
  }

  inline TVector(const TVector<ROW> &vector)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] = vector._[i];
  }
  
	inline TVector(const float _[ROW])
	{
    Set(_);
	}

  inline TVector(const Vector & vector)
  {
    const unsigned int k = (ROW<=vector.row?ROW:vector.row);    
    for (unsigned int i = 0; i < k; i++)
      _[i] = vector._[i];
    for (unsigned int i = k; i < ROW; i++)
      _[i] = 0.0f;
  }
  
  inline TVector<ROW>& Set(const float _[ROW])
  {
    for (unsigned int i = 0; i < ROW; i++)
      this->_[i] = _[i];
    return *this;
  }
  
  inline float *GetArray(){
    return _;
  }  
  
  inline void Zero()
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] = 0;    
  }
  
  inline float& operator[] (const unsigned int row)
  {
    if(row<ROW)
      return _[row];
    return undef; 
  }

  inline float& operator() (const unsigned int row)
  {
    if(row<ROW)
      return _[row];
    return undef; 
  }

  inline float Get(const unsigned int row) const 
  {
    if(row<ROW)
      return _[row];
    return undef; 
  }
  
  inline TVector<ROW>& Set(const TVector<ROW> &vector)
  {        
    for (unsigned int i = 0; i < ROW; i++)
      _[i] = vector._[i];
    return *this;    
  }   

  
  inline TVector<ROW>& operator = (const TVector<ROW> &vector)
  {        
    for (unsigned int i = 0; i < ROW; i++)
      _[i] = vector._[i];
    return *this;    
  }
     

	inline TVector<ROW> operator - () const
	{
		TVector<ROW> result;
		for (unsigned int i = 0; i < ROW; i++)
			result._[i] = -_[i];
    return result;
	}

  inline TVector<ROW> Minus() const
  {
    TVector<ROW> result;
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = -_[i];
    return result;
  }


	inline TVector<ROW>& operator += (const TVector<ROW> &vector)
	{
		for (unsigned int i = 0; i < ROW; i++)
			_[i] += vector._[i];
    return *this;
	}

	inline TVector<ROW>& operator -= (const TVector<ROW> &vector)
	{
		for (unsigned int i = 0; i < ROW; i++)
			_[i] -= vector._[i];
    return *this;
	}

  inline TVector<ROW>& operator *= (const TVector<ROW> &vector)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] *= vector._[i];
    return *this;
  }

  inline TVector<ROW>& operator /= (const TVector<ROW> &vector)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] /= vector._[i];
    return *this;
  }

  inline TVector<ROW>& operator += (float scalar)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] += scalar;
    return *this;
  }

  inline TVector<ROW>& operator -= (float scalar)
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] -= scalar;
    return *this;
  }

	inline TVector<ROW>& operator *= (float scalar)
	{
		for (unsigned int i = 0; i < ROW; i++)
			_[i] *= scalar;
    return *this;
	}

	inline TVector<ROW>& operator /= (float scalar)
	{
		scalar = 1.0f / scalar;
		for (unsigned int i = 0; i < ROW; i++)
			_[i] *= scalar;
    return *this;
	}


	inline TVector<ROW> operator + (const TVector<ROW> &vector) const
	{
		TVector<ROW> result;
		for (unsigned int i = 0; i < ROW; i++)
			result._[i] = _[i] + vector._[i];
    return result;
	}

	inline TVector<ROW> operator - (const TVector<ROW> &vector) const
	{
		TVector<ROW> result;
		for (unsigned int i = 0; i < ROW; i++)
			result._[i] = _[i] - vector._[i];
    return result; 
	}

  inline TVector<ROW> operator + (float scalar) const
  {
    TVector<ROW> result;
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] + scalar;
    return result;
  }

  inline TVector<ROW> operator - (float scalar) const
  {
    TVector<ROW> result;
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] - scalar;
    return result;
  }

  inline TVector<ROW> operator * (float scalar) const
  {
    TVector<ROW> result;
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] * scalar;
    return result;
  }

  inline TVector<ROW> operator / (float scalar) const
  {
    TVector<ROW> result;
    scalar = 1.0f/scalar;
    for (unsigned int i = 0; i < ROW; i++)
      result._[i] = _[i] * scalar;
    return result;
  }

  inline bool operator == (const TVector<ROW>& vector) const
  {
    for (unsigned int i = 0; i < ROW; i++)
      if(_[i] != vector._[i]) return false;
    return true;
  }

  inline bool operator != (const TVector<ROW>& vector) const
  {
    return !(*this ==  vector);
  }


	inline float Norm() const 
	{
		return sqrtf(Norm2());
	}

  inline float Norm2() const 
  {
    float result = 0.0f;
    for (unsigned int i = 0; i < ROW; i++)
      result += _[i]*_[i];
    return result;
  }

	inline void Normalize()
	{
		float scalar = 1.0f / Norm();
    (*this)*=scalar;
	}
  
  inline float Dot(const TVector<ROW> &vector) const
  {
    float result = 0.0f;
    for (unsigned int i = 0; i < ROW; i++)
      result += _[i]*vector._[i];
    return result;     
  }

  inline TVector<ROW> Trunc(const TVector<ROW> &minVal, const TVector<ROW> &maxVal) 
  {
    for (unsigned int i = 0; i < ROW; i++)
      _[i] = TRUNC(_[i],minVal._[i],maxVal._[i]);
    return *this;     
  }
  
  void Print() const
  {
    std::cout << "Vector" <<ROW<<std::endl;;
    for (unsigned int i = 0; i < ROW; i++)
      std::cout << _[i] <<" ";
    std::cout << std::endl;    
  }
};

template<unsigned int ROW> const TVector<ROW> TVector<ROW>::ZERO;

template<unsigned int ROW> float TVector<ROW>::undef = 0.0f;


#endif
