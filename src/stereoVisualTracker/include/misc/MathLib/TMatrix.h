#ifndef __TMATRIX_H__
#define __TMATRIX_H__

#define  USE_T_EXTENSIONS


#include "TVector.h"
#include "Matrix.h"


template<unsigned int ROW> class TMatrix
{
public:
  friend class Vector;
  friend class TVector<ROW>;
  friend class Vector3;
  friend class Matrix;
  friend class Matrix3;
  friend class Matrix4;
       
protected:
	static float _s[ROW*ROW];
  
  float _[ROW][ROW];
  static float undef;
public:
	inline TMatrix()
  {
    Zero();
  }

  inline TMatrix(const TMatrix<ROW> & matrix)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] = matrix._[j][i];    
  }

	inline TMatrix(const float _[ROW][ROW])
	{
		for (unsigned int j = 0; j < ROW; j++)
			for (unsigned int i = 0; i < ROW; i++)
				this->_[j][i] = _[j][i];
	}

  inline TMatrix(const float *_)
  {
    int cnt = 0;
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        this->_[j][i] = *(_+ (cnt++));
  }

  inline TMatrix(const Matrix & matrix)
  {
    const unsigned int kj = (ROW<=matrix.row?ROW:matrix.row);    
    const unsigned int ki = (ROW<=matrix.column?ROW:matrix.column);    
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < ki; i++)
        _[j][i] = matrix._[j*matrix.column+i];
      for (unsigned int i = ki; i < ROW; i++)
        _[j][i] = 0.0f;
    }
    for (unsigned int j = kj; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] = 0.0f;      
  }  

  inline TMatrix<ROW>& operator = (const TMatrix<ROW> &matrix)
  {    
    for (unsigned int j = 0; j < ROW; j++){
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] = matrix._[j][i];
    }
    return *this;    
  }  

  inline float& operator() (const unsigned int row, const unsigned int col)
  {
    if((row<ROW)&&(col<ROW))
      return _[row][col];
    return TMatrix<ROW>::undef; 
  }


  inline TVector<ROW> GetColumn(unsigned int col) const
  {
    TVector<ROW> vector;
    return GetColumn(col,vector);    
  }
  
  inline TVector<ROW>& GetColumn(unsigned int col, TVector<ROW> &vector) const
  {
    if(col<ROW){
      for (unsigned int j = 0; j < ROW; j++)
        vector._[j] = _[j][col];
      return vector;
    }      
    return vector;
  }

  inline TVector<ROW> GetRow(unsigned int row) const
  {
    TVector<ROW> vector;
    return GetRow(row,vector);    
  }
  
  inline TVector<ROW>& GetRow(unsigned int row, TVector<ROW> &vector) const
  {
    if(row<ROW){
      for (unsigned int i = 0; i < ROW; i++)
        vector._[i] = _[row][i];
      return vector;
    }
    return vector;
  }
  
  inline TMatrix<ROW>& SetColumn(const TVector<ROW> &vector, unsigned int col)
  {
    if(col<ROW)
      for (unsigned int j = 0; j < ROW; j++)
        _[j][col] = vector._[j];
    return *this;
  }

  inline TMatrix<ROW>& SetRow(const TVector<ROW> &vector, unsigned int row)
  {
    if(row<ROW)
      for (unsigned int i = 0; i < ROW; i++)
        _[row][i] = vector._[i];
    return *this;
  }

  inline float* GetArray() const
  {
    return (float*)_; 
  }

  inline TMatrix<ROW>& SetArray(float * array)
  {
    memcpy(_,array,ROW*ROW*sizeof(float));
    return *this; 
  }

	inline TMatrix<ROW> operator - () const
	{
		TMatrix<ROW> result;
		for (unsigned int j = 0; j < ROW; j++)
			for (unsigned int i = 0; i < ROW; i++)
				result._[j][i] = -_[j][i];
    return result;
	}
  
  inline TMatrix<ROW>& Zero(){
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] = 0.0f;
    return *this;
  }

	inline TMatrix<ROW>& operator += (const TMatrix<ROW> &matrix)
	{
		for (unsigned int j = 0; j < ROW; j++)
			for (unsigned int i = 0; i < ROW; i++)
				_[j][i] += matrix._[j][i];
    return *this;
	}

	inline TMatrix<ROW>& operator -= (const TMatrix<ROW> &matrix)
	{
		for (unsigned int j = 0; j < ROW; j++)
			for (unsigned int i = 0; i < ROW; i++)
				_[j][i] -= matrix._[j][i];
    return *this;
	}

  inline TMatrix<ROW>& operator ^= (const TMatrix<ROW> &matrix)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] *= matrix._[j][i];
    return *this;
  }

  inline TMatrix<ROW>& operator *= (const TMatrix<ROW> &matrix)
  {
    TMatrix<ROW> copy(*this);
    for (unsigned int j = 0; j < ROW; j++){
      for (unsigned int i = 0; i < ROW; i++){
        _[j][i] = 0.0f;
        for(unsigned int k = 0; k< ROW; k++)    
          _[j][i] += copy._[j][k] * matrix._[k][i];
      }
    }
    return *this;
  }

  inline TMatrix<ROW>& operator /= (const TMatrix<ROW> &matrix)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] /= matrix._[j][i];
    return *this;
  }  

  inline TMatrix<ROW>& operator += (float scalar)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] += scalar;
    return *this;
  }

  inline TMatrix<ROW>& operator -= (float scalar)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] -= scalar;
    return *this;
  }

	inline TMatrix<ROW>& operator *= (float scalar)
	{
		for (unsigned int j = 0; j < ROW; j++)
			for (unsigned int i = 0; i < ROW; i++)
				_[j][i] *= scalar;
    return *this;
	}

	inline TMatrix<ROW>& operator /= (float scalar)
	{
		scalar = 1.0f / scalar;
		for (unsigned int j = 0; j < ROW; j++)
			for (unsigned int i = 0; i < ROW; i++)
				_[j][i] *= scalar;
    return *this;
	}

	inline TMatrix<ROW> operator + (const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return Add(matrix,result);
  }

  inline TMatrix<ROW>& Add(const TMatrix<ROW> &matrix, TMatrix<ROW> & result) const
  {
		for (unsigned int j = 0; j < ROW; j++)
			for (unsigned int i = 0; i < ROW; i++)
				result._[j][i] = _[j][i] + matrix._[j][i];		
    return result;
	}

	inline TMatrix<ROW> operator - (const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return Sub(matrix,result);
  }

  inline TMatrix<ROW>& Sub(const TMatrix<ROW> &matrix, TMatrix<ROW> & result) const
  {
		for (unsigned int j = 0; j < ROW; j++)
			for (unsigned int i = 0; i < ROW; i++)
				result._[j][i] = _[j][i] - matrix._[j][i];
    return result;
	}

  inline TMatrix<ROW> operator ^ (const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return PMult(matrix,result);
  }

  inline TMatrix<ROW>& PMult(const TMatrix<ROW> &matrix, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = _[j][i] * matrix._[j][i];    
    return result;
  }

  inline TMatrix<ROW> operator * (const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return Mult(matrix,result);
  }

  inline TMatrix<ROW> Mult(const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return Mult(matrix,result);
  }

  inline TMatrix<ROW>& Mult(const TMatrix<ROW> &matrix, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++){
      for (unsigned int i = 0; i < ROW; i++){
        result._[j][i] = 0.0f;
        for(unsigned int k = 0; k< ROW; k++)    
          result._[j][i] += _[j][k] * matrix._[k][i];
      }
    }
    return result;
  }

  inline TMatrix<ROW> operator + (float scalar) const
  {
    TMatrix<ROW> result;
    return Add(scalar,result);
  }

  inline TMatrix<ROW>& Add(float scalar, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = _[j][i] + scalar;
    return result;
  }

  inline TMatrix<ROW> operator - (float scalar) const
  {
    TMatrix<ROW> result;
    return Sub(scalar,result);
  }

  inline TMatrix<ROW>& Sub(float scalar, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = _[j][i] - scalar;
    return result;
  }

	inline TMatrix<ROW> operator * (float scalar) const
  {
    TMatrix<ROW> result;
    return Mult(scalar,result);
  }

  inline TMatrix<ROW>& Mult(float scalar, TMatrix<ROW> & result) const
  {
		for (unsigned int j = 0; j < ROW; j++)
			for (unsigned int i = 0; i < ROW; i++)
				result._[j][i] = _[j][i] * scalar;
    return result;
	}

	inline TMatrix<ROW> operator / (float scalar) const
  {
    TMatrix<ROW> result;
    return Div(scalar,result);
  }

  inline TMatrix<ROW>& Div(float scalar, TMatrix<ROW> & result) const
  {
		scalar = 1.0f / scalar;
		for (unsigned int j = 0; j < ROW; j++)
			for (unsigned int i = 0; i < ROW; i++)
				result._[j][i] = _[j][i] * scalar;
    return result;
	}

  inline TVector<ROW> operator * (const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return Mult(vector,result);
  }

  inline TVector<ROW> Mult(const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return Mult(vector,result);
  }

  inline TVector<ROW>& Mult(const TVector<ROW> &vector, TVector<ROW> & result) const
  {
		for (unsigned int j = 0; j < ROW; j++)
		{
			result._[j] = 0.0f;
			for (unsigned int i = 0; i < ROW; i++)
				result._[j] += _[j][i] * vector._[i];
		}
		return result;
	}  

  inline TMatrix<ROW> Transpose() const
  {
    TMatrix<ROW> result;
    return Transpose(result);
  }

	inline TMatrix<ROW>& Transpose(TMatrix<ROW>& result) const
	{
		for (unsigned int j = 0; j < ROW; j++)
		{
			for (unsigned int i = 0; i < ROW; i++)
				result._[i][j] = _[j][i];
		}
    return result;
	}
  
  inline TMatrix<ROW>& Identity()
  {
    Zero();
    for (unsigned int i = 0; i < ROW; i++)
      _[i][i] = 1.0f;
    return *this;    
  }
  
  float* RowOrder(float result[ROW*ROW]=NULL) const
  {        
    float *iter,*res;    
    iter=res=(result?result:_s);  
    for (unsigned int i = 0; i < ROW; i++)
      for (unsigned int j = 0; j < ROW; j++)
        *(iter++) = _[j][i];

    return res;
  }
  
  void Print() const
  {
    std::cout << "Matrix " <<ROW<<"x"<<ROW<<std::endl;;
    for (unsigned int j = 0; j < ROW; j++){
      for (unsigned int i = 0; i < ROW; i++)
        std::cout << _[j][i] <<" ";
      std::cout << std::endl;
    }
  }

};


template<unsigned int ROW> float TMatrix<ROW>::_s[ROW*ROW];
template<unsigned int ROW> float TMatrix<ROW>::undef=0.0;


#endif
