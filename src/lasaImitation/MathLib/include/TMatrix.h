#ifndef __TMATRIX_H__
#define __TMATRIX_H__

#include "MathLibCommon.h"

#define  USE_T_EXTENSIONS


#include "TVector.h"
#include "Matrix.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

/**
 * \class TMatrix
 * 
 * \ingroup MathLib
 * 
 * \brief The basic template square matrix class. See the Matrix class for the explanation of the functions.
 * 
 * This template square matrix class can be used for doing various matrix manipulation.
 * This should be combined with the TVector class for doing almost anything
 * you ever dreamt of. 
 */
template<unsigned int ROW> class TMatrix
{
public:
  /*
  friend class Vector;
  friend class TVector<ROW>;
  friend class Vector3;
  friend class Matrix;
  friend class Matrix3;
  friend class Matrix4;
  */     
protected:
  static REALTYPE _s[ROW*ROW];
  static float    _sf[ROW*ROW];
  static REALTYPE undef;

public:  
  REALTYPE _[ROW][ROW];
  
public:
  /// Empty constructor
  inline TMatrix()
  {
    Zero();
  }

  /// Copy constructor
  inline TMatrix(const TMatrix<ROW> & matrix)
  {
    Set(matrix);
  }

  /// Constructor with data pointer to be copied in the matrix
  inline TMatrix(const REALTYPE _[ROW][ROW])
  {
    Set(_);
  }

  /// Constructor with data pointer to be copied in the matrix
  inline TMatrix(const REALTYPE *_)
  {
    Set(_);
  }

  /*
  /// Constructor which copy the data of a normal Matrix object
  inline TMatrix(const Matrix & matrix)
  {
    Set(matrix);
  } 
  */



  /// Self copy of the argument
  inline TMatrix<ROW>& Set(const TMatrix<ROW> & matrix)
  {
    return Set(matrix._);
  }

  /// Assigment of data pointer
  inline TMatrix<ROW>& Set(const REALTYPE *array)
  {
    memcpy(_,array,ROW*ROW*sizeof(REALTYPE));
    return *this;
  }

  /// Assigment of data pointer
  inline TMatrix<ROW>& SetForceFloat(const float *array)
  {
    const float *arrayPos = array; 
    for (unsigned int j = 0; j < ROW; j++){
      for (unsigned int i = 0; i < ROW; i++){
        _[j][i] = float(*(arrayPos++));
      }
    }
    return *this;
  }

  /// Assigment of data pointer
  inline TMatrix<ROW>& Set(const REALTYPE array[ROW][ROW])
  {
    Set((REALTYPE*)(array)); 
    return *this;
  }

  /// Copy the data from a normal Matrix object
  inline TMatrix<ROW>& Set(const Matrix & matrix)
  {
    const unsigned int kj = (ROW<=matrix.row?ROW:matrix.row);    
    const unsigned int ki = (ROW<=matrix.column?ROW:matrix.column);    
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < ki; i++)
        _[j][i] = matrix._[j*matrix.column+i];
      for (unsigned int i = ki; i < ROW; i++)
        _[j][i] = R_ZERO;
    }
    for (unsigned int j = kj; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] = R_ZERO;
    return *this;      
  }  

  /// Gets the data array
  inline REALTYPE* GetArray() const
  {
    return (REALTYPE*)_; 
  }

  /// Sets all values to zero
  inline TMatrix<ROW>& Zero(){
    memset(_,0,ROW*ROW*sizeof(REALTYPE));
    return *this;
  }

  /// Sets the matrix to identity
  inline TMatrix<ROW>& Identity()
  {
    Zero();
    for (unsigned int i = 0; i < ROW; i++)
      _[i][i] = R_ONE;
    return *this;    
  }

  /// Gets a reference to the given element
  inline REALTYPE& operator() (const unsigned int row, const unsigned int col)
  {
    if((row<ROW)&&(col<ROW))
      return _[row][col];
    return undef; 
  }

  /// Gets the value of the given element
  inline REALTYPE Get(const unsigned int row, const unsigned int col) const
  {
    if((row<ROW)&&(col<ROW))
      return _[row][col];
    return undef;     
  }

  /// Returns the given column
  inline TVector<ROW> GetColumn(unsigned int col) const
  {
    TVector<ROW> vector;
    return GetColumn(col,vector);    
  }
  
  /// Returns the given column in the vector
  inline TVector<ROW>& GetColumn(unsigned int col, TVector<ROW> &vector) const
  {
    if(col<ROW){
      for (unsigned int j = 0; j < ROW; j++)
        vector._[j] = _[j][col];
      return vector;
    }      
    return vector;
  }

  /// Returns the given row
  inline TVector<ROW> GetRow(unsigned int row) const
  {
    TVector<ROW> vector;
    return GetRow(row,vector);    
  }
  
  /// Returns the given column in the vector
  inline TVector<ROW>& GetRow(unsigned int row, TVector<ROW> &vector) const
  {
    if(row<ROW){
      for (unsigned int i = 0; i < ROW; i++)
        vector._[i] = _[row][i];
      return vector;
    }
    return vector;
  }
  
  
  /// Sets the given column 
  inline TMatrix<ROW>& SetColumn(const TVector<ROW> &vector, unsigned int col)
  {
    if(col<ROW)
      for (unsigned int j = 0; j < ROW; j++)
        _[j][col] = vector._[j];
    return *this;
  }

  /// Sets the given row
  inline TMatrix<ROW>& SetRow(const TVector<ROW> &vector, unsigned int row)
  {
    if(row<ROW)
      for (unsigned int i = 0; i < ROW; i++)
        _[row][i] = vector._[i];
    return *this;
  }

  /// Assignment operator
  inline TMatrix<ROW>& operator = (const TMatrix<ROW> &matrix)
  {
    return Set(matrix);    
  }  

  /// Inverse operator
  inline TMatrix<ROW> operator - () const
  {
    TMatrix<ROW> result;
    return Minus(result);
  }

  /// Inverse operator
  inline TMatrix<ROW>& Minus(TMatrix<ROW>& result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = -_[j][i];
    return result;
  }
  
  
  /// Assignment data-wise operations
  inline TMatrix<ROW>& operator += (const TMatrix<ROW> &matrix)
  {
    return SAdd(matrix);
  }

  inline TMatrix<ROW>& operator -= (const TMatrix<ROW> &matrix)
  {
    return SSub(matrix);
  }

  inline TMatrix<ROW>& operator ^= (const TMatrix<ROW> &matrix)
  {
    return SPMult(matrix);
  }

  inline TMatrix<ROW>& operator /= (const TMatrix<ROW> &matrix)
  {
    return SPDiv(matrix);
  }  

  inline TMatrix<ROW>& operator *= (const TMatrix<ROW> &matrix)
  {
    return SMult(matrix);
  }

  inline TMatrix<ROW>& SAdd(const TMatrix<ROW> &matrix)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] += matrix._[j][i];
    return *this;
  }

  inline TMatrix<ROW>& SSub(const TMatrix<ROW> &matrix)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] -= matrix._[j][i];
    return *this;
  }

  inline TMatrix<ROW>& SPMult(const TMatrix<ROW> &matrix)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] *= matrix._[j][i];
    return *this;
  }

  inline TMatrix<ROW>& SPDiv(const TMatrix<ROW> &matrix)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] /= matrix._[j][i];
    return *this;
  }  

  inline TMatrix<ROW>& SMult(const TMatrix<ROW> &matrix)
  {
    TMatrix<ROW> copy(*this);
    copy.Mult(matrix,*this);
    return *this;
  }







  inline TMatrix<ROW>& operator += (REALTYPE scalar)
  {
    return SAdd(scalar);
  }

  inline TMatrix<ROW>& operator -= (REALTYPE scalar)
  {
    return SSub(scalar);
  }

  inline TMatrix<ROW>& operator *= (REALTYPE scalar)
  {
    return SMult(scalar);
  }

  inline TMatrix<ROW>& operator /= (REALTYPE scalar)
  {
    return SDiv(scalar);
  }

  inline TMatrix<ROW>& SAdd(REALTYPE scalar)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] += scalar;
    return *this;
  }

  inline TMatrix<ROW>& SSub(REALTYPE scalar)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] -= scalar;
    return *this;
  }

  inline TMatrix<ROW>& SMult(REALTYPE scalar)
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[j][i] *= scalar;
    return *this;
  }

  inline TMatrix<ROW>& SDiv(REALTYPE scalar)
  {
    scalar = R_ONE / scalar;
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

  inline TMatrix<ROW> operator - (const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return Sub(matrix,result);
  }

  inline TMatrix<ROW> operator ^ (const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return PMult(matrix,result);
  }

  inline TMatrix<ROW> operator / (const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return PDiv(matrix,result);
  }

  inline TMatrix<ROW> operator * (const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return Mult(matrix,result);
  }

  inline TMatrix<ROW> Add(const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return Add(matrix,result);
  }

  inline TMatrix<ROW> Sub(const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return Sub(matrix,result);
  }

  inline TMatrix<ROW> PMult(const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return PMult(matrix,result);
  }

  inline TMatrix<ROW> PDiv(const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return PDiv(matrix,result);
  }

  inline TMatrix<ROW> Mult(const TMatrix<ROW> &matrix) const
  {
    TMatrix<ROW> result;
    return Mult(matrix,result);
  }

  inline TMatrix<ROW>& Add(const TMatrix<ROW> &matrix, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = _[j][i] + matrix._[j][i];    
    return result;
  }

  inline TMatrix<ROW>& Sub(const TMatrix<ROW> &matrix, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = _[j][i] - matrix._[j][i];
    return result;
  }

  inline TMatrix<ROW>& PMult(const TMatrix<ROW> &matrix, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = _[j][i] * matrix._[j][i];    
    return result;
  }

  inline TMatrix<ROW>& PDiv(const TMatrix<ROW> &matrix, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = _[j][i] / matrix._[j][i];    
    return result;
  }

  inline TMatrix<ROW>& Mult(const TMatrix<ROW> &matrix, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++){
      for (unsigned int i = 0; i < ROW; i++){
        result._[j][i] = R_ZERO;
        for(unsigned int k = 0; k< ROW; k++)    
          result._[j][i] += _[j][k] * matrix._[k][i];
      }
    }
    return result;
  }











  inline TMatrix<ROW> operator + (REALTYPE scalar) const
  {
    TMatrix<ROW> result;
    return Add(scalar,result);
  }

  inline TMatrix<ROW> operator - (REALTYPE scalar) const
  {
    TMatrix<ROW> result;
    return Sub(scalar,result);
  }

  inline TMatrix<ROW> operator * (REALTYPE scalar) const
  {
    TMatrix<ROW> result;
    return Mult(scalar,result);
  }

  inline TMatrix<ROW> operator / (REALTYPE scalar) const
  {
    TMatrix<ROW> result;
    return Div(scalar,result);
  }

  inline TMatrix<ROW> Add(REALTYPE scalar) const
  {
    TMatrix<ROW> result;
    return Add(scalar,result);
  }

  inline TMatrix<ROW> Sub(REALTYPE scalar) const
  {
    TMatrix<ROW> result;
    return Sub(scalar,result);
  }

  inline TMatrix<ROW> Mult(REALTYPE scalar) const
  {
    TMatrix<ROW> result;
    return Mult(scalar,result);
  }

  inline TMatrix<ROW> Div(REALTYPE scalar) const
  {
    TMatrix<ROW> result;
    return Div(scalar,result);
  }


  inline TMatrix<ROW>& Add(REALTYPE scalar, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = _[j][i] + scalar;
    return result;
  }

  inline TMatrix<ROW>& Sub(REALTYPE scalar, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = _[j][i] - scalar;
    return result;
  }

  inline TMatrix<ROW>& Mult(REALTYPE scalar, TMatrix<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        result._[j][i] = _[j][i] * scalar;
    return result;
  }

  inline TMatrix<ROW>& Div(REALTYPE scalar, TMatrix<ROW> & result) const
  {
    scalar = R_ONE / scalar;
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
    for (unsigned int j = 0; j < ROW; j++) {
      result._[j] = R_ZERO;
      for (unsigned int i = 0; i < ROW; i++)
        result._[j] += _[j][i] * vector._[i];
    }
    return result;
  }  

  inline TVector<ROW> MultTranspose(const TVector<ROW> &vector) const
  {
    TVector<ROW> result;
    return MultTranspose(vector,result);
  }

  inline TVector<ROW>& MultTranspose(const TVector<ROW> &vector, TVector<ROW> & result) const
  {
    for (unsigned int j = 0; j < ROW; j++) {
      result._[j] = R_ZERO;
      for (unsigned int i = 0; i < ROW; i++)
        result._[j] += _[i][j] * vector._[i];
    }
    return result;
  }  

  /// Tests equality of two matrices
  inline bool operator == (const TMatrix<ROW>& matrix) const
  {
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        if(_[j][i] != matrix._[j][i]) return false;
    return true;
  }

  /// tests inequality of two vectors
  inline bool operator != (const TMatrix<ROW>& matrix) const
  {
    return !(*this ==  matrix);
  }

  inline TMatrix<ROW>& SAbs(){
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        _[i][j] = fabs(_[i][j]);
    return *this;  
  }

  inline REALTYPE Sum(){
    REALTYPE res = R_ZERO;
    for (unsigned int j = 0; j < ROW; j++)
      for (unsigned int i = 0; i < ROW; i++)
        res += _[i][j];
    return res;  
  }


  /// Self transpose
  inline TMatrix<ROW> STranspose()
  {
    REALTYPE tmp;
    for (unsigned int j = 0; j < ROW-1; j++){
      for (unsigned int i = j+1; i < ROW; i++){
        tmp     = _[j][i];
        _[j][i] = _[i][j];
        _[i][j] = tmp;
      }
    }
    return *this;
  }

  /// Returns the transpose
  inline TMatrix<ROW> Transpose() const
  {
    TMatrix<ROW> result;
    return Transpose(result);
  }

  /// Returns the transpose in the result
  inline TMatrix<ROW>& Transpose(TMatrix<ROW>& result) const
  {
    for (unsigned int j = 0; j < ROW; j++)
    {
      for (unsigned int i = 0; i < ROW; i++)
        result._[i][j] = _[j][i];
    }
    return result;
  }
  
  
  /**
   * \brief Return a data pointer with data ordered in rows and not in column. Useful for opengl matrix manipulation 
   * \param result The data array. If null, this function uses the internal static member of the matrix template.
   * \return The pointer to the data
   */    
  REALTYPE* RowOrder(REALTYPE result[ROW*ROW]=NULL) const
  {        
    REALTYPE *iter,*res;    
    iter=res=(result?result:_s);  
    for (unsigned int i = 0; i < ROW; i++)
      for (unsigned int j = 0; j < ROW; j++)
        *(iter++) = _[j][i];

    return res;
  }

  /// Same as previous but force the target array to be of type float (usefl for OpenGL)
  float* RowOrderForceFloat(float result[ROW*ROW]=NULL) const
  {        
    float *iter,*res;    
    iter=res=(result?result:_sf);  
    for (unsigned int i = 0; i < ROW; i++)
      for (unsigned int j = 0; j < ROW; j++)
        *(iter++) = float(_[j][i]);

    return res;
  }
  
  /// Prints out the vector to stdout
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


template<unsigned int ROW> REALTYPE TMatrix<ROW>::_s[ROW*ROW];
template<unsigned int ROW> float    TMatrix<ROW>::_sf[ROW*ROW];
template<unsigned int ROW> REALTYPE TMatrix<ROW>::undef=0.0;

#ifdef USE_MATHLIB_NAMESPACE
}
#endif

#endif
