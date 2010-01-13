#ifndef MATRIX_H
#define MATRIX_H
#include <vector>
using namespace std;

#include "MathLibCommon.h"


#include "Macros.h"
#include "Vector.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

#ifdef  USE_T_EXTENSIONS
template<unsigned int ROW> class TMatrix;
#endif


/**
 * \class Matrix
 * 
 * \ingroup MathLib
 * 
 * \brief The basic matrix class
 * 
 * This matrix class can be used for doing various matrix manipulation.
 * This shFould be combined with the Vector class for doing almost anything
 * you ever dreamt of. Please understand almost as almost...
 */
    
class Matrix
{
  friend class Vector;
#ifdef  USE_T_EXTENSIONS
  template<unsigned int ROW> friend class TMatrix;
#endif
  
protected: 
  static int bInverseOk;    ///< Tell if last inverse operation was sucessfull
  
  unsigned int  row;        ///< Number of rows
  unsigned int  column;     ///< Number of columns
  REALTYPE        *_;          ///< The data array

public:

  /// Empty constructor
  inline Matrix() {
    row    = 0;
    column = 0;
    _      = NULL;
  }
  
  /// Destructor
  inline virtual ~Matrix(){
    Release(); 
  }

  /// Copy constructor
  inline Matrix(const Matrix &matrix)
  {
    row    = 0;
    column = 0;
    _      = NULL;
    Resize(matrix.row,matrix.column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = matrix._[j*column+i];
  }

  /**
   * \brief Create a sized matrix
   * \param rowSize  The row size
   * \param colSize  The column size
   * \param clear Tell if the matrix sould be set to 0
   */    
  inline Matrix(unsigned int rowSize, unsigned int colSize, bool clear = true)
  {
    row    = 0;
    column = 0;
    _      = NULL;
    Resize(rowSize,colSize,false);
    if(clear)
      Zero();
  }
  
  /**
   * \brief Create a matrix by copying a memory area
   * \param _     The array of REALTYPE to be copied
   * \param rowSize  The row size
   * \param colSize  The column size
   */  
  inline Matrix(const REALTYPE _[], unsigned int rowSize, unsigned int colSize)
  {
    row       = 0;
    column    = 0;
    this->_   = NULL;
    Set(_,rowSize,colSize);
  }

#ifdef  USE_T_EXTENSIONS
  /// Copy Contructor of a template Matrix (see TMatrix)  
  template<unsigned int ROW> inline Matrix(const TMatrix<ROW> &matrix)
  {
    row    = 0;
    column = 0;
    _      = NULL;
    Resize(ROW,ROW,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = matrix._[j][i];
  }
#endif

  /**
   * \brief Set a matrix by copying a memory area
   * \param _     The array of REALTYPE to be copied
   * \param rowSize  The row size
   * \param colSize  The column size
   */  
  inline Matrix& Set(const REALTYPE _[], unsigned int rowSize, unsigned int colSize){
    Resize(rowSize,colSize,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        this->_[j*column+i] = _[j*column+i];
    return *this;    
  }

   
  /// Get the number of rows  
  inline unsigned int RowSize() const{
    return row;
  }
  /// Get the number of columns
  inline unsigned int ColumnSize() const{
    return column;
  } 
  /// Get the data array
  inline REALTYPE *Array() const{
    return _;
  }

  /// Clear the matrix
  inline Matrix& Zero()
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = R_ZERO;
    return *this;
  }
    
  /// Identity matrix
  inline Matrix& Identity()
  {
    const unsigned int k = (row>column?column:row);
    Zero();
    for (unsigned int i = 0; i < k; i++)
      _[i*column+i] = R_ONE;
    return *this;    
  }

  /// Access to a reference of any element of the matrix
  inline REALTYPE& operator() (const unsigned int row, const unsigned int col)
  {
    if((row<this->row)&&(col<this->column))
      return _[row*column+col];
    return Vector::undef; 
  }

  /// Access to an element of the matrix
  inline REALTYPE At(const unsigned int row, const unsigned int col) const
  {
    if((row<this->row)&&(col<this->column))
      return _[row*column+col];
    return Vector::undef; 
  }

  /// Get a copy of a row into a vector
  inline Vector GetRow(const unsigned int row) const
  {
    Vector result(column,false);    
    return GetRow(row,result);     
  }

  /// Get a copy of a row into a vector
  inline Vector& GetRow(const unsigned int row, Vector& result) const
  {
    result.Resize(column,false);
    for (unsigned int i = 0; i < column; i++)
      result._[i] = _[row*column+i];
    return result;     
  }

  /// Get a copy of a column into a vector
  inline Vector GetColumn(const unsigned int col) const
  {
    Vector result(row,false);    
    return GetColumn(col,result);     
  }

  /// Get a copy of a column into a vector
  inline Vector& GetColumn(const unsigned int col, Vector& result) const
  {
    result.Resize(row,false);
    if(col<column){
      for (unsigned int j = 0; j < row; j++)
        result._[j] = _[j*column+col];
    }else{
      result.Zero();
    }
    return result;     
  }


  /**
   * \brief Get a matrix spanning several rows of the matrix
   * \param row      The starting row
   * \param rowSize  The number of rows
   * \return         The resulting matrix
   */  
  inline Matrix GetRowSpace(const unsigned int row, const unsigned int len) const
  {
    if(len>0){
      Matrix result(len,column,false);    
      return GetRowSpace(row,len,result);
    }else
      return Matrix();     
  }

  /**
   * \brief Get a matrix spanning several rows of the matrix
   * \param row      The starting row
   * \param len      The number of rows
   * \param result   The target matrix
   * \return         The resulting matrix
   */  
  inline Matrix& GetRowSpace(const unsigned int row, const unsigned int len, Matrix &result) const
  {      
    if(len>0){
      const unsigned int end  = row+len-1;
      const unsigned int size = len; 
      result.Resize(size,column,false);
      
      if(row<this->row){
        const unsigned int k = (end+1<=this->row?end+1:this->row);  
        
        for (unsigned int j = 0; j < column; j++)
          for (unsigned int i = row; i < k; i++)
            result._[(i-row)*column+j] = _[i*column+j];
        for (unsigned int j = 0; j < column; j++)
          for (unsigned int i = k; i < end+1; i++)
           result._[(i-row)*column+j] = R_ZERO;            
      }else{
        result.Zero();
      }
    }else{
      result.Resize(0,0,false);
    }
    return result;     
  }

  /**
   * \brief Get a matrix spanning several columns of the matrix
   * \param row      The starting column
   * \param len      The number of columns
   * \return         The resulting matrix
   */  
  inline Matrix GetColumnSpace(const unsigned int col, const unsigned int len) const  
  {
    if(len>0){
      Matrix result(row,len,false);    
      return GetColumnSpace(col,len,result);
    }else
      return Matrix();     
  }

  /**
   * \brief Get a matrix spanning several columns of the matrix
   * \param row      The starting column
   * \param len      The number of columns
   * \param result   The target matrix
   * \return         The resulting matrix
   */  
  inline Matrix& GetColumnSpace(const unsigned int col, const unsigned int len, Matrix &result) const
  {
    if(len>0){
      const unsigned int end  = col+len-1;    
      const unsigned int size = len; 
      result.Resize(row,size,false);
      
      if(col<column){
        const unsigned int k = (end+1<=column?end+1:column);  
        
        for (unsigned int i = col; i < k; i++)
          for (unsigned int j = 0; j < row; j++)
            result._[j*size+(i-col)] = _[j*column+i];
        for (unsigned int i = k; i < end+1; i++)
          for (unsigned int j = 0; j < row; j++)
           result._[j*size+(i-col)] = R_ZERO;            
      }else{
        result.Zero();
      }
    }else{
      result.Resize(0,0,false);
    }
    return result;     
  }

   /**
   * \brief Assign a value to a matrix row
   * \param value    The value
   * \param row      The row
   */  
  inline Matrix& SetRow(const REALTYPE value, const unsigned int row)
  {
    if(row<this->row){    
      for (unsigned int i = 0; i < column; i++)
        _[row*column+i] = value; 
    }
    return *this;     
  }

   /**
   * \brief Assign a vector to a matrix row
   * \param vector   The input vector
   * \param row      The row
   */  
  inline Matrix& SetRow(const Vector &vector, const unsigned int row)
  {
    if(row<this->row){    
      const unsigned int ki = (column<=vector.row?column:vector.row);
      for (unsigned int i = 0; i < ki; i++)
        _[row*column+i] = vector._[i]; 
    }
    return *this;     
  }

  /**
   * \brief Assign a vector to a matrix column
   * \param vector   The input vector
   * \param col      The column
   */  
  inline Matrix& SetColumn(const Vector &vector, const unsigned int col)
  {
    if(col<this->column){    
      const unsigned int kj = (row<=vector.row?row:vector.row);
      for (unsigned int j = 0; j < kj; j++)
        _[j*column+col] = vector._[j];
    }
    return *this;
  }

  /**
   * \brief Assign a matrix to the current matrix rows
   * \param vector   The input matrix
   * \param row      The starting row
   */  
  inline Matrix& SetRowSpace(const Matrix &matrix, const unsigned int row)
  {
    if(row<this->row){
      const unsigned int ki = (column<=matrix.column?column:matrix.column);
      const unsigned int kj = (row+matrix.row<=this->row?row+matrix.row:this->row);
      for (unsigned int j = row; j < kj; j++)
        for (unsigned int i = 0; i < ki; i++)
          _[j*column+i] = matrix._[(j-row)*matrix.column+i]; 
    }
    return *this;     
  }

  /**
   * \brief Assign a matrix to the current matrix columns
   * \param vector   The input matrix
   * \param col      The starting column
   */  
  inline Matrix& SetColumnSpace(const Matrix &matrix, const unsigned int col)
  {
    if(col<this->column){    
      const unsigned int kj = (row<=matrix.row?row:matrix.row);
      const unsigned int ki = (col+matrix.column<=this->column?col+matrix.column:this->column);
      for (unsigned int j = 0; j < kj; j++)
        for (unsigned int i = col; i < ki; i++)
          _[j*column+i] = matrix._[j*matrix.column+(i-col)];
    }
    return *this;
  }


  inline Matrix GetRowSpace(const Vector& ids, Matrix &result) const
  {
    int s = ids.Size();
    IndicesVector id; 
    for(int i=0;i<s;i++) id.push_back(int(ROUND(ids.At(i))));
    return GetRowSpace(id,result);
  }

  /**
   * \brief Get a matrix spanning several rows of the matrix
   * \param ids      The indices of the desired rows
   * \return         The resulting matrix
   */  
  inline Matrix GetRowSpace(const IndicesVector& ids) const
  {
    Matrix result(ids.size(),column);
    return GetRowSpace(ids,result);
  }

  inline Matrix& GetRowSpace(const IndicesVector& ids, Matrix &result) const
  {
    const unsigned int k=ids.size();
    result.Resize(k,column,false);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g      = ids[i];
      const unsigned int offset = i*column;
      if(g<row){
        for(unsigned int j=0;j<column;j++)
          result._[offset+j] = _[g*column+j];
      }else{
        for(unsigned int j=0;j<column;j++)
          result._[offset+j] = R_ZERO;
      }
    }
    return result;
  }

  inline Matrix GetColumnSpace(const Vector& ids, Matrix &result) const
  {
    int s = ids.Size();
    IndicesVector id; 
    for(int i=0;i<s;i++) id.push_back(int(ROUND(ids.At(i))));
    return GetColumnSpace(id,result);
  }
  /**
   * \brief Get a matrix spanning several columns of the matrix
   * \param ids      The indices of the desired columns
   * \return         The resulting matrix
   */  
  inline Matrix GetColumnSpace(const IndicesVector& ids) const
  {
    Matrix result(row,ids.size());
    return GetColumnSpace(ids,result);
  }

  inline Matrix& GetColumnSpace(const IndicesVector& ids, Matrix &result) const
  {
    const unsigned int k=ids.size();
    result.Resize(row,k);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g = ids[i];
      if(g<column){
        for(unsigned int j=0;j<row;j++)
          result._[j*k+i] = _[j*column+g];
      }else{
        for(unsigned int j=0;j<row;j++)
          result._[j*k+i] = R_ZERO;        
      }
    }
    return result;     
  }

  inline Matrix GetMatrixSpace(const Vector& rowIds, const Vector& colIds, Matrix &result) const
  {
    int r = rowIds.Size();
    int c = colIds.Size();
    IndicesVector idr,idc; 
    for(int i=0;i<r;i++) idr.push_back(int(ROUND(rowIds.At(i))));
    for(int i=0;i<c;i++) idc.push_back(int(ROUND(colIds.At(i))));
    return GetMatrixSpace(idr,idc,result);
  }
  /**
   * \brief Get a matrix spanning several rows and columns of the matrix
   * \param rowIds      The indices of the desired rows
   * \param colIds      The indices of the desired columns
   * \return            The resulting matrix
   */  
  inline Matrix GetMatrixSpace(const IndicesVector& rowIds,const IndicesVector& colIds) const
  {
    Matrix result(rowIds.size(),colIds.size());
    return GetMatrixSpace(rowIds,colIds,result);
  }

  inline Matrix& GetMatrixSpace(const IndicesVector& rowIds,const IndicesVector& colIds, Matrix &result) const
  {
    const unsigned int k1=rowIds.size();
    const unsigned int k2=colIds.size();
    result.Resize(k1,k2);
    for(unsigned int i=0;i<k1;i++){
      const unsigned int g1 = rowIds[i];
      if(g1<row){
        for(unsigned int j=0;j<k2;j++){      
          const unsigned int g2 = colIds[j];
          if(g2<column){
            result._[i*k2+j] = _[g1*column+g2];            
          }else{
            result._[i*k2+j] = R_ZERO;
          }
        }
      }else{
        for(unsigned int j=0;j<k2;j++)
          result._[i*k2+j] = R_ZERO;
      }
    }
    return result;     
  }

  /**
   * \brief Set a specified column set of the matrix with columns of the source matrix
   * \param ids      The indices of the desired columns
   * \param source   The source matrix
   * \return         The resulting matrix
   */  
  inline Matrix& SetColumnSpace(const IndicesVector& ids, const Matrix &source)
  {
    const unsigned int k      = MIN(ids.size(),source.column);
    const unsigned int r      = MIN(row,source.row);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g = ids[i];
      if(g<column){
        for(unsigned int j=0;j<r;j++)
          _[j*column+g] = source._[j*source.column+i];
      }
    }
    return *this;     
  }


  inline Matrix& InsertSubRow(unsigned int startRow, unsigned int startColumn, 
                              const Matrix& matrix, 
                              unsigned int matrixRow,
                              unsigned int matrixStartColumn, unsigned int matrixColumnLength){

    // Check submatrix boundaries
    if((matrixRow         >= matrix.row)||
       (matrixStartColumn >= matrix.column)) return *this;
    
    // Check matrix boundaries
    if((startRow    >= row)||
       (startColumn >= column)) return *this;

    if(matrixStartColumn+matrixColumnLength > matrix.column) matrixColumnLength = matrix.column-matrixStartColumn;

    if(startColumn+matrixColumnLength > column) matrixColumnLength = column-startColumn;
     
    unsigned int rowOffset       = startRow*column;
    unsigned int matrixRowOffset = matrixRow*matrix.column;
    unsigned int colOffset       = startColumn;
    unsigned int matrixColOffset = matrixStartColumn;
    for(unsigned int j=0;j<matrixColumnLength;j++){
      _[rowOffset+colOffset] = matrix._[matrixRowOffset+matrixColOffset];
      colOffset++;
      matrixColOffset++;
    }
    return *this;
  } 

  inline Matrix& InsertSubColumn(unsigned int startRow, unsigned int startColumn, 
                                 const Matrix& matrix, 
                                 unsigned int matrixStartRow, unsigned int matrixRowLength,
                                 unsigned int matrixColumn){

    if((matrixStartRow >= matrix.row)||
       (matrixColumn   >= matrix.column)) return *this;
    if((startRow    >= row)||
       (startColumn >= column)) return *this;
    
    if(matrixStartRow+   matrixRowLength    > matrix.row)    matrixRowLength    = matrix.row   -matrixStartRow;
    
    if(startRow+   matrixRowLength    > row)    matrixRowLength    = row   -startRow;
     
    unsigned int rowOffset       = startRow*column;
    unsigned int matrixRowOffset = matrixStartRow*matrix.column;
    for(unsigned int i=0;i<matrixRowLength;i++){
      _[rowOffset+startColumn] = matrix._[matrixRowOffset+matrixColumn];
      rowOffset       +=column;
      matrixRowOffset +=matrix.column;
    }      
    return *this;
  } 

  inline Matrix& InsertSubMatrix(unsigned int startRow, unsigned int startColumn, 
                                 const Matrix& matrix, 
                                 unsigned int matrixStartRow, unsigned int matrixRowLength,
                                 unsigned int matrixStartColumn, unsigned int matrixColumnLength){

    // Check submatrix boundaries
    if((matrixStartRow    >= matrix.row)||
       (matrixStartColumn >= matrix.column)) return *this;
    
    // Check matrix boundaries
    if((startRow    >= row)||
       (startColumn >= column)) return *this;

    if(matrixStartRow+   matrixRowLength    > matrix.row)    matrixRowLength    = matrix.row   -matrixStartRow;
    if(matrixStartColumn+matrixColumnLength > matrix.column) matrixColumnLength = matrix.column-matrixStartColumn;

    if(startRow+   matrixRowLength    > row)    matrixRowLength    = row   -startRow;
    if(startColumn+matrixColumnLength > column) matrixColumnLength = column-startColumn;
     
    unsigned int rowOffset       = startRow*column;
    unsigned int matrixRowOffset = matrixStartRow*matrix.column;
    for(unsigned int i=0;i<matrixRowLength;i++){
      unsigned int colOffset       = startColumn;
      unsigned int matrixColOffset = matrixStartColumn;
      for(unsigned int j=0;j<matrixColumnLength;j++){
        _[rowOffset+colOffset] = matrix._[matrixRowOffset+matrixColOffset];
        colOffset++;
        matrixColOffset++;
      }
      rowOffset       +=column;
      matrixRowOffset +=matrix.column;
    }      
    return *this;
  } 
  
  inline Matrix operator - () const
  {
    Matrix result(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = -_[j*column+i];
    return result;
  }
  
  inline Matrix& SMinus()
  {
    for (unsigned int j = 0; j < row*column; j++)
        _[j] = -_[j];
    return *this;
  }
  
  
  inline virtual Matrix& operator = (const Matrix &matrix)
  {
    Resize(matrix.row,matrix.column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = matrix._[j*column+i];
    return *this;    
  }

  inline Matrix& operator += (const Matrix &matrix)
  {
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++)
      for (unsigned int i = 0; i < ki; i++)
        _[j*column+i] += matrix._[j*column+i];
    return *this;
  }

  inline Matrix& operator -= (const Matrix &matrix)
  {
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++)
      for (unsigned int i = 0; i < ki; i++)
        _[j*column+i] -= matrix._[j*column+i];
    return *this;
  }

  /// Element-wise multiplication
  inline Matrix& operator ^= (const Matrix &matrix)
  {
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++)
      for (unsigned int i = 0; i < ki; i++)
        _[j*column+i] *= matrix._[j*column+i];
    return *this;
  }

  /// Element-wise division
  inline Matrix& operator /= (const Matrix &matrix)
  {
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++)
      for (unsigned int i = 0; i < ki; i++)
        _[j*column+i] /= matrix._[j*column+i];
    return *this;
  }

  inline Matrix& operator += (REALTYPE scalar)
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] += scalar;
    return *this;
  }

  inline Matrix& operator -= (REALTYPE scalar)
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] -= scalar;
    return *this;
  }

  inline Matrix& operator *= (REALTYPE scalar)
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] *= scalar;
    return *this;
  }

  inline Matrix& operator /= (REALTYPE scalar)
  {
    scalar = R_ONE/scalar;
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] *= scalar;
    return *this;
  }
  
  inline Matrix operator + (const Matrix &matrix) const
  {
    Matrix result(row,column,false);  
    return Add(matrix,result);
  }
  
  inline Matrix& Add(const Matrix &matrix, Matrix &result) const
  {   
    result.Resize(row,column,false);
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < ki; i++)
        result._[j*column+i] = _[j*column+i] + matrix._[j*column+i];
      for (unsigned int i = ki; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    }
    for (unsigned int j = kj; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    return result;
  }
  
  inline Matrix operator - (const Matrix &matrix) const
  {
    Matrix result(row,column,false);  
    return Sub(matrix,result);
  }
  
  inline Matrix& Sub(const Matrix &matrix, Matrix &result) const
  {   
    result.Resize(row,column,false);
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < ki; i++)
        result._[j*column+i] = _[j*column+i] - matrix._[j*column+i];
      for (unsigned int i = ki; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    }
    for (unsigned int j = kj; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    return result;
  }
  
  /// Element-wise multiplication
  inline Matrix operator ^ (const Matrix &matrix) const
  {
    Matrix result(row,column,false);  
    return PMult(matrix,result);
  }
  
  /// Element-wise multiplication
  inline Matrix& PMult(const Matrix &matrix, Matrix &result) const
  {   
    result.Resize(row,column,false);
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < ki; i++)
        result._[j*column+i] = _[j*column+i] * matrix._[j*column+i];
      for (unsigned int i = ki; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    }
    for (unsigned int j = kj; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    return result;
  }

  inline Matrix operator ^ (const Vector &vector) const
  {
    Matrix result(row,column,false);  
    return PMult(vector,result);
  }
  
  /// Row-Wise multiplication .. 
  inline Matrix& PMult(const Vector &vector, Matrix &result) const
  {   
    result.Resize(row,column,false);
    const unsigned int kj = (row<=vector.row?row:vector.row);
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] * vector._[j];
    }
    for (unsigned int j = kj; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    return result;
  }
  
  /// Element-wise division
  inline Matrix operator / (const Matrix &matrix) const
  {
    Matrix result(row,column,false);  
    return PDiv(matrix,result);
  }
  
  /// Element-wise division
  inline Matrix& PDiv(const Matrix &matrix, Matrix &result) const
  {   
    result.Resize(row,column,false);
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < ki; i++)
        result._[j*column+i] = _[j*column+i] / matrix._[j*column+i];
      for (unsigned int i = ki; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    }
    for (unsigned int j = kj; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    return result;
  }

  inline Matrix operator + (REALTYPE scalar) const
  {
    Matrix result(row,column,false);  
    return Add(scalar,result);    
  }

  inline Matrix& Add(REALTYPE scalar, Matrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] + scalar;    
    return result;
  }

  inline Matrix operator - (REALTYPE scalar) const
  {
    Matrix result(row,column,false);  
    return Sub(scalar,result);    
  }
  
  inline Matrix& Sub(REALTYPE scalar, Matrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] - scalar;    
    return result;
  }

  inline Matrix operator * (REALTYPE scalar) const
  {
    Matrix result(row,column,false);  
    return Mult(scalar,result);    
  }

  inline Matrix& Mult(REALTYPE scalar, Matrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] * scalar;    
    return result;
  }


  inline Matrix operator / (REALTYPE scalar) const
  {
    Matrix result(row,column,false);  
    return Div(scalar,result);    
  }

  inline Matrix& Div(REALTYPE scalar, Matrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] / scalar;    
    return result;    
  }

  inline bool operator == (const Matrix& matrix) const
  {
    if((row!=matrix.row)||(column!=matrix.column)) return false;
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        if(_[j*column+i] != matrix._[j*column+i]) return false;
    return true;
  }

  inline bool operator != (const Matrix& matrix) const
  {
    return !(*this ==  matrix);
  }

  /// Matrix multiplication against a vector
  inline Vector operator * (const Vector &vector) const
  {
    Vector result(row,false);  
    return Mult(vector,result);    
  }  

  /// Matrix multiplication against a vector
  inline Vector Mult(const Vector &vector) const
 {
    Vector result(row,false);  
    return Mult(vector,result);    
  }

  /// Matrix multiplication against a vector
  inline Vector& Mult(const Vector &vector, Vector &result) const
  {
    result.Resize(row,false);
    const unsigned int ki = (column<=vector.row?column:vector.row);
    for (unsigned int j = 0; j < row; j++){
      result._[j] = R_ZERO;
      for (unsigned int i = 0; i < ki; i++)
        result._[j] += _[j*column+i] * vector._[i];
    }
    return result;
  }

  /// Transpose Matrix multiplication against a vector
  inline Vector MultTranspose(const Vector &vector) const
  {
    Vector result(row,false);  
    return MultTranspose(vector,result);    
  }
  
  /// Transpose Matrix multiplication against a vector
  inline Vector& MultTranspose(const Vector &vector, Vector &result) const
  {
    result.Resize(column,false);
    const unsigned int ki = (row<vector.row?row:vector.row);
    for (unsigned int j = 0; j < column; j++){
      result._[j] = R_ZERO;
      for (unsigned int i = 0; i < ki; i++){      
        result._[j] += _[i*column+j] * vector._[i];
      }
    }
    return result;
  }

  /// Transpose then Multiply by the matrix : 
  // result = this^T * matrix (avoiding a useless call to .Transpose() )
  inline Matrix& MultTranspose(const Matrix &matrix, Matrix &result) const
  {
    result.Resize(column,matrix.column,false);
    const unsigned int rrow = result.row;
    const unsigned int rcol = result.column;
    const unsigned int kk = (row<=matrix.row?row:matrix.row);
    for (unsigned int j = 0; j < rrow; j++){
      for (unsigned int i = 0; i < rcol; i++){
        result._[j*rcol+i] = 0.0f;
        for(unsigned int k = 0; k< kk; k++)    
          result._[j*rcol+i] += _[k*column+j] * matrix._[k*rcol+i];
      }
    }       
    return result;
  }

  inline Matrix MultRow(const Vector &vector){
    Matrix result(row,column);
    return MultRow(vector,result);      
  } 

  inline Matrix MultColumn(const Vector &vector){
    Matrix result(row,column);
    return MultColumn(vector,result);      
  } 

  /// Multiplication of each ith row by the ith element of a vector
  inline Matrix& MultRow(const Vector &vector, Matrix& result) 
  {    
    result.Resize(row,column,false);
    const unsigned int ki = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < ki; i++){
      REALTYPE v = vector._[i];
      const unsigned int offset = i*column;  
      for (unsigned int j = 0; j < column; j++){
        result._[offset+j] = _[offset+j] * v;
      }
    }
    for (unsigned int i = ki; i < row; i++){
      const unsigned int offset = i*column;  
      for (unsigned int j = 0; j < column; j++){
        result._[offset+j] = R_ZERO;
      }
    }    
    return result;
  }

  /// Multiplication of each ith column by the ith element of a vector
  inline Matrix& MultColumn(const Vector &vector, Matrix& result) 
  {    
    result.Resize(row,column,false);
    const unsigned int ki = (column<=vector.row?column:vector.row);
    for (unsigned int i = 0; i < ki; i++){
      REALTYPE v = vector._[i];
      unsigned int offset = 0;  
      for (unsigned int j = 0; j < row; j++){
        result._[offset+i] = _[offset+i] * v;
        offset+=column;
      }
    }
    for (unsigned int i = ki; i < column; i++){
      unsigned int offset = 0;  
      for (unsigned int j = 0; j < row; j++){
        result._[offset+i] = R_ZERO;
        offset+=column;
      }
    }    
    return result;
  }

  /// Self multiplication of each ith row by the ith element of a vector
  inline Matrix& SMultRow(const Vector &vector) 
  {    
    const unsigned int ki = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < ki; i++){
      REALTYPE v = vector._[i];
      const unsigned int offset = i*column;  
      for (unsigned int j = 0; j < column; j++){
        _[offset+j] *= v;
      }
    }
    for (unsigned int i = ki; i < row; i++){
      const unsigned int offset = i*column;  
      for (unsigned int j = 0; j < column; j++){
        _[offset+j] = R_ZERO;
      }
    }    
    return *this;
  }


  /// Matrix multiplication
  inline Matrix operator * (const Matrix &matrix) const  
  {
    Matrix result(row,matrix.column,false);  
    return Mult(matrix,result);
  }  

  /*
  /// Matrix multiplication
  inline Matrix& Mult(const Matrix &matrix, Matrix &result) const
  {
    result.Resize(row,matrix.column,false);
    const unsigned int rrow = result.row;
    const unsigned int rcol = result.column;
    const unsigned int kk = (column<=matrix.row?column:matrix.row);
    for (unsigned int j = 0; j < rrow; j++){
      for (unsigned int i = 0; i < rcol; i++){
        result._[j*rcol+i] = R_ZERO;
        for(unsigned int k = 0; k< kk; k++)    
          result._[j*rcol+i] += _[j*column+k] * matrix._[k*rcol+i];
      }
    }    
    return result;
  }
  */
  /// Matrix multiplication
  inline Matrix& Mult(const Matrix &matrix, Matrix &result) const
  {
    result.Resize(row,matrix.column,false);
    result.Zero();
    //const unsigned int rrow = result.row;
    const unsigned int rcol = result.column;
    const unsigned int kk = (column<=matrix.row?column:matrix.row);

    REALTYPE *cP1   = _; 
    REALTYPE *eP1   = cP1 + row*column;    
    REALTYPE *cD    = result._;

    while(cP1!=eP1){
        REALTYPE *currP1  = cP1; 
        REALTYPE *endP1   = currP1 + kk;
        REALTYPE *currP2  = matrix._; 
        while(currP1!=endP1){
            REALTYPE *currPD  = cD; 
            REALTYPE  curr1   = *currP1;
            REALTYPE *endP2   = currP2 + rcol;    
            while(currP2!=endP2){
                (*currPD++) += curr1 * (*(currP2++));
            }
            currP1++;
        }
        cD  += rcol;
        cP1 += column;
    }        
    return result;
  }

  /// Set a diagonal matrix given a vector of diagonal elements
  inline Matrix& Diag(const Vector &vector)
  {
    Resize(vector.row,vector.row,false);
    //const unsigned int k = (row>column?column:row);
    //const unsigned int k2 = (k>vector.row?vector.row:k);
    Zero();
    for (unsigned int i = 0; i < row; i++)
      _[i*column+i] = vector._[i];
    return *this;    
  }

  /// Set a random matrix with value uniformly distributed between 0 and 1
  inline Matrix& Random(){
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = RND(R_ONE);    
    return *this;    
  }

  /// Return the transpose of a matrix
    inline Matrix Transpose() const
    {
    Matrix result(column,row,false);
    return Transpose(result);    
    }

  /// Computr the transpose of a matrix
  inline Matrix& Transpose(Matrix &result) const
  {    
    result.Resize(column,row,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[i*row+j] = _[j*column+i];
    return result;    
  }

  /// Compute the self transpose 
  inline Matrix& STranspose()
  { 
    if(column!=row){
      Matrix tmp(*this);
      return tmp.Transpose(*this);        
    } 
    
    REALTYPE tmp;
    for (unsigned int j = 0; j < row-1; j++){
      for (unsigned int i = j+1; i < row; i++){
        tmp           = _[j*column+i];
        _[j*column+i] = _[i*column+j];
        _[i*column+j] = tmp;
      }
    }
    return *this;    
  }

  /// Return the vertical concatenation with another matrix
  inline Matrix VCat(const Matrix& matrix)
  {
    Matrix result;
    return VCat(matrix,result);    
  }
  
  /// Return the vertical concatenation with another matrix in result
  inline Matrix& VCat(const Matrix& matrix, Matrix & result)
  {
    unsigned int k1 = (column>matrix.column?column:matrix.column);
    result.Resize(row+matrix.row,k1,false);
    for (unsigned int j = 0; j < row; j++){
      for (unsigned int i = 0; i < column; i++)
        result._[j*k1+i] = _[j*column+i];
      for (unsigned int i = column; i < k1; i++)
        result._[j*k1+i] = R_ZERO;
    }
    for (unsigned int j = 0; j < matrix.row; j++){
      for (unsigned int i = 0; i < matrix.column; i++)
        result._[(row+j)*k1+i] = matrix._[j*matrix.column+i];
      for (unsigned int i = matrix.column; i < k1; i++)
        result._[(row+j)*k1+i] = R_ZERO;
    }
    return result;
  }

  /// Return the horizontal concatenation with another matrix
  inline Matrix HCat(const Matrix& matrix)
  {
    Matrix result;
    return VCat(matrix,result);    
  }
  
  /// Return the horizontal concatenation with another matrix in result  
  inline Matrix& HCat(const Matrix& matrix, Matrix & result)
  {
    unsigned int k1 = (row>matrix.row?row:matrix.row);
    unsigned int k2 = column+matrix.column;
    result.Resize(k1,k2,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*k2+i] = _[j*column+i];
    for (unsigned int j = row; j < k1; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*k2+i] = R_ZERO;

    for (unsigned int j = 0; j < matrix.row; j++)
      for (unsigned int i = 0; i < matrix.column; i++)
        result._[j*k2+i+column] = matrix._[j*matrix.column+i];
    for (unsigned int j = matrix.row; j < k1; j++)
      for (unsigned int i = 0; i < matrix.column; i++)
        result._[j*k2+i+column] = R_ZERO;
    
    return result;
  }

  
  
  /**
   * \brief Inverse the matrix (One should check, using IsInverseOk() if the inversion succeeded)
   * \param determinant If not NULL, allows to get the determinant of the matrix
   * \return         The resulting inverse matrix
   */  
  inline Matrix Inverse(REALTYPE *determinant=NULL) const
  {
    Matrix result;
    return Inverse(result,determinant);
  }  

  /**
   * \brief Inverse the matrix (One should check, using IsInverseOk() if the inversion succeeded)
   * \param result        A reference to the resulting inverse matrix
   * \param determinant   If not NULL, allows to get the determinant of the matrix
   * \param baseDetFactor Don't touch this param, please...
   * \return         The resulting inverse matrix
   */  
  inline Matrix& Inverse(Matrix &result, REALTYPE *determinant=NULL, REALTYPE baseDetFactor = R_ONE, Matrix *work=NULL) const
  {
    bInverseOk = TRUE;       
    if(row==column){ // Square matrix
      
      Matrix *MM = work;    
      if(MM==NULL){
        MM = new Matrix(*this);
      }else{
        MM->Set(_,row,column);
      }
      
      if(determinant!=NULL) *determinant = baseDetFactor;
      result.Resize(row,column,false);
      const unsigned int n = row;
      //Matrix MM(*this);
      MM->Set(_,row,column);
      result.Identity();
      for(unsigned int i=0;i<n;i++){
        REALTYPE pivot = MM->_[i*column+i]; 
        if(fabs(pivot)<=EPSILON){
          for(unsigned int j=i+1;j<n;j++){
            if((pivot = MM->_[j*column+i])!=R_ZERO){
              MM->SwapRow(i,j);
              result.SwapRow(i,j);
              break;  
            }
          }            
          if(fabs(pivot)<=EPSILON){
            bInverseOk = FALSE;
            if(determinant!=NULL) *determinant = R_ZERO;
            return result;
          }                      
        }
        if(determinant!=NULL) *determinant *= pivot;
        pivot = R_ONE/pivot;
        for(unsigned int j=0;j<n;j++){
          MM->_[i*column+j]   *= pivot;
          result._[i*column+j] *= pivot;
        }
        for(unsigned int k=0;k<n;k++){
          if(k!=i){
            const REALTYPE mki = MM->_[k*column+i];
            for(unsigned int j=0;j<n;j++){
               MM->_[k*column+j]   -= MM->_[i*column+j]   *mki;
               result._[k*column+j] -= result._[i*column+j] *mki;              
            }            
          }
        }
      }        
    }else{ // Moore-Penrose pseudo inverse
      if(determinant!=NULL) *determinant = R_ZERO;
      if(row>column){ // (JtJ)^(-1)Jt
        Matrix MT,SQ,SQInv;
        Transpose(MT);
        MT.Mult(*this,SQ);
        SQ.Inverse(SQInv);
        SQInv.Mult(MT,result); 
      }else{ // Jt(JJt)^(-1)
        Matrix MT,SQ,SQInv;
        Transpose(MT);
        Mult(MT,SQ);
        SQ.Inverse(SQInv);
        MT.Mult(SQInv,result);         
      }
    }
    return result;    
  }

  /// After an inverse operation, tell if it was a success
  inline static int IsInverseOk(){
    return bInverseOk;
  }

  /// Echange two row of the matrix
  inline Matrix& SwapRow(unsigned int j1, unsigned int j2){
    if((j1<row)&&(j2<row)){
      REALTYPE tmp;
      for (unsigned int i = 0; i < column; i++){
        tmp            = _[j1*column+i];
        _[j1*column+i] = _[j2*column+i];
        _[j2*column+i] = tmp;        
      }        
    }
    return *this; 
  }
 
  /// Echange two columns of the matrix
  inline Matrix& SwapColumn(unsigned int i1, unsigned int i2){
    if((i1<column)&&(i2<column)){
      REALTYPE tmp;
      for (unsigned int j = 0; j < row; j++){
        tmp            = _[j*column+i1];
        _[j*column+i1] = _[j*column+i2];
        _[j*column+i2] = tmp;        
      }        
    }
    return *this; 
  }
  
  /// Do the square root on each matrix element
  inline Matrix& Sqrt(){
    for (unsigned int j = 0; j < row; j++){
      for (unsigned int i = 0; i < column; i++){
        _[j*column+i] = sqrt(fabs(_[j*column+i]));
      }
    }  
    return *this;
  }
  
  /// Print the matrix of stdout
  void Print() const
  {
    std::cout << "Matrix " <<row<<"x"<<column<<std::endl;;
    for (unsigned int j = 0; j < row; j++){
      for (unsigned int i = 0; i < column; i++)
        std::cout << _[j*column+i] <<" ";
      std::cout << std::endl;
    }
  }
  
  /**
   * \brief Do a QR Decomposition of the matrix, where A=QR, and Q is a base and ortonornal matrix and R a triangular matrix 
   * \param Q The resulting Q
   * \param R The resulting R
   */  
  void QRDecomposition(Matrix & Q, Matrix & R){
    Matrix QR;
    QRDecomposition(Q,R,QR);
  }
  
  /**
   * \brief Do a QR Decomposition of the matrix, where A=QR, and Q is a base and ortonornal matrix and R a triangular matrix 
   * \param Q The resulting Q
   * \param R The resulting R
   * \param QR A temporary processing matrix
   */  
  void QRDecomposition(Matrix & Q, Matrix & R, Matrix & QR){    
    if(row>=column){
      QR = *this;
    }else{
      Transpose(QR);
    }
    unsigned int m = QR.row;
    unsigned int n = QR.column;
    Vector RDiag(n);
        
    for(unsigned int k=0;k<n;k++){
      REALTYPE nrm = R_ZERO;
      for (unsigned int i = k; i < m; i++) {
        nrm = hypot_s(nrm, QR(i,k));
      }
      if (nrm != R_ZERO) {
        if(QR(k,k)<R_ZERO){
          nrm = -nrm;
        }
        for (unsigned int i = k; i < m; i++) {
            QR(i,k) /= nrm;
        }
        QR(k,k)+=R_ONE;

        for (unsigned int j = k + 1; j < n; j++) {
          REALTYPE s = R_ZERO;
          for (unsigned int i = k; i < m; i++) {
              s += QR(i,k) * QR(i,j);
          }
          s = -s / QR(k,k);
          for (unsigned int i = k; i < m; i++) {
              QR(i,j) += s * QR(i,k);
          }
        }
      }
      RDiag(k) = -nrm;
    }
    
    R.Resize(n,n);
    for(unsigned int i = 0; i < n; i++) {
      for(unsigned int j = 0; j < n; j++) {
        if(i<j){
          R(i,j) = QR(i,j); 
        }else if(i==j){
          R(i,j) = RDiag(i);
        }else{
          R(i,j) = R_ZERO;
        }
      }
    }

    Q.Resize(m,n);
    for(int k= n-1;k>=0;k--){
      for(unsigned int i = 0; i < m; i++) {
        Q(i,k) = R_ZERO;
      }
      Q(k,k)=R_ONE;
      for(unsigned int j = k; j < n; j++) {
        if(QR(k,k)!=R_ZERO){
          REALTYPE s = R_ZERO;
          for(unsigned int i = k; i < m; i++) {
            s += QR(i,k) * Q(i,j);
          }
          s = -s / QR(k,k);
          for(unsigned int i = k; i < m; i++) {
            Q(i,j) = Q(i,j) + s*QR(i,k);
          }
        }
      }       
    }
  }


  /**
   * \brief Perform a one-shot eigen values and vectors decomposition  
   * \param eigenvalues  the resulting eigen values
   * \param eigenVectors the resulting eigen vectors
   * \param maxIter      the maximum number of iteration
   */  
  void EigenValuesDecomposition(Vector &eigenValues, Matrix& eigenVectors,int maxIter = 30){
    Matrix tri;
    Tridiagonalize(tri,eigenVectors);
    tri.TriEigen(eigenValues, eigenVectors,maxIter);
  }


  
  /**
   * \brief Do a compressed tridiagonalization of the matrix (Use TriDiag() to recover a normal form of the matrix) 
   * \param result The resulting matrix. It has three rows, one for the diagonal element, and the others of the up and down diagonal elements 
   * \param trans  The resulting transformation matrix
   */  
  Matrix& Tridiagonalize(Matrix &result,Matrix &trans){
    result.Resize(2,row);
    Matrix A(*this);
    trans = A;
    if(row==0) return result;
    
    
    int n = row;
    int l,k,j,i;
    REALTYPE scale,hh,h,g,f;
    for(i=n-1;i>=1;i--){
      l = i-1;
      h = scale = R_ZERO;
      if(l>0){
        for(k=0;k<=l;k++)
          scale += fabs(A._[i*column+k]);
        if(scale == R_ZERO){
          result._[column+i] = A._[i*column+l];
        }else{
          for(k=0;k<=l;k++){
            A._[i*column+k] /= scale;
            h += A._[i*column+k]*A._[i*column+k];
          }
          f= A._[i*column+l];
          g=(f>=R_ZERO?-sqrt(h):sqrt(h));
          result._[column+i] = scale*g;
          h-=f*g;
          A._[i*column+l] = f-g;
          f=R_ZERO;
          for(j=0;j<=l;j++){
            A._[j*column+i] = A._[i*column+j] /h;
            g=R_ZERO;
            for(k=0;k<=j;k++)
              g+=  A._[j*column+k]*A._[i*column+k];
            for(k=j+1;k<=l;k++)
              g+=  A._[k*column+j]*A._[i*column+k];
            result._[column+j] = g/h;
            f+= result._[column+j]*A._[i*column+j];
          }
          hh = f/(h+h);
          for(j=0;j<=l;j++){
            f = A._[i*column+j];
            result._[column+j]=g=result._[column+j]-hh*f;
            for(k=0;k<=j;k++)
              A._[j*column+k] -= (f*result._[column+k]+g*A._[i*column+k]);            
          }             
        }
      }else{
        result._[column+i] = A._[i*column+l];        
      }
      result._[i]=h;  
    }
    result._[0]=R_ZERO;  
    result._[column+0]=R_ZERO;
    for(i=0;i<n;i++){
      l=i-1;
      if(result._[i]){
        for(j=0;j<=l;j++){
          g=R_ZERO;
          for(k=0;k<=l;k++)
            g+= A._[i*column+k]*A._[k*column+j]; 
          for(k=0;k<=l;k++)
            A._[k*column+j] -= g*A._[k*column+i]; 
        }  
      }
      result._[i] = A._[i*column+i];
      A._[i*column+i] = R_ONE;
      for(j=0;j<=l;j++) A._[j*column+i]=A._[i*column+j]=R_ZERO;
    }
    trans = A;
    return result;
  }
    
  /**
   * \brief Produce a tridiagnal matrix from the compressed tridiagonal matrix resulting from Tridiaglonalize()
   * \param tri The compressed tridiagonal matrix 
   */  
  Matrix& TriDiag(Matrix &tri){
    Resize(tri.ColumnSize(),tri.ColumnSize(),false);
    Zero();
    for(unsigned int i=0;i<column;i++){
      _[i*(column+1)] = tri._[i];
      if(i<column-1)
        _[i*(column+1)+1] = tri._[column+i+1];
      if(i>0)  
        _[i*(column+1)-1] = tri._[column+i];
    }
    return *this;
  }
  
  /**
   * \brief Compute the eigen values and eigen vectors from a compressed tridiagonal matrix resulting from Tridiaglonalize()
   * \param eigenvalues  the resulting eigern values
   * \param eigenVectors the resulting eigen vectors
   */  
  int TriEigen(Vector &eigenValues, Matrix& eigenVectors,int maxIter = 120){
    bInverseOk = true;
    if(row!=2) return -1;
    if(column==0) return -1;
    GetRow(0,eigenValues);
    Vector e;
    GetRow(1,e);
    
    const int n = column;
    int m,l,iter,i,k;
    REALTYPE s,r,p,g,f,dd,c,b;
    int cumIter = 0;
    for(i=1;i<n;i++) e._[i-1] = e._[i];
    e._[n-1] = R_ZERO;

    for(l=0;l<n;l++){
      iter=0;
      do{
        for(m=l;m<=n-2;m++){
          dd = fabs(eigenValues._[m])+fabs(eigenValues._[m+1]);
          if((REALTYPE)(fabs(e._[m])+dd) == dd) break;  
        }
        if(m!=l){
          if(iter++==maxIter) {
            bInverseOk = false;
            break;
          }
          g=(eigenValues._[l+1]-eigenValues._[l])/(2.0f*e[l]);
          r=hypot_s(g,R_ONE);
          g=eigenValues._[m]-eigenValues._[l]+e._[l]/(g+SIGN2(r,g));
          s=c=R_ONE;
          p=R_ZERO;
          for(i=m-1;i>=l;i--){
            f=s*e._[i];
            b=c*e._[i];
            e._[i+1] =(r=hypot_s(f,g));
            if(r==R_ZERO){
              eigenValues._[i+1]-=p;
              e._[m] = R_ZERO;
              break;  
            }
            s=f/r;
            c=g/r;
            g=eigenValues._[i+1]-p;
            r=(eigenValues._[i]-g)*s+2.0f*c*b;
            eigenValues._[i+1]=g+(p=s*r);
            g=c*r-b;
            for(k=0;k<n;k++){
              f=eigenVectors._[k*n+i+1];
              eigenVectors._[k*n+i+1]=s*eigenVectors._[k*n+i]+c*f;
              eigenVectors._[k*n+i]=c*eigenVectors._[k*n+i]-s*f;                
            }            
          }
          if((r==R_ZERO)&&(i>=0)) continue;
          eigenValues._[l]-=p;
          e._[l] = g;
          e._[m] = R_ZERO;
        }        
      }while(m!=l); 
      cumIter+=iter;
    } 
    if(!bInverseOk){
        fprintf(stderr,"Error: too many ierations...%f/%d\n",REALTYPE(cumIter)/REALTYPE(n),maxIter);
    }
    return cumIter;
  }

  /*
     \param[in] N   Number of columns and rows of the matrix
   \param[in] Ns  Stride parameter, i.e. offset between the first element of adjacent columns
   \param[in,out] R   Upper triangular Cholesky factor. Also serves as input matrix if <em>A==NULL</em>
   \param[in] A   Matrix to decompose. May be NULL, in which case the decomposition of R is done in place.
   \return  
      - 1 in case of succes
      - 0 in case of failure (e.g. input matrix is not positive definite)
  */
  //int Cholesky(int N,int Ns,double *R,const double *A) {
  Matrix& Cholesky(Matrix &result) {  
    result.Resize(row,column,false);
    result.Set(_,row,column);
    return result.SCholesky();
  }

  Matrix& SCholesky() {  
    
    if(row==column){ // Square matrix
      bInverseOk = TRUE;
               
      REALTYPE A_00, R_00;

      A_00 = _[0];
      if(A_00 <= R_ZERO) {bInverseOk = FALSE; return *this;}
      _[0] = R_00 = sqrt(A_00);
      
      if(row > 1) {
        REALTYPE A_01 = _[column+0];
        REALTYPE A_11 = _[column+1];
        REALTYPE R_01;
        REALTYPE diag;
        REALTYPE sum;
          
        R_01 = A_01 / R_00;
        diag = A_11 - R_01 * R_01;
    
        if(diag<=R_ZERO) {bInverseOk = FALSE; return *this;}        
    
        _[column+0] = R_01;
        _[column+1] = sqrt(diag); 
    
        for(unsigned int k=2; k<row; k++) {
          const unsigned int koffset = k*column;
          
          REALTYPE A_kk = _[koffset+k];
             
          for(unsigned int i=0; i<k; i++) {
            const unsigned int ioffset = i*column;
            REALTYPE A_ik = _[koffset+i];
            REALTYPE A_ii = _[ioffset+i];
      
            sum = R_ZERO;
            for(unsigned int j=0;j<i;j++){
              sum += _[ioffset+j] * _[koffset+j];
            }
      
            A_ik = (A_ik-sum)/A_ii;
            _[koffset+i] = A_ik;
          }
          
          sum = R_ZERO;
          for(unsigned int j=0;j<k;j++){
            sum += _[koffset+j] * _[koffset+j];
          }
          
          diag = A_kk - sum;
          if(diag <= R_ZERO) {bInverseOk = FALSE; return *this;}
          
          _[koffset+k] = sqrt(diag);
        }
      }
      for(unsigned int j=0;j<row-1;j++){
        for (unsigned int i=j+1;i<row;i++){
          _[j*column+i] = R_ZERO;
        }
      }

    }else{
      bInverseOk = FALSE;
    }
    return *this;
  }

  Matrix& InverseLowerTriangular(Matrix& result){
    result.Resize(row,column,false);
    result.Set(_,row,column);
    return result.SInverseLowerTriangular();
  }

  Matrix& SInverseLowerTriangular(){
    if(row==column){ // Square matrix
      bInverseOk = TRUE;
    
      for(unsigned int i=0;i<row;i++){       
        for(unsigned int j=0;j<i;j++){
          _[i*column+j] = -_[i*column+j] * _[j*column+j];
                    
          for(unsigned int k=j+1;k<i;k++){
            _[i*column+j] -= _[i*column+k] * _[k*column+j];
          }
        }
        
        REALTYPE pivot = R_ONE/_[i*column+i];        
        if(fabs(pivot)<=EPSILON) {bInverseOk = FALSE; return *this;}
        
        _[i*column+i] = pivot; 
        for(unsigned int j=0;j<i;j++){
          _[i*column+j] *= pivot;
        }
      }
    }else{
      bInverseOk = FALSE;
    }
    return *this;
  }

  Matrix& InverseSymmetric(Matrix & result, 
                REALTYPE * determinant=NULL,  
                Matrix * work=NULL){

    Matrix *temp = work;    
    if(temp==NULL){
      temp = new Matrix(*this);
    }else{
      temp->Set(_,row,column);
    }
    temp->SCholesky();
    REALTYPE det = 1.;
    for(int i=0;i<row;i++)
      det *= (*temp)._[i*row + i];
    temp->SInverseLowerTriangular();
    
    /*Matrix T(result);
    T.STranspose();
    Matrix copy(result);
    T.Mult(copy,result);*/

    result.Resize(row,column,false);
    for(unsigned int i=0;i<row;i++){
      for(unsigned int j=0;j<column;j++){
        const unsigned int start = (i<j?j:i);
        REALTYPE res = R_ZERO;
        unsigned int koffset = start*column;
        for(unsigned int k=start;k<row;k++){
          //res += (*work)._[i*column+k] * (*work)._[k*column+j];
          res += (*temp)._[koffset+i] * (*temp)._[koffset+j];
          koffset+=column;
        }
        result._[i*column+j] = res;
      }      
    }

    if(work==NULL){
      delete temp;
    }
    if(determinant) *determinant = det;
    
    return result;
  }


  /// Sort the column of the matrix according to the indices of the input vector elements sorted according to thir abolut value
  Matrix& SortColumnAbs(Vector & values){
    const int k = (values.Size()<column?values.Size():column);
    REALTYPE cmax;
    int maxId;
    for(int i=0;i<k-1;i++){
      cmax  = fabs(values._[i]);
      maxId = i;
      for(int j=i+1;j<k;j++){
        if(cmax<fabs(values._[j])){
          cmax = fabs(values._[j]);
          maxId = j;  
        }            
      }
      if(maxId!=i){
        REALTYPE tmp       = values._[i];
        values._[i]     = values._[maxId];
        values._[maxId] = tmp;
        SwapColumn(i,maxId);
      }     
    }  
    return *this;
  }

  Matrix& SquareRoot(Matrix& result){
    Vector eigValD;
    Matrix eigVal;
    Matrix eigVec; 
    EigenValuesDecomposition(eigValD, eigVec);
    for(int i=0;i<7;i++) eigValD(i) = sqrt(eigValD(i));
    eigVal.Diag(eigValD);
    eigVec.Mult(eigVal,result);
    eigVal = result;
    eigVec.STranspose();
    eigVal.Mult(eigVec,result);
    return result;
  }

  Matrix& InverseSquareRoot(Matrix& result){
    Vector eigValD;
    Matrix eigVal;
    Matrix eigVec; 
    EigenValuesDecomposition(eigValD, eigVec);
    for(int i=0;i<7;i++) eigValD(i) = sqrt(1.0/eigValD(i));
    eigVal.Diag(eigValD);
    eigVec.Mult(eigVal,result);
    eigVal = result;
    eigVec.STranspose();
    eigVal.Mult(eigVec,result);
    return result;
  }

  /// Do a Gram-Schmidt ortonormalization of the matrix  but an extra column (the base) is added. Using RemoveZeroColumn may then help clean the matrix.
  Matrix& GramSchmidt(Vector &base){
    Matrix unit(row,1);
    unit.SetColumn(base,0);
    Matrix ext;
    unit.HCat(*this,ext);
    ext.GramSchmidt();
    (*this) = ext;
    return *this;  
  }

  /// Do a Gram-Schmidt ortonormalization of the matrix  
  Matrix& GramSchmidt(){
    Vector res(row),tmp(row),tmp2(row),tmp3(row);
    for(unsigned int i=0;i<column;i++){
      GetColumn(i,tmp);
      res = tmp;        
      for(unsigned int j=0;j<i;j++){
        GetColumn(j,tmp2);
        res-=tmp2.Mult((tmp2.Dot(tmp)),tmp3);          
      }
      REALTYPE norm = res.Norm();
      if(norm>EPSILON){
        res /= norm;
      }else{
        res.Zero();
      }
      SetColumn(res,i);  
    }  
    return *this;    
  }

  /// Remove the columns of the matrix being zero
  Matrix& RemoveZeroColumns(){
    int zeroCnt = 0;
    int colCnt  = 0;
    while(colCnt < int(column)-zeroCnt){

      bool bIsZero = true;
      for(unsigned int j=0;j<row;j++){
        if(fabs(_[j*column+colCnt])>EPSILON){
          bIsZero = false;
          break;
        }
      }
      if(bIsZero){
        if(colCnt<int(column)-1-zeroCnt){
          SwapColumn(colCnt,int(column)-1-zeroCnt);
        }
        zeroCnt++;
      }else{
        colCnt++;
      }              
    }
    Resize(row,column-zeroCnt,true);
    return *this;       
  }

  /// Set a given column to zero
  Matrix& ClearColumn(unsigned int col){
    if(col<column){
      for(unsigned int i=0;i<row;i++){
        _[i*column+col] = R_ZERO;
      }      
    }  
    return *this;
  }

  /// Sum each row and give the result in a vector
  Vector SumRow(){
    Vector result(column);
    return SumRow(result);
  }

  /// Sum each row and give the result in a vector
  Vector & SumRow(Vector & result){
    result.Resize(column,false);
    result.Zero();
    for(unsigned int i=0;i<column;i++){
      for(unsigned int j=0;j<row;j++){
        result._[i] += _[j*column+i];
      }      
    }
    return result;  
  }

  /// Sum each column and give the result in a vector
  Vector SumColumn(){
    Vector result(row);
    return SumColumn(result);
  }
  
  /// Sum each column and give the result in a vector
  Vector & SumColumn(Vector & result){
    result.Resize(row,false);
    result.Zero();
    for(unsigned int j=0;j<row;j++){
      for(unsigned int i=0;i<column;i++){
        result._[j] += _[j*column+i];
      }      
    }
    return result;  
  }
  
  bool Load(const char* filename);
  bool Save(const char* filename);
  
protected:
  string RemoveSpaces(string s);
    
  inline void Release(){
    if(_!=NULL) delete [] _; 
    row    = 0;
    column = 0;
    _      = NULL;
  }  
public:  
  /// Resize the matrix
  inline virtual void Resize(unsigned int rowSize, unsigned int colSize, bool copy = true){
    if((row!=rowSize)||(column!=colSize)){
      if((rowSize)&&(colSize)){
        REALTYPE *arr = new REALTYPE[rowSize*colSize];
        if(copy){
          unsigned int mj = (row<rowSize?row:rowSize);
          unsigned int mi = (column<colSize?column:colSize);
          
          for (unsigned int j = 0; j < mj; j++){
            for (unsigned int i = 0; i < mi; i++)
              arr[j*colSize+i] = _[j*column+i];
            for (unsigned int i = mi; i < colSize; i++)
              arr[j*colSize+i] = R_ZERO;
          }
          for (unsigned int j = mj; j < rowSize; j++){
            for (unsigned int i = 0; i < colSize; i++)
              arr[j*colSize+i] = R_ZERO;            
          }
        }
        if(_!=NULL) delete [] _; 
        _      = arr;
        row    = rowSize;
        column = colSize;        
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
