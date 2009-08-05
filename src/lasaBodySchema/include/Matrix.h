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
#ifndef MATRIX_H
#define MATRIX_H

#ifdef WIN32
#include "windows.h"
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "Macros.h"
#include "Vector.h"

#ifdef  USE_T_EXTENSIONS
template<unsigned int ROW> class TMatrix;
#endif
    
class Matrix
{
  friend class Vector;
#ifdef  USE_T_EXTENSIONS
  template<unsigned int ROW> friend class TMatrix;
#endif
  
protected: 
  static int bInverseOk;
  
  unsigned int  row;    //number of rows
  unsigned int  column;//number of columns
  float        *_;

public:

  inline Matrix() {
    row    = 0;
    column = 0;
    _      = NULL;
  }
  
  inline virtual ~Matrix(){
    Release(); 
  }

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

  inline Matrix(unsigned int rowSize, unsigned int colSize, bool clear = true)
  {
    row    = 0;
    column = 0;
    _      = NULL;
    Resize(rowSize,colSize,false);
    if(clear)
      Zero();
  }
  
  inline Matrix(const float _[], unsigned int rowSize, unsigned int colSize)
  {
    row       = 0;
    column    = 0;
    this->_   = NULL;
    Resize(rowSize,colSize,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        this->_[j*column+i] = _[j*column+i];
  }

#ifdef  USE_T_EXTENSIONS
  template<unsigned int ROW> inline Matrix(const TMatrix<ROW> &matrix)
  {
    row    = 0;
    column = 0;
    _      = NULL;
    Resize(ROW,ROW,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = matrix._[j*column+i];
  }
#endif
   
  inline Matrix& Zero()
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = 0.0f;
    return *this;
  }

  //actually the size of the column (or the number of rows)    
  inline unsigned int RowSize() const{
    return row;
  }

  //actually the size of the row (or the number of columns)    
  inline unsigned int ColumnSize() const{
    return column;
  } 
  inline float *Array() const{
    return _;
  }

  inline float& operator() (const unsigned int row, const unsigned int col)
  {
    if((row<this->row)&&(col<this->column))
      return _[row*column+col];
    return Vector::undef; 
  }

  inline Vector GetRow(const unsigned int row) const
  {
    Vector result(column,false);    
    return GetRow(row,result);     
  }

  inline Vector& GetRow(const unsigned int row, Vector& result) const
  {
    result.Resize(column,false);
    for (unsigned int i = 0; i < column; i++)
      result._[i] = _[row*column+i];
    return result;     
  }

  inline Vector GetColumn(const unsigned int col) const
  {
    Vector result(row,false);    
    return GetColumn(col,result);     
  }

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

  inline Matrix GetColumnSpace(const unsigned int col, const unsigned int len) const  
  {
    if(len>0){
      Matrix result(row,len,false);    
      return GetColumnSpace(col,len,result);
    }else
      return Matrix();     
  }

  inline Matrix GetRowSpace(const unsigned int row, const unsigned int len) const
  {
    if(len>0){
      Matrix result(len,column,false);    
      return GetRowSpace(row,len,result);
    }else
      return Matrix();     
  }


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
           result._[j*size+(i-col)] = 0.0f;            
      }else{
        result.Zero();
      }
    }else{
      result.Resize(0,0,false);
    }
    return result;     
  }

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
           result._[(i-row)*column+j] = 0.0f;            
      }else{
        result.Zero();
      }
    }else{
      result.Resize(0,0,false);
    }
    return result;     
  }

  inline Matrix& SetRow(const Vector &vector, const unsigned int row)
  {
    if(row<this->row){    
      const unsigned int ki = (column<=vector.row?column:vector.row);
      for (unsigned int i = 0; i < ki; i++)
        _[row*column+i] = vector._[i]; 
    }
    return *this;     
  }

  inline Matrix& SetColumn(const Vector &vector, const unsigned int col)
  {
    if(col<this->column){    
      const unsigned int kj = (row<=vector.row?row:vector.row);
      for (unsigned int j = 0; j < kj; j++)
        _[j*column+col] = vector._[j];
    }
    return *this;
  }

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

  inline Matrix GetRowSpace(const Vector &ids) const
  {
    Matrix result(ids.Size(),column);
    return GetRowSpace(ids,result);
  }

  inline Matrix GetColumnSpace(const Vector &ids) const
  {
    Matrix result(row,ids.Size());
    return GetColumnSpace(ids,result);
  }

  inline Matrix GetMatrixSpace(const Vector &rowIds,const Vector &colIds) const
  {
    Matrix result(rowIds.Size(),colIds.Size());
    return GetMatrixSpace(rowIds,colIds,result);
  }

  inline Matrix& GetColumnSpace(const Vector &ids, Matrix &result) const
  {
    const unsigned int k=ids.Size();
    result.Resize(row,k);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g = (unsigned int)(fabs(ROUND(ids._[i])));
      if(g<column){
        for(unsigned int j=0;j<row;j++)
          result._[j*k+i] = _[j*column+g];
      }else{
        for(unsigned int j=0;j<row;j++)
          result._[j*k+i] = 0.0f;        
      }
    }
    return result;     
  }

  inline Matrix& GetRowSpace(const Vector &ids, Matrix &result) const
  {
    const unsigned int k=ids.Size();
    result.Resize(k,column);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g = (unsigned int)(fabs(ROUND(ids._[i])));
      if(g<row){
        for(unsigned int j=0;j<column;j++)
          result._[i*column+j] = _[g*column+j];
      }else{
        for(unsigned int j=0;j<column;j++)
          result._[i*column+j] = 0.0f;
      }
    }
    return result;     
  }

  inline Matrix& GetMatrixSpace(const Vector &rowIds,const Vector &colIds, Matrix &result) const
  {
    const unsigned int k1=rowIds.Size();
    const unsigned int k2=colIds.Size();
    result.Resize(k1,k2);
    for(unsigned int i=0;i<k1;i++){
      const unsigned int g1 = (unsigned int)(fabs(ROUND(rowIds._[i])));
      if(g1<row){
        for(unsigned int j=0;j<k2;j++){      
          const unsigned int g2 = (unsigned int)(fabs(ROUND(colIds._[j])));
          if(g2<column){
            result._[i*k2+j] = _[g1*column+g2];            
          }else{
            result._[i*k2+j] = 0.0f;
          }
        }
      }else{
        for(unsigned int j=0;j<k2;j++)
          result._[i*k2+j] = 0.0f;
      }
    }
    return result;     
  }

  inline Matrix operator - () const
  {
    Matrix result(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = -_[j*column+i];
    return result;
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

  inline Matrix& operator ^= (const Matrix &matrix)
  {
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++)
      for (unsigned int i = 0; i < ki; i++)
        _[j*column+i] *= matrix._[j*column+i];
    return *this;
  }

  inline Matrix& operator /= (const Matrix &matrix)
  {
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++)
      for (unsigned int i = 0; i < ki; i++)
        _[j*column+i] /= matrix._[j*column+i];
    return *this;
  }

  inline Matrix& operator += (float scalar)
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] += scalar;
    return *this;
  }

  inline Matrix& operator -= (float scalar)
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] -= scalar;
    return *this;
  }

  inline Matrix& operator *= (float scalar)
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] *= scalar;
    return *this;
  }

  inline Matrix& operator /= (float scalar)
  {
    scalar = 1.0f/scalar;
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
  
  inline Matrix operator ^ (const Matrix &matrix) const
  {
    Matrix result(row,column,false);  
    return PMult(matrix,result);
  }
  
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
  
  inline Matrix operator / (const Matrix &matrix) const
  {
    Matrix result(row,column,false);  
    return PDiv(matrix,result);
  }
  
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

  inline Matrix operator + (float scalar) const
  {
    Matrix result(row,column,false);  
    return Add(scalar,result);    
  }

  inline Matrix& Add(float scalar, Matrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] + scalar;    
    return result;
  }

  inline Matrix operator - (float scalar) const
  {
    Matrix result(row,column,false);  
    return Sub(scalar,result);    
  }
  
  inline Matrix& Sub(float scalar, Matrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] - scalar;    
    return result;
  }

  inline Matrix operator * (float scalar) const
  {
    Matrix result(row,column,false);  
    return Mult(scalar,result);    
  }

  inline Matrix& Mult(float scalar, Matrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] * scalar;    
    return result;
  }


  inline Matrix operator / (float scalar) const
  {
    Matrix result(row,column,false);  
    return Div(scalar,result);    
  }

  inline Matrix& Div(float scalar, Matrix& result) const
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

  inline Vector operator * (const Vector &vector) const
  {
    Vector result(row,false);  
    return Mult(vector,result);    
  }  

  inline Vector Mult(const Vector &vector) const
  {
    Vector result(row,false);  
    return Mult(vector,result);    
  }

  inline Vector& Mult(const Vector &vector, Vector &result) const
  {
    result.Resize(row,false);
    const unsigned int ki = (column<=vector.row?column:vector.row);
    for (unsigned int j = 0; j < row; j++){
      result._[j] = 0.0f;
      for (unsigned int i = 0; i < ki; i++)
        result._[j] += _[j*column+i] * vector._[i];
    }
    return result;
  }


  inline Matrix operator * (const Matrix &matrix) const  
  {
    Matrix result(row,matrix.column,false);  
    return Mult(matrix,result);
  }  

  inline Matrix& Mult(const Matrix &matrix, Matrix &result) const
  {
    result.Resize(row,matrix.column,false);
    const unsigned int rrow = result.row;
    const unsigned int rcol = result.column;
    const unsigned int kk = (column<=matrix.row?column:matrix.row);
    for (unsigned int j = 0; j < rrow; j++){
      for (unsigned int i = 0; i < rcol; i++){
        result._[j*rcol+i] = 0.0f;
        for(unsigned int k = 0; k< kk; k++)    
          result._[j*rcol+i] += _[j*column+k] * matrix._[k*rcol+i];
      }
    }    
    return result;
  }



  inline Matrix& Identity()
  {
    const unsigned int k = (row>column?column:row);
    Zero();
    for (unsigned int i = 0; i < k; i++)
      _[i*column+i] = 1.0f;
    return *this;    
  }

  inline Matrix& Diag(const Vector &vector)
  {
    const unsigned int k = (row>column?column:row);
    const unsigned int k2 = (k>vector.row?vector.row:k);
    Zero();
    for (unsigned int i = 0; i < k2; i++)
      _[i*column+i] = vector._[i];
    return *this;    
  }

  inline Matrix& Random(){
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] =((float)rand())/((float)(RAND_MAX+1.0));    
    return *this;    
  }

  inline Matrix Transpose() const
  {
    Matrix result(row,column,false);
    return Transpose(result);    
  }

  inline Matrix& Transpose(Matrix &result) const
  {    
    result.Resize(column,row,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[i*row+j] = _[j*column+i];
    return result;    
  }

  inline Matrix VCat(const Matrix& matrix)
  {
    Matrix result;
    return VCat(matrix,result);    
  }
  
  inline Matrix& VCat(const Matrix& matrix, Matrix & result)
  {
    unsigned int k1 = (column>matrix.column?column:matrix.column);
    result.Resize(row+matrix.row,k1,false);
    for (unsigned int j = 0; j < row; j++){
      for (unsigned int i = 0; i < column; i++)
        result._[j*k1+i] = _[j*column+i];
      for (unsigned int i = column; i < k1; i++)
        result._[j*k1+i] = 0.0f;
    }
    for (unsigned int j = 0; j < matrix.row; j++){
      for (unsigned int i = 0; i < matrix.column; i++)
        result._[(row+j)*k1+i] = matrix._[j*matrix.column+i];
      for (unsigned int i = matrix.column; i < k1; i++)
        result._[(row+j)*k1+i] = 0.0f;
    }
    return result;
  }

  inline Matrix HCat(const Matrix& matrix)
  {
    Matrix result;
    return VCat(matrix,result);    
  }
  
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
        result._[j*k2+i] = 0.0f;

    for (unsigned int j = 0; j < matrix.row; j++)
      for (unsigned int i = 0; i < matrix.column; i++)
        result._[j*k2+i+column] = matrix._[j*matrix.column+i];
    for (unsigned int j = matrix.row; j < k1; j++)
      for (unsigned int i = 0; i < matrix.column; i++)
        result._[j*k2+i+column] = 0.0f;
    
    return result;
  }

  inline static int IsInverseOk(){
    return bInverseOk;
  }

  inline Matrix Inverse() const
  {
    Matrix result;
    return Inverse(result);
  }  

  inline Matrix& Inverse(Matrix &result) const
  {
    bInverseOk = TRUE;   
    if(row==column){ // Square matrix
      result.Resize(row,column,false);
      const unsigned int n = row;
      Matrix MM(*this);
      result.Identity();
      for(unsigned int i=0;i<n;i++){
        float pivot = MM._[i*column+i]; 
        if(fabs(pivot)<=EPSILON){
          for(unsigned int j=i+1;j<n;j++){
            if((pivot = MM._[j*column+i])!=0.0f){
              MM.SwapRow(i,j);
              result.SwapRow(i,j);
              break;  
            }
          }
          if(fabs(pivot)<=EPSILON){
            bInverseOk = FALSE;
            return result;
          }            
        }
        pivot = 1.0f/pivot;
        for(unsigned int j=0;j<n;j++){
          MM._[i*column+j]   *= pivot;
          result._[i*column+j] *= pivot;
        }
        for(unsigned int k=0;k<n;k++){
          if(k!=i){
            const float mki = MM._[k*column+i];
            for(unsigned int j=0;j<n;j++){
               MM._[k*column+j]   -= MM._[i*column+j]   *mki;
               result._[k*column+j] -= result._[i*column+j] *mki;              
            }            
          }
        }
      }        
    }else{ // Moore-Penrose pseudo inverse
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

  inline Matrix& SwapRow(unsigned int j1, unsigned int j2){
    if((j1<row)&&(j2<row)){
      float tmp;
      for (unsigned int i = 0; i < column; i++){
        tmp            = _[j1*column+i];
        _[j1*column+i] = _[j2*column+i];
        _[j2*column+i] = tmp;        
      }        
    }
    return *this; 
  }
 
  inline Matrix& SwapColumn(unsigned int i1, unsigned int i2){
    if((i1<column)&&(i2<column)){
      float tmp;
      for (unsigned int j = 0; j < row; j++){
        tmp            = _[j*column+i1];
        _[j*column+i1] = _[j*column+i2];
        _[j*column+i2] = tmp;        
      }        
    }
    return *this; 
  }
  
  void Print() const
  {
    std::cout << "Matrix " <<row<<"x"<<column<<std::endl;;
    for (unsigned int j = 0; j < row; j++){
      for (unsigned int i = 0; i < column; i++)
        std::cout << _[j*column+i] <<" ";
      std::cout << std::endl;
    }
  }
  
  void QRDecomposition(Matrix & Q, Matrix & R){
    Matrix QR;
    QRDecomposition(Q,R,QR);
  }
  
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
      float nrm = 0.0f;
      for (unsigned int i = k; i < m; i++) {
        nrm = hypot_s(nrm, QR(i,k));
      }
      if (nrm != 0.0f) {
        if(QR(k,k)<0.0f){
          nrm = -nrm;
        }
        for (unsigned int i = k; i < m; i++) {
            QR(i,k) /= nrm;
        }
        QR(k,k)+=1.0f;

        for (unsigned int j = k + 1; j < n; j++) {
          float s = 0.0f;
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
          R(i,j) = 0.0f;
        }
      }
    }

    Q.Resize(m,n);
    for(int k= n-1;k>=0;k--){
      for(unsigned int i = 0; i < m; i++) {
        Q(i,k) = 0.0f;
      }
      Q(k,k)=1.0f;
      for(unsigned int j = k; j < n; j++) {
        if(QR(k,k)!=0.0f){
          float s = 0.0f;
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

  
  Matrix& Tridiagonalize(Matrix &result,Matrix &trans){
    result.Resize(2,row);
    Matrix A(*this);
    
    int n = row;
    int l,k,j,i;
    float scale,hh,h,g,f;
    for(i=n-1;i>=1;i--){
      l = i-1;
      h = scale = 0.0f;
      if(l>0){
        for(k=0;k<=l;k++)
          scale += fabs(A._[i*column+k]);
        if(scale == 0.0f){
          result._[column+i] = A._[i*column+l];
        }else{
          for(k=0;k<=l;k++){
            A._[i*column+k] /= scale;
            h += A._[i*column+k]*A._[i*column+k];
          }
          f= A._[i*column+l];
          g=(f>=0.0f?-sqrt(h):sqrt(h));
          result._[column+i] = scale*g;
          h-=f*g;
          A._[i*column+l] = f-g;
          f=0.0f;
          for(j=0;j<=l;j++){
            A._[j*column+i] = A._[i*column+j] /h;
            g=0.0f;
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
    result._[0]=0.0f;  
    result._[column+0]=0.0f;
    for(i=0;i<n;i++){
      l=i-1;
      if(result._[i]){
        for(j=0;j<=l;j++){
          g=0.0f;
          for(k=0;k<=l;k++)
            g+= A._[i*column+k]*A._[k*column+j]; 
          for(k=0;k<=l;k++)
            A._[k*column+j] -= g*A._[k*column+i]; 
        }  
      }
      result._[i] = A._[i*column+i];
      A._[i*column+i] = 1.0f;
      for(j=0;j<=l;j++) A._[j*column+i]=A._[i*column+j]=0.0f;
    }
    trans = A;
    return result;
  }
    
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
  
  int TriEigen(Vector &eigenValues, Matrix& eigenVectors,int maxIter = 30){
    GetRow(0,eigenValues);
    Vector e;
    GetRow(1,e);
    
    const int n = column;
    int m,l,iter,i,k;
    float s,r,p,g,f,dd,c,b;
    
    for(i=1;i<n;i++) e._[i-1] = e._[i];
    e._[n-1] = 0.0f;

    for(l=0;l<n;l++){
      iter=0;
      do{
        for(m=l;m<=n-2;m++){
          dd = fabs(eigenValues._[m])+fabs(eigenValues._[m+1]);
          if((float)(fabs(e._[m])+dd) == dd) break;  
        }
        if(m!=l){
          if(iter++==maxIter) printf("Error: too many ierations...\n");
          g=(eigenValues._[l+1]-eigenValues._[l])/(2.0f*e[l]);
          r=hypot_s(g,1.0f);
          g=eigenValues._[m]-eigenValues._[l]+e._[l]/(g+SIGN2(r,g));
          s=c=1.0f;
          p=0.0f;
          for(i=m-1;i>=l;i--){
            f=s*e._[i];
            b=c*e._[i];
            e._[i+1] =(r=hypot_s(f,g));
            if(r==0.0f){
              eigenValues._[i+1]-=p;
              e._[m] = 0.0f;
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
          if((r==0.0f)&&(i>=0)) continue;
          eigenValues._[l]-=p;
          e._[l] = g;
          e._[m] = 0.0f;
        }        
      }while(m!=l); 
    } 
       
    return iter;
  }



  


  Matrix& SortColumnAbs(Vector & values){
    const int k = (values.Size()<column?values.Size():column);
    float cmax;
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
        float tmp       = values._[i];
        values._[i]     = values._[maxId];
        values._[maxId] = tmp;
        SwapColumn(i,maxId);
      }     
    }  
    return *this;
  }

  
protected:

  inline void Release(){
    if(_!=NULL) delete [] _; 
    row    = 0;
    column = 0;
    _      = NULL;
  }  
public:  
  inline virtual void Resize(unsigned int rowSize, unsigned int colSize, bool copy = true){
    if((row!=rowSize)||(column!=colSize)){
      if((rowSize)&&(colSize)){
        float *arr = new float[rowSize*colSize];
        if(copy){
          unsigned int mj = (row<rowSize?row:rowSize);
          unsigned int mi = (column<colSize?column:colSize);
          
          for (unsigned int j = 0; j < mj; j++){
            for (unsigned int i = 0; i < mi; i++)
              arr[j*colSize+i] = _[j*colSize+i];
            for (unsigned int i = mi; i < colSize; i++)
              arr[j*colSize+i] = 0.0f;
          }
          for (unsigned int j = mj; j < rowSize; j++){
            for (unsigned int i = 0; i < colSize; i++)
              arr[j*colSize+i] = 0.0f;            
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


#endif
