#ifndef DTW_H
#define DTW_H

#include "MathLibCommon.h"

#include "Regression.h"
#include "Vector.h"
#include "Matrix.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

/**
 * \class DTW
 * 
 * \ingroup MathLib
 * 
 * \brief Dynamical time wraping functions
 * 
 */
class DTW
{
public:

      
  /**
   * \brief Dynamical time wraping function
   * The others functions of this class should not be used (not necessary)
   * 
   * \param Ref   Reference data (can be multidimentional). The time base flows along rows.
   * \param In    Data to align
   * \param Out   Result of the DTW
   * \param Dist  Resulting distrance measure
   * \param Cost  Resulting cost of the dtw
   * \return      The parameter Out
   */    
  static Matrix&  Process(Matrix &Ref, Matrix &In, Matrix &Out, Matrix& Dist, Matrix& Cost);

  /// Distance function between two vectors (Norm-2)
  static float    Distance(const Vector &x,const Vector &y);
  /// Distance function between two multidimentional sets of data
  static Matrix&  Distance(const Matrix &x,const Matrix &y, Matrix &result);
  /// Smooth return path
  static float*   SmoothPath(int* pathIn, float* pathOut, int size, int window);  

};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif
