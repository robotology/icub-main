#ifndef SPLINEFIT_H
#define SPLINEFIT_H
#include "MathLibCommon.h"

#include "Matrix.h"
#include "Vector.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

/**
 * \class SplineFit
 * 
 * \ingroup MathLib
 * 
 * \brief A set of regression function
 * 
 */
class SplineFit
{
public:
  /**
   * \brief Do a Hermitte spline fit (multidimensional data at once).
   * 
   * \param inputData        The input data series (with time base in the first column). Time is assumed to be strictly increasing.
   * \param outputData       The result (with output time base filled in the first column) . Time is assumed to be strictly increasing (errors can be expected otherwise...).
   * \return                 outputData
   */    
  static Matrix& HermitteSplineFit(const Matrix& inputData, Matrix& outputData);

  static Matrix& HermitteSplineFitQuat(const Matrix& inputData, Matrix& outputData);

  /**
   * \brief Do a Hermitte spline fit (multidimensional data to one vector).
   * 
   * \param inputData        The input data series (with time base in the first column). Time is assumed to be strictly increasing.
   * \param outputData       The result (with output time base filled in the first column).
   * \param startRowGuess    A guess as to where to start the search (0 is the default)
   * \return                 outputData
   */    
  static Vector& HermitteSplineFit(const Matrix& inputData, Vector& outputData, int *startRowGuess = NULL, REALTYPE *phase=NULL);
  
  static Matrix& KinematicSplineCoefs(const Vector& pos, const Vector& vel, const Vector& acc, const Vector& targetPos, const Vector& targetVel, const Vector& targetAcc, const REALTYPE targetTime, Matrix& resultCoefs);    
  static void    KinematicSplineFit(const Matrix& coefs, const REALTYPE targetTime, const REALTYPE time, Vector* pos, Vector *vel, Vector* acc);    

  static Matrix& KinematicSplineCoefs2(const Vector& pos, const Vector& vel, const Vector& targetPos, const Vector& targetVel, const REALTYPE targetTime, Matrix& resultCoefs);    
  static void    KinematicSplineFit2(const Matrix& coefs, const REALTYPE targetTime, const REALTYPE time, Vector* pos, Vector *vel, Vector* acc);    
  static void    KinematicSplineFitMax2(const Matrix& coefs, const REALTYPE targetTime, Vector* pos, Vector* vel, Vector* acc);

protected:
  static Matrix  mKinenaticSplineFitMatrix;

  static Matrix  mKinenaticSplineFit2Matrix;

};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif

#endif
