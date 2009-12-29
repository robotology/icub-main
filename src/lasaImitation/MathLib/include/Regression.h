#ifndef REGRESSION_H
#define REGRESSION_H
#include "MathLibCommon.h"


#include "Matrix4.h"
#include "Matrix.h"
#include "Vector.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

/**
 * \class Regression
 * 
 * \ingroup MathLib
 * 
 * \brief A set of regression function
 * 
 */
class Regression
{
private:
	bool Create(int matrixSize, int covarianceMatricesCount);

public:
  /**
   * \brief Do a Hermitte spline fit
   * 
   * \param inData        The input data series
   * \param inTimeBase    The time base of the input data
   * \param outTimeBase   The desired time base for the output data
   * \param outData       The result
   * \return              outData
   */    
	static Vector& HermitteSplineFit(const Vector& inData, const Vector& inTimeBase, const Vector& outTimeBase, Vector& outData);

  /**
   * \brief Do a Hermitte spline fit (multidimensional data at once)
   * 
   * \param inData        The input data series
   * \param inTimeBase    The time base of the input data
   * \param outTimeBase   The desired time base for the output data
   * \param outData       The result
   * \return              outData
   */    
	static Matrix& HermitteSplineFit(const Matrix& inData, const Vector& inTimeBase, const Vector& outTimeBase, Matrix& outData);

  /**
   * \brief Do a Hermitte spline fit assuming constant time steps
   * 
   * \param inData        The input data series
   * \param nbSteps       The desired number of output steps
   * \param outData       The result
   * \return              outData
   */    
	static Vector& HermitteSplineFit(const Vector& inData, int nbSteps, Vector& outData);

  /**
   * \brief Do a Hermitte spline fit assuming constant time steps (multidimensional data at once)
   * 
   * \param inData        The input data series
   * \param nbSteps       The desired number of output steps
   * \param outData       The result
   * \return              outData
   */    
	static Matrix& HermitteSplineFit(const Matrix& inData, int nbSteps, Matrix& outData);
  


	static void SaveData(Matrix &data, const char fileName[]);
	static bool LoadData(const char fileName[], Matrix &result);

	Matrix *covarianceMatrices;
	int covarianceMatricesCount;

  Matrix  covMatInRow;

  Matrix  smoothCovMatrix;


	Regression() : covarianceMatrices(NULL) {}
	~Regression() {Release();}
	void Release();

  /**
   * \brief Do a locally weighted regression (LWR)
   * 
   * Note: the resulting covariance matrices are stored in the class member covarianceMatrices.
   * 
   * \param inData        The input data multidimensional data (a data point is set in a row)
   * \param outData       The output data (the result). The columns indexed by xIndices should be filled with appropriate data. 
   * \param xIndices      An array of indices denoting the columns of the data to be used for the regression  
   * \param xIndicesCount The size of xIndices  
   * \param yIndices      An array of indices denoting the columns of the data to be regressed (guessed...)  
   * \param yIndicesCount The size of yIndices  
   * \param spam          An array of sigmas          
   * \param covSquare     Keep if to false...
   */    
	bool LocallyWeightedRegression(Matrix &inData, Matrix &outData, int xIndices[], int yIndices[], int xIndicesCount, int yIndicesCount, const Matrix &spam, bool covSquare = false);
	
  /**
   * \brief Do a gaussian product of two gaussian sets produced by LocallyWeightedRegression()  
   * 
   * \param data0         The data of the first gaussian set
   * \param data1         The data of the second gaussian set
   * \param yIndices      An array of indices denoting the columns of the data to be regressed (guessed...)  
   * \param yIndicesCount The size of yIndices  
   * \param covarianceMatrices0 A pointer to the covariance matrices of the first gaussian set
   * \param covarianceMatricesCount0 The size of the corresponding array
   * \param covarianceMatrices1 A pointer to the covariance matrices of the second gaussian set
   * \param covarianceMatricesCount1 The size of the corresponding array
   * \param resultData    The result
   * \param resultC       Don't know. Just pass something...
   */    
  bool GaussProduct(
		Matrix &data0,
		Matrix &data1,
		const int yIndices[],
		int yIndicesCount,
		Matrix *covarianceMatrices0,
		int covarianceMatricesCount0,
		Matrix *covarianceMatrices1,
		int covarianceMatricesCount1,
		Matrix &resultData,
		Vector &resultC,
    bool keepAlignment = false, bool pdiff=false,
    Matrix * eigDiff = NULL); 
	void SaveCovarianceMatrices(const char fileName[]);
  
  void SmoothCovarianceMatrices(int window);
  void SetSmoothCovarianceMatrix();
  void SmoothCovarianceMatrix(REALTYPE alpha);

  /**
   * \brief Find the best plane passing through n points
   * 
   * \param inData        The input data series
   * \param nbSteps       The desired number of output steps
   * \param outData       The result
   * \return              outData
   */    
  static Matrix4& GetBestPlaneFromPoints(const Matrix &points, const Vector3& zero, const Vector3& firAxis, const Vector3& secAxis, Matrix4 &out);

};


#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif
