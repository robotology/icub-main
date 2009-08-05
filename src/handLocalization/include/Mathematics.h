#ifndef _MATHEMATICA_
#define _MATHEMATICA_

#include <includes.h>

#include <math.h>
#include <algorithm>
#include <vector>

// namespaces 
namespace thesis {
    /**
     * mathematics to display information (of more complex stuff).
     */
    namespace mathematics {
		class Geometry;
		class Statistics;
		class Cluster;
    }
}


// ***************************************************************************
// ********               namespace thesis::mathematics               ********
// ***************************************************************************

/**
 *  Class Geometry: definition 
 *
 */
class thesis::mathematics::Geometry {
private:
	// Declaration of hidden class variables
	double	double_tmp;
	double *double_tmp_array_ptr;
	int		int_tmp;
	Bottle	bottle_tmp;
	// Declaration of hidden methods

public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Geometry();
	~Geometry();

	double		median(Bottle *_bottle_ptr);
	double		median(double *_array_ptr, int _array_size);
	int			median(int *_array_ptr, int _array_size);

	Bottle		lowPass(double *_array_ptr, int _array_size);

	int			getAngle(dPoint2D _v1, dPoint2D _v2);
	int			getAngle(iPoint2D _v1, iPoint2D _v2);
	int			getAngle(dPoint2D _v1);
	int			getAngle(iPoint2D _v1);
	double		getArcCos(dPoint2D _v1);

};

// ***************************************************************************

/**
 *  Class Statistics: definition 
 *
 */
class thesis::mathematics::Statistics {
private:
	// Declaration of hidden class variables
	Bottle bottle_tmp;
	dPoint2D dPoint2D_tmp;
	// Declaration of hidden methods
	double		parameter90CI_ND(int _nofMeasureEntities);

public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Statistics();
	~Statistics();

	double		average(Bottle *_b);
	/*double		variance(Bottle *_b);
	double		covariance(Bottle *_b_x, Bottle *_b_y);*/
	double		standardDeviationND(Bottle *_b);
	double		standardDeviationND(Bottle *_b, double _average);
	//double		correlationCoefficient(Bottle *_b_x, Bottle *_b_y);
	dPoint2D	confidenceIntervalND(Bottle *_b, int _percentage);

	double		average(double *_arr_ptr, int _size);
	double		covariance(double *_arr_x_ptr, double *_arr_y_ptr, int _size);
	double		standardDeviationND(double *_arr_ptr, int _size);
	double		correlationCoefficient(double *_arr_x_ptr, double *_arr_y_ptr, int _size);
	double		covariance(double *_arr_x_ptr, double *_arr_y_ptr, double _avg_x, double _avg_y, int _size);
};

// ***************************************************************************

/**
 *  Class Cluster: definition
 */
class thesis::mathematics::Cluster {
protected:
	// Declaration of hidden class variables
	// Declaration of hidden methods

public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Cluster();
	~Cluster();

	void kMeans(double **_data_ptr, string _method, int _clusterNum, int _dataNum, int _dimNum, double _haltRatio, int _maxIters, int *_label_ptr, double **_clusterCenter_ptr);
};


#endif
