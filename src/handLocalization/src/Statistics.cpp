// include header
#include <Mathematics.h>

// namespaces
using namespace thesis::mathematics;

//using std::sort;

// ***************************************************************************

/**
 * Implementation of Statistics class
 *
 * This "module" is working with .
 */

// Default constructor
Statistics::Statistics() {
	//printf("Start:\t[Statistics]\n");
	this->bottle_tmp = Bottle ();
	this->bottle_tmp.clear();
	this->dPoint2D_tmp.x = 0.0;
	this->dPoint2D_tmp.y = 0.0;
}

// Destructor
Statistics::~Statistics() {
	//printf("Quit:\t[Statistics]\n");
}

/**
 *
 * This method returns the Parameter in order to compute the 90% confidence interval for the standard deviation
 *
 */
double Statistics::parameter90CI_ND(int _nofMeasureEntities) {
	if ((_nofMeasureEntities > 1) & (_nofMeasureEntities < 10)) {
		double values [] = { 6.31, 2.92, 2.35, 2.13, 2.02, 1.94, 1.90, 1.86};
		return values[_nofMeasureEntities-2];
	}
	else if ((_nofMeasureEntities >= 10) & (_nofMeasureEntities < 15)) {
		return 1.83;
	}
	else if ((_nofMeasureEntities >= 15) & (_nofMeasureEntities < 20)) {
		return 1.77;
	}
	else if ((_nofMeasureEntities >= 20) & (_nofMeasureEntities < 30)) {
		return 1.73;
	}
	else if ((_nofMeasureEntities >= 30) & (_nofMeasureEntities < 50)) {
		return 1.70;
	}
	else if ((_nofMeasureEntities >= 50) & (_nofMeasureEntities < 100)) {
		return 1.68;
	}
	else {
		return 1.66;
	}
}

/**
 *
 * This method returns the average of a test series stored in a bottle of double values.
 * x_ = 1/n * Sum[ x_i ] 
 *
 */
double Statistics::average(Bottle *_b) {
	int bottlesize = _b->size();
	double x_, x_i;

	x_  = 0.0;
	x_i = 0.0;
	for (int b = 0; b < bottlesize; b++) {
		x_i			= _b->get(b).asDouble();
		x_			= x_ + x_i;
	} 
	x_ = x_ / (double)bottlesize;

	return x_;
}

double Statistics::average(double *_arr_ptr, int _size) {
	double x_, x_i;

	x_  = 0.0;
	x_i = 0.0;
	for (int i=0; i<_size; i++) {
		x_i			= _arr_ptr[i];
		x_			= x_ + x_i;
	} 
	x_ = x_ / (double)_size;

	return x_;
}

/**
 *
 * This method returns the variance of a test series stored in a bottle of double values.
 * s² =  1/(n-1) * Sum[ (x_i - x_)^2 ] 
 *
double Statistics::variance(Bottle *_b) {
	double variance;
	variance = standardDeviationND(_b);
	variance = variance * variance;
	return variance;
}
 */

/**
 *
 * This method returns the standard deviation of a test series stored in a bottle of double values.
 * s = sqrt( 1/(n-1) * Sum[ (x_i - x_)^2 ] )
 *
 */
double Statistics::standardDeviationND(Bottle *_b, double _average) {
	int bottlesize = _b->size();
	//printf("bottlesize %d \n", bottlesize);
	double x_, x_i, s, term;
	x_			= _average;
	x_i			= 0.0;
	s			= 0.0;
	term		= 0.0;

	if (bottlesize > 1) {

		for (int b = 0; b < bottlesize; b++) {
			x_i		= _b->get(b).asDouble();
			term	= x_i - x_;
			s = s + term*term;
		} // end for b
		s = sqrt(s / (double)(bottlesize - 1));
	} 
	else {
		s = _b->get(0).asDouble();
	}
	//printf("standard deviation s = %f\n", s);
	return s;
}

double Statistics::standardDeviationND(Bottle *_b) {
	double average, s;
	average = this->average(_b);
	s		= this->standardDeviationND(_b, average);
	//printf("standard deviation s = %f\n", s);
	return s;
}

double Statistics::standardDeviationND(double *_arr_ptr, int _size) {
	double x_, s, tmp;
	if (_size > 0) {
		tmp		= 0.0;
		x_		= this->average(_arr_ptr, _size);
		for ( int i=0; i<_size; i++) {
			tmp = tmp + ((_arr_ptr[i] - x_)*(_arr_ptr[i] - x_));
		}
		s = sqrt(tmp / (_size-1));
	}
	else {
		printf("i do not compute std with less than 2 elements!\n");
		exit(99);
	}
	return s;
}

/**
 *
 * This method returns the empirical covariance of a two-dim test series stored in a bottle of n double values.
 * s_xy =  1/(n-1) * Sum[ (x_i - x_)(y_i - y_) ] 
 *
double Statistics::covariance(Bottle *_b_x, Bottle *_b_y) {
	int bottlesize = _b_x->size();
	double s_xy, x_, y_, tmp;
	tmp = 0.0;
	s_xy = 0.0;
	if (_b_x->size() == _b_y->size()) {
		if (bottlesize >1 ) {
			x_ = this->average(_b_x);
			y_ = this->average(_b_y);
			for (int b = 0; b < bottlesize; b++) {
				tmp = tmp + ((_b_x->get(b).asDouble() - x_)*(_b_y->get(b).asDouble() - y_));
			}
			s_xy = tmp / (bottlesize -1);
		}
		else {
			printf("more elements not available? arsch!\n");
			exit(-99);
		}
	}
	else {
		printf("the covariance of two different sized data sets i don't want to compute.\n");
		exit(-99);
	}

	return s_xy;
}
 */

double Statistics::covariance(double *_arr_x_ptr, double *_arr_y_ptr, int _size) {
	double s_xy, x_, y_, tmp;
	tmp = 0.0;
	s_xy = 0.0;
	if (_size > 1 ) {
		x_ = this->average(_arr_x_ptr, _size);
		y_ = this->average(_arr_y_ptr, _size);
		for (int b = 0; b < _size; b++) {
			tmp = tmp + (_arr_x_ptr[b]*_arr_y_ptr[b]);
		}
		tmp = tmp - _size*x_*y_;
		s_xy = tmp / (_size - 1);
	}
	else {
		printf("more elements not available? arsch!\n");
		exit(-99);
	}
	return s_xy;
}

double Statistics::covariance(double *_arr_x_ptr, double *_arr_y_ptr, double _avg_x, double _avg_y, int _size) {
	double cov_xy, tmp;
	tmp = 0.0;
	cov_xy = 0.0;
	if (_size > 1 ) {
		for (int b = 0; b < _size; b++) {
			tmp = tmp + (_arr_x_ptr[b]*_arr_y_ptr[b]);
		}
		tmp = tmp - _size*_avg_x*_avg_y;
		cov_xy = tmp / (_size - 1);
	}
	else {
		printf("more elements not available? arsch!\n");
		exit(-99);
	}
	return cov_xy;
	
}

/**
 *
 * This method returns the empirical correlation coefficient of a test series stored in a bottle of n double values.
 * r =  s_xy / (s_x*s_y)
 *
double Statistics::correlationCoefficient(Bottle *_b_x, Bottle *_b_y) {
	double r, s_x, s_y, s_xy;
	s_x = this->standardDeviationND(_b_x);
	s_y = this->standardDeviationND(_b_y);
	s_xy = this->covariance(_b_x, _b_y);
	if ((s_x != 0 ) & (s_y != 0)) {
		r = s_xy / (s_x*s_y);
	}
	else {
		printf("the correlation coefficient can not be computed if there are constant test sets\n");
		exit(-99);
	}
	return r;
}
 */

double Statistics::correlationCoefficient(double *_arr_x_ptr, double *_arr_y_ptr, int _size) {
	double r, s_x, s_y, s_xy;
	s_x = this->standardDeviationND(_arr_x_ptr, _size);
	s_y = this->standardDeviationND(_arr_y_ptr, _size);
	s_xy = this->covariance(_arr_x_ptr, _arr_y_ptr, _size);
	if ((s_x != 0 ) & (s_y != 0)) {
		r = s_xy / (s_x*s_y);
	}
	else {
		printf("the correlation coefficient can not be computed if there are constant test sets\n");
		exit(-99);
	}
	return r;

}

/**
 *
 * This method returns the x % confidence interval for the standard deviation of a normal distribution
 * the intervalborders are stored in a dPoint2D structure.
 *
 * x_ - t*s/sqrt(n) <= mu <= x_ + t*s/sqrt(n)
 *
 */
dPoint2D Statistics::confidenceIntervalND(Bottle *_b, int _percentage) {

	dPoint2D tmp;
	double s, x_, t, n;
	x_ = this->average(_b);
	s  = this->standardDeviationND(_b, x_);
	n  = _b->size();

	if (_percentage == 90) {
		t = this->parameter90CI_ND(((int)n));
	}
	else {
		//printf("\t[Statistics]\t{parameter for %d pc confidence intervall is not yet implemented}\n", _percentage);
		exit(-99);
	}
	tmp.x = x_ - ( t * s / sqrt((double)n));
	tmp.y = x_ + ( t * s / sqrt((double)n));
	
	//printf("confidence interval 90 pc [%f, %f]\n", tmp.x, tmp.y);
	
	return tmp;
}
