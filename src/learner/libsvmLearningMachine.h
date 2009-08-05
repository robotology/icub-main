// libsvmLearningMachine.h : header file for libsvm-based SVM learning machines
//

#ifndef __libsvmLearningMachineh__
#define __libsvmLearningMachineh__

#include "learningMachine.h"
#include "libsvm.h"

// -------------------------------------------------------
// libsvm learning machine
// -------------------------------------------------------

class libsvmLearningMachine : public LearningMachine {
public:

    class params : public LearningMachine::params {
    public:
        params() : _filter(false) {
	        // default libsvm parameters
    	    _svmparams.svm_type = EPSILON_SVR;
        	_svmparams.kernel_type = RBF;
	        _svmparams.degree = 3;
	        _svmparams.gamma = 1/(real)_domainSize;
	        _svmparams.coef0 = 0;
	        _svmparams.nu = 0.5;
	        _svmparams.cache_size = 10;
	        _svmparams.C = 1;
	        _svmparams.eps = 1e-3;
	        _svmparams.p = 0.1;
	        _svmparams.shrinking = 1;
	        _svmparams.probability = 0;
	        _svmparams.nr_weight = 0;
	        _svmparams.weight_label = 0;
	        _svmparams.weight = 0;
        }
        // parameters of the libsvm SVM machine
        svm_parameter _svmparams;
		// non-SVs filtering
		bool _filter;
    };

	libsvmLearningMachine( Normaliser*, params& );
	~libsvmLearningMachine( void );

	void reset( void );
	void save( void );
	bool load( void );
	void setC( const double );

	bool addExample( const real[], const real );
	void train( void );
	real predict( const real[] );

	virtual bool isExampleWorthAdding ( const real[], const real );
	void filterSVs( void );

protected:

    // libsvm parameters
	params _params;

private:

	// libsvm "problem"
	svm_problem _problem;
	// libsvm model pointer
	svm_model* _model;

};

// -------------------------------------------------------
// libsvm uniform learning machine
// -------------------------------------------------------
// this one samples according to set of a tolerance values -
// avoids taking samples which are too close to one another

class libsvmUniLearningMachine : public libsvmLearningMachine {
public:

	libsvmUniLearningMachine( Normaliser*, params&, real* );
	~libsvmUniLearningMachine( void );

	bool isExampleWorthAdding ( const real[], const real );

private:

	// array of tolerances
	real* _tolerance;

};

// -------------------------------------------------------
// libsvm feedback learning machine
// -------------------------------------------------------
// variant: this one compares the predicted value of a sample with the
// value of the example, and only accepts the new sample if the error is "big"

class libsvmFBLearningMachine : public libsvmLearningMachine {
public:

	libsvmFBLearningMachine( Normaliser* norm, params& params, real thr ) 
		: libsvmLearningMachine(norm, params), _threshold(thr) {}
	~libsvmFBLearningMachine( void ) {}

	bool isExampleWorthAdding ( const real[], const real );

private:

	// error threshold: error less than -> don't sample
	real _threshold;

};

#endif
