// libsvmLearningMachine.cpp : behaviour of libsvm-based SVM learning machines
//

#include "libsvmLearningMachine.h"

// -------------------------------------------------------
// libsvm learning machine
// -------------------------------------------------------

libsvmLearningMachine::libsvmLearningMachine( Normaliser* norm, params& params )
    : LearningMachine(norm, params), _params(params), _model(0)
{

    // allocate the problem's x's
    lmAlloc(_problem.x, _params._capacity);
   	{ foreach(_params._capacity,i) {
		lmAlloc(_problem.x[i], _params._domainSize+1);
        { foreach(_params._domainSize,j) {
			_problem.x[i][j].index = j+1;
			_problem.x[i][j].value = 0.0;
        } }
		_problem.x[i][_params._domainSize].index = -1;
    } }
    // allocate the problem's y's
	lmAlloc(_problem.y, _params._capacity);

}
    
libsvmLearningMachine::~libsvmLearningMachine()
{

    // destroy problem's y's
    delete[] _problem.y;
    // destroy problems's x's
	{ foreach(_params._capacity,i) delete[] _problem.x[i]; }
	delete[] _problem.x;
    // destroy model
    if ( _model != 0 ) svm_destroy_model( _model );
    // destroy paramters
    svm_destroy_param( &_params._svmparams );

}

void libsvmLearningMachine::reset()
{

	// destroy models and reset them to 0
	if ( _model != 0 ) svm_destroy_model( _model );
	_model = 0;

	LearningMachine::reset();

}

void libsvmLearningMachine::save()
{

	// save model
	string modelFileName = _params._name + ".model";
	if ( svm_save_model(modelFileName.c_str(), _model ) == -1 ) {
		cout << "ERROR: could not save model." << endl;
		return;
	}
	cout << "saved model to " << modelFileName << "." << endl;

	// save data
	LearningMachine::save();

}


bool libsvmLearningMachine::load()
{

	// load a previously saved model
	string modelFileName = _params._name + ".model";
	_model = svm_load_model(modelFileName.c_str());
	if ( _model == 0 ) {
		cout << "no previously saved model found." << endl;
		reset();
		return false;
	}
	cout << "loaded model from " << modelFileName << "." << endl;

	// try and load data
	if ( LearningMachine::load() == false ) {
		reset();
		return false;
	}

	return true;

}

void libsvmLearningMachine::setC( const double C )
{

	_params._svmparams.C = C;

}

bool libsvmLearningMachine::addExample( const real x[], const real y )
{

	// if the buffer is full, stop
	if ( _count == _params._capacity ) {
		return false;
	} else {
		// otherwise, is this example worth adding to the current pool?
		if ( isExampleWorthAdding(x, y) ) {
			// yes: then add it and then bail out.
            real* tmpVector;
            lmAlloc(tmpVector, _params._domainSize+1);
            tmpVector[0] = y;
            { foreach(_params._domainSize,i) tmpVector[i+1] = x[i]; }
            _rawData.add(tmpVector);
            delete[] tmpVector;
            _count++;
			return true;
		} else {
			// no. then return false
			return false;
		}
	}

}

void libsvmLearningMachine::filterSVs( void )
{

	// create a data set for the SV-only data set
	dataSet svDataSet(_params._capacity,_params._domainSize+1);

	unsigned int svCount = 0;
	// doing epoch management? then scan support vectors array
	{ foreach(_model->l,i) {
		// find the SV in the normalised data set
		{ foreach(_count,j) {
			// is this datum a support vector?
			bool isASV = true;
			{ foreach(_params._domainSize,k) {
				if ( _normalData(j,k+1) != _model->SV[i][k].value ) {
					isASV = false;
					break;
				}
			} }
			if ( isASV ) {
				// if so, copy the corresponding non-normal vector
				// to the new normalised data set
				{ foreach(_params._domainSize+1,k) {
					svDataSet(svCount,k) = _rawData(j,k);
				} }
				svCount++;
				break;
			} else {
				// otherwise check next datum
				continue;
			}
		} }
	} }
	// now svDataSet contains the non-normal SVs; copy them to _rawData and go ahead
	_rawData.reset();
	{ foreach(svCount,i) { 
		{ foreach(_params._domainSize,j) {
			_rawData(i,j) = svDataSet(i,j);
		} }
	} }
	_rawData.setCount(svCount);
	cout << "filtered out " << _count-svCount << " non-SV data." << endl;
	_count = svCount;

}

void libsvmLearningMachine::train()
{

	// destroy old model
	if ( _model != 0 ) svm_destroy_model( _model );

	// normalise data
    _norm->normaliseAll();

    // tell libsvm how may examples we have
	_problem.l = _count;

    // now copy the normalised data set and values into the problem's data structures
    // allocate the problem's x's
   	{ foreach(_count,i) {
		_problem.y[i] = _normalData(i,0);
        { foreach(_params._domainSize,j) {
			_problem.x[i][j].value = _normalData(i,j+1);
        } }
    } }

	// check problem consistency
	if ( svm_check_parameter( &_problem, &_params._svmparams ) != 0 ) {
		cout << "FATAL ERROR: libsvm parameters are incorrect." << endl;
		exit(-1);
	}

	// train !!
	_model = svm_train( &_problem, &_params._svmparams );

}


real libsvmLearningMachine::predict( const real x[] )
{

	// if model is void, do not even try to predict, but return zeroes
	if ( _model == 0 ) {
		return 0.0;
	}

	// otherwise, build sample to predict (normalise it first)
    svm_node* tmpSample;
    lmAlloc(tmpSample, _params._domainSize+1);
    { foreach(_params._domainSize,i) {
		tmpSample[i].index = i+1;
		tmpSample[i].value = _norm->normalise(x[i], i+1);
		}
	}
	tmpSample[_params._domainSize].index = -1;

	// and then predict (un-normalise before bailing out)
	real y = svm_predict(_model, tmpSample);
    delete[] tmpSample;
    return _norm->unNormalise( y, 0 );

}


bool libsvmLearningMachine::isExampleWorthAdding ( const real x[], const real y )
{

    return true;

}

// -------------------------------------------------------
// libsvm uniform learning machine
// -------------------------------------------------------

libsvmUniLearningMachine::libsvmUniLearningMachine( Normaliser* norm, params& params, real* tol) 
  : libsvmLearningMachine(norm, params)
{

	lmAlloc(_tolerance, _params._domainSize);
	{ foreach(_params._domainSize,i) { _tolerance[i] = tol[i]; } }

}

libsvmUniLearningMachine::~libsvmUniLearningMachine( void )
{

	delete[] _tolerance;

}

bool libsvmUniLearningMachine::isExampleWorthAdding ( const real x[], const real y )
{

	bool closeToMe;

	// so: if the example to be added is "close" to any previous one, don't add it
	{ foreach(_count,i) {
		closeToMe = true;
		foreach(_params._domainSize,j) {
			if ( abs(x[j]-_rawData(i,j+1)) > _tolerance[j] ) {
				closeToMe = false;
				break;
			}
		}
		if ( closeToMe ) {
			return false;
		}
	} }

	return true;

}

// -------------------------------------------------------
// libsvm feedback learning machine
// -------------------------------------------------------

bool libsvmFBLearningMachine::isExampleWorthAdding ( const real x[], const real y )
{

	if ( fabs(predict(x)-y) > _threshold ) {
		return true;
	} else {
		return false;
	}

}
