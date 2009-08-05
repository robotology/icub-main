// learningMachine.h : header file for learning machines
//

#ifndef __learningMachineh__
#define __learningMachineh__

#include "dataSet.h"
#include "normaliser.h"

// -------------------------------------------------------
// definitions
// -------------------------------------------------------
// - LEARNING MACHINE: a map R^m -> R, with the notable special case of classification,
//     in which R^n really consists of labels
// - INPUT SPACE: the space to map from (e.g., R^m)
// - OUTPUT SPACE: the space to map to (R)
// - SAMPLE: element of the input space
// - VALUE: element of the output space
// - EXAMPLE: a pair <SAMPLE,VALUE>. examples are used for training, whereas a sample
//     maps to a value through prediction
// all numeric values are stored as "real".

// -------------------------------------------------------
// plain learning machine
// -------------------------------------------------------
// only does data bookkeeping. has two data sets (normalised and
// non normalised) and a template pointer. can save and load its own status.
// notice that data set are created of domainSize+1 size, since the first
// column (0) is the OUTPUT SPACE.

class LearningMachine {
public:
    // parameters of a learning machine
    class params {
	  public:
		params(unsigned int capacity, unsigned int domainSize, string name)
            : _capacity(capacity), _domainSize(domainSize), _name(name) {}
		params() : _capacity(100), _domainSize(1), _name("learner") {}
		unsigned int _capacity;
		unsigned int _domainSize;
		string _name;
	};

	// initialise with parameters or use default
    LearningMachine( Normaliser* norm, params& params )
	 : _params(params), _count(0), _rawData(_params._capacity,_params._domainSize+1),
       _normalData(_params._capacity,_params._domainSize+1),
	   _norm(norm)
	{
		_norm->setDataSets(&_rawData,&_normalData);
	}
    LearningMachine( Normaliser* norm )
	 : _count(0), _rawData(_params._capacity,_params._domainSize+1),
       _normalData(_params._capacity,_params._domainSize+1),
	   _norm(norm)
	{
		_norm->setDataSets(&_rawData,&_normalData);
	}
    virtual ~LearningMachine( void ) {}

	// viewing counters
	unsigned int getDomainSize( void ) const { return _params._domainSize; }
	unsigned int getCapacity( void ) const { return _params._capacity; }
	unsigned int getCount( void ) const { return _count; }
	// resetting the machine
    void reset( void ) { _count = 0; _rawData.reset(); _normalData.reset(); }
    // loading and saving status
    void save( void );
    bool load( void );

	// abstract methods. any concrete learning machine must be able at least
	// to add an example, train its models and predict a new value given a sample
	virtual bool addExample( const real[], const real ) = 0;
	virtual void train( void ) = 0;
	virtual real predict( const real[] ) = 0;

protected:

    // the parameters
    params _params;
    // how many samples considered so far?
    unsigned int _count;
    // raw and normalised data
	dataSet _rawData;
    dataSet _normalData;
    // the normaliser
	Normaliser* _norm;

};

#endif
