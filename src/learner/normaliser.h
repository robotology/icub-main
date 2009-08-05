// normaliser.h : header file for normalisers
//

#ifndef __normaliserh__
#define __nornaliserh__

#include "dataSet.h"

// -------------------------------------------------------
// normalisers
// -------------------------------------------------------
// a normaliser holds references to a source data set (non normalised)
// and a destination data set (normalised).

class Normaliser {
public:
  virtual ~Normaliser() {}

    // set up the data sets
	void setDataSets(dataSet* source, dataSet* dest) { _source = source, _dest = dest; }
    // (un)normalise a single value
    virtual real normalise( real, unsigned int ) = 0;
	virtual real unNormalise( real, unsigned int ) = 0;
    // (un)normalise all
    virtual void normaliseAll( void ) = 0;
    virtual void unNormaliseAll( void ) = 0;
    virtual void evalStatistics( void ) = 0;
protected:
	dataSet* _source;
    dataSet* _dest;
};

// null normaliser (does nothing)
class nullNormaliser : public Normaliser {
public:
	real normalise( real, unsigned int );
	real unNormalise( real, unsigned int );
    void normaliseAll( void );
    void unNormaliseAll( void );
    void evalStatistics( void ) {}
};

// fixed statistics normaliser (pass them to the constructor)
class fixNormaliser : public Normaliser {
public:
	fixNormaliser(real mean, real stdv)	: _mean(mean), _stdv(stdv) {}
    ~fixNormaliser() {}
	real normalise( real, unsigned int );
	real unNormalise( real, unsigned int );
    void normaliseAll( void );
    void unNormaliseAll( void );
    void evalStatistics( void ) {}
private:
	real _mean, _stdv;
};

// mean/standard deviation normaliser (evaluates them on-the-fly)
class msNormaliser : public Normaliser {
public:
	msNormaliser(unsigned int);
    ~msNormaliser();
	real normalise( real, unsigned int );
	real unNormalise( real, unsigned int );
    void normaliseAll( void );
    void unNormaliseAll( void );
    void evalStatistics( void );
private:
	real* _mean, * _stdv;
};

// max/min normaliser
class mmNormaliser : public Normaliser {
public:
	mmNormaliser(unsigned int);
    ~mmNormaliser();
	real normalise( real, unsigned int );
	real unNormalise( real, unsigned int );
    void normaliseAll( void );
    void unNormaliseAll( void );
    void evalStatistics( void );
private:
	real* _max, * _min;
};

#endif
