// dataSet.h : header file for data sets
//

#ifndef __dataSeth__
#define __dataSeth__

#include "basics.h"

// -------------------------------------------------------
// data set
// -------------------------------------------------------
// a data set holds L vectors in R^n: it is a matrix with L rows and n columns

class dataSet {
public:

    // initialise with max.capacity and size of the data elements
    dataSet(unsigned int, unsigned int);
    ~dataSet();

    // access element (i,j)
    real& operator()(unsigned int i, unsigned int j) { return _data[i*_size+j]; }
    // get/set counters
    unsigned int getSize() const { return _size; }
    unsigned int getCapacity() const { return _capacity; }
    unsigned int getCount() const { return _count; }
    void setCount(unsigned int count) { _count = count; }
    // add 
    void add(real[]);
    // reset data set
    void reset(void);
    // save/load data set
    void save(string&);
    bool load(string&);

private:
	unsigned int _capacity, _count, _size;
    real* _data;
};

#endif
