// dataSet.cpp : behaviour of data sets
//

#include "dataSet.h"

// -------------------------------------------------------
// data set
// -------------------------------------------------------

dataSet::dataSet(unsigned int capacity, unsigned int size) : _capacity(capacity), _count(0), _size(size)
{

    lmAlloc(_data, _size*_capacity);

}

dataSet::~dataSet()
{

    delete[] _data;

}

void dataSet::add(real* datum)
{

    { foreach(_size,i) (*this)(_count,i) = datum[i]; }
    _count++;

}

void dataSet::reset()
{

    _count = 0;

}

void dataSet::save(string& name)
{

    // try and open file
	ofstream dataOfstream(name.c_str());
	if ( dataOfstream.is_open() == 0 ) {
		cout << "ERROR: could not save data." << endl;
		return;
	}

	// file header: size of the data vectors and number of them
	dataOfstream << _count << " " << _size << endl;

	// save data
    { foreach(_count,i) {
        foreach(_size,j) dataOfstream << (*this)(i,j) << " ";
    	dataOfstream << endl;
    } }

}

bool dataSet::load(string& name)
{

	ifstream dataIfstream(name.c_str());
	if ( dataIfstream.is_open() == false ) {
		cout << "no previously saved data found." << endl;
		return false;
	}

	// load file header and check consistency of previously saved data...
	unsigned int size, count;
	dataIfstream >> count >> size;
	// size must match
	if ( size != _size ) {
		cout << "ERROR: previously saved data file does not match." << endl;
		return false;
	}
	// number of stored examples can't be bigger than what this machine can hold
	if ( count > _capacity ) {
		cout << "ERROR: more saved data than allowed for this machine." << endl;
		return false;
	}

	// set new number of examples
	_count = count;

	// load them
    { foreach(_count,i) { 
        foreach(_size,j) dataIfstream >> (*this)(i,j);
    } }
	
	return true;

}
