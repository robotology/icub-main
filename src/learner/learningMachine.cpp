// learningMachine.cpp : behaviour of learning machines
//

#include "learningMachine.h"

// -------------------------------------------------------
// plain learning machine
// -------------------------------------------------------

void LearningMachine::save( void )
{

    // save non-normalised data
    string dataFileName = _params._name + ".raw.data";
    _rawData.save(dataFileName);
    cout << "saved raw data to " << dataFileName << "." << endl;
    // save normalised data
    dataFileName = _params._name + ".norm.data";
    _normalData.save(dataFileName);
    cout << "saved normalised data to " << dataFileName << "." << endl;

}

 
bool LearningMachine::load( void )
{

    // load non-normalised data
    string dataFileName = _params._name + ".raw.data";
    if ( _rawData.load(dataFileName) ==  false ) {
        return false;
    }
    // set new number of examples
	_count = _rawData.getCount();
    // loaded data are non normalised: normalise
	_norm->evalStatistics();
	_norm->normaliseAll();
    cout << "loaded " << _count << " data from " << dataFileName << "." << endl;
    return true;
    
}
