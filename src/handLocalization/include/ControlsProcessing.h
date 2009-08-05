#ifndef _CONTROL_PROC_
#define _CONTROL_PROC_

#include <includes.h>

#include <math.h>

// namespaces 
namespace thesis {
    /**
     * Motor Controls Processing.
     */
    namespace controlsprocessing {
		class MICube;
    }
}


// ***************************************************************************
// ***************    namespace thesis::controlsprocessing     ***************
// ***************************************************************************

/**
 *  Class MICube: definition
 */
class thesis::controlsprocessing::MICube {
protected:
	// Declaration of hidden class variables
	int counter;
	int time;
	int nof_joints;
	double **jointsCube_ptr;

	// Declaration of hidden  methods
	double convertBottleElementIntoDouble(Value _el);

public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	MICube (Bottle *_controls_ptr, int _timewindow);
	~MICube();

	bool		reset();
	bool		paramChange(int _timewindow);
	void		update(Bottle *_controls_ptr);
	double**	getAccessToMotorInformationValues();
	int			getNofJoints();
};

// ***************************************************************************

#endif
