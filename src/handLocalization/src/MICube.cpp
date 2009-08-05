// include header
#include <controlsprocessing.h>

// namespaces
using namespace thesis::controlsprocessing;

// ***************************************************************************

/**
 * Implementation of MICube class
 *
 * This "module" is working on the images within a timeperiod and saves 
 * the motor information to MICube
 *
 */

// Init constructor
MICube::MICube(Bottle *_controls_ptr, int _timewindow) {
	//printf("Start:\t[MICube]\n");
	
	// Declarations & Initializing
	this->counter		= 0;
	this->time			= _timewindow;
	this->nof_joints	= _controls_ptr->size();

	printf("Info:\t[MICube]\t{will record %d units for %d joints}\n", this->time, this->nof_joints);
	this->jointsCube_ptr = new double* [this->nof_joints];
	for (int j = 0; j < this->nof_joints; j++) {
		this->jointsCube_ptr[j] = new double[this->time];
		for (int t=0; t < this->time; t++) {
			this->jointsCube_ptr[j][t] = 0.0;
		}
	}
	this->update(_controls_ptr);
}

// Destructor
MICube::~MICube() {
	for (int t=0; t<this->nof_joints; t++) {
			delete [] this->jointsCube_ptr[t];
	}
	delete [] this->jointsCube_ptr;
	//printf("Quit:\t[MICube]\n");
}

bool MICube::reset() {
	this->counter = 0;
	for (int j = 0; j < this->nof_joints; j++) {
		this->jointsCube_ptr[j] = new double[this->time];
		for (int t=0; t < this->time; t++) {
			this->jointsCube_ptr[j][t] = 0.0;
		}
	}
	return true;
}

bool MICube::paramChange(int _timewindow) {
	this->counter = 0;
	int j;
	for (j = 0; j < this->nof_joints; j++) {
		delete [] this->jointsCube_ptr[j];
	}
	this->time = _timewindow;
	for (j = 0; j < this->nof_joints; j++) {
		this->jointsCube_ptr[j] = new double[this->time];
		for (int t=0; t < this->time; t++) {
			this->jointsCube_ptr[j][t] = 0.0;
		}
	}
	return true;

}
/*
 *	This method is updating the MI Cube with its new motorsensory information 
 **/
void MICube::update(Bottle *_controls_ptr) {
	double val_d;
	int len = _controls_ptr->size();
	if (len == this->nof_joints) {
		for (int i = 0; i<len; i++) {
			Value el = _controls_ptr->get(i);
			if (el.isDouble()) {
				val_d = el.asDouble();
			}
			else {
				val_d = this->convertBottleElementIntoDouble((Value)el.asString());
			}
			this->jointsCube_ptr[i][this->counter] = val_d;
		}
	}
	this->counter++;
}

int MICube::getNofJoints() {
	return this->nof_joints;
}

double** MICube::getAccessToMotorInformationValues() {
	return this->jointsCube_ptr;
}

/**
 * If reading the values from the file, the string values have to be converted into double values.
 **/
double MICube::convertBottleElementIntoDouble(Value _el) {

	int i, k, vz, startpos, temp_int;
	double potenz;
	double d = 0.0;

    string val = string(_el.asString().c_str());
	int len = val.length();
	int comma = val.find(".");
	int sign = val.find("-");

	if (sign == -1) {
		startpos	= 0;
		k			= 1;
		vz			= 1;
	}
	else {
		startpos	= 1;
		k			= 0;
		vz			= -1;
	}
	i = startpos;
	while (i < len) {
		potenz = 1.0;
		if (i < comma) {
			while (k < (comma-startpos-i)) {
				potenz = 10.0 * potenz;
				k++;
			}
		}
		else if (i > comma) {
			potenz = 1.0;
			k = 0;
			while (k < (i-comma)) {
				potenz = potenz * 0.1;
				k++;
			}
		}
		if (i != comma) {
			temp_int = val.at(i) - 48;
			d = (double)(d + ((double)temp_int * potenz));
		}
		i++;
	}
	d = (double)vz * d;
	return d;
}
