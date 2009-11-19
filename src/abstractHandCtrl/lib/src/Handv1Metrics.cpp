/*******************************************************************************
 * Copyright (C) 2009 Christian Wressnegger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *******************************************************************************/

#include "iCub/vislab/Handv1Metrics.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

using namespace vislab::yarp;
using namespace vislab::yarp::util;

namespace vislab {
namespace control {

#define DISABLE_PIDVOODOO

const double Handv1Metrics::pidLimit = 933.0;
const double Handv1Metrics::residualVoltageThres = 70.0;

Handv1Metrics::Handv1Metrics(IEncoders* const encoders, IPidControl* const pidControl,
		IAmplifierControl* const ampControl, map<const string, Matrix>& constants) :
	HandMetrics(encoders, pidControl), objectSensingConstants(constants) {

	int numAxes;
	encoders->getAxes(&numAxes);
	if (numAxes != this->numAxes) {
		throw "The number of axes mismatches the expected number";
	}

	this->encoders = encoders;
	this->pidControl = pidControl;

#ifndef DISABLE_PIDVOODOO
	for (int i = 0; i < numAxes; i++) {
		ampControl->enableAmp(i);
	}

	for (int i = 0; i < numAxes; i++) {
		pidControl->enablePid(i);
	}

	pidControl->getPids(prevPids);

	pidControl->setPid(8, Pid(-120, -1000, 0, pidLimit, 4, pidLimit));
	Pid pid(100, 1000, 0, pidLimit, 4, pidLimit);
	for (int i = 9; i < numAxes - 1; i++) {
		pidControl->setPid(i, pid);
	}
	pidControl->setPid(numAxes - 1, Pid(-120, -1250, 0, pidLimit, 5, pidLimit));

	for (int i = 0; i < numAxes; i++) {
		pidControl->enablePid(i);
	}
#endif

	string neededConstants[] = { "derivate_gain", "offsets", "springs" };
	for (int i = 0; i < 3; i++) {
		map<const string, Matrix>::iterator itr = objectSensingConstants.find(neededConstants[i]);
		if (itr == objectSensingConstants.end() || itr->second.cols() != numAxes) {
			ostringstream ss;
			ss << "Missing or incomplete constant `" << neededConstants[i] << "`";
			throw ss.str();
		}
	}
	prevPosition.resize(numAxes);
	prevVoltageOffset.resize(numAxes);
	prevSpringStiffness.resize(numAxes);
	error.resize(numAxes);

	Time::turboBoost();
	prevTime = Time::now();

	snapshot();
}

Handv1Metrics::~Handv1Metrics() {
#ifndef DISABLE_PIDVOODOO
	for (int i = 0; i < numAxes; i++) {
		pidControl->disablePid(i);
	}

	pidControl->setPids(prevPids);

	for (int i = 0; i < numAxes; i++) {
		pidControl->enablePid(i);
	}
#endif
}

Vector& Handv1Metrics::getVoltage() {
	if (voltage.size() <= 0) {
		voltage.resize(numAxes);
		Vector v = getVelocity();
		v = vislab::yarp::operator *(objectSensingConstants["derivate_gain"].getRow(0), v);

		for (int i = 0; i < v.size(); i++) {

			// too many assignments! but hey -- it's readable ;)
			double offset = prevVoltageOffset[i];
			offset = v[i] < -residualVoltageThres ? objectSensingConstants["offsets"][0][i] : offset;
			offset = v[i] > residualVoltageThres ? -objectSensingConstants["offsets"][0][i] : offset;
			prevVoltageOffset[i] = offset;

			// dto
			double spring = prevSpringStiffness[i];
			spring = position[i] > 0 ? objectSensingConstants["springs"][0][i] : spring;
			spring = position[i] > 30 ? objectSensingConstants["springs"][1][i] : spring;
			spring = position[i] > 60 ? objectSensingConstants["springs"][2][i] : spring;
			prevSpringStiffness[i] = spring;

			voltage[i] = v[i] + offset + spring * position[i];

			voltage[i] = min(voltage[i], (double) pidLimit);
			voltage[i] = max(voltage[i], -(double) pidLimit);
		}
#ifdef DEBUG
		cout << "derivate = ";
		printVector(v);
		cout << "--- position = ";
		printVector(position);
		cout << "*offset = ";
		printVector(prevVoltageOffset);
		cout << "*spring = ";
		printVector(prevSpringStiffness);
		cout << "voltage = ";
		printVector(voltage);
#endif
	}
	return voltage;
}

Vector& Handv1Metrics::getError() {
	if (error.size() <= 0) {
		error = targetVoltage - getVoltage();
	}
	return error;
}

}
}
