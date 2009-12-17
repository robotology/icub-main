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

#include "iCub/vislab/HandMetrics.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

using namespace vislab::yarp;
using namespace vislab::yarp::util;

namespace vislab {
namespace control {

HandMetrics::HandMetrics(IEncoders* const encoders, ::yarp::dev::IPidControl* const pidControl) {

	int numAxes;
	encoders->getAxes(&numAxes);
	if (numAxes != this->numAxes) {
		throw invalid_argument("The number of axes mismatches the expected number");
	}

	this->encoders = encoders;
	this->pidControl = pidControl;
	prevPosition.resize(numAxes);
	position.resize(numAxes);

	Time::turboBoost();
	prevTime = Time::now();

	snapshot();
}

HandMetrics::~HandMetrics() {
}

void HandMetrics::snapshot() {
	mutex.wait();
	bool newSnapshot = position.size() > 0 || velocity.size() > 0;

	if (newSnapshot) {
		prevPosition = position;
	}

	// reset hand metrics (force re-computations)
	position.clear();
	velocity.clear();

	if (newSnapshot) {
		prevTime = curTime;
	}
	curTime = Time::now();
	mutex.post();
}

#ifdef DEBUG
void HandMetrics::printSnapshotData(ostream& s) {
	mutex.wait();
	s << "delta t = " << curTime - prevTime << endl;
	s << "position = ";
	printVector(position, s);
	s << "prePosition = ";
	printVector(prevPosition, s);
	s << "targetVoltage = ";
	printVector(targetVoltage, s);
	mutex.post();
}
#endif

const Vector HandMetrics::getPosition(const bool sync) {
	if (sync) {
		mutex.wait();
	}
	if (position.size() <= 0) {
		position.resize(numAxes);
		Vector error(numAxes), output(numAxes);
		encoders->getEncoders(position.data());
		pidControl->getErrors(error.data());

#ifdef DEBUG
		cout << "original position = ";
		printVector(position);
		cout << "error = ";
		printVector(error);
#endif

		position = position + error;
	}
	Vector v = position;
	if (sync) {
		mutex.post();
	}
	return v;
}

const Vector HandMetrics::getPosition() {
	return getPosition(true);
}

const Vector HandMetrics::getVelocity(const bool sync) {
	if (sync) {
		mutex.wait();
	}
	if (velocity.size() <= 0) {
		Vector deltaR = getPosition(false) - prevPosition;
		double deltaT = getTimeInterval(false);
		velocity = deltaR / deltaT;

#ifdef DEBUG
		cout << "deltaR = ";
		printVector(deltaR);
		cout << "deltaT = " << deltaT << endl;
#endif
	}
	Vector v = velocity;
	if (sync) {
		mutex.post();
	}
	return v;
}

const Vector HandMetrics::getVelocity() {
	return getVelocity(true);
}

double HandMetrics::getTimeInterval(const bool sync) {
	if (sync) {
		mutex.wait();
	}
	double d = abs(curTime - prevTime);
	if (sync) {
		mutex.post();
	}
	return d;
}

double HandMetrics::getTimeInterval() {
	return getTimeInterval(true);
}

}
}
