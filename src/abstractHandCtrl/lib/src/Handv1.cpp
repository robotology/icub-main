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

#include "iCub/vislab/Handv1.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;

using namespace vislab::yarp::util;
using namespace vislab::yarp;

namespace vislab {
namespace control {

const double Handv1::FunctionSmoother::e = 1.0;
const double Handv1::FunctionSmoother::t = 2.0;
const double Handv1::FunctionSmoother::l = 0.5;

Handv1::FunctionSmoother::FunctionSmoother() {
}

Handv1::FunctionSmoother::FunctionSmoother(const Vector& residualVoltageThresholds) {
	thresholds = residualVoltageThresholds;
	prevValues.resize(thresholds.size());
	prevSmoothedValues.resize(prevValues.length());
}

Handv1::FunctionSmoother::~FunctionSmoother() {

}

Vector& Handv1::FunctionSmoother::smooth(const Vector& v, Vector& smoothedValues,
		const double deltaT) {
	if (v.length() != prevValues.size()) {
		throw "The value vector size mismatches the size of the previous values. \
				   Therefore they are for sure not corresponding and shouldn't be compared.";
	}

	smoothedValues.resize(v.length());

	for (int i = 0; i < v.length() && i < thresholds.length(); i++) {
		double d = (prevValues[i] + v[i]) * l;
		if (deltaT > t) { // reset integral
			smoothedValues[i] = 0;
		}
		if (abs(d - prevValues[i]) < e) {
			smoothedValues[i] = 0;
		} else {
			bool isOpening = thresholds[i] > 0;
			if ((isOpening && d > 0) || (!isOpening && d < 0)) {
				smoothedValues[i] = prevSmoothedValues[i] + (d / (700 * deltaT));
			} // otherwise remain at 0
		}
		prevValues[i] = d;
	}
	prevSmoothedValues = smoothedValues;
	return smoothedValues;
}

Handv1::Handv1(PolyDriver& controlBoard, map<const string, Matrix>& constants) :
	Hand(controlBoard), sensingConstants(constants) {

	if (sensingConstants.find("thresholds") == sensingConstants.end()) {
		throw "No threshold defined!";
	}
	fs = FunctionSmoother(sensingConstants["thresholds"].getRow(0));
}

void Handv1::defineHandMetrics() {
	// TODO: change call of defineHandMetrics in Hand-constructor

	bool isValid = true;
	isValid &= controlBoard.view(encoders);
	isValid &= controlBoard.view(pidControl);
	isValid &= controlBoard.view(posControl);
	isValid &= controlBoard.view(ampControl);

	if (!isValid) {
		throw "Insufficient control board";
	}

	handMetrics = new Handv1Metrics(encoders, pidControl, ampControl, sensingConstants);
	Hand::handMetrics = (HandMetrics*) handMetrics;
}

Handv1::~Handv1() {
}

void Handv1::calibrate() {
	throw "Not implemented yet!";
}

Handv1Metrics& Handv1::getMetrics() const {
	return *handMetrics;
}

void Handv1::stopBlockedJoints(std::set<int>* const blockedJoints) {
	Vector smoothedError;
	Vector thresholds = sensingConstants["thresholds"].getRow(0);
	fs.smooth(handMetrics->getError(), smoothedError, handMetrics->getTimeInterval());

	set<int>::const_iterator itr;
	for (itr = enabledJoints.begin(); itr != enabledJoints.end(); ++itr) {
		int i = *itr;
		if (i >= thresholds.length() || i < 0) {
			cout << "Warning: No thresholds for joint #" << i << " specified.";
		} else {
			bool isOpening = thresholds[i] > 0;
			if ((isOpening && smoothedError[i] > thresholds[i]) || (!isOpening && smoothedError[i]
					< thresholds[i])) {
				posControl->stop(i);
				cout << "joint #" << i << " blocked" << endl;
				if (blockedJoints != NULL) {
					blockedJoints->insert(i);
				}
			}
		}
	}
#ifdef DEBUG
	cout << "smoothed error: ";
	printVector(smoothedError);
	cout << "thresholds: ";
	printVector(thresholds);
#endif
}

}
}
