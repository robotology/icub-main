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

#include "vislab/yarp/util/MotionSequence.h"

#include <cmath>
#include <stdexcept>
#include <fstream>

#include "vislab/yarp/sig.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace vislab::yarp;

namespace vislab {
namespace yarp {
namespace util {

Motion::Motion() {
	dynamicallyAdjustVectorSizes = true;
	numJoints = 0;
}

Motion::Motion(unsigned int numJoints) {
	setNumJoints(numJoints);
}

Motion::~Motion() {

}

void Motion::adjustVectorSizes() {
	if (dynamicallyAdjustVectorSizes) {
		numJoints = max(position.length(), velocity.length());
	}
	::vislab::yarp::resize(position, numJoints, 0.0);
	resize(velocity, numJoints, 0.0);
}

void Motion::setPosition(const Vector& v, const bool isRelative) {
	this->isRelative = isRelative;
	position = v;
	adjustVectorSizes();
}

void Motion::setVelocity(const Vector& v) {
	velocity = v;
	adjustVectorSizes();
}

void Motion::setTiming(const double d) {
	timing = d;
}

const Vector& Motion::getPosition() const {
	return position;
}

const Vector& Motion::getVelocity() const {
	return velocity;
}

const double Motion::getTiming() const {
	return timing;
}

void Motion::setNumJoints(const int n) {
	if (n <= 0) {
		throw invalid_argument("The number of joints must not be lower or equal 0.");
	}
	if (numJoints != n) {
		numJoints = n;
		dynamicallyAdjustVectorSizes = false;
		adjustVectorSizes();
	}
}

const int Motion::getNumJoints() const {
	return numJoints;
}

ConstString Motion::toString() {
	ostringstream oss;

	oss << "jointPositions\t";
	printVector(getPosition(), oss);
	oss << "jointVelocities\t";
	printVector(getVelocity(), oss);
	oss << "timing\t" << getTiming();

	return oss.str().c_str();
}

MotionSequence::MotionSequence() {
	dynamicallyAdjustMotionSizes = true;
	numJoints = 0;
}

MotionSequence::MotionSequence(unsigned int numJoints) {
	this->numJoints = numJoints;
	dynamicallyAdjustMotionSizes = false;
}

void MotionSequence::addMotion(Motion& m) {
	if (dynamicallyAdjustMotionSizes && m.getNumJoints() > numJoints) {
		numJoints = m.getNumJoints();
		adjustMotionSizes();
	} else {
		m.setNumJoints(numJoints);
	}
	motionSequence.push_back(m);
}

void MotionSequence::adjustMotionSizes() {
	MotionSequence::iterator itr;
	for (itr = begin(); itr != end(); ++itr) {
		itr->setNumJoints(numJoints);
	}
}

void MotionSequence::setNumJoints(const int n) {
	if (n <= 0) {
		throw invalid_argument("The number of joints must not be lower or equal 0.");
	}
	if (numJoints != n) {
		numJoints = n;
		dynamicallyAdjustMotionSizes = false;
		adjustMotionSizes();
	}
}

const int MotionSequence::getNumJoints() const {
	return numJoints;
}

Motion& MotionSequence::operator[](const size_t i) {
	return motionSequence[i];
}

const Motion& MotionSequence::operator[](const size_t i) const {
	return motionSequence[i];
}

const size_t MotionSequence::length() const {
	return motionSequence.size();
}

void MotionSequence::clear() {
	motionSequence.clear();
}

MotionSequence MotionSequence::subsequence(const size_t start, const size_t end) {
	MotionSequence seq(numJoints);
	for (size_t i = max((size_t) 0, start); i <= min(length(), end - 1); i++) {
		seq.addMotion(operator[](i));
	}
	return seq;
}

ConstString MotionSequence::toString() {
	ostringstream oss;

	for (size_t i = 0; i < length(); i++) {
		oss << "[POSITION" << i << "]" << endl;
		oss << (*this)[i].toString().c_str() << endl;
		oss << endl;
	}
	oss << "[DIMENSIONS]" << endl;
	oss << "numberOfPoses\t" << length() << endl;
	oss << "numberOfJoints\t" << getNumJoints();

	return oss.str().c_str();
}

bool MotionSequence::toFile(ConstString filename) {
	ofstream f;
	f.open(filename.c_str());
	if (!f.is_open()) {
		return false;
	}
	f << toString().c_str() << endl;
	f.close();
	return !f.is_open();
}

}
}
}
