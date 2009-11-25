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

#include "iCub/vislab/Hand.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;

using namespace vislab::yarp::util;
using namespace vislab::yarp;

namespace vislab {
namespace control {

const int Hand::proximalJoints[] = { THUMB_PROXIMAL, INDEX_PROXIMAL, MIDDLE_PROXIMAL, PINKY };
const int Hand::distalJoints[] = { THUMB_DISTAL, INDEX_DISTAL, MIDDLE_DISTAL };
const int Hand::thumbJoints[] = { THUMB_OPPOSE, THUMB_PROXIMAL, THUMB_DISTAL };
const int Hand::allFingerJoints[] = { HAND_FINGER, THUMB_OPPOSE, THUMB_PROXIMAL, THUMB_DISTAL,
		INDEX_PROXIMAL, INDEX_DISTAL, MIDDLE_PROXIMAL, MIDDLE_DISTAL, PINKY };
const int Hand::allButThumb[] = { HAND_FINGER, INDEX_PROXIMAL, INDEX_DISTAL, MIDDLE_PROXIMAL,
		MIDDLE_DISTAL, PINKY };
const int Hand::completeHand[] = { WRIST_PROSUP, WRIST_PITCH, WRIST_YAW, HAND_FINGER, THUMB_OPPOSE,
		THUMB_PROXIMAL, THUMB_DISTAL, INDEX_PROXIMAL, INDEX_DISTAL, MIDDLE_PROXIMAL, MIDDLE_DISTAL,
		PINKY };

const set<int> Hand::PROXIMAL_JOINTS(Hand::proximalJoints, Hand::proximalJoints + 4);
const set<int> Hand::DISTAL_JOINTS(Hand::distalJoints, Hand::distalJoints + 3);
const set<int> Hand::THUMB_JOINTS(Hand::thumbJoints, Hand::thumbJoints + 1);
const set<int> Hand::ALL_FINGER_JOINTS(Hand::allFingerJoints, Hand::allFingerJoints + 9);
const set<int> Hand::ALL_BUT_THUMB(Hand::allButThumb, Hand::allButThumb + 8);
const set<int> Hand::COMPLETE_HAND(Hand::completeHand, Hand::completeHand + 12);

//const pair<int, int> Hand::limits[] = { std::make_pair(-96, 10), std::make_pair(0, 161),
//		std::make_pair(-37, 80), std::make_pair(5, 106), std::make_pair(-90, 90),
//		std::make_pair(-90, 0), std::make_pair(-20, 40), std::make_pair(0, 60),
//		std::make_pair(-15, 105), std::make_pair(0, 90), std::make_pair(0, 90), std::make_pair(0, 90),
//		std::make_pair(0, 90), std::make_pair(0, 90), std::make_pair(0, 90), std::make_pair(0, 115) };

const int Hand::limits[][2] = { { -96, 10 }, { 0, 161 }, { -37, 80 }, { 5, 106 },
		{ -90, 90 }, { -90, 0 }, { -20, 40 }, { 0, 60 }, { -15, 105 }, { 0, 90 }, { 0, 90 }, { 0, 90 },
		{ 0, 90 }, { 0, 90 }, { 0, 90 }, { 0, 115 } };

const vector<pair<int, int> > Hand::LIMITS = initLimits(Hand::limits, HandMetrics::numAxes);
const vector<int> Hand::RANGES = Hand::initRanges(Hand::LIMITS);

const vector<pair<int, int> > Hand::initLimits(const int arr[][2], size_t len) {
	vector<pair<int, int> > v;
	for (size_t i = 0; i < len; i++) {
		v.push_back(make_pair(arr[i][0], arr[i][1]));
	}
	return v;
}

const vector<int> Hand::initRanges(const vector<pair<int, int> > v) {
	vector<int> v_;
	for (size_t i = 0; i < v.size(); i++) {
		v_.push_back(abs(v[i].second - v[i].first));
	}
	return v_;
}

Hand::RecordingThread::RecordingThread(IEncoders* const encoders, int period) :
	RateThread(period) {
	if (encoders == NULL) {
		throw "Valid encoders are mandatory.";
	}

	int numAxes;
	encoders->getAxes(&numAxes);
	if (numAxes != HandMetrics::numAxes) {
		throw "The number of axes mismatches the expected number";
	}

	this->encoders = encoders;
	curRecording.setNumJoints(HandMetrics::numAxes);
}

Hand::RecordingThread::~RecordingThread() {
}

bool Hand::RecordingThread::start() {
	curRecording.clear();
	return RateThread::start();
}

void Hand::RecordingThread::stop() {
	RateThread::stop();
	recording = curRecording;
}

void Hand::RecordingThread::run() {
	Motion m(HandMetrics::numAxes);

	Vector v(HandMetrics::numAxes);
	encoders->getEncoders(v.data());
	m.setPosition(v);

	encoders->getEncoderSpeeds(v.data());
	m.setVelocity(v);
	m.setTiming(0.0);

	curRecording.addMotion(m);
}

MotionSequence Hand::RecordingThread::getRecording() {
	return recording;
}

Hand::Hand(PolyDriver& controlBoard) :
	controlBoard(controlBoard) {

	bool isValid = true;
	isValid &= controlBoard.view(encoders);
	isValid &= controlBoard.view(pidControl);
	isValid &= controlBoard.view(posControl);

	if (!isValid) {
		throw "Insufficient control board";
	}

	enabledJoints = COMPLETE_HAND;
	disabledJoints.clear();

	posControl->getRefSpeeds(prevSpeed);
#ifdef DEBUG
	printVector(prevSpeed, HandMetrics::numAxes);
#endif
	handMetrics = NULL;

	recordingThread = new RecordingThread(encoders);
	record(false);
}

void Hand::defineHandMetrics() {
	handMetrics = new HandMetrics(encoders, pidControl);
}

Hand::~Hand() {
	// Since derived classes are supposed to create the HandMetrics object,
	// we cannot rely on its existence.
	if (handMetrics != NULL) {
		delete handMetrics;
	}
	posControl->setRefSpeeds(prevSpeed);
	delete recordingThread;
}

HandMetrics& Hand::getMetrics() {
	if (handMetrics == NULL) {
		defineHandMetrics();
	}
	return *handMetrics;
}

void Hand::setEnable(const vector<int>& joints, const bool b) {
	if (!joints.empty()) {
		mutex.wait();
		if (b) { //enable
			for (unsigned int i = 0; i < joints.size(); i++) {
				disabledJoints.erase(joints[i]);
				if (joints[i] >= Hand::WRIST_PROSUP && joints[i] <= Hand::PINKY) {
					enabledJoints.insert(joints[i]);
				}
			}
			enabledJoints.insert(joints.begin(), joints.end());
		} else { // disable
			copy(joints.begin(), joints.end(), inserter(disabledJoints, disabledJoints.begin()));
			set<int>::const_iterator itr;
			for (itr = disabledJoints.begin(); itr != disabledJoints.end(); ++itr) {
				enabledJoints.erase(*itr);
			}
		}
		mutex.post();
	}
}

void Hand::disableJoints(const vector<int>& joints) {
	setEnable(joints, false);
}

void Hand::enableJoints(const vector<int>& joints) {
	setEnable(joints, true);
}

void Hand::getDisabledJoints(vector<int>& joints) {
	mutex.wait();
	joints.clear();
	copy(disabledJoints.begin(), disabledJoints.end(), inserter(joints, joints.end()));
	mutex.post();
}

void Hand::setVelocity(const double d, const int joint) {
	if (enabledJoints.find(joint) != enabledJoints.end()) { // contains(.)
		posControl->setRefSpeed(joint, d);
	}
}

void Hand::setVelocity(const double d, const std::set<int> joints) {
	Vector v(HandMetrics::numAxes);
	set<int>::const_iterator itr;
	for (itr = joints.begin(); itr != joints.end(); ++itr) {
		int joint = (*itr);
		if (joint >= v.size() || joint < Hand::WRIST_PROSUP) {
			cout << "Warning: joint #" << joint << " is not part of the hand." << endl;
		} else {
			v[joint] = d;
		}
	}
	setVelocity(v, joints);
}

void Hand::setVelocity(const ::yarp::sig::Vector& v, const std::set<int> joints) {
	set<int>::const_iterator itr;
	for (itr = joints.begin(); itr != joints.end(); ++itr) {
		int joint = (*itr);
		if (joint >= v.size() || joint < 0) {
			cout << "Warning: No value for joint #" << joint << " specified." << endl;
		} else {
			setVelocity(v[joint], joint);
		}
	}
}

void Hand::setVelocity(const ::yarp::sig::Vector& v, const int joint) {
	set<int> s;
	s.insert(joint);
	setVelocity(v, s);
}

bool Hand::move(const Vector& v, const set<int> joints) {
	bool result = true;
	mutex.wait();
	set<int> activeJoints;
	set<int>::const_iterator itr;
	for (itr = joints.begin(); itr != joints.end(); ++itr) {
		int joint = (*itr);
		if (enabledJoints.find(joint) == enabledJoints.end()) { // !contains(.)
			continue;
		}
		if (joint >= v.size() || joint < 0) {
			cout << "Warning: No value for joint #" << joint << " specified." << endl;
		} else {
			result = result && posControl->positionMove(joint, v[joint]);
			activeJoints.insert(joint);
		}
	}
	do {
		getMetrics().snapshot();
		stopBlockedJoints(activeJoints);
	} while (!motionDone(activeJoints));

	posControl->stop(); // just to make sure
	mutex.post();
	return result;
}

bool Hand::move(const Motion& m, const int joint) {
	set<int> s;
	s.insert(joint);
	return move(m, s);
}

bool Hand::move(const Motion& m, const std::set<int> joints) {
	setVelocity(m.getVelocity(), joints);
	bool result = move(m.getPosition(), joints);
	Time::delay(m.getTiming());
	return result;
}

bool Hand::move(const Matrix& m, const int joint, const bool invert) {
	set<int> s;
	s.insert(joint);
	return move(m, s, invert);
}

bool Hand::move(const Matrix& m, const set<int> joints, const bool invert) {
	bool result = true;
	set<int> activeJoints(joints.begin(), joints.end());

	for (int step = invert ? m.rows() - 1 : 0; (invert && step >= 0) || (!invert && step < m.rows()); step
			= invert ? step - 1 : step + 1) {
		result &= move(m.getRow(step), joints);
	}
	return result;
}

bool Hand::move(const MotionSequence& seq, const bool invert) {
	return move(seq, COMPLETE_HAND, invert);
}

bool Hand::move(const MotionSequence& seq, const int joint, const bool invert) {
	set<int> s;
	s.insert(joint);
	return move(seq, s, invert);
}

bool Hand::move(const MotionSequence& seq, const std::set<int> joints, const bool invert) {
	bool result = true;
	if (invert) {
		MotionSequence::const_reverse_iterator itr;
		for (itr = seq.rbegin(); itr != seq.rend(); ++itr) {
			move(*itr, joints);
		}
	} else {
		MotionSequence::const_iterator itr;
		for (itr = seq.begin(); itr != seq.end(); ++itr) {
			move(*itr, joints);
		}
	}
	return result;
}

bool Hand::motionDone(const set<int> joints) {
	Vector v = getMetrics().getVelocity();
	set<int>::const_iterator itr;
	for (itr = joints.begin(); itr != joints.end(); ++itr) {
		bool b;
		posControl->checkMotionDone(*itr, &b);
		if (abs(v[*itr]) > 0.01 && !b) {
			return false;
		}
	}
	return true;
}

void Hand::stopBlockedJoints(std::set<int>& activeJoints) {
	// to be detailed in derived classes
}

void Hand::record(const bool b) {
	if (isRecording() != b) {
		if (!b) {
			recordingThread->stop();
			// save file
		} else {
			recordingThread->start();
		}
	}
}

bool Hand::isRecording() {
	return recordingThread->isRunning();
}

MotionSequence Hand::getRecording() {
	return recordingThread->getRecording();
}

void Hand::setSamplingRate(int t) {
	recordingThread->setRate(t);
}

}
}
