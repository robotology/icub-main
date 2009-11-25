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

#include "iCub/vislab/HandModule.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;

using namespace vislab::util;
using namespace vislab::yarp::util;
using namespace vislab::yarp;

namespace vislab {
namespace control {

HandModule::HandModule(ConstString name) :
	AbstractRFModule(name) {
}

HandModule::~HandModule() {
}


bool HandModule::ReloadCommand::execute(const Bottle& params, Bottle& reply) const {
	HandModule* h = (HandModule*) parent;
	NetInt32 result = Vocab::encode("ack");

	try {
		h->motionSpecification.fromConfigFile(h->motionSpecificationFilename);
		h->workerThread->addMotionSpecification(h->motionSpecification);
	} catch (...) {
		result = Vocab::encode("fail");
	}

	reply.addVocab(result);
	return true;
}

bool HandModule::configure(ResourceFinder &rf) {
	if (!AbstractRFModule::configure(rf)) {
		return false;
	}

	ConstString str;
	ConstString prefix = "/";

	partName = rf.check("part", Value("right_arm"),
			"The robot's arm to work with (\"right_arm\" or \"left_arm\")").asString();

	// TODO: remove this once they finally resolved the "/" confusion
	str = getName(prefix + partName);
	replaceDoubleSlash(str);
	setName(str);

	if (!isHandlerAvailable()) {
		return false; // unable to open; let RFModule know so that it won't run
	}

	ConstString handType = rf.check("handType", Value("general"),
			"The typ of the icub's hands: \"general\" | \"v1\" (string)").asString();

	if (handType == "v1") {
		str = "Name of the configuration file specifying the object sensing constants (string)";
		ConstString sensingCalibrationFile =
				rf.check("sensingCalib", Value("object_sensing.ini"), str).asString();

		Property p;
		p.fromConfigFile(sensingCalibrationFile);

		string part = partName.c_str();
		transform(part.begin(), part.end(), part.begin(), ::toupper);
		sensingCalibration = p.findGroup(part.c_str());

		this->handType = v1;
	} else {
		this->handType = GENERAL;
	}

	motionSpecificationFilename = rf.check("motionSpec", Value("motion_specification.ini"),
			"Name of the configuration file specifying the motions (string)").asString();

	str = prefix + getName(rf.check("control", Value("/control"),
			"Control port for communicating with the control board").asString());
	dataPorts.add(id.Control, str, NULL); // opened by PolyDriver!

	ConstString controlboardName(prefix + robotName + prefix + partName);

	Property p;
	p.put("device", "remote_controlboard");
	p.put("local", dataPorts.getName(id.Control)); // client
	p.put("remote", controlboardName); // server
	Bottle config(p.toString());

	if (!controlBoard.open(config)) {
		cerr << "Could not open remote control board (port name: " << controlboardName << ")" << endl;
		return false;
	}

	IEncoders* encoders;
	if (!controlBoard.view(encoders)) {
		cerr << "Motor control board does not provide encoders." << endl;
		return false;
	}

	IPidControl* pidControl;
	if (!controlBoard.view(pidControl)) {
		cerr << "Motor control board does not provide PID control." << endl;
		return false;
	}

	IPositionControl* posControl;
	if (!controlBoard.view(posControl)) {
		cerr << "Motor control board does not provide position control." << endl;
		return false;
	}

	IAmplifierControl* ampControl;
	if (!controlBoard.view(ampControl)) {
		cerr << "Motor control board does not provide amplifier control." << endl;
		return false;
	}

	int numAxes;
	encoders->getAxes(&numAxes);
	if (numAxes != HandMetrics::numAxes) {
		cerr << "The number of axes mismatches the expected number. "
				<< "It might be that the remote control board to connect to " << controlboardName << endl;
		return false;
	}

	motionSpecification.fromConfigFile(motionSpecificationFilename);

	return true;
}

bool HandModule::startThread() {
	bool b = AbstractRFModule::startThread();
	workerThread = dynamic_cast<HandWorkerThread *>(AbstractRFModule::workerThread);
	return b;
}

//#define DEBUG

HandModule::HandWorkerThread::HandWorkerThread(const OptionManager& moduleOptions,
		const Contactables& ports, PolyDriver& controlBoard, HandType t) :
	AbstractWorkerThread(moduleOptions, ports), controlBoard(controlBoard) {

	handType = t;
	hand = NULL;
	handv1 = NULL;

	// configure control board
	bool isValid = true;
	isValid &= controlBoard.view(encoders);
	isValid &= controlBoard.view(pidControl);
	isValid &= controlBoard.view(posControl);
	isValid &= controlBoard.view(ampControl);

	if (!isValid) {
		throw "Insufficient control board";
	}

	createHand();
}

void HandModule::HandWorkerThread::createHand() {
	switch (handType) {
	case v1:
		try {
			if (hand != NULL) {
				// This is necessary in case the Hand constructor throws an
				// exception. ...which it actually does the first time, because
				// the sensing constants aren't properly set in the beginning.
				delete hand;
			}
			handv1 = new Handv1(controlBoard, sensingConstants);
			HandModule::HandWorkerThread::hand = (Hand*) handv1;
		} catch (char const*) {
			hand = new Hand(controlBoard);
		}
		break;
	case GENERAL:
	default:
		hand = new Hand(controlBoard);
		break;
	}
}
void HandModule::HandWorkerThread::setMotionSpecification(Searchable& s) {
	motions.clear();
	addMotionSpecification(s);
}

void HandModule::HandWorkerThread::addMotionSpecification(Searchable& s) {
	Bottle b(s.toString());
	readMotionSequences(b, motions);

#ifdef DEBUG
	map<const string, MotionSequence>::const_iterator itr1;
	for (itr1 = motions.begin(); itr1 != motions.end(); ++itr1) {
		cout << "Motion type: " << itr1->first << endl;
		cout << itr1->second.toString() << endl;
	}
#endif
}


void HandModule::HandWorkerThread::setSensingConstants(::yarp::os::Searchable& s) {
	if (handType != v1) {
		cout << "Warning: You are trying to set sensing constants which is not \
			    	 necessary/ meant for the type of hand you specified." << endl;
	}

	Bottle b(s.toString());
	sensingConstants.clear();
	readMatrices(b, sensingConstants);

#ifdef DEBUG
	cout << "Sensing Constants" << endl;
	map<const string, Matrix>::const_iterator itr2;
	for (itr2 = sensingConstants.begin(); itr2 != sensingConstants.end(); ++itr2) {
		cout << (*itr2).first << "..." << endl;
		printMatrix((*itr2).second);
	}
#endif

	string osValueNames[] = { "thresholds", "offsets", "springs", "derivate_gain" };

	for (int i = 0; i < 4; i++) {
		Matrix& m = sensingConstants[osValueNames[i]];

		if (m.cols() != HandMetrics::numAxes) {
			cout << "Warning: The expected size of `" << osValueNames[i] << "` is " << HandMetrics::numAxes
					<< ", but it was " << m.cols() << ". The sizes will be automatically adopted!" << endl;
			resize(m, m.rows(), HandMetrics::numAxes);
		}
	}
	createHand();
}

HandModule::HandWorkerThread::~HandWorkerThread() {
	// Since derived classes are supposed to create the Hand object,
	// we cannot rely on its existence.
	if (hand != NULL) {
		delete hand;
	}
}

void HandModule::HandWorkerThread::setEnable(const std::vector<int>& joints, const bool b) {
	hand->setEnable(joints, b);
}

void HandModule::HandWorkerThread::getDisabledJoints(std::vector<int>& joints) {
	hand->getDisabledJoints(joints);
}

}
}
