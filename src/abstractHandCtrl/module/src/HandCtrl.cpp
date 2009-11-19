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

#include "iCub/vislab/HandCtrl.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;

using namespace vislab::util;
using namespace vislab::yarp::util;

namespace vislab {
namespace control {

HandCtrl::HandCtrl() :
	HandModule("AbstractHandCtrl") {
}

HandCtrl::~HandCtrl() {
}

bool HandCtrl::DoCommand::execute(const Bottle& params, Bottle& reply) const {
	HandCtrl* g = (HandCtrl*) parent;

	const string motion = params.get(1).toString().c_str();
	vector<string> motionParameters;
	explode(motion, ",", motionParameters);
	if (!motionParameters.empty()) {
		if (motionParameters.size() <= 1) {
			// TODO: decide on further parameters for the motion
			// motionParameters.push_back("none");
		}
		g->workerThread->doMotion(motionParameters[0].c_str());
		reply.addVocab(Vocab::encode("ack"));
	} else {
		reply.addString("Incomplete request! No type specified (\"do MOTION_ID\")");
	}
	return true;
}

bool HandCtrl::BlockCommand::execute(const Bottle& params, Bottle& reply) const {
	HandCtrl* p = (HandCtrl*) parent;
	vector<int> blockParameters;
	parseListOf<int> (params.get(1).toString(), blockParameters);

	if (blockParameters.empty()) {
		p->workerThread->setEnable(blockParameters, false);
		ostringstream ss;
		if (blockParameters.empty()) {
			ss << "no blocked joints";
		} else {
			for (unsigned int i = 0; i < blockParameters.size(); i++) {
				ss << blockParameters[i];
				if (i < blockParameters.size() - 1) {
					ss << ",";
				}
			}
		}
		reply.addString(ss.str().c_str());
	} else {
		p->workerThread->setEnable(blockParameters, false);
		reply.addVocab(Vocab::encode("ack"));
	}
	return true;
}

bool HandCtrl::UnBlockCommand::execute(const Bottle& params, Bottle& reply) const {
	HandCtrl* p = (HandCtrl*) parent;
	vector<int> unblockParameters;
	parseListOf<int> (params.get(1).toString(), unblockParameters);

	if (!unblockParameters.empty()) {
		p->workerThread->setEnable(unblockParameters, true);
		reply.addVocab(Vocab::encode("ack"));
	} else {
		reply.addString("Incomplete request! No type specified (\"unblock 1,2,3\")");
	}
	return true;
}

bool HandCtrl::configure(ResourceFinder &rf) {
	if (!HandModule::configure(rf)) {
		return false;
	}

	ConstString str;

	addRemoteCommand(new DoCommand(this));
	addRemoteCommand(new BlockCommand(this));
	addRemoteCommand(new UnBlockCommand(this));

	str = "Dis-/Enables the motion recorder";
	addModuleOption(new Option("record", str, Option::ON_OFF, Option::OFF));
	str = "Dis-/Enables the direct control of the arm";
	addModuleOption(new Option("direct control", str, Option::ON_OFF, Option::OFF));

	str
			= "Input port providing the target configuration for the hand's joints as Vector(12) (c.f. http://eris.liralab.it/wiki/ICub_joints excl. shoulder & elbow)";
	str = prefix + getName(rf.check("q", Value("/q:i"), str).asString());
	dataPorts.add(id.Input_Q, str, new BufferedPort<Vector> );

	vector<unsigned int> errorLog;
	if (!dataPorts.open(&errorLog)) {
		for (unsigned int i = 0; i < errorLog.size(); i++) {
			cout << getName() << ": unable to open port " << dataPorts.getName(i) << endl;
		}
		return false; // unable to open; let RFModule know so that it won't run
	}

	str
			= "RPC port to communicate with the iKin arm controller via Bottle(Vocab:cmd) objects (c.f. iKinArmCtrl rpc).";
	str = prefix + getName(rf.check("iKin", Value(prefix +partName +"/rpc"), str).asString());
	dataPorts.add(id.RPC_iKin, str, new Port());

	workerThread = new WorkerThread(moduleOptions, dataPorts, id, motionSpecification, controlBoard, handType);
//	workerThread->addMotionSpecification(motionSpecification);
	workerThread->setSensingConstants(sensingCalibration);
	return workerThread->start();
}

bool HandCtrl::close() {
	/* stop the thread */
	return HandModule::close() && workerThread->stop();
}

HandCtrl::WorkerThread::WorkerThread(const OptionManager& moduleOptions, const Contactables& ports,
		const struct PortIds id, Searchable& motionSpec, PolyDriver& controlBoard, HandType t) :
	HandWorkerThread(moduleOptions, ports, controlBoard, t) {

	this->id = id;
	addMotionSpecification(motionSpec);
	iKinArmCtrl = (Port*) dataPorts[id.RPC_iKin];
}

void HandCtrl::WorkerThread::waitForIKinArmCtrl() {
	while (!iKinArmCtrlComm("run") && !isStopping()) {
		Time::delay(1);
	}
}

bool HandCtrl::WorkerThread::iKinArmCtrlComm(const ConstString cmd) {
	Bottle out, in;
	out.clear();
	out.addVocab(Vocab::encode(cmd));
	iKinArmCtrl->write(out, in);
	bool b = (in.get(0).asVocab() == VOCAB3('a', 'c', 'k'));

#ifdef DEBUG
	cout << "iKinCommand: " << (b ? "success" : "failure") << endl;
#endif
	return b;
}

void HandCtrl::WorkerThread::doMotion(const ConstString type) {
	mutex.wait();
	motionQueue.push(type);
	mutex.post();
}

void HandCtrl::WorkerThread::run() {
	BufferedPort<Vector>* q = (BufferedPort<Vector>*) dataPorts[id.Input_Q];

	MotionSequence prevSequence;
	Vector initPosition(numAxes); // all zero
	initPosition[Hand::HAND_FINGER] = 40;

	// just in case iKinArmCtrl is running
	iKinArmCtrlComm("susp");
	hand->move(initPosition, Hand::COMPLETE_HAND);

	while (!isStopping()) {

		if (moduleOptions["direct control"].getValue().asString() == "on") {
			Vector* v = q->read(false);
			if (v != NULL) {
				const int expectedVectorSize = 12;
				int offset = max(0, numAxes - (v->size() - expectedVectorSize));

				Vector position(numAxes);
				for (int i = 0; i < numAxes - offset; i++) {
					position[offset + i] = (*v)[i];
				}
				hand->move(position);
			}
		} else {
			mutex.wait();
			bool isEmpty = motionQueue.empty();
			mutex.post();
			if (!isEmpty) {

				mutex.wait();
				ConstString cmd = motionQueue.front();
				motionQueue.pop();
				mutex.post();

				cout << "Motion Idtype: " << cmd << endl;

				if (motions.find(cmd.c_str()) != motions.end()) {
					MotionSequence& sequence = motions[cmd.c_str()];

					// Decide on which joints need to be rolled back
					hand->getMetrics().snapshot();
					Vector position = hand->getMetrics().getPosition();
					cout << "Position: ";
					printVector(position);

					set<int> joints;
					set<int>::const_iterator itr;
					for (itr = Hand::COMPLETE_HAND.begin(); itr != Hand::COMPLETE_HAND.end(); ++itr) {
						int i = *itr;
						if (abs(position[i] - initPosition[i]) >= 1.0) {
							joints.insert(i);
						}
					}
					hand->move(prevSequence, joints, true);
					hand->move(initPosition);
					hand->move(sequence);

					prevSequence = sequence;
				} else {
					cout << "Unknown command `" << cmd << "`" << endl;
				}
			}
		}
	}
}

bool HandCtrl::updateModule() {
	return HandModule::updateModule();
}

}
}
