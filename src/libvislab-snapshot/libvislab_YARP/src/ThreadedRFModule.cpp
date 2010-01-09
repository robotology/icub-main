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

#include "vislab/yarp/util/ThreadedRFModule.h"

#include <iostream>

using namespace yarp::os;
using namespace std;

using namespace vislab::util;

namespace vislab {
namespace yarp {
namespace util {

ThreadedRFModule::ThreadedRFModule(double period) :
	RFModule2(period) {
	workerThread = NULL;
}

ThreadedRFModule::ThreadedRFModule(ConstString name, double period) :
	RFModule2(name, period) {
	workerThread = NULL;
}

ThreadedRFModule::~ThreadedRFModule() {
}

bool ThreadedRFModule::startThread() {
	workerThread = createWorkerThread();
	if (workerThread == NULL) {
		cout << "The worker thread wasn't created properly" << endl;
		return false;
	}
	return workerThread->start();
}

bool ThreadedRFModule::interruptModule() {
	// Since derived classes are supposed to create the Thread object,
	// we cannot rely on its existence.
	bool b = (workerThread != NULL ? workerThread->stop() : true);
	return RFModule2::interruptModule() && b;
}

ThreadedRFModule::RFWorkerThread::RFWorkerThread(
		const vislab::yarp::util::OptionManager& moduleOptions, const Contactables& ports) {
	this->moduleOptions = moduleOptions;
	dataPorts = ports;
}

ThreadedRFModule::RFWorkerThread::~RFWorkerThread() {
}

bool ThreadedRFModule::RFWorkerThread::threadInit() {
	return true;
}

void ThreadedRFModule::RFWorkerThread::threadRelease() {
}

}
}
}
