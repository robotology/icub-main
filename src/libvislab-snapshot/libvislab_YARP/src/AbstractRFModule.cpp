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

#include "vislab/yarp/util/AbstractRFModule.h"

#include <iostream>

using namespace yarp::os;
using namespace std;

using namespace vislab::util;

namespace vislab {
namespace yarp {
namespace util {

#define WARNING "\nWARNING: AbstractRFModule class is deprecated uses ThreadedRFModule or RFModule2 instead.\n\n"

AbstractRFModule::AbstractRFModule(ConstString name) :
	ThreadedRFModule(name) {
	cerr << WARNING << endl;
	Time::delay(1);
}

AbstractRFModule::~AbstractRFModule() {
}

AbstractRFModule::AbstractWorkerThread::AbstractWorkerThread(const OptionManager& moduleOptions,
		const Contactables& ports) :
	ThreadedRFModule::RFWorkerThread(moduleOptions, ports) {
}

AbstractRFModule::AbstractWorkerThread::~AbstractWorkerThread() {
	cerr << WARNING << endl;
	Time::delay(1);
}

}
}
}
