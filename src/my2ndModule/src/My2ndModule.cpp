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

#include "iCub/vislab/My2ndModule.h"

using namespace std;

using namespace vislab::util;
using namespace vislab::yarp::util;

using namespace yarp::os;
using namespace yarp::sig;

namespace vislab {
namespace demo {

#ifndef IGNORE_THE_FANCY_EXTENSIONS
bool My2ndModule::WriteCommand::execute(const Bottle& params, Bottle& reply) const {
	My2ndModule* p = (My2ndModule*) parent;

	Vector v(3);
	v[0] = 1;
	v[1] = 2;
	v[2] = 3;

	Motion m;
	m.setTiming(3.0);
	m.setVelocity(v);
	m.setPosition(v);

	MotionSequence seq;
	seq.addMotion(m);

	if (!seq.toFile(p->getLogFilename().c_str())) {
		reply.addString("unable to write file.");
	}
	return true;
}

const ConstString My2ndModule::getLogFilename() {
	return moduleOptions["log"].getValue().asString();
}

#endif

My2ndModule::My2ndModule()
#ifndef IGNORE_THE_FANCY_EXTENSIONS
:
	RFModule2("my2ndModule")
#endif
{
}

bool My2ndModule::configure(ResourceFinder &rf) {
#ifndef IGNORE_THE_FANCY_EXTENSIONS

	if (!RFModule2::configure(rf)) {
		return false;
	}

	ConstString logFile = rf.check("log", Value("seq.log")).asString();

	// r1: Specify remote commmands.
	addRemoteCommand(new WriteCommand(this));

	// r2: Specify remote options
	ConstString str = "Contains the name of the output file.";
	addModuleOption(new Option("log", str, Option::SIMPLE, Value(logFile)));

	addModuleOption(new Option("v1", "unused boolean option", Option::BOOLEAN, Option::TRUE_));
	addModuleOption(new Option("v2", "another unused boolean option", Option::ON_OFF, Option::ON));
	addModuleOption(new Option("v3", "unused numeric option", Option::NUMERIC, Value(1.0)));

	Value* v = Value::makeList("1.0 2 3.3");
	addModuleOption(new Option("v4", "unused vector option", Option::VECTOR, *v));
	delete v;

	// p1: Add ports
	str = prefix + getName(rf.check("in", Value("/in"),
			"Dead input port. This won't be used and is only there for demo purposes.").asString());
	dataPorts.add(id.in, str, new BufferedPort<Bottle> );

	str = prefix + getName(rf.check("out", Value("/out"),
			"Output port providing the following string: \"my2ndModule demo\"").asString());
	dataPorts.add(id.out, str, new BufferedPort<Bottle> );

	// p2: Open ports
	vector<unsigned int> errorLog;
	if (!dataPorts.open(&errorLog)) {
		for (unsigned int i = 0; i < errorLog.size(); i++) {
			cout << getName() << ": unable to open port " << dataPorts.getName(i) << endl;
		}
		return false; // unable to open; let RFModule know so that it won't run
	}

#endif
	return true;
}

bool My2ndModule::updateModule() {
#ifndef IGNORE_THE_FANCY_EXTENSIONS

	// p3: Access ports

	//	BufferedPort < Bottle > *in = (BufferedPort<Bottle>*) dataPorts[id.in];
	BufferedPort < Bottle > *out = (BufferedPort<Bottle>*) dataPorts[id.out];

	Bottle &smth = out->prepare();
	smth.clear();
	smth.addString("my2ndModule demo");
	out->write();

#endif
	return true;
}

}
}
