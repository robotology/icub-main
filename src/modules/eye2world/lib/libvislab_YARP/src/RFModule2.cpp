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

#include "vislab/yarp/util/RFModule2.h"

#include <iostream>
#include <sstream>

using namespace yarp::os;
using namespace std;

using namespace vislab::util;

namespace vislab {
namespace yarp {
namespace util {

RFModule2::RFModule2(bool handlerPortAutomation, double period) {
	init("myModule", false, period);
}

RFModule2::RFModule2(const char* name, double period) {
	init(name, true, period);
}

RFModule2::~RFModule2() {
}

void RFModule2::init(const char* name, bool handlerPortAutomation, double period) {
	prefix = "/";
	moduleName = name;
	createHandlerPort = handlerPortAutomation;
	this->period = period;
}

bool RFModule2::EchoCommand::execute(const Bottle& params, Bottle& reply) const {
	reply.addString(params.get(1).asString().c_str());
	return true;
}

bool RFModule2::SetCommand::execute(const Bottle& params, Bottle& reply) const {
	if (params.size() <= 1) { // only the "set" command
		addMultilineString(reply, parent->moduleOptions.toString());
	} else {
		const Value key = params.get(1);
		const Value value = params.get(2);
		if (parent->moduleOptions.contains(key.toString())) {
			ostringstream oss;

			Option& o = parent->moduleOptions[key.toString()];
			o.setValue(value, &oss);
			ConstString str = oss.str().c_str();

			reply.addString(str.length() <= 0 ? o.toString() : str);
		} else {
			reply.addString("Unsupported option");
		}
	}
	return true;
}

bool RFModule2::HelpCommand::execute(const Bottle& params, Bottle& reply) const {
	ostringstream oss;
	oss << parent->getName().c_str() << " commands are:" << endl;
	oss << parent->remoteCommands.toString();
	if (!parent->moduleOptions.empty()) {
		oss << endl << parent->getName().c_str() << " options are:" << endl;
		oss << parent->moduleOptions.getDescription().c_str();
	}
	addMultilineString(reply, oss.str().c_str());
	return true;
}

void RFModule2::addRemoteCommand(Command* cmd) {
	remoteCommands.add(cmd);
}

void RFModule2::addModuleOption(Option* o) {
	moduleOptions.add(o);
}

bool RFModule2::configure(ResourceFinder &rf) {

	ConstString str = rf.check("name", Value(moduleName), "module name (string)").asString();
	setName(str.c_str()); // modulePortName

	robotName = rf.check("robot", Value("myRobot"), "The robot's name (string)").asString();

	addRemoteCommand(new SetCommand(this));
	addRemoteCommand(new EchoCommand(this));
	addRemoteCommand(new HelpCommand(this));

	// addModuleOption(new Option("foo", "bar", Option::SIMPLE, Value("bar")));
	// addModuleOption(new Option("foo", "bar", Option::ON_OFF, Option::ON)));
	// addModuleOption(new Option("foo", "bar", Option::NUMBER, Value("7.0")));
	// addModuleOption(new Option("foo", "bar", Option::BOOLEAN, Option::TRUE)));

	return !createHandlerPort || isHandlerAvailable();
}

void RFModule2::setName(const char *name) {
	RFModule::setName(name);

	if (createHandlerPort) {
		// Ensure that the RPC port is named after the latest module name.
		if (isHandlerAvailable()) {
			handlerPort.interrupt();
			handlerPort.close();
			//detachTerminal();
		}

		// attach a port of the same name as the module (prefixed with a /) to the module
		// so that messages received from the port are redirected to the respond method
		if (handlerPort.open(prefix + getName())) {
			attach(handlerPort);
#ifndef __unix__
			// TODO: Hmmm, the warning actually appears on windows and unix systems
			attachTerminal();
#endif
		} else {
			cout << getName() << ": unable to open port " << handlerPort.getName() << endl;
		}
	}
}

bool RFModule2::isHandlerAvailable() {
	return createHandlerPort ? handlerPort.where().isValid() : false;
}

bool RFModule2::interruptModule() {
	dataPorts.interrupt();
	handlerPort.interrupt();
	return true;
}

bool RFModule2::close() {
	dataPorts.close();
	handlerPort.close();
	return true;
}

bool RFModule2::respond(const Bottle& command, Bottle& reply) {
	bool b = remoteCommands.respond(command, reply);
	return b ? b : RFModule::respond(command, reply);
}

bool RFModule2::updateModule() {
	return true;
}

double RFModule2::getPeriod() {
	return period;
}

}
}
}
