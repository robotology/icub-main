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

#ifndef __VISLAB_YARP_UTIL_RFMODULE2_H_
#define __VISLAB_YARP_UTIL_RFMODULE2_H_

#include <yarp/os/all.h>
#include <vislab/util/all.h>

#include "Contactables.h"
#include "Command.h"
#include "CommandManager.h"
#include "Option.h"
#include "OptionManager.h"

namespace vislab {
namespace yarp {
namespace util {

#define BREAK_IF_IS_STOPPING {if (isStopping()) { break; }}
#define DEFAULT_CALLING_PERIOD 1.0

/**
 * This class implementation an extension to {@link yarp::os::RFModule} but keeps fully backwards compatibly.
 *
 * This implementation already processes the input parameters for the names of the module and the robot and
 * takes over the closing and interrupting of data port that were added to {@link RFModule2#dataPorts}:
 * \code
 * 		dataPorts.add(id.Input_X, str, new BufferedPort<Vector>)
 * \endcode
 * One can open all added port with the following piece of code:
 * \code
 *		std::vector<unsigned int> errorLog;
 *		if (!dataPorts.open(&errorLog)) {
 *			for (unsigned int i = 0; i < errorLog.size(); i++) {
 *				std::cout << getName() << ": unable to open port " << dataPorts.getName(i) << std::endl;
 *			}
 *			return false; // unable to open
 *		}
 * \endcode
 *
 * The configuration method on the other hand reads in and set the module's and the robot's name ("name" & "robot" program argument).
 *
 * Furthermore it already implements the RPC command interface with a few basic command ("echo", "set", "help", "quit").
 * New remote commands are added by using the addRemoteCommand(Command*) function:
 * \code
 * 		addRemoteCommand(new EchoCommand(this))
 * \endcode
 *
 * Options which can be accessed and modified with the "set" command, can be added as follows:
 * \code
 * 		addModuleOption(new Option("foo", "bar", Option::ON_OFF, Option::ON))
 * \endcode
 *
 * Valid option type are:
 * - SIMPLE: simple string value
 * - NUMERIC: number: integer | double
 * - BOOLEAN: boolean value: true | false
 * - ON_OFF: boolean vlaue: on | off
 * - VECTOR: vector of numeric values as comma seperated list
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class RFModule2: public ::yarp::os::RFModule {
	class EchoCommand: public Command {
		virtual bool execute(const ::yarp::os::Bottle& params, ::yarp::os::Bottle& reply) const;
	public:
		EchoCommand(RFModule2* const parent) :
			Command(parent, "echo", "Writes back the given string. echo \"<STRING>\"") {
		}
	};

	class SetCommand: public vislab::yarp::util::Command {
		virtual bool execute(const ::yarp::os::Bottle& params, ::yarp::os::Bottle& reply) const;
	public:
		SetCommand(RFModule2* const parent) :
					Command(
							parent,
							"set",
							"Sets a given value: 'set blubb bla' 'set \"something important\" \"1 2 3\"' 'set' gives you the key-values pairs") {
		}
	};
	friend class SetCommand;

	class HelpCommand: public Command {
		virtual bool execute(const ::yarp::os::Bottle& params, ::yarp::os::Bottle& reply) const;
	public:
		HelpCommand(RFModule2* const parent) :
			Command(parent, "help", "Shows the help messages.") {
		}
	};
	friend class HelpCommand;

	bool createHandlerPort;

protected:
	/** The interval in which the module's updateModule() functions gets called. */
	double period;
	/** The "standard" prefix for YARP ports "/". */
	::yarp::os::ConstString prefix;
	/** The module's name. */
	::yarp::os::ConstString moduleName;
	/** The name of the robot to be used. */
	::yarp::os::ConstString robotName;

	/** The ports to be used by the module. */
	Contactables dataPorts;
	/** The RPC communication port of the module. */
	::yarp::os::Port handlerPort; //< A port to handle command messages.

	/** The RPC commands of the module. */
	CommandManager remoteCommands;
	/** A {@link std::map} of module options stored as key value pairs. */
	OptionManager moduleOptions;

	/**
	 * Adds a new {@link Command} for the module's RPC communication.
	 * @param cmd The {@link Command} to be added.
	 */
	void addRemoteCommand(Command* cmd);
	/**
	 * Adds a new {@link Option} with can be altered by the module's RPC communication port.
	 * @param o The {@link Option} to be added.
	 */
	void addModuleOption(Option* o);

	void init(const char* name, bool handlerPortAutomation, double period);

public:
	/**
	 * The constructor.
	 * @param handlerPortAutomation Dis-/Enable the automated creation of the RPC handler as soon
	 * as the name of the module is set using the setName() functions.
	 * @param period The interval in which the module's updateModule() functions gets called.
	 */
	RFModule2(bool handlerPortAutomation = false, double period = DEFAULT_CALLING_PERIOD);
	/**
	 * The constructor.
	 * @param name The modules name.
	 * @param period The interval in which the module's updateModule() functions gets called.
	 */
	RFModule2(const char* name, double period = DEFAULT_CALLING_PERIOD);
	/**
	 * The destructor.
	 */
	virtual ~RFModule2();

	/**
	 * @see yarp::os::RFModule#configure(yarp::os::ResourceFinder)
	 */
	virtual bool configure(::yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
	/**
	 * @see yarp::os::RFModule#configure(const char*)
	 */
	void setName(const char* name);
	/**
	 * Returns an indicator of if the handler port is available/ opened.
	 * @return An indicator of if the handler port is available/ opened.
	 */
	bool isHandlerAvailable();
	/**
	 * @see yarp::os::RFModule#interruptModule()
	 */
	virtual bool interruptModule(); // interrupt, e.g., the ports
	/**
	 * @see yarp::os::RFModule#close()
	 */
	virtual bool close(); // close and shut down the module
	/**
	 * @see yarp::os::RFModule#respond(yarp::os::Bottle, yarp::os::Bottle)
	 */
	virtual bool respond(const ::yarp::os::Bottle& command, ::yarp::os::Bottle& reply);
	/**
	 * @see yarp::os::RFModule#getPeriod()
	 */
	virtual double getPeriod();
	/**
	 * @see yarp::os::RFModule#updateModule()
	 */
	virtual bool updateModule();
};

}
}
}
#endif /* __VISLAB_YARP_UTIL_RFMODULE2_H_ */
