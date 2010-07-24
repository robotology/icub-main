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

#ifndef __VISLAB_YARP_UTIL_THREADEDRFMODULE_H_
#define __VISLAB_YARP_UTIL_THREADEDRFMODULE_H_

#include "Contactables.h"
#include "OptionManager.h"

#include <yarp/os/all.h>

#include "RFModule2.h"

namespace vislab {
namespace yarp {
namespace util {

/**
 * The "abstract" implementation of a {@link yarp::os::RFModule}. C++ doesn't know abstract classes that doesn't
 * contain abstract function as for instance Java does. However, this class is supposed to inherited.
 *
 * This implementation already processes the input parameters for the names of the module and the robot and
 * takes over the closing and interrupting of data port that were added to {@link AbstractRFModule#dataPorts}:
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
 * The configuration method already reads in and set the module's and the robot's name ("name" & "robot" program argument).
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
 * @author Christian Wressnegger
 * @date 2009
 */
class ThreadedRFModule: public RFModule2 {
protected:
	/**
	 * The worker thread for ({@link ThreadedRFModule}s.
	 *
	 * @author Christian Wressnegger
	 * @date 2009
	 */
	class RFWorkerThread: public ::yarp::os::Thread {
	protected:
		/** The ports to be used by the module. */
		vislab::yarp::util::Contactables dataPorts;
		/** The options of the parent module. */
		vislab::yarp::util::OptionManager moduleOptions;

	public:

		/**
		 * The constructor.
		 * @param moduleOptions The module's options.
		 * @param ports The ports to be used by the module.
		 */
		RFWorkerThread(const vislab::yarp::util::OptionManager& moduleOptions,
				const vislab::yarp::util::Contactables& ports);
		/**
		 * The destructor.
		 */
		virtual ~RFWorkerThread();

		/**
		 * @see Thread#threadInit()
		 */
		virtual bool threadInit();
		/**
		 * @see Thread#threadRelease()
		 */
		virtual void threadRelease();
	};

	::yarp::os::Thread* workerThread;

	virtual bool startThread();
	virtual ::yarp::os::Thread* createWorkerThread() = 0;

public:
	/**
	 * The constructor.
	 * @param period The interval in which the module's updateModule() functions gets called.
	 */
	ThreadedRFModule(double period = DEFAULT_CALLING_PERIOD);
	/**
	 * The constructor.
	 * @param name The modules name.
	 * @param period The interval in which the module's updateModule() functions gets called.
	 */
	ThreadedRFModule(::yarp::os::ConstString name, double period = DEFAULT_CALLING_PERIOD);
	/**
	 * The destructor.
	 */
	virtual ~ThreadedRFModule();
	/**
	 * @see RFModule2#interruptModule()
	 */
	virtual bool interruptModule(); // interrupt, e.g., the ports
};

}
}
}
#endif /* __VISLAB_YARP_UTIL_THREADEDRFMODULE_H_ */
