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

#ifndef __VISLAB_YARP_UTIL_ABSTRACTRFMODULE_H_
#define __VISLAB_YARP_UTIL_ABSTRACTRFMODULE_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>

#include <iCub/eye2world/contactables.h>
#include <iCub/eye2world/command.h>
#include <iCub/eye2world/commandManager.h>
#include <iCub/eye2world/option.h>
#include <iCub/eye2world/optionManager.h>
#include <iCub/eye2world/threadedRFModule.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace vislab {
namespace yarp {
namespace util {

/**
 * @deprecated
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class AbstractRFModule: public ThreadedRFModule {
protected:
	/**
	 * @deprecated
	 *
	 * @author Christian Wressnegger
	 * @date 2009
	 */
	class AbstractWorkerThread: public ThreadedRFModule::RFWorkerThread {
	public:
		/**
		 * The constructor.
		 * @param moduleOptions The module's options.
		 * @param ports The ports to be used by the module.
		 */
		AbstractWorkerThread(const vislab::yarp::util::OptionManager& moduleOptions,
				const vislab::yarp::util::Contactables& ports);
		/**
		 * The destructor.
		 */
		virtual ~AbstractWorkerThread();
	};

public:
	/**
	 * The constructor.
	 * @param name The modules name.
	 */
	AbstractRFModule(::yarp::os::ConstString name);
	/**
	 * The destructor.
	 */
	virtual ~AbstractRFModule();
};

}
}
}
#endif /* __VISLAB_YARP_UTIL_ABSTRACTRFMODULE_H_ */
