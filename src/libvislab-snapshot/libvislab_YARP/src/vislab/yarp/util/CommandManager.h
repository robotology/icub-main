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

#ifndef __VISLAB_YARP_UTIL_COMMANDMANAGER_H_
#define __VISLAB_YARP_UTIL_COMMANDMANAGER_H_

#include "Command.h"
#include "SynchronizedNameMap.h"

#include <sstream>

#include <vislab/util/all.h>
#include <yarp/os/all.h>


namespace vislab {
namespace yarp {
namespace util {

/**
 * Manage individual {@link Command}s. This manager is supposed to be used in connection
 * with a {@link yarp::os::RFModule}'s response function.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class CommandManager : public SynchronizedNameMap<Command> {

public:
	/**
	 * The constructor.
	 */
	CommandManager();
	/**
	 * The destructor.
	 */
	virtual ~CommandManager();

	/**
	 * Respond to the provided message/ command according to the {@link Command}s hosted
	 * by this manager.
	 * @param command The identifier of the {@link Command} to be executed.
	 * @param reply The result of the requested {@link Command}.
	 * @return "false" if there was an unknown command aka according to YARP if there was no critical failure ô_O.
	 */
	bool respond(const ::yarp::os::Bottle& command, ::yarp::os::Bottle& reply);
};

}
}
}

#endif /* __VISLAB_YARP_UTIL_COMMANDMANAGER_H_ */
