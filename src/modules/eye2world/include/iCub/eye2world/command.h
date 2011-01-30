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

#ifndef __VISLAB_YARP_UTIL_COMMAND_H_
#define __VISLAB_YARP_UTIL_COMMAND_H_

#include <sstream>
#include <string>

#include <yarp/os/all.h>

namespace vislab {
namespace yarp {
namespace util {

class RFModule2;

/**
 * Represent a remote command that can be used for the communicate with {@link yarp::os::RFModule}s
 * as part of the RPC channel.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class Command {

	const ::yarp::os::ConstString name;
	const ::yarp::os::ConstString description;

protected:
	/** The hosting {@link yarp::os::RFModule}. */
	RFModule2* parent;

public:
	/**
	 * The constructor.
	 * @param parent The hosting {@link yarp::os::RFModule}.
	 * @param name The name/ identifier of the {@link Command}.
	 * @param description The description of the {@link Command}.
	 */
	Command(RFModule2* const parent, const ::yarp::os::ConstString name,
			const ::yarp::os::ConstString description = "");
	/**
	 * The destructor.
	 */
	virtual ~Command();

	/**
	 * Returns the name of the {@link Command}.
	 * @return The name of the {@link Command}.
	 */
	const ::yarp::os::ConstString getName() const;
	/**
	 * Returns the description of the {@link Command}.
	 * @return The description of the {@link Command}.
	 */
	virtual const ::yarp::os::ConstString getDescription() const;

	/**
	 * Returns a human readable string that describes this object.
	 * @return A human readable string that describes this object.
	 */
	virtual ::yarp::os::ConstString toString() const;
	/**
	 * The actual action of the {@link Command}.
	 * @param params The parameters for this action.
	 * @param reply The result of this action.
	 * @return An indicator of if the action was successfully executed.
	 * This is not related to the success of the desired functionality.
	 */
	virtual bool execute(const ::yarp::os::Bottle& params, ::yarp::os::Bottle& reply) const = 0;

};

}
}
}

#endif /* __VISLAB_YARP_UTIL_COMMAND_H_ */
