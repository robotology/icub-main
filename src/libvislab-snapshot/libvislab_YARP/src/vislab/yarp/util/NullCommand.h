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

#ifndef __VISLAB_UTIL_YARP_NULLCOMMAND_H_
#define __VISLAB_UTIL_YARP_NULLCOMMAND_H_

#include "Command.h"
#include <yarp/os/all.h>

namespace vislab {
namespace yarp {
namespace util {

/**
 * Represent a dummy {@link Command} that doesn't have a parent {@link yarp::os::RFModule}
 * and doesn't execute a particular action.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class NullCommand: public Command {
public:
	/**
	 * The constructor.
	 * @param name The name of this command.
	 * @param description The description of this command.
	 */
	NullCommand(const ::yarp::os::ConstString name, const ::yarp::os::ConstString description = "") :
		Command(NULL, name, description) {
	}
	;
	/**
	 * The destructor.
	 */
	virtual ~NullCommand();

	/**
	 * @see Command#respond(yarp::os::Bottle, yarp::os::Bottle)
	 */
	virtual bool execute(const ::yarp::os::Bottle& params, ::yarp::os::Bottle& reply) {
		return true;
	}
};

}
}
}

#endif /* __VISLAB_UTIL_YARP_NULLCOMMAND_H_ */
