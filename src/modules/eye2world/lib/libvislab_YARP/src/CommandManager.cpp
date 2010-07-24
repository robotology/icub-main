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

#include "vislab/yarp/util/CommandManager.h"

#include <iostream>

using namespace std;
using namespace yarp::os;

namespace vislab {
namespace yarp {
namespace util {

CommandManager::CommandManager() {
}

CommandManager::~CommandManager() {
}

bool CommandManager::respond(const Bottle& command, Bottle& reply) {
	reply.clear();
	ConstString cmd = command.get(0).asString();

	if (contains(cmd.c_str())) {
		return operator[](cmd.c_str()).execute(command, reply);
	}
	return false;
}

}
}
}
