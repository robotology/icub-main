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

#include "vislab/yarp/util/OptionManager.h"

using namespace std;
using namespace yarp::os;

namespace vislab {
namespace yarp {
namespace util {

OptionManager::OptionManager() {
}

OptionManager::~OptionManager() {
}

ConstString OptionManager::getDescription() {
	ostringstream oss;
	if (entries.empty()) {
		oss << "No entries available";
	} else {
		SynchronizedNameMap<Option>::const_iterator itr = begin();
		oss << itr->getName().c_str() << ": " << itr->getDescription().c_str();
		++itr;
		for (; itr != end(); ++itr) {
			oss << std::endl << itr->getName().c_str() << ": " << itr->getDescription().c_str();
		}
	}
	return oss.str().c_str();
}

}
}
}
