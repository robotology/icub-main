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

#ifndef __VISLAB_YARP_UTIL_OPTIONMANAGER_H_
#define __VISLAB_YARP_UTIL_OPTIONMANAGER_H_

#include <vislab/util/all.h>
#include "Option.h"
#include "SynchronizedNameMap.h"

#include <sstream>
#include <yarp/os/all.h>

namespace vislab {
namespace yarp {
namespace util {

class OptionManager: public SynchronizedNameMap<Option> {
public:
	OptionManager();
	virtual ~OptionManager();

	::yarp::os::ConstString getDescription();
};

}
}
}

#endif /* __VISLAB_YARP_UTIL_OPTIONMANAGER_H_ */
