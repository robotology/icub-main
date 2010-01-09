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

#ifndef __VISLAB_YARP_UTIL_TIMEDMOTIONSEQUENCE__H_
#define __VISLAB_YARP_UTIL_TIMEDMOTIONSEQUENCE__H_

#include <ostream>
#include <vector>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>

#include <ace/OS.h>

#include "common.h"

namespace vislab {
namespace yarp {
namespace util {

/**
 * Represent a sequence of motions. Therefore it makes use of {@link Motion} objects.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class TimedMotionSequence : public MotionSequence {

	std::vector<double> motionTimestamps;

public:
	/**
	 * The constructor.
	 */
	TimedMotionSequence();
	/**
	 * The constructor.
	 * @param numJoints The number of joint to be moved in the scope of this motion.
	 */
	TimedMotionSequence(unsigned int numJoints);

	/**
	 * Add a new {@link Motion} specification to the sequence. Once the number of joints is fixed
	 * (MotionSequence#setNumJoints(const int)) one is only able to pass {@link Motion} specifications
	 * with the specified number of joints.
	 * @param m The motion to be added.
	 */
	void addMotion(Motion& m);
};

}
}
}

#endif /* __VISLAB_YARP_UTIL_TIMEDMOTIONSEQUENCE__H_ */
