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

#ifndef __ICUB_VISLAB_HANDMETRICS_H_
#define __ICUB_VISLAB_HANDMETRICS_H_

#include <vislab/yarp/all.h>
#include <vislab/yarp/util/all.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <cmath>
#include <stdexcept>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

namespace vislab {
namespace control {

/**
 * @ingroup libHandCtrl
 *
 * This class provides several metrics for a hand of the iCub.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class HandMetrics {
	bool newSnapshot;

protected:
	/** The encoders of the hand. */
	::yarp::dev::IEncoders* encoders;
	/** The PID controller of the hand. */
	::yarp::dev::IPidControl* pidControl;

	/**
	 * The current position of the hand in joint space. Use HandMetrics#getPosition()
	 * for valid values of the current position.
	 */
	::yarp::sig::Vector position;
	/** The previous position of the hand in joint space. */
	::yarp::sig::Vector prevPosition;
	/**
	 * The velocity of the individual joints. Use HandMetrics#getVelocity() for valid
	 * values of the current velocity.
	 */
	::yarp::sig::Vector velocity;

	/** The previous and current time stamp. */
	double prevTime, curTime;

public:
	/** The expected number of axes. */
	const static int numAxes = 16;

	/**
	 * The constructor.
	 * @param encoders The encoders of the hand.
	 * @param pidControl The PID controller of the hand.
	 * @param ampControl The amplifier controller of the hand.
	 * @param constants The sensing constants of the hand.
	 */
	HandMetrics(::yarp::dev::IEncoders* const encoders, ::yarp::dev::IPidControl* const pidControl);
	/**
	 * The destructor.
	 */
	virtual ~HandMetrics();

	/**
	 * Make a snapshot of the current state of the hand controller.
	 * Use this to receive updated values from the other functions.
	 */
	void snapshot();
#ifdef DEBUG
	/**
	 * Writes the data retrieved by the snapshot function in a human readable form to the provided stream.
	 * @param s The output stream.
	 */
	void printSnapshotData(std::ostream& s = std::cout);
#endif

	/**
	 * Returns the current positions of the hand's joints.
	 * @return The current positions of the hand's joints.
	 */
	const ::yarp::sig::Vector& getPosition();
	/**
	 * Returns the current velocity of the hand's joints.
	 * @return The current velocity of the hand's joints.
	 */
	const ::yarp::sig::Vector& getVelocity();
	/**
	 * Returns the time interval of the latest snapshot.
	 * @return The time interval of the latest snapshot.
	 */
	double getTimeInterval();
};

}
}

#endif /* __ICUB_VISLAB_HANDMETRICS_H_ */
