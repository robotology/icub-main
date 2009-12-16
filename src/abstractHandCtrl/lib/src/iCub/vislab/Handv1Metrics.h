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

#ifndef __ICUB_VISLAB_HANDv1METRICS_H_
#define __ICUB_VISLAB_HANDv1METRICS_H_

#include <vislab/yarp/all.h>
#include <vislab/yarp/util/all.h>


#include <iostream>
#include <sstream>
#include <cmath>
#include <string>
#include <map>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include "HandMetrics.h"

namespace vislab {
namespace control {

/**
 * @ingroup libHandCtrl
 *
 * This class extends the initially provided metrics for a hand of the iCub in order to
 * determine the actually used voltage and the error with respect to the supposed voltage.
 * This allows us in further follow to determine if the hand holds an object or not without
 * any tacktile sensors.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class Handv1Metrics : public HandMetrics {

	/** The output limit of all hand PIDs. */
	const static double pidLimit;

	/** The threshold of the residual voltage at which.... */
	const static double residualVoltageThres;

	/** The amplifier controller of the hand. */
	::yarp::dev::IAmplifierControl* ampControl;
	/** The value of the PIDs before they were changed by this class. */
	::yarp::dev::Pid prevPids[numAxes];

	/** The sensing constants of the hand (cf. Handv1#calibrate). */
	std::map<const std::string, ::yarp::sig::Matrix>& objectSensingConstants;

	/**
	 * The current voltage of the hand in joint space. Use HandMetrics#getVoltage()
	 * for valid values of the current voltage.
	 */
	::yarp::sig::Vector voltage;
	/** The target voltage of the hand in joint space. */
	::yarp::sig::Vector targetVoltage;
	/** The error of the current voltage with respect to the target voltage. */
	::yarp::sig::Vector error;

	/** The previous voltage offset. */
	::yarp::sig::Vector prevVoltageOffset;
	/** The previous spring stiffness constants. */
	::yarp::sig::Vector prevSpringStiffness;

protected:
	/**
	 * Returns the current voltage of the hand's joints.
	 * @param sync Synchronized executing of the function.
	 * @return The current voltage of the hand's joints.
	 */
	const ::yarp::sig::Vector getVoltage(const bool sync);
	/**
	 * Returns the current error of the hand's joints.
	 * @param sync Synchronized executing of the function.
	 * @return The current error of the hand's joints.
	 */
	const ::yarp::sig::Vector getError(const bool sync);

public:
	/**
	 * The constructor.
	 * @param encoders The encoders of the hand.
	 * @param pidControl The PID controller of the hand.
	 * @param ampControl The amplifier controller of the hand.
	 * @param constants The sensing constants of the hand.
	 */
	Handv1Metrics(::yarp::dev::IEncoders* const encoders, ::yarp::dev::IPidControl* const pidControl,
			::yarp::dev::IAmplifierControl* const ampControl, std::map<const std::string, ::yarp::sig::Matrix>& constants);
	/**
	 * The Handv1Metrics.
	 */
	virtual ~Handv1Metrics();

	/**
	 * Returns the current voltage of the hand's joints.
	 * @return The current voltage of the hand's joints.
	 */
	const ::yarp::sig::Vector getVoltage();
	/**
	 * Returns the current error of the hand's joints.
	 * @return The current error of the hand's joints.
	 */
	const ::yarp::sig::Vector getError();
};

}
}

#endif /* __ICUB_VISLAB_HANDv1METRICS_H_ */
