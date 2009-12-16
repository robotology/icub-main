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

#ifndef __ICUB_VISLAB_HANDv1_H_
#define __ICUB_VISLAB_HANDv1_H_

#include "Handv1Metrics.h"
#include "Hand.h"

#include <vislab/util/all.h>
#include <vislab/yarp/util/all.h>
#include <vislab/yarp/all.h>

#include <iostream>
#include <set>
#include <map>
#include <stdexcept>

#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>

namespace vislab {
namespace control {

/**
 * @ingroup libHandCtrl
 *
 * Provides an abstraction layer for the hand of the iCub "v1".
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class Handv1 : Hand {
	class FunctionSmoother {

		static const double e; // "epsilon"
		static const double t; // "tau"
		static const double l; // "lambda"

		unsigned int dimension;
		::yarp::sig::Vector prevValues;
		::yarp::sig::Vector prevSmoothedValues;
		::yarp::sig::Vector thresholds;

	public:
		FunctionSmoother();
		FunctionSmoother(const ::yarp::sig::Vector& residualVoltageThresholds);
		virtual ~FunctionSmoother();

		::yarp::sig::Vector& smooth(const ::yarp::sig::Vector& v, ::yarp::sig::Vector& smoothedValues,
				const double deltaT);
	};

	std::map<const std::string, ::yarp::sig::Matrix>& sensingConstants;
	FunctionSmoother fs;

protected:

	/** Provides several metrics for this hand. */
	Handv1Metrics* handMetrics;
	/** The PID controller of the hand. */
	::yarp::dev::IPidControl* pidControl;
	/** The amplifier controller of the hand. */
	::yarp::dev::IAmplifierControl* ampControl;

	/**
	 * Extend the base defintion of the {@link HandMetrics} in order to provide special metrics
	 * for this version of the iCub.
	 *
	 * @see Hand#defineMetrics()
	 */
	virtual void defineHandMetrics();

	/**
	 * Encodes rules for sensing objects based on the {@link HandMetrics} dedicated to
	 * this version of the iCub.
	 * @param blockedJoints The set of blocked joints.
	 *
	 * @see Hand#stopBlockedJoints(std::set<int>* const)
	 */
	virtual void stopBlockedJoints(std::set<int>* const blockedJoints = NULL);

public:

	/**
	 * The constructor.
	 * @param controlBoard The control board of the hand.
	 * @param constants The sensing constants of the hand.
	 */
	Handv1(::yarp::dev::PolyDriver& controlBoard,
			std::map<const std::string, ::yarp::sig::Matrix>& constants);

	/**
	 * The destructor.
	 */
	virtual ~Handv1();

	/**
	 * Calibrates the hand, i.e. the sensing constants are determined.
	 */
	void calibrate();
	/**
	 * Returns the metrics object of this hand.
	 * @return The metrics object of this hand.
	 */
	virtual Handv1Metrics& getMetrics();
};

}
}

#endif /* __ICUB_VISLAB_HANDv1_H_ */
