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

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>

#include <gsl/gsl_math.h>


double norm(const yarp::sig::Vector &v);


class HandMetrics
{
	bool newSnapshot;

	// Expected number of axes
	const static int numAxes = 16;

	// output limit of all hand PIDs
	const static double pidLimit;

	const static double residualVoltageThres;

	::yarp::dev::IEncoders         *encoders;
	::yarp::dev::IPidControl       *pidControl;
	::yarp::dev::IAmplifierControl *ampControl;
	::yarp::dev::Pid prevPids[numAxes];

	std::map<const std::string, ::yarp::sig::Matrix>& objectSensingConstants;

	::yarp::sig::Vector position;
	::yarp::sig::Vector velocity;
	::yarp::sig::Vector voltage;
	::yarp::sig::Vector targetVoltage;
	::yarp::sig::Vector error;

	::yarp::sig::Vector prevPosition;
	double prevTime, curTime;
	::yarp::sig::Vector prevVoltageOffset;
	::yarp::sig::Vector prevSpringStiffness;

public:
	/**
	 * The constructor.
	 * @param encoders The encoders of the hand.
	 * @param pidControl The PID controller of the hand.
	 * @param ampControl The amplifier controller of the hand.
	 * @param constants The sensing constants of the hand.
	 */
	HandMetrics(::yarp::dev::IEncoders* const encoders, ::yarp::dev::IPidControl* const pidControl,
			    ::yarp::dev::IAmplifierControl* const ampControl, std::map<const std::string, ::yarp::sig::Matrix>& constants);
	/**
	 * The destructor.
	 */
	virtual ~HandMetrics();

	/**
	 * Make a snapshot of the current state of the hand controller.
	 * Use this to receive updated values from the other functions.
	 */
	void snapshot();
	/**
	 * Writes the data retrieved by the snapshot function in a human readable form to the provided stream.
	 * @param s The output stream.
	 */
	void printSnapshotData(std::ostream& s = std::cout);

	/**
	 * Returns the current positions of the hand's joints.
	 * @return The current positions of the hand's joints.
	 */
	::yarp::sig::Vector& getPosition();
	/**
	 * Returns the current velocity of the hand's joints.
	 * @return The current velocity of the hand's joints.
	 */
	::yarp::sig::Vector& getVelocity();
	/**
	 * Returns the current voltage of the hand's joints.
	 * @return The current voltage of the hand's joints.
	 */
	::yarp::sig::Vector& getVoltage();
	/**
	 * Returns the current error of the hand's joints.
	 * @return The current error of the hand's joints.
	 */
	::yarp::sig::Vector& getError();
	/**
	 * Returns the time interval of the latest snapshot.
	 * @return The time interval of the latest snapshot.
	 */
	double getTimeInterval();
};


class FunctionSmoother
{
private:
    static const double e; // "epsilon"
    static const double t; // "tau"
    static const double l; // "lambda"

    unsigned int dimension;
    yarp::sig::Vector prevValues;
    yarp::sig::Vector thresholds;

public:
    FunctionSmoother() { }
    FunctionSmoother(const yarp::sig::Vector &residualVoltageThresholds);
    virtual ~FunctionSmoother() { }

    yarp::sig::Vector& smooth(const yarp::sig::Vector &v, yarp::sig::Vector &smoothedValues,
                              const double deltaT);
};


#endif /* __ICUB_VISLAB_HANDMETRICS_H_ */
