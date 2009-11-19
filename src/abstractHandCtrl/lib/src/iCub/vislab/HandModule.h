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

#ifndef __ICUB_VISLAB_HANDMODULE_H__
#define __ICUB_VISLAB_HANDMODULE_H__

#include "Hand.h"
#include "Handv1.h"

#include <vislab/util/all.h>
#include <vislab/yarp/util/all.h>
#include <vislab/yarp/all.h>

#include <iostream>
#include <string>
#include <vector>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

namespace vislab {
namespace control {

/**
 * The base module for our implementations related to iCub's hand.
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class HandModule: public vislab::yarp::util::AbstractRFModule {
public:
	/** The type of the hand. */
	enum HandType {
		GENERAL, v1
	};

	/**
	 * The worker thread of the {@link HandModule} module.
	 *
	 * @author Christian Wressnegger
	 * @date 2009
	 */
	class HandWorkerThread: public vislab::yarp::util::AbstractRFModule::AbstractWorkerThread {
	protected:

		/** The motion sequence specifications. */
		std::map<const std::string, vislab::yarp::util::MotionSequence> motions;
		/** The sensing constants of the hand (cf. Handv1#calibrate). */
		std::map<const std::string, ::yarp::sig::Matrix> sensingConstants;

		/** The control board of the robot's arm. */
		::yarp::dev::PolyDriver& controlBoard;
		/** The encoders of the hand. */
		::yarp::dev::IEncoders* encoders;
		/** The PID controller of the hand. */
		::yarp::dev::IPidControl* pidControl;
		/** The position controller of the hand. */
		::yarp::dev::IPositionControl* posControl;
		/** The amplifier controller of the hand. */
		::yarp::dev::IAmplifierControl* ampControl;

		/** The type of the hands. */
		HandType handType;
		/** The abstracted hand. */
		vislab::control::Hand* hand;
		/** The abstracted hand for the iCub v1. */
		vislab::control::Handv1* handv1;

		/**
		 * This function is supposed to create a {@link Hand} implementation. Override it
		 * in order to change the {@link Hand} used by this module.
		 */
		virtual void createHand();

	public:

		/**
		 * Enables or disables the specified joints according to the given boolean value.
		 * @param joints The IDs of the joints to be dis-/enabled.
		 * @param b The flag.
		 */
		void setEnable(const std::vector<int>& joints, const bool b);
		/**
		 * Populates the given {@link std::vector} with the IDs of the currently disabled joints.
		 * @param joints The resulting {@link std::vector} holding the IDs of the disabled joints.
		 */
		void getDisabledJoints(std::vector<int>& joints);

		/**
		 * Adds new motion specifications to the internal set.
		 * @param s The {@link Searchable} containing the motion specifications.
		 */
		void addMotionSpecification(::yarp::os::Searchable& s);

		/**
		 * Sets the sensing constants of the robot's hand (cf. Hand#calibrate).
		 * @param s The {@link Searchable} containing the sensing constants.
		 *          \code
		 *						thresholds    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
		 *						offsets       0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
		 *						springs       0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
		 *						springs       0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
		 *						springs       0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
		 *						derivate_gain 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
		 *          \endcode
		 */
		void setSensingConstants(::yarp::os::Searchable& s);

		/**
		 * The constructor.
		 * @param moduleOptions The option used by the module.
		 * @param ports The ports to be used by the module.
		 * @param controlBoard The control board of the hand.
		 */
		HandWorkerThread(const vislab::yarp::util::OptionManager& moduleOptions,
				const vislab::yarp::util::Contactables& ports, ::yarp::dev::PolyDriver& controlBoard,
				HandType t);
		/**
		 * The destructor.
		 */
		virtual ~HandWorkerThread();

	};

private:

	struct PortIds {
		unsigned int Control;
	} id;

protected:
	/** The arm identifier as human readable string. */
	::yarp::os::ConstString partName;
	/** The motion sequence specifications. */
	::yarp::os::Property motionSpecification;
	/** The sensing constants of the hand (cf. Handv1#calibrate). */
	::yarp::os::Bottle sensingCalibration;

	/** The control board of the hand. */
	::yarp::dev::PolyDriver controlBoard;
	/** The type of the hands. */
	HandType handType;

public:

	/** The number of axes. */
	static const int numAxes = 16;

	/**
	 * The constructor.
	 * @param name The module's name.
	 */
	HandModule(::yarp::os::ConstString name);
	/**
	 * The destructor.
	 */
	virtual ~HandModule();

	/**
	 * @see AbstractRFModule#configure(ResourceFinder)
	 */
	virtual bool configure(::yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
};

}
}

#endif // __ICUB_VISLAB_HANDMODULE_H__
