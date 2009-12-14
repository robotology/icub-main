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
 * @ingroup libHandCtrl
 *
 * The base module for our implementations related to iCub's hand.
 *
 *
 * \section lib_sec Dependencies
 *
 * - YARP (YARP_{OS,dev,sig,math})
 *   - ACE
 * - OpenVislab (libvislab, libvislab_YARP): http://OpenVislab.sf.net
 *
 * \section parameters_sec Parameters
 *
 * \subsection cmdline_parameters_sec Command-line Parameters
 *
 * The following key-value pairs can be specified as command-line parameters by prefixing "--" to the key
 * (e.g. --from conf.ini). The value part can be changed to suit your needs; the default values are shown below.
 *
 * - from &lt;STRING&gt; <br />
 *   specifies the configuration file. <br />
 *   default: "conf.ini"
 *
 * - context &lt;STRING&gt; <br />
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file. <br />
 *   default: "eye2world"
 *
 * - name &lt;STRING&gt; <br />
 *   specifies the name of the module (used to form the stem of module port names). <br />
 *   default: "eye2world"
 *
 * - robot &lt;STRING&gt; <br />
 *   specifies the name of the robot (used to form the root of robot port names). <br />
 *   default: "icub"
 *
 *
 * \subsection conffile_parameters_sec Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file
 * (they can also be specified as command-line parameters).
 * The value part can be changed to suit your needs; the default values are shown below.
 *
 * - part &lt;STRING&gt; <br />
 *   specifies name of the arm to use. <br />
 *   default: "right_arm"
 *
 * - handType &lt;STRING&gt; <br />
 *   specifies the type of the hand of the iCub: "general" | "v1" <br />
 *   default: "general"
 *
 * - motionSpec &lt;FILE&gt; <br />
 *   specifies the file name of the configuration file containing the motions <br />
 *   default: "motion_specification.ini" <br />
 *
 * - sensingCalib &lt;FILE&gt; <br />
 *   <strong>ONLY USED IF</strong> <handType> is specified as "v1". <br />
 *   specifies the file name of the calibration file containing hand's the sensing constants. <br />
 *   default: "object_sensing.ini"
 *
 * - control &lt;PORT&gt; <br />
 *   specifies the communication port for communicating with the control board of the arm. <br />
 *   default: /abstractHandCtrl/control
 *
 * - q &lt;PORT&gt; <br />
 *   specifies the port to directly receive joint configurations as Vector(12) to control the hand. <br />
 *   default: /q:i
 *
 *
 * \section portsa_sec Ports Accessed
 *
 * - /<robot>/<part>/control
 *   The port of the control board of the arm to be controlled. Used and managed by the PolyDriver class.
 *
 * \section portsc_sec Ports Created
 *
 * \subsection inputports_sec Input ports
 *
 *  - /abstractHandCtrl <br />
 *    This port is used to change the parameters of the module at run time or stop the module.
 *    The following commands are available:
 *    - set &#91; &lt;id&gt; &lt;value&gt; &#93; <br />
 *      no options available:
 *    - echo <str>
 *    - help
 *    - quit
 *
 *    Note that the name of this port mirrors whatever is provided by the --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc
 *    directive: yarp rpc /eye2world This opens a connection from a terminal to the port and allows
 *    the user to then type in commands and receive replies.
 *
 *
 * \subsection outputports_Sec Output ports
 *
 *  - /eye2world <br />
 *    see above
 *
 *
 * \section in_files_sec Input Data Files
 *
 * \subsection sensing_sec Sensing Constants
 *
 * \code
 *	 thresholds    0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
 *	 offsets       0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
 *	 springs       0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
 *	 springs       0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
 *	 springs       0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
 * 	 derivate_gain 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
 * \endcode
 *
 * \subsection motion_sec Motion Specification
 *
 * \code
 *   (...)
 *   [include X "y.ini"]
 *   (...)
 * \endcode
 *
 * where X is the identifier of the motion which can be used for the "do" command and y is some
 * file name where the file should have a structure like the following:
 *
 * \code
 *   [POSITION0]
 *   jointPositions   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0
 *   jointVelocities 10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0
 *   timing 0.0
 *
 *   [POSITION1]
 *   jointPositions   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0
 *   jointVelocities 10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0  10.0
 *   timing 0.0
 *
 *   [DIMENSIONS]
 *   numberOfPoses   2
 *   numberOfJoints 16
 * \endcode
 *
 * The number of "POSITION${x}" entries is controlled by the "numberOfPoses" value in the "DIMENSIONS"
 * section. The size of each vector in those section is specified by "numberOfJoints": 16 in this case.
 * For a more extensive description of the file format, check the documenation of the robotMotorGui.
 *
 * Right now only the position value are taken into account. The velocities an timing are supposed
 * to follow soon!
 *
 *
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

protected:
	class ReloadCommand: public vislab::yarp::util::Command {
		virtual bool execute(const ::yarp::os::Bottle& params, ::yarp::os::Bottle& reply) const;
	public:
		ReloadCommand(HandModule* const parent) :
			Command(parent, "reload motion specifications",
					"Reloads the motion specification from the previously declared file.") {
		}
	};
	friend class DoCommand;

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
		 * Replaces the internal set of motions with those specified as parameter.
		 * @param s The {@link Searchable} containing the motion specifications.
		 */
		void setMotionSpecification(::yarp::os::Searchable& s);
		/**
		 * Adds new motion specifications to the internal set.
		 * @param s The {@link Searchable} containing the motion specifications.
		 */
		void addMotionSpecification(::yarp::os::Searchable& s);
		/**
		 * Returns the internal set of motion specifications.
		 * @return The internal set of motion specifications.
		 */
		const std::map<const std::string, vislab::yarp::util::MotionSequence>
				& getMotionSpecifications();

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

	::yarp::os::ConstString motionSpecificationFilename;

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

	HandWorkerThread* workerThread;

	virtual bool startThread();

public:

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
