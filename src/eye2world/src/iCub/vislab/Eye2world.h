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

/**
 * @ingroup icub_module
 *
 * \defgroup icub_eye2world eye2world
 *
 * This module provides a projection from the image plane to the world represented as
 * plane in front of the robot relatively specified to robot's base coordinates (e.g.
 * think of a table the robot is operating on)
 *
 * 
 * \section lib_sec Dependencies
 *
 * - YARP (YARP_{OS,dev,sig,math})
 *   - ACE
 * - iKin
 *   - IPOPT
 *   - ctrlLib
 * - OpenVislab (libvislab, libvislab_YARP): http://OpenVislab.sf.net
 *
 * \section parameters_sec Parameters
 * 
 * \section cmdline_parameters_sec Command-line Parameters
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
 * (e.g. --from conf.ini). The value part can be changed to suit your needs; the default values are shown below.
 *
 * --from &lt;STRING&gt; <br />
 *   specifies the configuration file. <br />
 *   default: "conf.ini"
 *
 * --context &lt;STRING&gt; <br />
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file. <br />
 *   default: "eye2world"
 *
 * --name &lt;STRING&gt; <br />
 *   specifies the name of the module (used to form the stem of module port names). <br />
 *   default: "eye2world"
 *
 * --robot &lt;STRING&gt; <br />
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
 * --eyeCalibration &lt;FILE&gt; <br />
 *   specifies the file name to the calibration file for the iCub's eyes.
 *
 * --tableConfiguration &lt;FILE&gt; <br />
 *   specifies the file name to configuration of the file.
 *
 * --in &lt;PORT&gt; <br />
 *   specifies the input port name for the 2D coordinate to be transformed. <br />
 *   default: /in
 *
 * --headState &lt;PORT&gt; <br />
 *   specifies the input port name for head's state (position/ pose). <br />
 *   default: /icub/head/state:i
 *
 * --torsoState &lt;PORT&gt; <br />
 *   specifies the input port name for head's state (position/ pose). <br />
 *   default: /icub/torso/state:i
 *
 * --out &lt;PORT&gt; <br />
 *   specifies the output port name for the resulting 3D coordinates. <br />
 *   default: /out
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - /icub/head/state:o <br />
 *   The coordinates are expected to be wrapped in a Bottle object as double values:
 *   @code Bottle(double:neckPitch, double:neckRoll, double:neckYaw, double:eyesTilt, double:eyesVesion, double:eyesVergence) @endcode
 *
 * - /icub/torso/state:o <br />
 *   The coordinates are expected to be wrapped in a Bottle object as double values:
 *   @code Bottle(double:yaw, double:roll, double:pitch) @endcode
 *                      
 * \section portsc_sec Ports Created
 *
 * \subsection inputports_sec Input ports
 *
 *  - /eye2world <br />
 *    This port is used to change the parameters of the module at run time or stop the module
 *    The following commands are available:
 *    - set &#91; &lt;id&gt; &lt;value&gt; &#93; <br />
 *      available options:
 *      - heightOffset (numeric) <br />
 *        This value will be added to the z-offset of the projection plan to the robot's base
 *        coordinates. This may help you time find the correct position of the projection plane
 *        without modifying the configuration and restarting the module.
 *      - motor2eye (numeric) <br />
 *        This value is supposed to compensate the error introduced by the fact that the distance
 *        between the lens and the motor of the eye is not taken into account for the kinematics.
 *        In order to achieve this, the value is simply added to the z coordinate of the
 *        transformation to the projection plane but is again subtracted from the result afterwards.
 *        That way it is possible to adjust the accuracy of the project without messing about the
 *        real relative distance to the projection plane.
 *      - scale (numeric) <br />
 *        This one will be multiplied to the resulting x and y coordinate. This is another
 *        Possibility to tweak accuracy of the projection.
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
 *  - /eye2world/in <br />
 *    cf. Input parameter "--in" <br />
 *    The coordinates are wrapped in a Bottle object as double values: @code Bottle([ConstString:cam,] double:x, double:y[, double:height]) @endcode
 *    Optionally it is possible to specify the camera/ eye to use ("left", "right") and an additional offset with respect to the
 *    z-axes of the projection plane.
 *
 * \subsection outputports Output ports
 *
 *  - /eye2world <br />
 *    see above
 *
 *  - /eye2world/out <br />
 *    cf. Input parameter "--out" <br />
 *    The coordinates are wrapped in a Bottle object as double values: @code Bottle(double:x, double:y, double:z) @endcode
 *
 *
 * \section in_files_sec Input Data Files
 *
 * \subsection eyecalib_sec Eye calibration:
 *
 * \code
 *   [CAMERA_CALIBRATION_${X}]
 *   (...)
 *   fx 216.171
 *   fy 216.265
 *   cx 173.283
 *   cy 141.662
 *   (...)
 * \endcode
 *
 * where ${X} might be "LEFT" or "RIGHT"
 *
 * \subsection tableconf_sec Table configuration
 *
 * \code
 *   [POSITION]
 *   x  0.0
 *   y  0.0
 *   z -0.06
 * \endcode
 *
 * where (x,y,z) is the 3D offset of the projection plane relative to the robot's reference frame in meters.
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * conf.ini in $ICUB_ROOT/icub/app/Eye2world/
 * 
 * \section tested_os_sec Tested OS
 *
 * most extensively on
 * Linux version 2.6.30-gentoo-r8 (gcc version 4.3.4 (Gentoo 4.3.4 p1.0, pie-10.1.5) ) but compiles
 * and runs fine on Windows XP SP3 MSVC++8 as well.
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <dl>
 * 	<dt>eye2world</dt>
 * 		<dd>--in /in<dd>
 *    <dd>--eyeCalibration "iCubLisboa01/conf/icubEyes.ini"</dd>
 * 		<dd>--tableConfiguration "iCubLisboa01/conf/table.ini" </dd>
 * 		<dd>--out /out</dd>
 * </dl>
 *
 *
 * \author Christian Wressnegger
 * 
 * Copyright (C) 2009 Christian Wressnegger<br />
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_EYE2WORLD_MODULE_H__
#define __ICUB_EYE2WORLD_MODULE_H__

#include "Projection.h"

#include <vislab/yarp/all.h>
#include <vislab/yarp/util/all.h>

#include <iostream>
#include <string>
#include <vector>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/math/Math.h>

#include <iCub/iKinFwd.h>
#include <iCub/ctrlMath.h>

namespace vislab {
namespace math {

/**
 * Implements a {@link RFModule} that determines the 3D position of a 2D coordinate pair
 * by projecting from the image plane to the world represented as plane in front of the robot
 * relatively specified to robot's base coordinates (e.g. think of a table the robot is
 * operating on)
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class Eye2world: public vislab::yarp::util::AbstractRFModule {

	struct PortIds {
		unsigned int Input_Coordinates2d, Input_HeadPosition, Input_TorsoPosition, Output_Coordinates3d;
	} id;

	class WorkerThread: public vislab::yarp::util::AbstractRFModule::AbstractWorkerThread {
	private:

		::yarp::os::Bottle* inputCoordinates;
		std::map<const std::string, CameraCalibration> cameras;
		::yarp::sig::Vector tabletopPosition;
		struct PortIds id;

	public:

		WorkerThread(const vislab::yarp::util::OptionManager& moduleOptions,
				const vislab::yarp::util::Contactables& ports, struct PortIds ids, std::map<
						const std::string, ::yarp::os::Property*>& cameras, ::yarp::os::Property& table);
		bool threadInit();
		void threadRelease();
		void run();
	};

	::yarp::os::Property leftEyeCalibration;
	::yarp::os::Property rightEyeCalibration;
	::yarp::os::Property tabletopPosition;

	std::map<const std::string, ::yarp::os::Property*> cameras;

protected:
	WorkerThread *workerThread;
	virtual ::yarp::os::Thread* createWorkerThread();

public:
	/**
	 * The default constructor.
	 */
	Eye2world();

	/**
	 * @see AbstractRFModule#configure(::yarp::os::ResourceFinder&)
	 */
	// configure all the module parameters and return true if successful
	bool configure(::yarp::os::ResourceFinder& rf);
};

}
}

#endif // __ICUB_EYE2WORLD_MODULE_H__
