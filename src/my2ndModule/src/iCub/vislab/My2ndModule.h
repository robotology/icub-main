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
 * \defgroup icub_my2ndModule my2ndModule
 *
 * This module aims at demonstrating the use of the YARP extension RFModule2 introduced by
 * the OpenVislab libraries (http://OpenVislab.sf.net). Furthermore it shows the use of motions
 * and motion sequences which also were proposed for inclusion into the YARP library.
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
 * \section cmdline_parameters_sec Command-line Parameters
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
 * (e.g. --from conf.ini). The value part can be changed to suit your needs; the default values are shown below.
 *
 * - from &lt;STRING&gt; <br />
 *   specifies the configuration file. <br />
 *   default: "conf.ini"
 *
 * - context &lt;STRING&gt; <br />
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file. <br />
 *   default: "my2ndModule/conf"
 *
 * - name &lt;STRING&gt; <br />
 *   specifies the name of the module (used to form the stem of module port names). <br />
 *   default: "my2ndModule"
 *
 * - robot &lt;STRING&gt; <br />
 *   specifies the name of the robot (used to form the root of robot port names). <br />
 *   default: "myRobot"
 *
 *
 * \subsection conffile_parameters_sec Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters).
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - log &lt;FILE&gt; <br />
 *   specifies the file name for the output of the "write" remote command (within the given context).
 *
 * - in &lt;PORT&gt; <br />
 *   specifies the input port name. <br />
 *   default: /in
 *
 * - out &lt;PORT&gt; <br />
 *   specifies the output port name. <br />
 *   default: /out
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * None
 *                      
 * \section portsc_sec Ports Created
 *
 * \subsection inputports_sec Input ports
 *
 *  - /my2ndModule <br />
 *    This port is used to change the parameters of the module at run-time or stop the module
 *    The following commands are available:
 *    - write <br />
 *      writes out a pre-defined motion sequence to a file.
 *    - set &#91; &lt;id&gt; &lt;value&gt; &#93; <br />
 *      available options:
 *      - log (string) <br />
 *        This value will be added to the z-offset of the projection plan to the robot's base
 *        coordinates. This may help you time find the correct position of the projection plane
 *        without modifying the configuration and restarting the module.
 *    - echo <str>
 *    - help
 *    - quit
 *
 *    Note that the name of this port mirrors whatever is provided by the --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc
 *    directive: yarp rpc /my2ndModule This opens a connection from a terminal to the port and allows
 *    the user to then type in commands and receive replies.
 *
 *
 * \subsection outputports Output ports
 *
 *  - /my2ndModule <br />
 *    see above
 *
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * A file specified by the program argument "log"/ the remote option "logFile" or the argument of
 * the remote command "write" containing a motion sequence as produced by the {@link MotionSequence}
 * class.
 *
 * \section conf_file_sec Configuration Files
 *
 * conf.ini in $ICUB_ROOT/icub/app/My2ndModule/
 * 
 * \section tested_os_sec Tested OS
 *
 * most extensively on
 * Linux version 2.6.31-gentoo-r6 (gcc version 4.3.4 (Gentoo 4.3.4 p1.0, pie-10.1.5) ) but most probably
 * compiles and runs fine on Windows XP SP3 MSVC++8 as well :).
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <dl>
 * 	<dt>my2ndModule</dt>
 *    <dd>--log "test.txt"</dd>
 * </dl>
 *
 *
 * \author Christian Wressnegger
 * 
 * Copyright (C) 2009 Christian Wressnegger<br />
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_MY2NDMODULE_MODULE_H__
#define __ICUB_MY2NDMODULE_MODULE_H__

#include <yarp/os/all.h>
#include <vislab/yarp/util/all.h>

//#define IGNORE_THE_FANCY_EXTENSIONS

namespace vislab {
namespace demo {

/**
 * Implements a demo module in order to demonstrate how to use {@link RFModule2} of the OpenVislab
 * libraries (http://OpenVislab.sf.net)
 *
 * @author Christian Wressnegger
 * @date 2009
 */
class My2ndModule: public vislab::yarp::util::RFModule2 {
#ifndef IGNORE_THE_FANCY_EXTENSIONS

	// p0: Declare port identifiers
	struct PortIds {
		unsigned int in, out;
	}id;

	// r0: Declare commands to use
	class WriteCommand: public vislab::yarp::util::Command {
		virtual bool execute(const ::yarp::os::Bottle& params, ::yarp::os::Bottle& reply) const;
	public:
		WriteCommand(My2ndModule* const parent) :
		Command(parent, "write", "Writes out a pre-defined motion sequence to a file") {
		}
	};
	friend class WriteCommand;

	const ::yarp::os::ConstString getLogFilename();

#endif

public:
	/**
	 * The default constructor.
	 */
	My2ndModule();

	/**
	 * @see RFModule2#configure(::yarp::os::ResourceFinder&)
	 */
	// configure all the module parameters and return true if successful
	bool configure(::yarp::os::ResourceFinder& rf);
	/**
	 * @see RFModule2#updateModule()
	 */
	bool updateModule();
};

}
}

#endif // __ICUB_MY2NDMODULE_MODULE_H__
