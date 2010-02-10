/*******************************************************************************
 * Copyright (C) 2009-2010 Christian Wressnegger
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
 * \defgroup icub_gloveCtrl gloveCtrl
 *
 * This module reads joints angles from CyberGlove(R) data glove, maps them to the joint ranges
 * of the iCub and writes them to a port. Together with the abstractHandCtrl module this can be
 * use to teleoperate the iCub using the CyberGlove(R).
 *
 * \section lib_sec Dependencies
 *
 * - YARP (YARP_{OS,dev,sig,math})
 *   - ACE
 * - libHandCtrl
 * - OpenVislab (libvislab, libvislab_YARP): http://OpenVislab.sf.net
 * - libglove (libvislab, libvislab_YARP): http://libglove.sf.net
 * - [optional] Flock of Birds
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
 *   default: "gloveCtrl/conf"
 *
 * - name &lt;STRING&gt; <br />
 *   specifies the name of the module (used to form the stem of module port names). <br />
 *   default: "gloveCtrl"
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
 *
 * - devMgr.address &lt;TCP/IP ADDRESS&gt; <br />
 *   specifies the TCP/IP address of the remote device manager (gloved) <br />
 *   default: "127.0.0.1"
 *
 * - devMgr.port &lt;INTEGER&gt; <br />
 *   specifies the port of the remote device manager (gloved) <br />
 *   default: "12345" <br />
 *
 * - gloveCalib &lt;FILE&gt; <br />
 *   specifies the name of the file containing calibration for the data glove. <br />
 *   default: "glove.calib"
 *
 * - out &lt;PORT&gt; <br />
 *   specifies the port to directly write joint configurations as Vector(16) to control the hand. <br />
 *   default: /out <br />
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
 *  - /gloveCtrl <br />
 *    This port is used to change the parameters of the module at run time or stop the module.
 *    The following commands are available:
 *    - set &#91; &lt;id&gt; &lt;value&gt; &#93; <br />
 *      available options:
 *      - streaming (on/off) <br />
 *        Enables/ Disables the streaming of glove values to the port specified above.
 *    - start Set the previously mentioned option "streaming" to "on"
 *    - stop Set the previously mentioned option "streaming" to "on"
 *    - echo <str>
 *    - help
 *    - quit
 *
 *    Note that the name of this port mirrors whatever is provided by the --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc
 *    directive: yarp rpc /gloveCtrl This opens a connection from a terminal to the port and allows
 *    the user to then type in commands and receive replies.
 *
 *
 * \subsection outputports_Sec Output ports
 *
 *  - /gloveCtrl <br />
 *    see above
 *
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * conf.ini in $ICUB_ROOT/icub/app/gloveControl/
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
 *   <dt>gloveCtrl</dt>
 *     <dd>--robot icub</dd>
 *     <dd>--devMgr.address 127.0.0.1</dd>
 *     <dd>--devMgr.port 12345</dd>
 *     <dd>--gloveCalib "res/glove.calib"</dd>
 * </dl>
 *
 *
 *
 * \author Christian Wressnegger
 *
 * Copyright (C) 2009-2010 Christian Wressnegger<br />
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __ICUB_VISLAB_GLOVECTRL_MODULE_H__
#define __ICUB_VISLAB_GLOVECTRL_MODULE_H__


#include <vislab/yarp/util/all.h>

#include <glove.hpp>

namespace vislab {
namespace control {

/**
 *
 *
 * @author Christian Wressnegger and Carla Gonzalez
 * @date 2009-2010
 */
class GloveCtrl: public vislab::yarp::util::ThreadedRFModule {

  struct PortIds {
    unsigned int Output_Q;
  } id;

  class StartCommand: public vislab::yarp::util::Command {
    virtual bool execute(const ::yarp::os::Bottle& params, ::yarp::os::Bottle& reply) const;
  public:
    StartCommand(GloveCtrl* const parent) :
      Command(parent, "start", "Starts streaming data of the CyberGlove.") {
    }
  };
  friend class StartCommand;

  class StopCommand: public vislab::yarp::util::Command {
    virtual bool execute(const ::yarp::os::Bottle& params, ::yarp::os::Bottle& reply) const;
  public:
    StopCommand(GloveCtrl* const parent) :
      Command(parent, "stop", "Stops streaming data of the CyberGlove") {
    }
  };
  friend class StopCommand;

  class WorkerThread: public vislab::yarp::util::ThreadedRFModule::RFWorkerThread {
  private:
    struct PortIds id;
    glove::devices::CyberGlove* glove;

  public:

    void enableStreaming(const bool b);

    WorkerThread(const vislab::yarp::util::OptionManager& moduleOptions,
        const vislab::yarp::util::Contactables& ports, const struct PortIds id,
        glove::devices::CyberGlove* const glove);

    void run();
  };
  friend class WorkerThread;

  glove::devices::remote::RemoteDeviceManager* devMgr;
  glove::devices::CyberGlove* glove;

protected:
  WorkerThread* workerThread;
  virtual ::yarp::os::Thread* createWorkerThread();

public:
  /**
   * The constructor.
   */
  GloveCtrl();
  /**
   * The destructor.
   */
  virtual ~GloveCtrl();

  /**
   * @see HandModule#configure(ResourceFinder)
   */
  bool configure(::yarp::os::ResourceFinder &rf);
  // configure all the module parameters and return true if successful

};

}
}

#endif // __ICUB_VISLAB_GLOVECTRL_MODULE_H__
