/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/** 
 * @ingroup icub_module
 * \defgroup icub_lumaChroma lumaChroma
 *
 * Receives an rgb image, depending on the image type the user wants to work on (YUV or HSV) the module extracts the Y, U and V or H, S, and V planes and performs centre-surround processing with a construction of uniqueness maps (intensity saliency, colour, hue, saturation and value ) via a difference-of-Gaussian pyramid filter bank. 
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP 
 * OpenCV (version >= 2.0) 
 *
 * \section parameters_sec Parameters
 * 
 * Command-line Parameters
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c lumaChroma.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c lumaChroma/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c lumaChroma \n   
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c image \c lumaChroma \n   
 *   specifies the type of the module to work on (yuv or hsv)  
 * 
 * Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c imageInputPort \c /image:i \n    
 *   specifies the input port name (this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 * - \c YPort \c /Y/image:o \n  
 *   specifies the output port name when YUV is selected(this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 * - \c UVPort \c /UV/image:o \n  
 *   specifies the output port name when YUV is selected(this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 * - \c HPort \c /H/image:o \n  
 *   specifies the output port name when HSV is selected(this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 * - \c SPort \c /S/image:o \n  
 *   specifies the output port name when HSV is selected(this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 * - \c VPort \c /V/image:o \n  
 *   specifies the output port name when HSV is selected(this string will be prefixed by \c /yuvProc 
 *   or whatever else is specifed by the name parameter
 *
 *
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 * Input ports
 *
 *  - \c /lumaChroma \n 
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /yuvProc
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /lumaChroma/image:i \n
 *
 * Output ports
 *
 *  - \c /lumaChroma \n
 *    see above
 *
 *  - \c /lumaChroma/Y/image:o when YUV is selected\n
 *  - \c /lumaChroma/UV/image:o when YUV is selected\n
 *
 *  - \c /lumaChroma/H/image:o when HSV is selected\n
 *  - \c /lumaChroma/S/image:o when HSV is selected\n
 *  - \c /lumaChroma/V/image:o when HSV is selected\n
 *
 * Port types
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >    \c inputPort; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortY;   
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortUV;   
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortV;   
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c lumaChroma.ini  in \c $ICUB_ROOT/app/lumaChroma/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux: Ubuntu 9.10, Debian Stable, squeeze and windows 
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>lumaChroma --name lumaChroma --image yuv (or hsv) --context lumaChroma/conf --from config.ini </tt>
 *
 * \author 
 * 
 * Vadim Tikhanoff
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include "iCub/lumaChroma.h"

using namespace yarp::os;

int main(int argc, char * argv[])
{
    /* initialize yarp network */ 
    Network yarp;

    /* create the module */
    lumaChroma module; 

    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultConfigFile( "config.ini" ); //overridden by --from parameter
    rf.setDefaultContext( "lumaChroma/conf" );   //overridden by --context parameter
    rf.configure( "ICUB_ROOT", argc, argv );
 
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

    return 0;
}

