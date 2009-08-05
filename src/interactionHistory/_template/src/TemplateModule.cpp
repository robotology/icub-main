// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
 * email:   assif.mirza@robotcub.org
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


#include <iCub/iha/TemplateModule.h>

#include <iCub/iha/iha_utils.h>

/*ASTERIX
 * @addtogroup icub_iha_Template
 *
\section intro_sec Description
DESC for IHA

\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg <INT>                : debug printing level
--name <STR>               : process name for ports
--file <STR>               : configuration from given file

--connect_to_coords <STR>  : connect to specified port for face
\endverbatim

\section portsa_sec Ports Accessed

\section portsc_sec Ports Created

- /iha/sm/quit  - module quit port
 
\section conf_file_sec Configuration Files
conf/ihaTemplate.ini

Sample INI file:
\verbatim
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaTemplate --file conf/ihaTemplate.ini

\see iCub::contrib::TemplateModule

\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/template/src/TemplateModule.cpp.
*/

TemplateModule::TemplateModule(){
}

TemplateModule::~TemplateModule(){ 
}


bool TemplateModule::open(Searchable& config){
   
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());

    if (config.check("help","if present, display usage message")) {
		cerr << "Usage : " << "\n"
		<< "------------------------------------------" << "\n"
		<< "  --dbg [INT]   : debug printing level" << "\n"
		<< "  --name [STR]  : process name for ports" << "\n"
		<< "  --file [STR]  : config file" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --connect_to_coords [STR]       : connect to specified port for face" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "\n";
        return false;
    }

    bool ok = true;

    // Read parameters
	template_int_param = config.check("template_int_param",Value(4)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"template_int_param:%d\n",template_int_param);

	// create names of ports
    ConstString coordsPortName = getName("coords:in");
	
	// open the coordinates reading port
	coordsPort.open(coordsPortName.c_str());

	// if required we can connect to the facedetector coordinates port
	if (config.check("connect_to_coords")) {
		if (connectToParam(config,"connect_to_coords",coordsPortName.c_str(), 0.25, this)) {
            IhaDebug::pmesg(DBGL_INFO,"Connected to Face Coords\n");
        } else {
            ok = false;
        }
	}

    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);
    return ok;
}

bool TemplateModule::close(){
    coordsPort.close();
    return true;
}

bool TemplateModule::interruptModule(){
    coordsPort.interrupt();
    return true;
}

bool TemplateModule::updateModule(){

    return true;
}

bool TemplateModule::respond(const Bottle &command,Bottle &reply){
        
    return false;
} 	
