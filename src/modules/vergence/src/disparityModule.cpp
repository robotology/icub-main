// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Vadim Tikhanoff
 * email:   francesco.rea@iit.it
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

#include <iCub/disparityModule.h>
// std
#include <stdio.h>
#include <iostream>

using namespace std;

disparityModule::disparityModule() {
    init_flag = false;
    currentProcessor = 0;
    ratio = 4.00;
    robotHead = 0;
    robotTorso = 0;
}

disparityModule::~disparityModule() {
   cmdPort.close();
} 

bool disparityModule::configure( ResourceFinder &rf ) {
    
    if ( rf.check( "moduleName" ) ) {
        moduleName = rf.find( "moduleName" ).asString();
        moduleName = "/" + moduleName;
    }
    else    
        moduleName ="/vergence";

    ctrlType = rf.check("ctrl", 
               Value("ctrlGaze"), 
               "controller type (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName( moduleName.c_str() );

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    if ( rf.check( "robot" ) )
        robotName=rf.find( "robot" ).asString();
    else
        robotName="icub";

    robotPortName = "/" + robotName + "/head";
    
    currentProcessor = new disparityProcessor();
    currentProcessor->setName( moduleName, robotName, ctrlType );
    currentProcessor->start();

    cmdPort.open(getName("/cmd:i"));
    attach(cmdPort);

    
    return true;
}

bool disparityModule::close() {
    currentProcessor->stop();
    delete currentProcessor;
	return true;
}

bool disparityModule::interruptModule() {

	return true;
}

bool disparityModule::updateModule() {	

	return true;
}

//make gcc happy

