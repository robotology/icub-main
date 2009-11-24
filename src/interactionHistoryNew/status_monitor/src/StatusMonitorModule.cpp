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


#include <iCub/iha/StatusMonitorModule.h>

#include <iCub/iha/iha_utils.h>

/**
 * @addtogroup icub_iha_StatusMonitor

\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg [INT]   : debug printing level
--name [STR]  : process name for ports
--file [STR]  : config file

--status_output_rate [INT] : output rate
\endverbatim

\section portsa_sec Ports Accessed
- connect and status out port to this process

\section portsc_sec Ports Created
- /iha/status/monitor:in
- /iha/status/monitor:out
 
- /iha/sm/quit  - module quit port

\section conf_file_sec Configuration Files
conf/ihaStatusMonitor.ini

Sample INI file:
\verbatim
status_output_rate 10
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaStatusMonitor --name /iha/status --file conf/ihaStatusMonitor.ini

See script $ICUB_ROOT/app/iha_manual/monitor.sh

\see iCub::contrib::StatusMonitorModule

\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/status_monitor/src/StatusMonitorModule.cpp.
 */

StatusMonitorModule::StatusMonitorModule(){
}

StatusMonitorModule::~StatusMonitorModule(){ 
}


bool StatusMonitorModule::open(Searchable& config){
   
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());

    if (config.check("help","if present, display usage message")) {
		cerr << "Usage : " << "\n"
		<< "------------------------------------------" << "\n"
		<< "  --dbg [INT]   : debug printing level" << "\n"
		<< "  --name [STR]  : process name for ports" << "\n"
		<< "  --file [STR]  : config file" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --status_output_rate [INT] : output rate" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "\n";
        return false;
    }

    bool ok = true;

    // Read parameters
	status_output_rate = config.check("status_output_rate",Value(10)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"status_output_rate:%d\n",status_output_rate);

	// create names of ports
    ConstString monitorInPortName = getName("monitor:in");
    ConstString monitorOutPortName = getName("monitor:out");
	
	// open the monitor read/write ports
	monitorInPort.open(monitorInPortName.c_str());
	monitorOutPort.open(monitorOutPortName.c_str());

    //---------------------------------------------------------
    // Start the status output thread
	IhaDebug::pmesg(DBGL_STATUS1, "Starting Status Output Thread\n");
    statusOutputThread = new StatusOutputThread((int)(1000.0/status_output_rate), &systemStatus, monitorOutPort);
	statusOutputThread->start();

    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);
    return ok;
}

bool StatusMonitorModule::close(){
    IhaDebug::pmesg(DBGL_DEBUG1,"StatusMonitorModule::close()\n");
    monitorInPort.close();
    monitorOutPort.close();
    statusOutputThread->stop();
    
    IhaDebug::pmesg(DBGL_DEBUG1,"Deleting status objects ...");
    for (map<string,SubsystemStatus*>::iterator ssit = systemStatus.begin(); ssit != systemStatus.end() ; ssit++) {
        delete ssit->second;
    }
    IhaDebug::pmesg(DBGL_DEBUG1," done.\n");
    return true;
}

bool StatusMonitorModule::interruptModule(){
    IhaDebug::pmesg(DBGL_DEBUG1,"StatusMonitorModule::interruptModule()\n");
    monitorInPort.interrupt();
    monitorOutPort.interrupt();
    return true;
}

bool StatusMonitorModule::updateModule(){
    // read from port
	Bottle b;
    monitorInPort.read(b);

    if (isStopping()) {
        ACE_OS::fprintf(stderr,"StatusMonitorModule: stopping\n");
        return false;
    }

	IhaDebug::pmesg(DBGL_DEBUG2,"READ:%s\n",b.toString().c_str());
    if (b.size() <3) {
        IhaDebug::pmesg(DBGL_STATUS2,"StatusMonitorModule: data not valid\n");
        return true;
    }

	ConstString _subsys = b.get(0).asString();
	ConstString _item = b.get(1).asString();
	int _value = b.get(2).asInt();

    // If this is a new subsystem, register it
    if (systemStatus.count(_subsys.c_str())==0) {
        IhaDebug::pmesg(DBGL_DEBUG1,"StatusMonitorModule: Adding subsystem %s\n",_subsys.c_str());
        systemStatus[_subsys.c_str()] = new SubsystemStatus();
        systemStatus[_subsys.c_str()]->setName(_subsys.c_str());
    }

    // Add the item and value
    IhaDebug::pmesg(DBGL_DEBUG1,"Adding: %s %s %d\n",_subsys.c_str(),_item.c_str(),_value);
    systemStatus[_subsys.c_str()]->int_values[_item.c_str()] = _value;

    addMissingItems();
    debugPrintSystemStatus(DBGL_DEBUG2);
    
    return true;
}

bool StatusMonitorModule::respond(const Bottle &command,Bottle &reply){
        
    return false;
} 	

void StatusMonitorModule::addMissingItems() {
    for (map<string,SubsystemStatus*>::iterator ssit = systemStatus.begin(); ssit != systemStatus.end() ; ssit++) {
            SubsystemStatus* ss=ssit->second;
            for (map<string, int>::iterator it=ss->int_values.begin();it!=ss->int_values.end();it++) {
                // Add the item to the other subsystems
                for (map<string,SubsystemStatus*>::iterator ssit2 = systemStatus.begin(); ssit2 != systemStatus.end() ; ssit2++) {
                    IhaDebug::pmesg(DBGL_DEBUG1,"Add %s to subsystem %s\n",it->first.c_str(), ssit2->first.c_str());
                    if ( systemStatus[ssit2->first.c_str()]->int_values.count(it->first.c_str())==0 ) {
                        systemStatus[ssit2->first.c_str()]->int_values[it->first.c_str()] = 0;
                    }
                }
                
            }
    }
}

void StatusOutputThread::run() {
    Bottle b;

    // Print the headers
    if ( systemStatus->size()==0) return;

    SubsystemStatus* ss=systemStatus->begin()->second; // just get from first system
    IhaDebug::pmesg(DBGL_STATUS1,"%10s", "");
    for (map<string, int>::iterator it=ss->int_values.begin();it!=ss->int_values.end();it++) {
        IhaDebug::pmesg(DBGL_STATUS1," %10s", it->first.c_str());
    }
    IhaDebug::pmesg(DBGL_STATUS1,"\n");

    // And now the items
    // Also add the items to the bottle
    for (map<string,SubsystemStatus*>::iterator ssit = systemStatus->begin(); ssit != systemStatus->end() ; ssit++) {
        IhaDebug::pmesg(DBGL_STATUS1,"%10s:",ssit->first.c_str());
        b.addString(ssit->first.c_str());
        SubsystemStatus* ss=ssit->second;
        for (map<string, int>::iterator it=ss->int_values.begin();it!=ss->int_values.end();it++) {
            IhaDebug::pmesg(DBGL_STATUS1," %10d", it->second);
            b.addString(it->first.c_str());
            b.addInt(it->second);
        }
        IhaDebug::pmesg(DBGL_STATUS1,"\n");
    }

    // write the bottle to the output port
    outPort->write(b);

}
