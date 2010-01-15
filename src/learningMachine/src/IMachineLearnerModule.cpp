/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Arjan Gijsberts
 * email:   arjan.gijsberts@iit.it
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

#include <iostream>
#include <stdexcept>

#include "iCub/IMachineLearnerModule.h"
#include "iCub/FileReaderT.h"


namespace iCub {
namespace learningmachine {


void IMachineLearnerModule::registerPort(Contactable& port, std::string name) {
    if(port.open(name.c_str()) != true) {
        std::string msg("could not register port ");
        msg+=name;
        throw std::runtime_error(msg);
    }
}


bool IMachineLearnerModule::close() {
    unregisterAllPorts();
    return true;
}

std::string IMachineLearnerModule::findFile(std::string fname) {
    std::string full_fname;
    bool ok;

    /*
     * ResourceFinder does _not_ allow us to search directly for a path. So we
     * need to hack this by setting a unique fake key and then search for that
     * key. :(
     * Uncomment this as soon as searching directly for files is disabled. At
     * the moment it is deprecated.
     */
    //ok = this->getResourceFinder().setDefault(fname.c_str(), fname.c_str());
    //if(!ok) {
    //    throw std::runtime_error("Could not inject dummy key in resource finder.");
    //}

    full_fname = std::string(this->getResourceFinder().findFile(fname.c_str()).c_str());
    if(full_fname == "") {
        throw std::runtime_error("Could not find file '" + fname +"' in the standard search paths.");
    }

    return full_fname;
}

void IMachineLearnerModule::loadCommandFile(std::string fname, Bottle* out) {
    FileReaderT<Bottle> str;
    Bottle* cmd = (Bottle*) 0;
    Bottle reply;

    // open reader
    str.open(fname);

    // send all bottles from file to respond method
    while(str.hasNext()) {
        cmd = str.getNext();
        reply.clear();
        if(*cmd != Bottle::getNullBottle()) {
            if(out != (Bottle*) 0) {
                out->addString(cmd->toString().c_str());
            } else {
                std::cout << /*">>> " <<*/ cmd->toString().c_str() << std::endl;
            }
            this->safeRespond(*cmd, reply);
            // imitate RFModule way of reporting
            if(out != (Bottle*) 0) {
                out->addString(reply.toString().c_str());
            } else {
                if(reply.get(0).toString() == "help") {
                    for(int i=0; i<reply.size(); i++) {
                        std::cout << reply.get(i).toString().c_str() << std::endl;
                    }
                } else {
                    std::cout << reply.toString().c_str() << std::endl;
                }
            }
        }
        // clean up
        delete cmd;
        cmd = (Bottle*) 0;

    }
}

} // learningmachine
} // iCub


