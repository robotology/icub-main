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

#ifndef __TEMPLATEY_MODULE_H__
#define __TEMPLATEY_MODULE_H__

#include <stdio.h>
#include <string>
#include <iostream>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/debug.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace iha {
        class TemplateModule;
    }
}

using namespace iCub::contrib;
using namespace iCub::iha;

/*ASTERIX
 *
 * DESC Module class
 *
 * \brief See \ref icub_iha_Template
 */
class iCub::contrib::TemplateModule : public yarp::os::Module {

private:

    // ports
    yarp::os::BufferedPort<yarp::os::Bottle> quitPort;
    yarp::os::BufferedPort<yarp::os::Bottle> coordsPort;

    // parameters read from ini file/command line
    int template_int_param;
public:

    TemplateModule();
    virtual ~TemplateModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

};


#endif
