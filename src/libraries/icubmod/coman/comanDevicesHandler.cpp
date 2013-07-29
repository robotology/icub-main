/*
 * Copyright (C) 2013 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "comanDevicesHandler.hpp"
#include "Boards_iface.h"

using namespace yarp::os;
using namespace yarp::dev;

comanDevicesHandler* comanDevicesHandler::_handle = NULL;
yarp::os::Semaphore comanDevicesHandler::comanDevicesHandler_mutex = 1;

comanDevicesHandler::comanDevicesHandler()
{
    _handle = this;
    _initted = false;
}

comanDevicesHandler::~comanDevicesHandler() { };

comanDevicesHandler *comanDevicesHandler::instance()
{
    yTrace();
    comanDevicesHandler_mutex.wait();
    if (NULL == _handle)
    {
        yTrace() << "Calling comanDevicesHandler Constructor";
        _handle = new comanDevicesHandler();

        if (NULL == _handle)
            yError() << "While calling comanDevicesHandler constructor";
        else
            yDebug() << "comanDevicesHandler succesfully instantiated";
    }
    comanDevicesHandler_mutex.post();

    return _handle;
}



bool comanDevicesHandler::open(yarp::os::Searchable& config)
{
    comanDevicesHandler_mutex.wait();

    if(_initted)
    {
        yDebug() << "Already initted!";
        comanDevicesHandler_mutex.post();
        return true;
    }

    /* find the path to the Yaml config file...
       ugly, but for now this class will use 2 config files, one xml and a yamlFile
    */
    Value yamlFile;
    std::cout << config.toString().c_str() << std::endl;

    yarp::os::Bottle general = config.findGroup("GENERAL");
    if(general.isNull())
    {
        yError() << "Missing GENERAL group into config file... cannot proceed!";
        return false;
    }

    if( (yamlFile=general.find("YAML")).isNull() )
    {
        yError() << "Missing Yaml config file... cannot proceed!";
        return false;
    }
    else
    {
        _board_crtl = new Boards_ctrl(yamlFile.asString());
    }


    /* Do all the things needed to a correct initialization of the boards and sockets!!
     * Those thing were done in the _init function of the user application
     */

    // pthread stuff
    _board_crtl->init();
    _board_crtl->start_stop_bc_boards(false);
    yarp::os::Time::delay(0.2);

    yDebug() << "Scan for active boards ....";
    int numActive = _board_crtl->scan4active();
    if(numActive == 0)
    {
        yError() << "No boards found, quitting!!";
        return false;
    }
    else
        yDebug() << "Found " <<  numActive << "boards";

    _board_crtl->configure_boards();

   // tell to ALL dps to start broadcast data
    _board_crtl->start_stop_bc_boards(true);

    _initted = true;
    comanDevicesHandler_mutex.post();
    return true;
}

Boards_ctrl *comanDevicesHandler::getBoard_ctrl_p()
{
    return _board_crtl;
}

bool comanDevicesHandler::close()
{
    yTrace();
    _board_crtl->start_stop_control(false);
    _board_crtl->stop_rx_udp();
    _board_crtl->start_stop_bc_boards(false);
    _board_crtl->start_stop_bc_boards(false);
    return true;
}


