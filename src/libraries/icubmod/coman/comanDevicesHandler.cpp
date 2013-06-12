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

    yDebug() << "Scan for active boards ....";
    int numActive = _board_crtl->scan4active();
    yDebug() << "Found " <<  numActive << "boards";

    _board_crtl->configure_boards();

#ifdef _HOMING_AND_START_CONTROLS_IN_MOTION_CONTROL_CLASS_

    _board_crtl->body_homing(r_pos, r_vel, r_tor);
    _board_crtl->set_velocity(r_vel, sizeof(r_vel));
    _board_crtl->set_position(r_pos, sizeof(r_pos));
    _board_crtl->set_torque(r_tor, sizeof(r_tor));

    // test settings
    test();
    // ... WAIT  to let dsp thinking .... LEAVE HERE
    sleep(1);

    // tell to dps sets to start the controller
    yDebug() << "Start control r_leg\n";
    _board_crtl->start_control_body(r_leg);

    yDebug() << "Start control l_leg\n";
    _board_crtl->start_control_body(l_leg);

    yDebug() << "Start control waist\n";
    _board_crtl->start_control_body(waist);

    yDebug() << "Start control r_arm\n";
    _board_crtl->start_control_body(r_arm);

    yDebug() << "Start control l_arm\n";
    _board_crtl->start_control_body(l_arm);

    yDebug() << "Start control neck\n";
    _board_crtl->start_control_body(neck);
 

   // tell to ALL dps to start broadcast data
    _board_crtl->start_stop_bc_boards(true);

    // wait for homing ....
    //ts.tv_sec = 5;
    //clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
    sleep(5);
#endif

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


