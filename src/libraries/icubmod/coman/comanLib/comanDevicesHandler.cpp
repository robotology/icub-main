/*
 * Copyright (C) 2013 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <yarp/os/Time.h>
#include "comanDevicesHandler.hpp"
//#include "Boards_iface.h"
#include <robolli/Boards_exception.h>
using namespace yarp::os;
using namespace yarp::dev;

#define MAX_MILLIAMPERE 25000

comanDevicesHandler* comanDevicesHandler::_handle = NULL;
int comanDevicesHandler::_usedBy = 0;
yarp::os::Semaphore comanDevicesHandler::comanDevicesHandler_mutex = 1;

comanDevicesHandler::comanDevicesHandler():RateThread(500)
{
    _handle = this;
    _initted = false;
    _board_crtl = NULL;
}

comanDevicesHandler::~comanDevicesHandler()
{
    if(_usedBy != 0)
        yError() << "Singleton closed while is still used by " <<  _usedBy << "devices. This souldn't happens\n";
    close();
}

comanDevicesHandler *comanDevicesHandler::instance()
{
    yTrace();
    comanDevicesHandler_mutex.wait();
    if (NULL == _handle)
    {
        yTrace() << "Calling comanDevicesHandler Constructor";
        _handle = new comanDevicesHandler();

        if (NULL == _handle)
        {
            yError() << "While calling comanDevicesHandler constructor";
        }
        else
        {
            yDebug() << "comanDevicesHandler succesfully instantiated";
            _usedBy++;
        }
    }
    else
        _usedBy++;
    comanDevicesHandler_mutex.post();

    return _handle;
}

bool comanDevicesHandler::comanDevicesHandler::deInstance()
{
    yTrace();
    comanDevicesHandler_mutex.wait();
    if (NULL == _handle)
    {
        yError() << "deInstance called but singleton doesn't exists";
    }
    else
    {
        _usedBy--;
        if(0 == _usedBy)
        {
            yDebug() << "closing comanDevicesHandler singleton";
            delete _handle;
            _handle = NULL;
        }
        else if(_usedBy < 0)
        {
            yError() << "comanDevicesHandler singleton _usedBy is " << _usedBy << ", this souldn't happen!!";
            if (NULL != _handle)
            {
                delete _handle;
                _handle = NULL;
            }
        }
    }
    comanDevicesHandler_mutex.post();

    return _handle;
}

bool comanDevicesHandler::open(yarp::os::Searchable& config)
{
    comanDevicesHandler_mutex.wait();

    if(_initted)
    {
//        yDebug() << "Coman handler already initted!";
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
        _board_crtl = new Boards_ctrl(yamlFile.asString().c_str());
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
//        return false;
    }
    else
        yDebug() << "Found " <<  numActive << "boards";

    _board_crtl->configure_boards();

   // tell to ALL dps to start broadcast data
    _board_crtl->start_stop_bc_boards(true);

    initGravityWorkAround();
    _initted = true;
    comanDevicesHandler_mutex.post();
    this->start();
    return true;
}

void comanDevicesHandler::initGravityWorkAround()
{
    for(int i=0; i<_board_crtl->getActiveNum(); i++)
    {
        _gravityOffsets.push_back(0);
    }
    _gravityOffsetsVectorSize = sizeof(int) * _board_crtl->getActiveNum();
}

bool comanDevicesHandler::setGravityOffset(int bId, int offset)
{
    _gravityOffsets[bId] = offset;
    return !_board_crtl->set_gravity_compensation(_gravityOffsets.data(), _gravityOffsetsVectorSize);
}

Boards_ctrl *comanDevicesHandler::getBoard_ctrl_p()
{
    return _board_crtl;
}


void comanDevicesHandler::run()
{
    Dsp_Board * b;
    std::vector<int> crashed_boards;
    std::vector<std::pair<int,int>> faulted_boards;
    std::vector<int> currents;
    _board_crtl->get_bc_data(_ts_bc_data);
    Boards_ctrl::mcs_map_t boards=_board_crtl->get_mcs_map();
    int sumCurrent=0; //mA
    for (std::map<int, McBoard*>::iterator it = boards.begin(); it !=boards.end(); ++it) {
        try {
            b = it->second;
            if ( ! b->stopped) {
                b->check_bc_data_rx();
            }
            mc_bc_data_t& data = _ts_bc_data[b->bId-1].raw_bc_data.mc_bc_data;
            if (data.faults())
                faulted_boards.push_back(std::make_pair(b->bId,data.faults()));
            sumCurrent +=abs(data.Current);
            currents.push_back(data.Current);
        } catch ( boards_error &e ) {
            printf("FATAL ERROR in %s ... %s\n", __FUNCTION__, e.what());
            crashed_boards.push_back(b->bId);
        } catch ( boards_warn &e ) {
            printf("WARNING in %s ... %s\n", __FUNCTION__, e.what());
        }
    }

    int total_connected_boards=_board_crtl->scan4active();

    if (faulted_boards.size()>0)
    {
        std::cout<<"faulted boards:";
        for (int i=0;i<faulted_boards.size();i++)
        {
            std::cout<<" "<<faulted_boards[i].first<<
            ((faulted_boards[i].second == 1)?"Current too high":"")<<
            ((faulted_boards[i].second == 2)?"Temperature too high":"")<<
            ((faulted_boards[i].second == 4)?"DC Supply too high":"")<<
            ((faulted_boards[i].second == 8)?"Motor Stalled":"")<<
            ((faulted_boards[i].second == 16)?"Emergency Stop":"")<<", ";
        }
        std::cout<<std::endl;
    }
    if (crashed_boards.size()>0)
    {
        std::cout<<"crashed boards:";
        for (int i=0;i<crashed_boards.size();i++)
        {
            std::cout<<" "<<crashed_boards[i];
        }
        std::cout<<std::endl;
    }
    if (total_connected_boards-_board_crtl->get_fts_map().size() !=boards.size())
    {
        std::cout<<"the boards connected"<< total_connected_boards-_board_crtl->get_fts_map().size()<<" are less than the starting ones "<<boards.size()<<std::endl;
    }
    if (currents.size()>0)
    {
        //std::cout<<"boards currents";
        for (int i=0;i<currents.size();i++)
        {
            //std::cout<<" "<<currents[i]<<",";
        }
        //std::cout<<std::endl;
    }
    if (sumCurrent>MAX_MILLIAMPERE)
    {
        std::cout<<"the total current: "<<sumCurrent<< " was higher than "<<MAX_MILLIAMPERE<<std::endl;

    }


}




bool comanDevicesHandler::close()
{
    yTrace();
    _board_crtl->start_stop_control(false);
    _board_crtl->stop_rx_udp();
    _board_crtl->start_stop_bc_boards(false);
    _board_crtl->start_stop_bc_boards(false);

    if(_board_crtl != NULL)
    {
        delete _board_crtl;
        _board_crtl = NULL;
    }

    return true;
}


