/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ilaria Gori, Ugo Pattacini
 * email:  ilaria.gori@iit.it, ugo.pattacini@iit.it
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

#ifndef __D4C_CLIENT_H__
#define __D4C_CLIENT_H__

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/d4c/d4c.h>

namespace iCub
{

namespace d4c
{

class D4CClient : public D4C
{
protected:
    bool isOpen;
    int  verbosity;

    std::string remote;
    std::string local;
    std::string carrier;

    mutable yarp::sig::Vector field;
    mutable yarp::sig::Vector xdot;
    mutable yarp::sig::Vector x;
    mutable yarp::sig::Vector xhat;
    mutable yarp::sig::Vector qhat;

    mutable yarp::os::BufferedPort<yarp::os::Property> data;
    yarp::os::RpcClient rpc;

    int printMessage(const int level, const char *format, ...) const;

public:
    D4CClient();
    bool open(const yarp::os::Property &options);
    void close();
    bool addItem(const yarp::os::Property &options, int &item);
    bool eraseItem(const int item);
    bool clearItems();
    bool getItems(yarp::os::Bottle &items);
    bool setProperty(const int item, const yarp::os::Property &options);
    bool getProperty(const int item, yarp::os::Property &options);
    bool enableField();
    bool disableField();
    bool getFieldStatus(bool &status);
    bool enableControl();
    bool disableControl();
    bool enableSimulation();
    bool disableSimulation();
    bool getSimulationStatus(bool &status);
    bool getControlStatus(bool &status);
    bool setPeriod(const int period);
    bool getPeriod(int &period);
    bool setPointStateToTool();
    bool attachToolFrame(const yarp::sig::Vector &x, const yarp::sig::Vector &o);
    bool getToolFrame(yarp::sig::Vector &x, yarp::sig::Vector &o);
    bool removeToolFrame();
    bool getTool(yarp::sig::Vector &x, yarp::sig::Vector &o);
    bool setPointState(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &xdot, const yarp::sig::Vector &odot);
    bool setPointOrientation(const yarp::sig::Vector &o, const yarp::sig::Vector &odot);
    bool getPointState(yarp::sig::Vector &x, yarp::sig::Vector &o,
                       yarp::sig::Vector &xdot, yarp::sig::Vector &odot);
    bool getField(yarp::sig::Vector &field);
    bool getSimulation(yarp::sig::Vector &xhat, yarp::sig::Vector &ohat,
                       yarp::sig::Vector &qhat);
    bool getActiveIF(std::string &activeIF);
    bool setActiveIF(const std::string &activeIF);
    bool getTrajectory(std::deque<yarp::sig::Vector> &trajPos,
                       std::deque<yarp::sig::Vector> &trajOrien,
                       const unsigned int maxIterations=D4C_DEFAULT_MAXITERATIONS,
                       const double Ts=D4C_DEFAULT_TS_DISABLED);
    bool executeTrajectory(const std::deque<yarp::sig::Vector> &trajPos,
                           const std::deque<yarp::sig::Vector> &trajOrien,
                           const double trajTime);
    virtual ~D4CClient();
};

}

}

#endif


