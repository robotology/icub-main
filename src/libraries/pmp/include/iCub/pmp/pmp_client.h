/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#ifndef __PMP_CLIENT_H__
#define __PMP_CLIENT_H__

#include <string>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <iCub/pmp/pmp.h>

namespace iCub
{

namespace pmp
{

class PmpClient : public Pmp
{
protected:
    bool isOpen;
    int  verbosity;

    std::string remote;
    std::string local;

    mutable yarp::sig::Vector field;
    mutable yarp::sig::Vector xdot;
    mutable yarp::sig::Vector x;
    mutable yarp::sig::Vector xhat;
    mutable yarp::sig::Vector qhat;

    mutable yarp::os::BufferedPort<yarp::os::Property> data;
    yarp::os::Port rpc;

    int printMessage(const int level, const char *format, ...) const;

public:
    PmpClient();
    bool open(const yarp::os::Property &options);
    void close();
    bool addItem(const yarp::os::Property &options, int &item);
    bool eraseItem(const int item);
    bool clearItems();
    bool getItems(yarp::os::Bottle &items) const;
    bool setProperty(const int item, const yarp::os::Property &options);
    bool getProperty(const int item, yarp::os::Property &options) const;
    bool enableField();
    bool disableField();
    bool getFieldStatus(bool &status) const;
    bool enableControl();
    bool disableControl();
    bool enableSimulation();
    bool disableSimulation();
    bool getSimulationStatus(bool &status) const;
    bool getControlStatus(bool &status) const;
    bool setPeriod(const int period);
    bool getPeriod(int &period) const;
    bool setPointStateToTool();
    bool attachToolFrame(const yarp::sig::Vector &x, const yarp::sig::Vector &o);
    bool getToolFrame(yarp::sig::Vector &x, yarp::sig::Vector &o) const;
    bool removeToolFrame();
    bool getTool(yarp::sig::Vector &x, yarp::sig::Vector &o) const;
    bool setPointState(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &xdot, const yarp::sig::Vector &odot);
    bool getPointState(yarp::sig::Vector &x, yarp::sig::Vector &o,
                       yarp::sig::Vector &xdot, yarp::sig::Vector &odot) const;
    bool getField(yarp::sig::Vector &field) const;
    bool getSimulation(yarp::sig::Vector &xhat, yarp::sig::Vector &ohat,
                       yarp::sig::Vector &qhat) const;
    bool getActiveIF(std::string &activeIF) const;
    bool setActiveIF(const std::string &activeIF);
    bool getTrajectory(std::deque<yarp::sig::Vector> &trajPos,
                       std::deque<yarp::sig::Vector> &trajOrien,
                       const unsigned int maxIterations=PMP_MAX_ITERATIONS,
                       const double Ts=PMP_TS);
    virtual ~PmpClient();
};

}

}

#endif


