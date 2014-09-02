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

#ifndef __D4C_SERVER_H__
#define __D4C_SERVER_H__

#include <sstream>
#include <string>
#include <map>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

#include <iCub/ctrl/pids.h>
#include <iCub/d4c/d4c.h>

namespace iCub
{

namespace d4c
{

// ancestral class
class Item
{
public:
    std::string name;
    std::string type;
    bool active;

    yarp::sig::Vector center;
    yarp::sig::Vector orientation;
    yarp::sig::Vector radius;
    yarp::sig::Vector color;

    Item();
    virtual bool fromProperty(const yarp::os::Property &options);
    virtual yarp::os::Property toProperty() const;
    virtual yarp::sig::Vector getField(const yarp::sig::Vector &x,
                                       const yarp::sig::Vector &xdot)=0;
    virtual ~Item() { }
};


// target with mass-spring-damper force field
class Target_MSD : public Item
{
public:
    double K;
    double D;

    Target_MSD();
    bool fromProperty(const yarp::os::Property &options);
    yarp::os::Property toProperty() const;
    yarp::sig::Vector getField(const yarp::sig::Vector &x,
                               const yarp::sig::Vector &xdot);
};


// obstacle with gaussian force field
class Obstacle_Gaussian : public Item
{
public:
    double G;
    bool cut_tails;

    Obstacle_Gaussian();
    bool fromProperty(const yarp::os::Property &options);
    yarp::os::Property toProperty() const;
    yarp::sig::Vector getField(const yarp::sig::Vector &x,
                               const yarp::sig::Vector &xdot);
};


class D4CServer;    // forward declaration
class GuiReporter : public yarp::os::PortReport
{
private:
    D4CServer *server;

public:
    GuiReporter();
    void setServer(D4CServer *server);
    void report(const yarp::os::PortInfo &info);
};


class D4CServer : public D4C,
                  public yarp::os::RateThread,
                  public yarp::os::PortReader
{
protected:
    bool isOpen;
    bool fieldEnabled;
    bool controlEnabled;
    bool simulationEnabled;
    bool simulationFirstStep;
    bool offlineMode;
    bool initIntegration;
    bool doInitGuiTrajectory;
    int  verbosity;
    int  period;

    std::string device;
    std::string name;
    std::string robot;
    std::string part;
    std::string activeIF;

    std::map<int,Item*> table;
    int itCnt;
    int doTrajectoryCnt;
    double t0;

    yarp::sig::Vector xdot;
    yarp::sig::Vector x;
    yarp::sig::Vector xhat;
    yarp::sig::Vector qhat;

    yarp::sig::Matrix toolFrame;
    yarp::sig::Matrix invToolFrame;

    iCub::ctrl::Integrator If;
    iCub::ctrl::Integrator Iv;

    yarp::os::Mutex               mutex;
    yarp::dev::PolyDriver         dCtrlLeft;
    yarp::dev::ICartesianControl *iCtrlLeft;
    yarp::dev::PolyDriver         dCtrlRight;
    yarp::dev::ICartesianControl *iCtrlRight;
    yarp::dev::ICartesianControl *iCtrlActive;

    yarp::os::BufferedPort<yarp::os::Property> data;
    yarp::os::BufferedPort<yarp::os::Bottle> gui;
    yarp::os::RpcServer rpc;

    friend class GuiReporter;
    GuiReporter reporter;

    class GuiRequest
    {
        std::string type;
        std::string name;
        std::map<int,Item*>::iterator iter;

        GuiRequest();   // not accessible

    public:

        GuiRequest(const std::string &type, std::map<int,Item*>::iterator &iter)
        {
            std::ostringstream name;
            name<<iter->second->name<<":"<<std::hex<<iter->first;

            this->type=type;
            this->iter=iter;
            this->name=name.str();
        }

        std::string                   getType() const { return type; }
        std::string                   getName() const { return name; }
        std::map<int,Item*>::iterator getIter() const { return iter; }

        bool operator==(const GuiRequest &req)
        {
            return ((type==req.type)&&(iter==req.iter));
        }
    };

    std::deque<GuiRequest> guiQueue;

    Item* itemFactory(const yarp::os::Property &options);
    int   printMessage(const int level, const char *format, ...) const;
    bool  read(yarp::os::ConnectionReader &connection);
    void  scheduleInitGuiTrajectory();
    void  initGuiTrajectory();
    void  updateGuiTrajectory();
    void  eraseGuiTrajectory();
    void  updateGuiItem(const GuiRequest &req);
    void  eraseGuiItem(const GuiRequest &req);
    void  pushUpdateGuiItem(std::map<int,Item*>::iterator &it);
    void  pushEraseGuiItem(std::map<int,Item*>::iterator &it);
    void  handleGuiQueue();    
    void  run();

    yarp::os::Property prepareData();
    void getTargetForCartesianIF(yarp::sig::Vector &pos, yarp::sig::Vector &orien);

public:
    D4CServer();
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
    bool getControlStatus(bool &status);
    bool enableSimulation();
    bool disableSimulation();
    bool getSimulationStatus(bool &status);
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
    virtual ~D4CServer();
};

}

}

#endif


