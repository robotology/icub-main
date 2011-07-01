// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_REMOTESALIENCE_INC
#define ICUB_REMOTESALIENCE_INC

// std
#include <iostream>

// yarp
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Port.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Network.h>

// iCub
#include <iCub/vis/SalienceInterfaces.h>

namespace iCub {
    namespace vis {
        class RemoteSalience;
    }
}

using namespace std;
using namespace iCub::vis;
using namespace yarp::os;
using namespace yarp::dev;

/**
 * Remote access to a salience module and 
 * the general methods of a salience filters.
 */
class iCub::vis::RemoteSalience : public DeviceDriver,
                                      public ISalienceControls,
                                      public ISalienceModuleControls{
private:
    Port configPort;

    bool setCommand(int code);
    bool setCommand(int code, double v);
    bool setCommand(int code, int v);
    bool setCommand(int code, string s);
    bool getCommand(int code, double& v) const;
    bool getCommand(int code, int& v) const;
    bool getCommand(int code, string& s) const;
    bool setDouble (int code, int j, double val);
    bool setDoubleBottle(int v, Bottle &bot);
    bool getDouble(int v, int j, double *val);
    bool getDoubleBottle(int v, Bottle &bot);
    bool getString(int code, int j, string& s);
    bool setString(int code, int j, string s);
    

public:
    RemoteSalience();
    virtual ~RemoteSalience();

    virtual bool open(const char * name);
    virtual bool close();

    // ISalienceControls
    virtual bool setFilterName(std::string n);
    virtual bool setChildFilterName(int j, std::string n);
    virtual std::string getFilterName();
    virtual std::string getChildFilterName(int j);
    virtual bool setWeight(double w);
    virtual double getWeight();
    virtual int getChildCount();
    virtual bool setChildWeights(yarp::os::Bottle& subWeights);
    virtual bool getChildWeights(yarp::os::Bottle *subWeights);
    virtual bool setChildWeight(int j, double w);
    virtual double getChildWeight(int j);

    // ISalienceModuleControls
    double getSalienceThreshold();
    bool setSalienceThreshold(double thr);
    int getNumBlurPasses();
    bool setNumBlurPasses(int num);
    //int getTemporalBlur();
    //bool setTemporalBlur(int size);

};

#endif
