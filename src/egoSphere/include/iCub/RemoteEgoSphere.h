// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_REMOTEEGOSPHERE_INC
#define ICUB_REMOTEEGOSPHERE_INC

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
#include <iCub/EgoSphereInterfaces.h>

namespace iCub {
    namespace contrib {
        class RemoteEgoSphere;
    }
}

using namespace std;
using namespace iCub::contrib;
using namespace yarp::os;
using namespace yarp::dev;

/**
 * Remote access to EgoSphere module
 */
class iCub::contrib::RemoteEgoSphere : public DeviceDriver,
                                       public IEgoSphereControls{
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
    RemoteEgoSphere();
    virtual ~RemoteEgoSphere();

    virtual bool open(const char *name);
    virtual bool close();

    // IEgoSphereInterfaces
    virtual bool setSaccadicSuppression(bool on);
    virtual bool getSaccadicSuppression();
    virtual bool setSalienceDecay(double rate);
    virtual double getSalienceDecay();
	virtual bool addIORRegion(double azimuth, double elevation);
    virtual bool reset();

};

#endif
