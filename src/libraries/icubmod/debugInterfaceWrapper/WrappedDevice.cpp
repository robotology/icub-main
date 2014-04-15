// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2014 iCub Facility, Istituto Italiano di Tecnologia
* Authors: Alberto Cardellino
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/


#include <WrappedDevice.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace iCub::Wrapper::iDebug;

SubDevice::SubDevice()
{
    iDbg_subDev =  0;
    iPos_subDev  =  0;
    base_subDev = -1;
    top_subDev  = -1;
    axes_subDev =  0;

    subdevice   =  0;

    configured  = false;
    attached    = false;
}

bool SubDevice::configure(int base, int top, int axes, const std::string &key)
{
    configured=false;

    base_subDev = base;
    top_subDev  = top;
    axes_subDev = axes;
    id_subDev   = key;

    if (top_subDev < base_subDev)
    {
        cerr << "check configuration file top < base." << endl;
        return false;
    }

    if ((top_subDev-base_subDev+1) != axes_subDev)
    {
        cerr << "check configuration file, number of axes and top/base parameters do not match"<<endl;
        return false;
    }

    if (axes_subDev <= 0)
    {
        cerr << "check number of axes" << endl;
        return false;
    }

    configured = true;
    return true;
}

void SubDevice::detach()
{
    subdevice   = 0;
    iDbg_subDev = 0;

    configured = false;
    attached   = false;
}

bool SubDevice::attach(yarp::dev::PolyDriver *device, const std::string & key)
{
    if (id_subDev != key)
    {
        cerr << "Wrong device sorry." << endl;
        return false;
    }

    //configure first
    if (!configured)
    {
        cerr << "You need to call configure before you can attach any device" << endl;
        return false;
    }

    if (device == 0)
    {
        cerr << "Invalid device (null pointer)\n" << endl;
        return false;
    }

    subdevice = device;

    if(!subdevice->isValid())
    {
        cerr << "Invalid device " << key << " (isValid() returned false)" << endl;
        return false;
    }

    subdevice->view(iDbg_subDev);
    subdevice->view(iPos_subDev);

    if (iDbg_subDev == 0)
        cerr<< "--> Warning iDebug not valid interface\n";

    int deviceJoints=0;

    if ( iPos_subDev !=0)
    {
        if (!iPos_subDev->getAxes(&deviceJoints))
        {
            std::cerr << "Error: unable to gex nunmber of axis of attached device\n";
            return false;
        }

        if (deviceJoints < axes_subDev)
        {
            std::cerr << "check device configuration, number of joints of attached device less than the one specified during configuration.\n";
            return false;
        }

        attached = true;
        return true;
    }
    else
    {
        return false;
    }

    return false;
}

