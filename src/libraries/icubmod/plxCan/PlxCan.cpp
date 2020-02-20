// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2008 Robotcub Consortium
* Author: Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/
///
/// $Id: PlxCan.cpp,v 1.9 2008/03/08 10:07:01 babybot Exp $
///

#include "PlxCan.h"
#include "PexApi.h"
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>

using namespace yarp::dev;
using namespace yarp::os;

PlxCan::PlxCan()
{
    handle = new PLXCAN_HANDLE;
}

PlxCan::~PlxCan()
{
    if (handle!=0)
        delete handle;
}

bool PlxCan::canSetBaudRate(unsigned int rate)
{
    printf("PlxCan::canSetBaudRate not yet implemented\n");
    return true;
}

bool PlxCan::canGetBaudRate(unsigned int *rate)
{
    printf("PlxCan::canSetBaudRate not yet implemented\n");
    *rate=0;
    return true;
}

bool PlxCan::canIdAdd(unsigned int id)
{
    int res=plxCanIdAdd(*handle, id);
    if (res!=NTCAN_SUCCESS)
        return false;

    return true;
}

bool PlxCan::canIdDelete(unsigned int id)
{
    printf("PlxCan::canIdDelete not yet implemented\n");
    return true;
}

bool PlxCan::canRead(CanBuffer &msgs,
                     unsigned int size,
                     unsigned int *read,
                     bool wait)
{
    int res;
    *read=size;
    PLXCAN_MSG *tmp=reinterpret_cast<PLXCAN_MSG *>(msgs[0].getPointer());
    if (wait)
        res=plxCanRead(*handle, tmp, read);
    else
        res=plxCanTake(*handle, tmp, read);

    if(res!=NTCAN_SUCCESS)
        return false;

    return true;
}

bool PlxCan::canWrite(const CanBuffer &msgs,
                      unsigned int size,
                      unsigned int *sent,
                      bool wait)
{
    int res;
    *sent=size;

    CanBuffer &buffer=const_cast<CanBuffer &>(msgs);
    const PLXCAN_MSG *tmp=reinterpret_cast<const PLXCAN_MSG *>(buffer[0].getPointer());

    if (wait)
        res=plxCanWrite(*handle, const_cast<PLXCAN_MSG *>(tmp), sent);
    else
        res=plxCanSend(*handle, const_cast<PLXCAN_MSG *>(tmp), sent);

    if (res!=NTCAN_SUCCESS)
        return false;

    return true;
}

bool PlxCan::open(yarp::os::Searchable &par)
{
    int txQueueSize=0;
    int rxQueueSize=0;
    int netId =-1;
    int txTimeout=0;
    int rxTimeout=0;

                         netId=par.check("CanDeviceNum", Value(-1), "numeric identifier of the can device").asInt();
    if  (netId == -1)    netId=par.check("canDeviceNum", Value(-1), "numeric identifier of the can device").asInt();

                         txTimeout=par.check("CanTxTimeout", Value(0),  "timeout on transmission [ms]").asInt();
    if  (txTimeout == 0) txTimeout=par.check("canTxTimeout", Value(0),  "timeout on transmission [ms]").asInt();

                         rxTimeout=par.check("CanRxTimeout", Value(0), "timeout on receive when calling blocking read [ms]").asInt() ;
    if  (rxTimeout == 0) rxTimeout=par.check("canRxTimeout", Value(0), "timeout on receive when calling blocking read [ms]").asInt() ;

    int res = plxCanOpen (netId, txQueueSize, rxQueueSize, txTimeout, rxTimeout, handle);
    if (res != NTCAN_SUCCESS)
    {
        fprintf(stderr, "PlxCan::open() returning false\n");
        return false;
    }

    return true;
}

bool PlxCan::close()
{
    int res;
    if (!handle)
        return false;

    res=plxCanClose (*handle);

    if (res!=NTCAN_SUCCESS)
        return false;

    delete handle;
    handle=0;
    return true;
}

bool PlxCan::canGetErrors(CanErrors &err)
{
    PLXCAN_ERRORS error;
    if (plxCanErrors(*handle, &error)!=NTCAN_SUCCESS)
        return false;

    err.busoff=error.busoff;
    err.txCanErrors=error.errors;
    err.rxCanErrors=error.canOvr;
    err.rxCanFifoOvr=error.canOvr;
    err.txCanFifoOvr=error.inputBuffOvr;
    err.rxBufferOvr=error.inputBuffOvr;
    err.txBufferOvr=error.outputBuffOvr;

    return true;
}
