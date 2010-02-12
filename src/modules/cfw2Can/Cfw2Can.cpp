// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2010 Robotcub Consortium
* Author: Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/
///
/// $Id: Cfw2Can.cpp,v 1.9 2008/03/08 10:07:01 babybot Exp $
///

#include "libcfw002.h"
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>

using namespace yarp::dev;
using namespace yarp::os;

Cfw2Can::Cfw2Can()
{
    handle = new CFW2CAN_HANDLE;
}

Cfw2Can::~Cfw2Can()
{
    if (handle!=0)
        delete handle;
}

bool Cfw2Can::canSetBaudRate(unsigned int rate)
{
    printf("Cfw2Can::canSetBaudRate not yet implemented\n");
    return true;
}

bool Cfw2Can::canGetBaudRate(unsigned int *rate)
{
    printf("Cfw2Can::canSetBaudRate not yet implemented\n");
    *rate=0;
    return true;
}

bool Cfw2Can::canIdAdd(unsigned int id)
{
    int res=cfw2CanIdAdd(*handle, id);
    if (res!=0)
        return false;

    return true;
}

bool Cfw2Can::canIdDelete(unsigned int id)
{
    printf("Cfw2Can::canIdDelete not yet implemented\n");
    return true;
}

bool Cfw2Can::canRead(CanBuffer &msgs,
                     unsigned int size, 
                     unsigned int *read,
                     bool wait)
{
    int res;
    *read=size;
    CFW2CAN_MSG *tmp=reinterpret_cast<CFW2CAN_MSG *>(msgs[0].getPointer());
    if (wait)
        res=cfw2CanRead(*handle, tmp, read, 1);
    else
        res=cfw2CanRead(*handle, tmp, read, 0);

    if(res!=0)
        return false;

    return true;
}

bool Cfw2Can::canWrite(const CanBuffer &msgs,
                      unsigned int size,
                      unsigned int *sent,
                      bool wait)
{
    int res;
    *sent=size;

    CanBuffer &buffer=const_cast<CanBuffer &>(msgs);
    const CFW2CAN_MSG *tmp=reinterpret_cast<const CFW2CAN_MSG *>(buffer[0].getPointer());

    if (wait)
        res=cfw2CanWrite(*handle, const_cast<CFW2CAN_MSG *>(tmp), sent, 1);
    else
        res=cfw2CanWrite(*handle, const_cast<CFW2CAN_MSG *>(tmp), sent, 0);

    if (res!=NTCAN_SUCCESS)
        return false;

    return true;
}

bool Cfw2Can::open(yarp::os::Searchable &par)
{
    int netId=par.check("CanDeviceNum", Value(-1), 
        "numeric identifier of the can device").asInt();
    int txQueueSize=0;
    int rxQueueSize=0;
    int txTimeout=par.check("CanTxTimeout", Value(0),
        "timeout on transmission [ms]").asInt();
    int rxTimeout=par.check("CanRxTimeout", Value(0),
        "timeout on receive when calling blocking read [ms]").asInt() ;
    fprintf(stderr ,"TxQueueSize=%d RxQueueSize=%d\n", txTimeout, rxTimeout);
    fprintf(stderr ,"Warning, ignoring TxQueueSize and RxQueueSize\n");

    int res = cfw2CanOpen (netId, txQueueSize, rxQueueSize, txTimeout, rxTimeout, handle);
    if (res != NTCAN_SUCCESS)
    {
        fprintf(stderr, "Cfw2Can::open() returning false\n");
        return false;
    }

    return true;
}

bool Cfw2Can::close()
{
    int res;
    if (!handle)
        return false;

    res=cfw2CanClose (*handle);

    if (res!=NTCAN_SUCCESS)
        return false;

    delete handle;
    handle=0;
    return true;
}

bool Cfw2Can::canGetErrors(CanErrors &err)
{
    CFW2CAN_ERRORS error;
    if (cfw2CanErrors(*handle, &error)!=0)
        return false;

    err.busoff=error.busoff;
    err.overflow=error.canOvr;
    err.inBuffOvr=error.inputBuffOvr;
    err.outBuffOvr=error.outputBuffOvr;
    err.errors=error.errors;

    return true;
}
