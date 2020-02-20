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
#include <yarp/os/Value.h>
#include "Cfw2Can.h"

using namespace yarp::dev;
using namespace yarp::os;

Cfw2Can::Cfw2Can()
{
    handle = new CFWCAN_HANDLE;
}

Cfw2Can::~Cfw2Can()
{
    if (handle!=0)
        delete handle;
}

bool Cfw2Can::canSetBaudRate(unsigned int rate)
{
    //    printf("CfwCan::canSetBaudRate not yet implemented\n");
    return true;
}

bool Cfw2Can::canGetBaudRate(unsigned int *rate)
{
    //    printf("Cfw2Can::canSetBaudRate not yet implemented\n");
    *rate=0;
    return true;
}

bool Cfw2Can::canIdAdd(unsigned int id)
{

#ifdef CFW2_HAS_FILTERS
    int res;
    res = cfwCanSetIdFilter(*handle ,  id , 1);
#endif
    return true;
}

bool Cfw2Can::canIdDelete(unsigned int id)
{

#ifdef CFW2_HAS_FILTERS
    int res;
    res = cfwCanSetIdFilter(*handle ,  id , 0);
#endif
    return true;
}

bool Cfw2Can::canRead(CanBuffer &msgs,
                     unsigned int size,
                     unsigned int *read,
                     bool wait)
{
    int res;
    *read=size;
    CFWCAN_MSG *tmp=reinterpret_cast<CFWCAN_MSG *>(msgs[0].getPointer());
    if (wait)
        res=cfwCanRead(*handle, tmp, read, 1);
    else
        res=cfwCanRead(*handle, tmp, read, 0);

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
    const CFWCAN_MSG *tmp=reinterpret_cast<const CFWCAN_MSG *>(buffer[0].getPointer());

    if (wait)
        res=cfwCanWrite(*handle, const_cast<CFWCAN_MSG *>(tmp), sent, 1);
    else
        res=cfwCanWrite(*handle, const_cast<CFWCAN_MSG *>(tmp), sent, 0);

    if (res!=0)
        return false;

    return true;
}

bool Cfw2Can::open(yarp::os::Searchable &par)
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

    int res = cfwCanOpen (netId, txQueueSize, rxQueueSize, txTimeout, rxTimeout, handle);
    if (res != 0)
    {
        fprintf(stderr, "Cfw2Can::open() returning error %d\n", res);
        return false;
    }
#ifdef CFW2_HAS_FILTERS
    int rtxEnable = cfwCanRtxEnable(*handle);
    if( rtxEnable != 0) {
        fprintf(stderr, "Cfw2Can::RtxEnable() returning error %d\n", rtxEnable);
        return false;
    }
#endif

    return true;
}

bool Cfw2Can::close()
{
    int res;
    if (!handle)
        return false;
#ifdef CFW2_HAS_FILTERS
    int rtxDisable = cfwCanRtxDisable(*handle);
    if(rtxDisable != 0) {
        fprintf(stderr, "Cfw2Can::RtxDisabling() returning error %d\n", rtxDisable);
        return false;
    }
#endif
    res=cfwCanClose (*handle);

    if (res!=0)
        return false;

    delete handle;
    handle=0;
    return true;
}

/*
bool Cfw2Can::canGetErrors(CanErrors &err)
{
    return true;
}
*/

///////// CanMessage
#include <string.h>

CanMessage &Cfw2CanMessage::operator=(const CanMessage &l)
{
    // reinterpret_cast is used here for performance reasons, and it works
    // with the current version of the iCub software. However it can hide bugs
    // if this operator is used in a reckless way (i.e. assigning
    // a CanMessage of different type to a Cfw2CanMessage).
    // For more information, check https://github.com/robotology/icub-main/pull/82
    const Cfw2CanMessage &tmp=reinterpret_cast<const Cfw2CanMessage &>(l);
    memcpy(msg, tmp.msg, sizeof(CFWCAN_MSG));
    return *this;
}
