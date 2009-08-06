// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2008 Robotcub Consortium
* Author: Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/
///
/// $Id: EsdCan.cpp,v 1.6 2009/06/23 11:04:37 babybot Exp $
///

#include "ntcan.h"
#include "EsdCan.h"
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>

using namespace yarp::dev;
using namespace yarp::os;

const int TX_QUEUE_SIZE=2047;
const int RX_QUEUE_SIZE=2047;

EsdCan::EsdCan()
{
    handle = new NTCAN_HANDLE;
}

EsdCan::~EsdCan()
{
    if (handle!=0)
        delete handle;
}

bool EsdCan::canSetBaudRate(unsigned int rate)
{ 
#if WIN32
    DWORD baud=rate;
#else
    uint32_t baud=rate;
#endif

    int res=::canSetBaudrate(*handle, baud);

    if (res!=NTCAN_SUCCESS)
        return false;

    return true;
}

bool EsdCan::canGetBaudRate(unsigned int *rate)
{
#if WIN32
    DWORD baud;
#else
    uint32_t baud;
#endif

    int res=::canGetBaudrate(*handle, &baud);
    *rate=baud;

    if (res!=NTCAN_SUCCESS)
        return false;

    return true;
}

bool EsdCan::canIdAdd(unsigned int id)
{
    int res=::canIdAdd(*handle, static_cast<long>(id));
    if (res!=NTCAN_SUCCESS)
        return false;

    return true;
}

bool EsdCan::canIdDelete(unsigned int id)
{
    int res=::canIdDelete(*handle, static_cast<long>(id));
    if (res!=NTCAN_SUCCESS)
        return false;

    return true;
}

bool EsdCan::canRead(CanBuffer &msgs,
                     unsigned int size, 
                     unsigned int *read,
                     bool wait)
{
    int res;
    #ifdef WIN32
        long lRead=size;
    #else
        int32_t lRead=size;
    #endif

    CMSG *tmp=reinterpret_cast<CMSG *>(msgs[0].getPointer());
    if (wait)
        res=::canRead(*handle, tmp, &lRead, 0);
    else
        res=::canTake(*handle, tmp, &lRead);

    *read=lRead;

    if ( (res==NTCAN_SUCCESS)||(res==NTCAN_RX_TIMEOUT))
        return true;

    fprintf(stderr, "Error: canRead returned with code:%.4X\n", res);
    return false;
}

bool EsdCan::canWrite(const CanBuffer &msgs,
                      unsigned int size,
                      unsigned int *sent,
                      bool wait)
{
    int res;
    #ifdef WIN32
        long lRead=size;
    #else
        int32_t lRead=size;
    #endif
        
    CanBuffer &buffer=const_cast<CanBuffer &>(msgs);
    const CMSG *tmp=reinterpret_cast<const CMSG *>(buffer[0].getPointer());

 //   if (wait)
        res=::canWrite(*handle, const_cast<CMSG *>(tmp), &lRead, 0);
  //  else
   //     res=::canSend(*handle, const_cast<CMSG *>(tmp), &lRead);

    *sent=lRead;

    if (res!=NTCAN_SUCCESS)
        {
            fprintf(stderr, "Error: canWrite returned with code:%.4X\n", res);
            return false;
        }

    return true;
}

bool EsdCan::open(yarp::os::Searchable &par)
{
    int netId=par.check("CanDeviceNum", Value(-1), 
        "numeric identifier of the can device").asInt();
    int txQueueSize=par.check("CanTxQueue", Value(TX_QUEUE_SIZE),
        "length of tx buffer").asInt();
    int rxQueueSize=par.check("CanRxQueue", Value(RX_QUEUE_SIZE),
        "length of rx buffer").asInt();
    int txTimeout=par.check("CanTxTimeout", Value(500),
        "timeout on transmission [ms]").asInt();
    int rxTimeout=par.check("CanRxTimeout", Value(500),
        "timeout on receive when calling blocking read [ms]").asInt() ;

    int mode=0;
    int res = ::canOpen (netId, mode, txQueueSize, rxQueueSize, txTimeout, rxTimeout, handle);
    if (res != NTCAN_SUCCESS)
    {
        //fprintf(stderr, "EsdCan::open() returning false\n");
        return false;
    }

    return true;
}

bool EsdCan::close()
{
    int res;
    if (!handle)
        return false;

    res=::canClose (*handle);

    if (res!=NTCAN_SUCCESS)
        return false;

    delete handle;
    handle=0;
    return true;
}
