// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "ethResource.h"
#include <ethManager.h>
#include <yarp/os/Time.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

ethResources::ethResources()
{
    yTrace();
    how_many_features     = 0;
    ethManager            = NULL;
    lastRecvMsgTimestamp  = -1.0;
    isInRunningMode             = false;
}

ethResources::~ethResources()
{
    yTrace() << info;
    how_many_features   = 0;
    ethManager          = NULL;
}


bool ethResources::open(FEAT_ID request)
{
   // ACE_TCHAR remoteIp_string[64], localIp_string[64];

    //uint16_t loc_port, rem_port;
//    uint8_t   loc_ip1,loc_ip2,loc_ip3,loc_ip4;
//    uint8_t   rem_ip1,rem_ip2,rem_ip3,rem_ip4;

    // Get the pointer to the actual Singleton ethManager
    ethManager = TheEthManager::instance();

    transMutex.wait();

    // Fill 'info' field with human friendly string
    sprintf(info, "ethResources - referred to EMS: %s:%d",request.EMSipAddr.string, request.EMSipAddr.port);
    yTrace() << "Ems ip address " << info;

    //
    //  EMBOBJ INIT
    //
    bool ret;
    eOipv4addr_t eo_locIp = eo_common_ipv4addr(request.PC104ipAddr.ip1, request.PC104ipAddr.ip2, request.PC104ipAddr.ip3, request.PC104ipAddr.ip4);
    eOipv4addr_t eo_remIp = eo_common_ipv4addr(request.EMSipAddr.ip1, request.EMSipAddr.ip2, request.EMSipAddr.ip3, request.EMSipAddr.ip4);
    if(!init(eo_locIp, eo_remIp, request.EMSipAddr.port, rxBUFFERsize, request.boardNum))
    {
        ret = false;
        yError() << "cannot init transceiver... maybe wrong board number... check log and config file.";
    }
    else
    {
        ret = true;
        yDebug() << "Transceiver succesfully initted.";
    }

    boardNum = request.boardNum;
    ACE_UINT32 hostip = (request.EMSipAddr.ip1 << 24) | (request.EMSipAddr.ip2 << 16) | (request.EMSipAddr.ip3 << 8) | (request.EMSipAddr.ip4);
    ACE_INET_Addr myIP((u_short)request.EMSipAddr.port, hostip);
    remote_dev = myIP;
    transMutex.post();
    return ret;
}

bool ethResources::close()
{
    yTrace();
    return false;
}

bool ethResources::registerFeature(FEAT_ID *request)
{
    yTrace() << request->boardNum;
    transMutex.wait();
    getHostData(&(request->EPvector), &(request->EPhash_function) );
    how_many_features++;
    transMutex.post();
    return true;
}

int ethResources::deregisterFeature(FEAT_ID request)
{
    yTrace() << request.boardNum;
    transMutex.wait();
    how_many_features--;
    if(1 == how_many_features)
        yWarning() << "1 feature left";
    if(0 == how_many_features)
        yWarning() << "0 feature left";
    int ret = how_many_features;
    transMutex.post();
    return ret;
}


// Invia un pacchetto sul socket, usato dal thread di invio. Può inviare qualunque cosa, no check su contenuto.
// int ethResources::send(void *data, size_t len)
// {
//     return ethManager->send(data, len, remote_dev);
// }


void ethResources::getPointer2TxPack(uint8_t **pack, uint16_t *size)
{
    transMutex.wait();
    getTransmit(pack, size);
    transMutex.post();
}

//void ethResources::getTxPack(uint8_t *pack, uint16_t *size)
//{
//    transMutex.wait();
//    getTransmit( (uint8_t**) &p_TxPkt, size);
//    memcpy(pack, p_TxPkt, *size);
//
//    transMutex.post();
//}

int  ethResources::getBufferSize()
{
    return (int) RECV_BUFFER_SIZE;
}

// I could Call directly the hostTransceiver method instead of this one, I do it here in order to add the mutex
// at EMS level
void ethResources::onMsgReception(uint8_t *data, uint16_t size)
{
    // transMutex.wait();  // spostato all'interno della funzione qui sotto, rimane più chiaro da leggere. Il mutex è lo stesso
    // perchè questa classe (ethResource) deriva da hostTransceiver
    lastRecvMsgTimestamp = yarp::os::Time::now();
    hostTransceiver::onMsgReception(data, size);
//     transMutex.post();
}

ACE_INET_Addr   ethResources::getRemoteAddress()
{
    return  remote_dev;
}


bool ethResources::goToConfig(void)
{
    yTrace() << info;
    eOnvID_t nvid;
//    EOnv tmp;

    // attiva il loop di controllo
    eOcfg_nvsEP_mn_applNumber_t dummy = 0;  // not used but there for API compatibility
    nvid = eo_cfg_nvsEP_mn_appl_NVID_Get(endpoint_mn_appl, dummy, applNVindex_cmmnds__go2state);

    eOmn_appl_state_t  desired  = applstate_config;
    if(!addSetMessage( nvid, endpoint_mn_appl, (uint8_t*) &desired))
    {
        yError() << "for var goToConfig";
        return false;
    }
    double          getLastRecvMsgTimestamp(void);

    isInRunningMode = false;
    return true;
}

bool ethResources::goToRun(void)
{
    yTrace() << info;
    eOnvID_t nvid;
//    EOnv tmp;

    // attiva il loop di controllo
    eOcfg_nvsEP_mn_applNumber_t dummy = 0;  // not used, here for API compatibility
    nvid = eo_cfg_nvsEP_mn_appl_NVID_Get(endpoint_mn_appl, dummy, applNVindex_cmmnds__go2state);

    eOmn_appl_state_t  desired  = applstate_running;
    if(!addSetMessage( nvid, endpoint_mn_appl, (uint8_t*) &desired))
    {
        yError() << "for var goToRun";
        return false;
    }

    isInRunningMode = true;
    return true;
}

double  ethResources::getLastRecvMsgTimestamp(void)
{
    return lastRecvMsgTimestamp;
}

bool ethResources::clearPerSigMsg(void)
{
    yTrace() << info;

    // Configure values to be sent regularly
    eOnvID_t                  nvid_ropsigcfgassign;       // nvID
    eOmn_ropsigcfg_command_t  ropsigcfgassign;
    EOarray                   *array;                     // array containing nvids to be signalled

    //get nvid
    nvid_ropsigcfgassign = eo_cfg_nvsEP_mn_comm_NVID_Get(endpoint_mn_comm, 0, commNVindex__ropsigcfgcommand);
    //prepare data
    ropsigcfgassign.array.head.capacity = NUMOFROPSIGCFG;
    ropsigcfgassign.array.head.itemsize = sizeof(eOropSIGcfg_t);
    ropsigcfgassign.array.head.size = 0;
    ropsigcfgassign.cmmnd = ropsigcfg_cmd_clear;

    //send set command
    if(!addSetMessage( nvid_ropsigcfgassign, endpoint_mn_comm, (uint8_t*) &ropsigcfgassign))
    {
        yError() << "in clearing periodic signal msg";
        return false;
    }

    return true;

}

bool ethResources::isRunning(void)
{
	return(isInRunningMode);
}
// eof


