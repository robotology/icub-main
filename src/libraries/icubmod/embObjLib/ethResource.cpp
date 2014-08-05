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
#include <yarp/os/Network.h>
#include <yarp/os/NetType.h>

// embobj
#include "EOropframe_hid.h"
#include "EOarray.h"
#include "EoProtocol.h"
#include "EoManagement.h"
#include "EoProtocolMN.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

#define DEFAULT_MAX_COUNT_STAT  60000 //1 minuts
#define DEFAULT_TIMEOUT_STAT    0.01 //expessed in sec
infoOfRecvPkts::infoOfRecvPkts()
{
    board = 0;      //after ethresource is opened, then this board num is set
    initted = false;
    isInError = false;
    count = 0;
    max_count = DEFAULT_MAX_COUNT_STAT;
    timeout = DEFAULT_TIMEOUT_STAT;
    _verbose = false;

    last_seqNum = 0;
    last_ageOfFrame = 0;
    last_recvPktTime = 0.0;

    totPktLost = 0;
    currPeriodPktLost = 0;
    stat_ageOfFrame = new StatExt();
    stat_periodPkt = new StatExt();
    stat_precessPktTime = new StatExt();


    ConstString statistcs_count_max = NetworkBase::getEnvironment("ETHREC_STATISTICS_COUNT_RECPKT_MAX");
    if (statistcs_count_max!="")
        max_count = NetType::toInt(statistcs_count_max);

    ConstString statistcs_timeout = NetworkBase::getEnvironment("ETHREC_STATISTICS_TIMEOUT_MSEC");
        if (statistcs_timeout!="")
        {
            timeout = (double)(NetType::toInt(statistcs_timeout))/1000; //because timeout is in sec
        }

    yTrace() << "Initialized with timeout " << timeout << "sec and count_rec_pkt " << max_count;


}

ethResources::ethResources()
{
    yTrace();
    how_many_features     = 0;
    ethManager            = NULL;
    lastRecvMsgTimestamp  = -1.0;
    isInRunningMode       = false;
    infoPkts              = new infoOfRecvPkts();
}

ethResources::~ethResources()
{
    yTrace() << info;
    how_many_features   = 0;
    ethManager          = NULL;
}


bool ethResources::open(yarp::os::Searchable &config, FEAT_ID request)
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
    if(config.findGroup("GENERAL").find("verbose").asBool())
    {
        infoPkts->_verbose = true;
    }
    else
    {
        infoPkts->_verbose = false;
    }
    //
    //  EMBOBJ INIT
    //
    bool ret;
    eOipv4addr_t eo_locIp = eo_common_ipv4addr(request.PC104ipAddr.ip1, request.PC104ipAddr.ip2, request.PC104ipAddr.ip3, request.PC104ipAddr.ip4);
    eOipv4addr_t eo_remIp = eo_common_ipv4addr(request.EMSipAddr.ip1, request.EMSipAddr.ip2, request.EMSipAddr.ip3, request.EMSipAddr.ip4);
    Bottle groupProtocol = Bottle(config.findGroup("PROTOCOL"));
    if(!init(groupProtocol, eo_locIp, eo_remIp, request.EMSipAddr.port, rxBUFFERsize, request.boardNum))
    {
        ret = false;
        yError() << "cannot init transceiver... maybe wrong board number... check log and config file.";
    }
    else
    {
        ret = true;
//        yDebug() << "Transceiver succesfully initted.";
    }

    boardNum = request.boardNum;
    ACE_UINT32 hostip = (request.EMSipAddr.ip1 << 24) | (request.EMSipAddr.ip2 << 16) | (request.EMSipAddr.ip3 << 8) | (request.EMSipAddr.ip4);
    ACE_INET_Addr myIP((u_short)request.EMSipAddr.port, hostip);
    remote_dev = myIP;
    transMutex.post();

    infoPkts->setBoardNum(boardNum);

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
    //getHostData(&(request->EPvector), &(request->EPhash_function) );
    how_many_features++;
    transMutex.post();
    return true;
}

int ethResources::deregisterFeature(FEAT_ID request)
{
    yTrace() << request.boardNum;
    transMutex.wait();
    how_many_features--;
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

void infoOfRecvPkts::setBoardNum(int boardnum)
{
    board = boardnum;
}

void infoOfRecvPkts::printStatistics(void)
{
    yDebug()<< "STAT board "<< board<< " curr pkt lost = " << currPeriodPktLost<< "   tot pkt lost = " << totPktLost;
    yDebug()<< "STAT board "<< board<< " age of frame: avg=" << stat_ageOfFrame->mean()<< "ms std=" << stat_ageOfFrame->deviation()<< "ms min=" << stat_ageOfFrame->getMin() << "ms max=" << stat_ageOfFrame->getMax()<< "ms on " << stat_ageOfFrame->count() << "values";
    yDebug()<< "STAT board "<< board<< " period of pkt: avg=" << stat_periodPkt->mean()*1000 << "ms std=" << stat_periodPkt->deviation()*1000 << "ms min=" << stat_periodPkt->getMin()*1000 << "ms max=" << stat_periodPkt->getMax()*1000 << "ms on " << stat_periodPkt->count() << "values";
    yDebug()<< "STAT board "<< board<< " pkt proccess time: avg=" << stat_precessPktTime->mean()*1000 << "ms std=" << stat_precessPktTime->deviation()*1000 << "ms min=" << stat_precessPktTime->getMin()*1000 << "ms max=" << stat_precessPktTime->getMax()*1000 << "ms on " << stat_precessPktTime->count() << "values\n";
}

void infoOfRecvPkts::clearStatistics(void)
{
    stat_ageOfFrame->clear();
    stat_periodPkt->clear();
    stat_precessPktTime->clear();
    currPeriodPktLost = 0;
    initted = false;
}


uint64_t infoOfRecvPkts::getSeqNum(uint8_t *packet)
{
    // marco.accame: ok, but ... it would be better to use a public method of EOropframe, which now we dont have
    EOropframeHeader_t *hdr_ptr = (EOropframeHeader_t*)packet;
    return(hdr_ptr->sequencenumber);
}

uint64_t infoOfRecvPkts::getAgeOfFrame(uint8_t *packet)
{
    // marco.accame: ok, but ... it would be better to use a public method of EOropframe, which now we dont have
    EOropframeHeader_t *hdr_ptr = (EOropframeHeader_t*)packet;
    return(hdr_ptr->ageofframe);
}

void infoOfRecvPkts::updateAndCheck(uint8_t *packet, double reckPktTime, double processPktTime)
{
    uint64_t curr_seqNum = getSeqNum(packet);
    uint64_t curr_ageOfFrame = getAgeOfFrame(packet);
    double curr_periodPkt;
    double diff_ageofframe;
    int diff;

    if(initted)
    {
        //1) check seq num
        if(curr_seqNum != last_seqNum+1)
        {
            int num_lost_pkts=0;

            if(curr_seqNum < (last_seqNum+1))
            {
                if(_verbose)
                    yError()<< "REC PKTS not in order!!!!" << board << " seq num rec=" << curr_seqNum << " expected=" << last_seqNum+1 << "!!!!!!!" ;
            }
            else
            {
                //i lost some pkts
                num_lost_pkts = curr_seqNum - last_seqNum -1;
            }
            currPeriodPktLost+= num_lost_pkts;
            totPktLost+= num_lost_pkts;

            if(_verbose)
                yError()<< "LOST "<< num_lost_pkts <<"  PKTS on board=" << board << " seq num rec="<< curr_seqNum << " expected=" << last_seqNum+1 << "!! curr pkt lost=" << currPeriodPktLost << "  Tot lost pkt=" << totPktLost;
        }

        //2) check ageOfPkt
        diff= (int)(curr_ageOfFrame - last_ageOfFrame);
        diff_ageofframe = (double)(diff/1000); //age of frame is expressed in microsec
        if( diff_ageofframe > (timeout*1000))
        {
            if(_verbose)
                yError() << "Board " << board << ": EMS time(ageOfFrame) between 2 pkts bigger then " << timeout * 1000 << "ms;\t Actual delay is" << diff_ageofframe << "ms diff="<< diff;
        }

        //3) check rec time
        curr_periodPkt = reckPktTime - last_recvPktTime;
        if(curr_periodPkt > timeout)
        {
            if(_verbose)
                yError() << "Board " << board << ": Gap of " << curr_periodPkt*1000 << "ms between two consecutive messages !!!";
        }

        stat_ageOfFrame->add(diff_ageofframe);
        stat_periodPkt->add(curr_periodPkt);
        stat_precessPktTime->add(processPktTime);
    }
    else
    {
        initted = true; //i rec fisrt pkt
        isInError = false;
    }

    //aggiorno i dati;
    last_seqNum = curr_seqNum;
    last_ageOfFrame = curr_ageOfFrame;
    last_recvPktTime = reckPktTime;
    count++;

    if(count == max_count)
    {
        printStatistics();
        clearStatistics();
        count = 0;
    }

}


void ethResources::checkIsAlive(double curr_time)
{
    if((infoPkts->isInError) || (!infoPkts->initted))
    {
        return;
    }

    if((curr_time - infoPkts->last_recvPktTime) > infoPkts->timeout)
    {
        yError() << "Board " << boardNum << ": more than " << infoPkts->timeout *1000 << "ms are passed without any news LAST=" << infoPkts->last_recvPktTime*1000 << "ms";
        infoPkts->isInError =true;
        infoPkts->printStatistics();
        infoPkts->clearStatistics();
    }

}
// I could Call directly the hostTransceiver method instead of this one, I do it here in order to add the mutex
// at EMS level
void ethResources::onMsgReception(uint8_t *data, uint16_t size)
{
     double curr_timeBeforeParsing = yarp::os::Time::now();

     // transMutex.wait();  // spostato all'interno della funzione qui sotto, rimane più chiaro da leggere. Il mutex è lo stesso
     // perchè questa classe (ethResource) deriva da hostTransceiver
     hostTransceiver::onMsgReception(data, size);
     //     transMutex.post();

     double curr_timeAfetrParsing = yarp::os::Time::now();

     if(isInRunningMode)
     {
         infoPkts->updateAndCheck(data, curr_timeBeforeParsing, (curr_timeAfetrParsing-curr_timeBeforeParsing));
     }
}

ACE_INET_Addr   ethResources::getRemoteAddress()
{
    return  remote_dev;
}


bool ethResources::goToConfig(void)
{
    yTrace() << info;

    // attiva il loop di controllo
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_appl, 0, eoprot_tag_mn_appl_cmmnds_go2state);

    eOmn_appl_state_t  desired  = applstate_config;
    if(!addSetMessage(protid, (uint8_t*) &desired))
    {
        yError() << "for var goToConfig";
        return false;
    }
    //double          getLastRecvMsgTimestamp(void);

    isInRunningMode = false;
    return true;
}

bool ethResources::goToRun(void)
{
    yTrace() << info;

    // attiva il loop di controllo
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_appl, 0, eoprot_tag_mn_appl_cmmnds_go2state);

    eOmn_appl_state_t  desired  = applstate_running;
    if(!addSetMessage(protid, (uint8_t*) &desired))
    {
        yError() << "for var goToRun";
        return false;
    }

    isInRunningMode = true;
    return true;
}

double  ethResources::getLastRecvMsgTimestamp(void)
{
    return(infoPkts->last_recvPktTime);
}

bool ethResources::clearPerSigMsg(void)
{
    yTrace() << info;

// remove values to be sent regularly


#if     defined(EOPROT_USE_MN_VERSION_1_0)

    eOmn_ropsigcfg_command_t cmdconfig  = {0};  
    eOropSIGcfg_t sigcfg                = {0};  
    eOprotID32_t IDcmdconfig            = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_ropsigcfg);
    EOarray *array                      = eo_array_New(NUMOFROPSIGCFG, sizeof(eOropSIGcfg_t), &cmdconfig.array); 

    cmdconfig.cmmnd                 = ropsigcfg_cmd_clear;
    cmdconfig.plustime              = 0;
    cmdconfig.plussign              = 0;
    cmdconfig.filler01              = 0;
    cmdconfig.signature             = eo_rop_SIGNATUREdummy;   

#else

    eOmn_cmd_config_t cmdconfig     = {0};
    eOropSIGcfg_t sigcfg            = {0};
    eOprotID32_t IDcmdconfig        = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_config);
    uint16_t targetcapacity         = (sizeof(cmdconfig.array)-sizeof(eOarray_head_t)) / sizeof(eOropSIGcfg_t);
    EOarray *array                  = eo_array_New((uint8_t)targetcapacity, sizeof(eOropSIGcfg_t), cmdconfig.array); // targetcapacity is never more than an uint8_t

    cmdconfig.opcpar.opc            = eomn_opc_config_REGROPs_clear;
    cmdconfig.opcpar.plustime       = 0;
    cmdconfig.opcpar.plussign       = 0;
    cmdconfig.opcpar.dummy01        = 0;
    cmdconfig.opcpar.signature      = eo_rop_SIGNATUREdummy;       

#endif     


 
    //send set command
    if(!addSetMessage(IDcmdconfig, (uint8_t*) &cmdconfig))
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




