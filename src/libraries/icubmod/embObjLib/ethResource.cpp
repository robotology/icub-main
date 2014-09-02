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
    how_many_features           = 0;
    ethManager                  = NULL;
    lastRecvMsgTimestamp        = -1.0;
    isInRunningMode             = false;
    infoPkts                    = new infoOfRecvPkts();
    ethResSem                   = new Semaphore(0);
    verifiedRemoteTransceiver   = false;
    boardEPsNumber              = 0;
}

ethResources::~ethResources()
{
    yTrace() << info;
    how_many_features   = 0;
    ethManager          = NULL;

    delete ethResSem;
}


bool ethResources::open(yarp::os::Searchable &cfgtransceiver, yarp::os::Searchable &cfgprotocol, FEAT_ID request)
{
    // Get the pointer to the actual Singleton ethManager
    ethManager = TheEthManager::instance();

    transMutex.wait();

    // Fill 'info' field with human friendly string
    snprintf(info, sizeof(info), "ethResources - referred to EMS: %s:%d", request.EMSipAddr.string, request.EMSipAddr.port);
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
    if(!init(cfgtransceiver, cfgprotocol, eo_locIp, eo_remIp, request.EMSipAddr.port, rxBUFFERsize, request.boardNum))
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


void ethResources::getPointer2TxPack(uint8_t **pack, uint16_t *size, uint16_t *numofrops)
{
    transMutex.wait();
    getTransmit(pack, size, numofrops);
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
    return sizeof(recv_msg);
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
    return(eo_ropframe_seqnum_Get((EOropframe*)packet));
}

uint64_t infoOfRecvPkts::getAgeOfFrame(uint8_t *packet)
{
    return(eo_ropframe_age_Get((EOropframe*)packet));
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

Semaphore* ethResources::GetSemaphore(eOprotEndpoint_t ep, uint32_t signature)
{
    return(ethResSem);
}

// marco.accame on 23 july 2014: i am with wip, thus in commit keep it defined until i have tested it well enough. 


bool ethResources::verifyRemoteTransceiver(yarp::os::Searchable &config)
{
#if !defined(_WIP_CHECK_PROTOCOL_VERSION_)
    return true;
#else
    if(verifiedRemoteTransceiver)
    {
        return(true);
    }

    // step 1: we ask the remote board the eoprot_tag_mn_comm_status variable and then we verify vs transceiver properties and .. mn protocol version

    const eoprot_version_t * pc104versionMN = eoprot_version_of_endpoint_get(eoprot_endpoint_management);
    const double timeout = 0.100;

    eOprotID32_t id = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_status);
    eOmn_comm_status_t brdstatus = {0};
    uint16_t size = 0;
    // the semaphore used for waiting for replies from the board
    yarp::os::Semaphore* sem = GetSemaphore(eoprot_endpoint_management, 0);


    // send ask message
    if(false == addGetMessage(id))
    {
        yError() << "ethResources::verifyRemoteTransceiver() cannot transmit a request about the communication status to board" << get_protBRDnumber()+1 << ": cannot proceed any further";
        return(false);
    }

    // wait for a say message arriving from the board. the eoprot_fun_UPDT_mn_comm_status() function shall release the waiting semaphore
    if(false == sem->waitWithTimeout(timeout))
    {
        yError() << "ethResources::verifyProtocol() had a timeout of" << timeout << "secs when asking the comm status to board" << get_protBRDnumber()+1 << ": cannot proceed any further";
        yError() << "ethResources::verifyProtocol() asks: can you ping the board? if so, is the MN version of board equal to (" << pc104versionMN->major << pc104versionMN->minor << ")? if not, perform FW upgrade. if so, was the ropframe transmitted in time?";
        return(false);
    }

    // get the reply
    if(false == readBufferedValue(id, (uint8_t*)&brdstatus, &size))
    {
        yError() << "ethResources::verifyRemoteTransceiver() cannot read the comm status of board" << get_protBRDnumber()+1 << ": cannot proceed any further";
        return(false);
    }


    // now i must verify that there is the same mn protocol version
    const eoprot_version_t * brdversionMN = pc104versionMN; // at the moment we cannot get it from remote board
    //const eoprot_version_t * brdversionMN = &brdstatus.mnprotocolversion;

    if(pc104versionMN->major != brdversionMN->major)
    {
        yError() << "ethResources::verifyRemoteTransceiver() detected different mn protocol major versions: local =" << pc104versionMN->major << ", remote =" << brdversionMN->major << ": cannot proceed any further. FW upgrade is required";
        return(false);
    }


    if(pc104versionMN->minor != brdversionMN->minor)
    {
        yWarning() << "ethResources::verifyRemoteTransceiver() detected different mn protocol minor versions: local =" << pc104versionMN->minor << ", remote =" << brdversionMN->minor << ": FW upgrade is advised";
    }

    yWarning() << "ethResources::verifyRemoteTransceiver() detected that it is working with MN protocol version = (" << pc104versionMN->major << pc104versionMN->minor << ")";


    // now i must check brdstatus.transceiver vs hostTransceiver::localTransceiverProperties

    if(localTransceiverProperties.listeningPort != brdstatus.transceiver.destinationPort)
    {
        yError() << "ethResources::verifyRemoteTransceiver() detected different ports: local listening =" << localTransceiverProperties.listeningPort << ", remote destination=" << brdstatus.transceiver.destinationPort << ": cannot proceed any further";
        return(false);
    }

    if(localTransceiverProperties.destinationPort != brdstatus.transceiver.listeningPort)
    {
        yError() << "ethResources::verifyRemoteTransceiver() detected different ports: local destination =" << localTransceiverProperties.destinationPort << ", remote listeing=" << brdstatus.transceiver.listeningPort << ": cannot proceed any further";
        return(false);
    }

    if(localTransceiverProperties.maxsizeRXpacket < brdstatus.transceiver.maxsizeTXpacket)
    {
        yError() << "ethResources::verifyRemoteTransceiver() detected too small rx packet: local rx =" << localTransceiverProperties.maxsizeRXpacket << ", remote tx=" << brdstatus.transceiver.maxsizeTXpacket << ": cannot proceed any further";
        return(false);
    }

    yWarning() << "ethResources::verifyRemoteTransceiver() detected that local max rx size of packet = " << localTransceiverProperties.maxsizeRXpacket << "can accept max board tx size of packet = " << brdstatus.transceiver.maxsizeTXpacket;

    if(localTransceiverProperties.maxsizeTXpacket > brdstatus.transceiver.maxsizeRXpacket)
    {
        yError() << "ethResources::verifyRemoteTransceiver() detected too big tx packet: local tx =" << localTransceiverProperties.maxsizeTXpacket << ", remote rx=" << brdstatus.transceiver.maxsizeRXpacket << ": verify";
        //return(false);
    }

    yWarning() << "ethResources::verifyRemoteTransceiver() detected that local max tx size of packet = " << localTransceiverProperties.maxsizeTXpacket << "can be accepted by remote board with max rx size = " << brdstatus.transceiver.maxsizeRXpacket;


//    if(localTransceiverProperties.maxsizeROPframeRegulars != brdstatus.transceiver. )
//    {
//        yError() << "ethResources::verifyRemoteTransceiver() detected different xxx: local =" << localTransceiverProperties. << ", remote=" << brdstatus.transceiver. << ": cannot proceed any further";
//        return(false);
//    }


//    if(localTransceiverProperties.maxsizeROPframeReplies != brdstatus.transceiver.maxsizeROPframeReplies )
//    {
//        yError() << "ethResources::verifyRemoteTransceiver() detected different xxx: local =" << localTransceiverProperties. << ", remote=" << brdstatus.transceiver. << ": cannot proceed any further";
//        return(false);
//    }



//    if(localTransceiverProperties.maxsizeROPframeOccasionals != brdstatus.transceiver.maxsizeROPframeOccasionals )
//    {
//        yError() << "ethResources::verifyRemoteTransceiver() detected different xxx: local =" << localTransceiverProperties. << ", remote=" << brdstatus.transceiver. << ": cannot proceed any further";
//        return(false);
//    }


    if(localTransceiverProperties.maxsizeROP != brdstatus.transceiver.maxsizeROP )
    {
        yError() << "ethResources::verifyRemoteTransceiver() detected different maxsizeROP: local =" << localTransceiverProperties.maxsizeROP << ", remote=" << brdstatus.transceiver.maxsizeROP << ": cannot proceed any further";
        return(false);
    }



//    if(localTransceiverProperties.maxnumberRegularROPs != brdstatus.transceiver.maxnumberRegularROPs )
//    {
//        yError() << "ethResources::verifyRemoteTransceiver() detected different xxx: local =" << localTransceiverProperties. << ", remote=" << brdstatus.transceiver. << ": cannot proceed any further";
//        return(false);
//    }



    // step 2: we ask the number of endpoints in the board. it is useful to verify that their number is not too high
    eOmn_command_t command = {0};
    // prepare message to send. we send a set<> with request of number of endpoints
    id = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_querynumof);
    memset(&command, 0, sizeof(command));
    command.cmd.opc                             = eomn_opc_query_numof_EPs;
    command.cmd.querynumof.opcpar.opc           = eomn_opc_query_numof_EPs;
    command.cmd.querynumof.opcpar.endpoint      = eoprot_endpoint_all;

    // send set message
    if(false == addSetMessage(id, (uint8_t*)&command))
    {
        yError() << "ethResources::verifyRemoteTransceiver() cannot transmit a request about the number of owned endpoints to board" << get_protBRDnumber()+1 << ": cannot proceed any further";
        return(false);
    }

    // wait for a sig message arriving from the board. the eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof() function shall release the waiting semaphore
    if(false == sem->waitWithTimeout(timeout))
    {
        yError() << "ethResources::verifyRemoteTransceiver() had a timeout of" << timeout << "secs when asking the number of endpoints to board" << get_protBRDnumber()+1 << ": cannot proceed any further";
        return(false);
    }

    // get the data of variable containing the reply about the number of endpoints
    id = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_replynumof);
    memset(&command, 0, sizeof(command));

    if(false == readBufferedValue(id, (uint8_t*)&command, &size))
    {
        yError() << "ethResources::verifyRemoteTransceiver() cannot read the number of endpoints of board" << get_protBRDnumber()+1 << ": cannot proceed any further";
        return(false);
    }

    boardEPsNumber = command.cmd.replynumof.opcpar.numberof;

    yWarning() << "ethResources::verifyRemoteTransceiver() detected" << boardEPsNumber << "endpoints in board" << get_protBRDnumber()+1;



    yWarning() << "ethResources::verifyRemoteTransceiver() has validated the transceiver of remote board " << get_protBRDnumber()+1;

    verifiedRemoteTransceiver = true;
    return(true);

#endif//_WIP_CHECK_PROTOCOL_VERSION_
}

bool ethResources::verifyProtocol(yarp::os::Searchable &config, eOprot_endpoint_t ep)
{
#if !defined(_WIP_CHECK_PROTOCOL_VERSION_)
    return true;
#else
    if(false == verifyRemoteTransceiver(config))
    {
        yError() << "ethResources::verifyProtocol() cannot verify transceiver in board" << get_protBRDnumber()+1 << ": cannot proceed any further";
        return(false);
    }

    // 1. send a set<eoprot_tag_mn_comm_cmmnds_command_queryarray> and wait for the arrival of a sig<eoprot_tag_mn_comm_cmmnds_command_replyarray>
    //    the opc to send is eomn_opc_query_array_EPdes which will trigger a opc in reception eomn_opc_reply_array_EPdes
    // 2. the resulting array will contains a eoprot_endpoint_descriptor_t item for the specifeid ep with the protocol version of the ems.



    const int capacityOfArrayOfEPDES = (EOMANAGEMENT_COMMAND_DATA_SIZE - sizeof(eOarray_head_t)) / sizeof(eoprot_endpoint_descriptor_t);
    const double timeout = 0.100;

    eOprotID32_t id = eo_prot_ID32dummy;
    eOmn_command_t command = {0};
    uint16_t size = 0;

    bool res = false;


    uint16_t numOfEPsInXML = 0;

    // the semaphore used for waiting for replies from the board
    yarp::os::Semaphore* sem = GetSemaphore(ep, 0);


    // then we compare with

    const eoprot_version_t * pc104versionMN = eoprot_version_of_endpoint_get(eoprot_endpoint_management);
    const eoprot_version_t * pc104versionEP = eoprot_version_of_endpoint_get(ep);

    uint16_t pc104entitiesinside = eoprot_entities_in_endpoint_numberof_get(get_protBRDnumber(), ep);


    if(boardEPsNumber > capacityOfArrayOfEPDES)
    {   // to support more than capacityOfArrayOfEPDES (= 16 on date of jul 22 2014) endpoints: just send two (or more) eoprot_tag_mn_comm_cmmnds_command_queryarray messages with setnumbers 0 and 1 (or more)
        yError() << "ethResources::verifyProtocol() detected that board" << get_protBRDnumber()+1 << "has" << boardEPsNumber << "endpoints and at most" << capacityOfArrayOfEPDES << "are supported: cannot proceed any further (review the code to support them all)";
        return(false);
    }

    // step 1: ask all the EP descriptors. from them we can extract protocol version of MN and of the target ep
    id = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_queryarray);
    memset(&command, 0, sizeof(command));
    command.cmd.opc                             = eomn_opc_query_array_EPdes;
    command.cmd.queryarray.opcpar.opc           = eomn_opc_query_array_EPdes;
    command.cmd.queryarray.opcpar.endpoint      = eoprot_endpoint_all;
    command.cmd.queryarray.opcpar.setnumber     = 0;
    command.cmd.queryarray.opcpar.setsize       = 0;

    if(false == addSetMessage(id, (uint8_t*)&command))
    {
        yError() << "ethResources::verifyProtocol() cannot transmit a request about the endpoint descriptors to board" << get_protBRDnumber()+1 << ": cannot proceed any further";
        return(false);
    }


    if(false == sem->waitWithTimeout(timeout))
    {
        yError() << "ethResources::verifyProtocol() had a timeout of" << timeout << "secs when asking the endpoint descriptors to board" << get_protBRDnumber()+1 << ": cannot proceed any further";
        return(false);
    }

    // now i get the array of descriptors
    id = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_command_replyarray);
    memset(&command, 0, sizeof(command));
    if(false == readBufferedValue(id, (uint8_t*)&command, &size))
    {
        yError() << "ethResources::verifyProtocol() cannot retrieve the endpoint descriptors of board" << get_protBRDnumber()+1 << ": cannot proceed any further";
        return(false);
    }

    // the array is ...
    eOmn_cmd_replyarray_t* cmdreplyarray = (eOmn_cmd_replyarray_t*)&command.cmd.replyarray;
    EOarray* array = (EOarray*)cmdreplyarray->array;

    yWarning() << "ethResources::verifyProtocol() -> xxx-debug head.capacity, itemsize, size" << array->head.capacity << array->head.itemsize << array->head.size;

    uint8_t sizeofarray = eo_array_Size(array);

    if(sizeofarray != boardEPsNumber)
    {
        yWarning() << "ethResources::verifyProtocol() retrieved from board" << get_protBRDnumber()+1 << ":" << sizeofarray << "endpoint descriptors, and there are" << boardEPsNumber << "endpoints";
    }


    for(int i=0; i<sizeofarray; i++)
    {
        eoprot_endpoint_descriptor_t *epd = (eoprot_endpoint_descriptor_t*)eo_array_At(array, i);
        if(epd->endpoint == eoprot_endpoint_management)
        {
            const eoprot_version_t * pc104versionMN = eoprot_version_of_endpoint_get(eoprot_endpoint_management);
            if(pc104versionMN->major != epd->version.major)
            {
                yError() << "ethResources::verifyProtocol() detected mismatching protocol version.major in board " << get_protBRDnumber()+1 << "for eoprot_endpoint_management: cannot proceed any further. FW upgrade is required";
                return(false);
            }
            if(pc104versionMN->minor != epd->version.minor)
            {
                yWarning() << "ethResources::verifyProtocol() detected mismatching protocol version.minor board " << get_protBRDnumber()+1 << "for eoprot_endpoint_management: FW upgrade is advised";
            }
        }
        if(epd->endpoint == ep)
        {
            const eoprot_version_t * pc104versionEP = eoprot_version_of_endpoint_get(ep);
            if(pc104versionEP->major != epd->version.major)
            {
                yError() << "ethResources::verifyProtocol() detected mismatching protocol version.major in board " << get_protBRDnumber()+1 << " for" << eoprot_EP2string(ep) << ": cannot proceed any further. FW upgrade is required";
                return(false);
            }
            if(pc104versionEP->minor != epd->version.minor)
            {
                yError() << "ethResources::verifyProtocol() detected mismatching protocol version.minor in board " << get_protBRDnumber()+1 << " for" << eoprot_EP2string(ep) << ": FW upgrade is advised";
                return(false);
            }
        }
        //yWarning() << "EP =" << epd->endpoint;
        //yWarning() << "entities inside = " << epd->entitiesinside;
        //yWarning() << "version.major =" << epd->version.major << "version.minor = " << epd->version.minor;
    }

    yWarning() << "ethResources::verifyProtocol() -> enters in a forever loop for(;;);";

    for(;;);

    return(true);
#endif//_WIP_CHECK_PROTOCOL_VERSION_
}

// eof




