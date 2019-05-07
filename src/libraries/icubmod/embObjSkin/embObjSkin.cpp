// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

// system includes
#include <iostream>
#include <string.h>

// Ace & Yarp includes
#include <yarp/os/Time.h>

#include <embObjSkin.h>

#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolSK.h"

#include "EoSkin.h"
#include "canProtocolLib/iCubCanProtocol.h"


#include <ethResource.h>
#include "../embObjLib/hostTransceiver.hpp"

#include "EOnv.h"

#include <yarp/os/NetType.h>

#include "EoCommon.h"

using namespace std;
using namespace iCub::skin::diagnostics;

bool isCANaddressValid(int adr)
{
    return ((adr>0) && (adr<15));
}


int SkinPatchInfo::checkCardAddrIsInList(int cardAddr)
{
    for (size_t i = 0; i< cardAddrList.size(); i++)
    {
        if(cardAddrList[i] == cardAddr)
            return i;
    }
    return -1;
}

EmbObjSkin::EmbObjSkin() :  mutex(1), _isDiagnosticPresent(false)
{
    res         = NULL;
    ethManager  = NULL;
    opened     = false;
    sensorsNum  = 0;
    _skCfg.numOfPatches = 0;
    _skCfg.totalCardsNum = 0;

    std::string tmp = NetworkBase::getEnvironment("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
        verbosewhenok = (bool)NetType::toInt(tmp);
    }
    else
    {
        verbosewhenok = false;
    }

    parser = NULL;

    memset(&ethservice.configuration, 0, sizeof(ethservice.configuration));
    ethservice.configuration.type = eomn_serv_NONE;
}


EmbObjSkin::~EmbObjSkin()
{
    if(NULL != parser)
    {
        delete parser;
        parser = NULL;
    }
}


bool EmbObjSkin::initWithSpecialConfig(yarp::os::Searchable& config)
{
    Bottle          bNumOfset;
    unsigned int    numOfSets;
    eOprotID32_t    protoid;
    unsigned int    p, j;
    SpecialSkinBoardCfgParam* boardCfgList = new SpecialSkinBoardCfgParam[_skCfg.totalCardsNum]; //please add cleanup!
    unsigned int    numofcfg;

    if(!_newCfg)
    {
        return true; //if we use old style config then return
    }

    //-----------------------------------------------------------------------------------------------------
    //------------ read special cfg board --------------------------------------------------------------

    numofcfg = _skCfg.totalCardsNum;//set size of my vector boardCfgList;
    //in output the function return number of special board cfg are in file xml
    bool ret = _cfgReader.readSpecialBoardCfg(config, boardCfgList, &numofcfg);

    if(!ret)
        return false;


    for(j=0; j<numofcfg; j++) //for each special board config
    {
        //check if patch exist
        for(p=0; p< _skCfg.patchInfoList.size(); p++)
        {
            if(_skCfg.patchInfoList[p].idPatch == boardCfgList[j].patch)
                break;
        }
        if(p>=_skCfg.patchInfoList.size())
        {
            yError() << "EmbObjSkin::initWithSpecialConfig(): in skin of BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << ": patch " << boardCfgList[j].patch << "not exists";
            return false;
        }
        //now p is the index of patch.

        eOcanport_t canport = _skCfg.patchInfoList[p].canport;

        //check if card address are in patch
        int boardIdx = -1;
        for(int a=boardCfgList[j].boardAddrStart; a<=boardCfgList[j].boardAddrEnd; a++)
        {
            boardIdx = _skCfg.patchInfoList[p].checkCardAddrIsInList(a);
            if(-1 == boardIdx)
            {
                yError() << "EmbObjSkin::initWithSpecialConfig(): in skin of BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << " card with address " << a << "is not present in patch " << _skCfg.patchInfoList[p].idPatch;
                return(false);
            }
        }

        // prepare data to send to ems
        eOsk_cmd_boardsCfg_t bcfg;
        // this message is for some addresses only
        bcfg.candestination[0] = bcfg.candestination[1] = 0;
        for(int adr=boardCfgList[j].boardAddrStart; adr<=boardCfgList[j].boardAddrEnd; adr++)
        {
            if(isCANaddressValid(adr))
            {
                eo_common_hlfword_bitset(&bcfg.candestination[canport], adr);
            }
        }

        bcfg.cfg.skintype = boardCfgList[j].cfg.skinType;
        bcfg.cfg.period = boardCfgList[j].cfg.period;
        bcfg.cfg.noload = boardCfgList[j].cfg.noLoad;

        // Init the data vector with special config values from "noLoad" param in config file.
        // This is to have a correct initilization for the data sent through yarp port
        for (size_t sensorId = 0; sensorId < 16; sensorId++)
        {
            size_t index = 16 * 12 * boardIdx + sensorId * 12;

            // Message head
            for(int k = 0; k < 12; k++)
            {
//                yDebug() << "readNewSpecialConfiguration size is: " << data.size() << " index is " << (index+k) << " value is: " << boardCfgList[j].cfg.noLoad;
                if((index+k) >= skindata.size())
                {
                    yError() << "readNewSpecialConfiguration: index too big";
                }
                skindata[index + k] = boardCfgList[j].cfg.noLoad;
            }
        }
//        //uncomment for debug only
//        yDebug() << "\n Special board cfg num " << j;
//        boardCfgList[j].debugPrint();

        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[p].indexNv, eoprot_tag_sk_skin_cmmnds_boardscfg);

        if(false == res->setRemoteValue(protoid, &bcfg))
        {
            yError() << "EmbObjSkin::initWithSpecialConfig(): in skin BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << " Error in send special board config for mtb with addr from"<< boardCfgList[j].boardAddrStart << " to addr " << boardCfgList[j].boardAddrEnd;
            return false;
        }

    } //end for for each special board cfg

    SystemClock::delaySystem(0.010); // 10 ms (m.a.a-delay: before it was 0.01)

    //-----------------------------------------------------------------------------------------------------
    //------------ read special cfg triangle --------------------------------------------------------------
    SpecialSkinTriangleCfgParam triangleCfg[SPECIAL_TRIANGLE_CFG_MAX_NUM];
    numofcfg = SPECIAL_TRIANGLE_CFG_MAX_NUM;    //set size of my vector boardCfgList;
                                                //in output the function return number of special board cfg are in file xml
    ret =  _cfgReader.readSpecialTriangleCfg(config, triangleCfg, &numofcfg);
    if(!ret)
        return false;


    for(j=0; j<numofcfg; j++)
    {
        for(p=0; p< _skCfg.patchInfoList.size(); p++)
        {
            if(_skCfg.patchInfoList[p].idPatch == triangleCfg[j].patch)
                break;
        }
        if(p >= _skCfg.patchInfoList.size())
        {
            yError() << "EmbObjSkin::initWithSpecialConfig(): in skin of bBOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << ": patch " << triangleCfg[j].patch << "not exists";
            return false;
        }
        //now p is index patch

        eOcanport_t canport = _skCfg.patchInfoList[p].canport;

        //check if bcfg.boardAddr is in my patches list
        if(-1 == _skCfg.patchInfoList[p].checkCardAddrIsInList(triangleCfg[j].boardAddr))
        {
            yError() << "EmbObjSkin::initWithSpecialConfig(): in skin of BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString <<  " card with address " << triangleCfg[j].boardAddr << "is not present in patch " << _skCfg.patchInfoList[p].idPatch;
            return(false);
        }

        //prepare data to send to ems
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[p].indexNv, eoprot_tag_sk_skin_cmmnds_trianglescfg);


        eOsk_cmd_trianglesCfg_t tcfg = {0};

        // this message is for one address only
        tcfg.candestination[0] = tcfg.candestination[1] = 0;
        int adr = triangleCfg[j].boardAddr;
        if(isCANaddressValid(adr))
        {
            eo_common_hlfword_bitset(&tcfg.candestination[canport], adr);
        }

        tcfg.idstart = triangleCfg[j].triangleStart;
        tcfg.idend = triangleCfg[j].triangleEnd;
        tcfg.cfg.CDCoffset = triangleCfg[j].cfg.cdcOffset;
        tcfg.cfg.enable = triangleCfg[j].cfg.enabled;
        tcfg.cfg.shift = triangleCfg[j].cfg.shift;


//        //uncomment for debug only
//        yDebug() << "\n Special triangle cfg num " << j;
//        boardCfgList[j].debugPrint();

        // allow the remote board to process one command at a time. 
        // otherwise the board sends up a diagnostics message telling that it cannot send a message in can bus
        // we do that by ... sic ... adding a small delay. 
        // the above "yDebug() << "\n Special triangle cfg num " << j;" used to test was probably doing the same effect as a delay of a few ms
        SystemClock::delaySystem(0.010);

        if(false == res->setRemoteValue(protoid, &tcfg))
        {
            yError() << "EmbObjSkin::initWithSpecialConfig(): in skin BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << " Error in send special triangle config for board CAN" << canport+1 << ":" << triangleCfg[j].boardAddr;
            return false;
        }
    }

    SystemClock::delaySystem(0.010); // 10 ms (m.a.a-delay: before it was 0.01)

    return true;
}


bool EmbObjSkin::fromConfig(yarp::os::Searchable& config)
{
    Bottle bPatches, bPatchList, xtmp;
    //reset total num of cards
    _skCfg.totalCardsNum = 0;


    servConfigSkin_t skinconfig;
    parser->parseService(config, skinconfig);

    bPatches = config.findGroup("patches", "skin patches connected to this device");
    if(bPatches.isNull())
    {
        yError() << "EmbObjSkin::fromConfig(): in skin BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << "patches group is missing";
        return(false);
    }

    _skCfg.numOfPatches=0;
    char tmp[80];
    for (int i=1; i<=2; i++)
    {
        sprintf(tmp,"skinCanAddrsPatch%d",i);
        if (bPatches.check(tmp))
        {
           _skCfg.numOfPatches++;
           bPatchList.addInt(i);
        }
    }

    _skCfg.patchInfoList.clear();
    _skCfg. patchInfoList.resize(_skCfg.numOfPatches);
    for(int j=1; j<_skCfg.numOfPatches+1; j++)
    {
        int id = bPatchList.get(j-1).asInt();
        if((id!=1) && (id!=2))
        {
            yError() << "EmbObjSkin::fromConfig(): in skin BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << "expecting at most 2 patches";
            return false;
        }
        _skCfg.patchInfoList[j-1].idPatch = id;
        _skCfg.patchInfoList[j-1].indexNv = convertIdPatch2IndexNv(id);
        _skCfg.patchInfoList[j-1].canport = (1 == id) ? eOcanport1 : eOcanport2;
        //yDebug("fromConfig: found CAN%d", _skCfg.patchInfoList[j-1].canport+1);
    }


    for(int i=0; i<_skCfg.numOfPatches; i++)
    {
        char tmp[80];
        int id = _skCfg.patchInfoList[i].idPatch;
        snprintf(tmp, sizeof(tmp), "skinCanAddrsPatch%d", id);

        xtmp = bPatches.findGroup(tmp);
        if(xtmp.isNull())
        {
            yError() << "EmbObjSkin::fromConfig(): skin BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << "doesn't find " << tmp << "in xml file";
            return false;
        }

        _skCfg.patchInfoList[i].cardAddrList.resize(xtmp.size()-1);

        for(int j=1; j<xtmp.size(); j++)
        {
            int addr = xtmp.get(j).asInt();
            _skCfg.totalCardsNum++;
            _skCfg.patchInfoList[i].cardAddrList[j-1] = addr;
        }
    }

    // impose the number of sensors (triangles found in config file)
    sensorsNum = 16*12*_skCfg.totalCardsNum;     // max num of card

    // resize the skindata holder
    mutex.wait();

    this->skindata.resize(sensorsNum);
    int ttt = this->skindata.size();
    for (int i=0; i < ttt; i++)
    {
        this->skindata[i]=(double)240;
    }

    mutex.post();

    // fill the ethservice ...

    ethservice.configuration.type = eomn_serv_SK_skin;


    ethservice.configuration.data.sk.skin.boardinfo.type = skinconfig.canboard.type;

    ethservice.configuration.data.sk.skin.boardinfo.protocol.major = skinconfig.canboard.protocol.major;
    ethservice.configuration.data.sk.skin.boardinfo.protocol.minor = skinconfig.canboard.protocol.minor;

    ethservice.configuration.data.sk.skin.boardinfo.firmware.major = skinconfig.canboard.firmware.major;
    ethservice.configuration.data.sk.skin.boardinfo.firmware.minor = skinconfig.canboard.firmware.minor;
    ethservice.configuration.data.sk.skin.boardinfo.firmware.build = skinconfig.canboard.firmware.build;


    ethservice.configuration.data.sk.skin.numofpatches = _skCfg.numOfPatches;
    if(ethservice.configuration.data.sk.skin.numofpatches > eomn_serv_skin_maxpatches)
    {
        yError() << "cannot have so many skin patches. detected" << ethservice.configuration.data.sk.skin.numofpatches << "max is" << eomn_serv_skin_maxpatches;
        return false;
    }
    // tagliato e cucito per le schede eb2 eb4 eb10 ed eb11. da migliorare sia i file xml che il parser.
    for(int np=0; np<ethservice.configuration.data.sk.skin.numofpatches; np++)
    {
        // patch np-th, can1 ... for each address put a bit using eo_common_hlfword_bitset
        ethservice.configuration.data.sk.skin.canmapskin[np][0] = 0;
        //eo_common_hlfword_bitset(&ethservice.configuration.data.sk.skin.canmapskin[np][0], 3);

        // patch np-th, can2 ... for each address put a bit using
        ethservice.configuration.data.sk.skin.canmapskin[np][1] = 0;
  
        eOcanport_t canport = _skCfg.patchInfoList[np].canport;
        
        int max = _skCfg.patchInfoList[np].cardAddrList.size();
        for(int n=0; n<max; n++)
        {
            int adr = _skCfg.patchInfoList[np].cardAddrList.at(n);
            adr = adr;
            if(isCANaddressValid(adr))
            {
                eo_common_hlfword_bitset(&ethservice.configuration.data.sk.skin.canmapskin[np][canport], adr);
                //yDebug("config of service: initting mask for bdr @ CAN%d:%d", canport+1, adr);
            }
        }
    }


    if( _cfgReader.isDefaultBoardCfgPresent(config) && _cfgReader.isDefaultTriangleCfgPresent(config))
    {
        _newCfg = true;
    }
    else
    {
        _newCfg = false;
        return true;
    }

    /*read skin board default configuration*/
    _brdCfg.setDefaultValues();
    if(!_cfgReader.readDefaultBoardCfg(config, &_brdCfg))
        return false;

    // Fill the data vector with default values from "noLoad" param in config file.
    for (int board_idx = 0; board_idx < _skCfg.totalCardsNum; board_idx++)
    {
        for (int triangleId = 0; triangleId < 16; triangleId++)
        {
            int index = 16*12*board_idx + triangleId*12;

            // Message head
            for (size_t k = 0; k < 12; k++)
            {
//                yDebug() << "EO readNewConfiguration (default) size is: " << data.size()
//                         << " index is " << (index+k) << " value is: " << _brdCfg.noLoad;
                if((index+k) >= skindata.size())
                    yError() << "readNewConfiguration: index too big";
                skindata[index + k] = _brdCfg.noLoad;
            }
        }
    }

    /*read skin triangle default configuration*/
    _triangCfg.setDefaultValues();
    if(! _cfgReader.readDefaultTriangleCfg(config, &_triangCfg))
        return false;

    return true;
}


bool EmbObjSkin::open(yarp::os::Searchable& config)
{
    // - first thing to do is verify if the eth manager is available. then i parse info about the eth board.

    ethManager = eth::TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "EmbObjSkin::open() fails to instantiate ethManager";
        return false;
    }

    if(false == ethManager->verifyEthBoardInfo(config, ipv4addr, boardIPstring, boardName))
    {
        yError() << "embObjSkin::open(): object TheEthManager fails in parsing ETH propertiex from xml file";
        return false;
    }
    // add specific info about this device ...



    // - now all other things

    if(NULL == parser)
    {
        parser = new ServiceParser;
    }

    // read config file
    if(false == fromConfig(config))
    {
        yError() << "embObjSkin::init() fails in function fromConfig() for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << ": CANNOT PROCEED ANY FURTHER";
        cleanup();
        return false;
    }



    // -- instantiate EthResource etc.

    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yError() << "embObjSkin::open() fails because could not instantiate the ethResource for BOARD w/ IP = " << boardIPstring << " ... unable to continue";
        return false;
    }

    // now we have an ip address, thus we can set the name in object SkinConfigReader
    char name[80];
    snprintf(name, sizeof(name), "embObjSkin on BOARD %s IP %s", res->getProperties().boardnameString.c_str(), res->getProperties().ipv4addrString.c_str());
    _cfgReader.setName(name);


    if(!res->verifyEPprotocol(eoprot_endpoint_skin))
    {
        cleanup();
        return false;
    }


    const eOmn_serv_parameter_t* servparam = &ethservice;

    if(false == res->serviceVerifyActivate(eomn_serv_category_skin, servparam, 5.0))
    {
        yError() << "embObjSkin::open() has an error in call of ethResources::serviceVerifyActivate() for board" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString;
        cleanup();
        return false;
    }



    if(!init())
        return false;

    /* Following delay is necessary in order to give enough time to skin boards to configure all its triangles */
    SystemClock::delaySystem(0.500);

    if(!initWithSpecialConfig(config))
    {
        cleanup();
        return false;
    }

    if(!configPeriodicMessage())
    {
        cleanup();
        return false;
    }

    if(!start())
    {
        cleanup();
        return false;
    }



    if(false == res->serviceStart(eomn_serv_category_skin))
    {
        yError() << "embObjSkin::open() fails to start skin service for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjSkin::open() correctly starts skin service of BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString;
        }
    }

    return true;
}



void EmbObjSkin::cleanup(void)
{
    if(ethManager == NULL) return;

    int ret = ethManager->releaseResource2(res, this);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
}

bool EmbObjSkin::close()
{
    cleanup();
    return true;
}



int EmbObjSkin::read(yarp::sig::Vector &out)
{
    mutex.wait();
    out = this->skindata;  //old - this needs the running thread
    mutex.post();

    return yarp::dev::IAnalogSensor::AS_OK;
}

int EmbObjSkin::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;;
}

int EmbObjSkin::getChannels()
{
    return sensorsNum;
}

int EmbObjSkin::calibrateSensor()
{
    return true;
}

int EmbObjSkin::calibrateChannel(int ch, double v)
{
    //NOT YET IMPLEMENTED
    return calibrateSensor();
}

int EmbObjSkin::calibrateSensor(const yarp::sig::Vector& v)
{
    //mutex.wait();
    //this->data=v;
    //mutex.post;
    return 0;
}

int EmbObjSkin::calibrateChannel(int ch)
{
    return 0;
}

bool EmbObjSkin::start()
{
    eOprotID32_t      protoid;
    uint8_t           dat;
    bool              ret = true;
    int               i;

    if(_newCfg)
    {
        dat = eosk_sigmode_signal;
    }
    else
    {
        dat = eosk_sigmode_signal_oldway;
    }

    for(i=0; i<_skCfg.numOfPatches;i++)
    {
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[i].indexNv, eoprot_tag_sk_skin_config_sigmode);
        ret = res->setRemoteValue(protoid, &dat);
        if(!ret)
        {
            yError() << "EmbObjSkin::start(): unable to start skin for BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << " on port " <<  _skCfg.patchInfoList[i].idPatch;
            return false;
        }
    }
    return ret;
}

bool EmbObjSkin::configPeriodicMessage(void)
{
    // - configure regular rops

    vector<eOprotID32_t> id32v(0);
    eOprotID32_t protoid = eo_prot_ID32dummy;

    // choose the variables and put them inside vector


    for(int i=0; i<_skCfg.numOfPatches; i++)
    {
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[i].indexNv, eoprot_tag_sk_skin_status_arrayofcandata);
        id32v.push_back(protoid);
    }


    if(false == res->serviceSetRegulars(eomn_serv_category_skin, id32v))
    {
        yError() << "EmbObjSkin::configPeriodicMessage() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjSkin::configPeriodicMessage() added" << id32v.size() << "regular rops to BOARD" << res->getProperties().boardnameString << "with IP" << res->getProperties().ipv4addrString;
            char nvinfo[128];
            for (size_t r = 0; r<id32v.size(); r++)
            {
                uint32_t id32 = id32v.at(r);
                eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
                yDebug() << "\t it added regular rop for" << nvinfo;
            }
        }
    }
    SystemClock::delaySystem(0.005);  // 5 ms (m.a.a-delay: before it was 0)

    return true;
}

bool EmbObjSkin::init()
{
    int j = 0;
    eOprotID32_t                protoid;

    // if old configuration style returns
    if(!_newCfg)
    {
        return true;
    }

    // send default board and triangle configuration (new configuration style)
    eOsk_cmd_boardsCfg_t  defBoardCfg = {0};
    eOsk_cmd_trianglesCfg_t defTriangleCfg = {0};
    size_t                  i, k;

    defBoardCfg.cfg.skintype    = _brdCfg.skinType;
    defBoardCfg.cfg.period      = _brdCfg.period;
    defBoardCfg.cfg.noload      = _brdCfg.noLoad;

    defTriangleCfg.idstart      = 0;
    defTriangleCfg.idend        = 15;
    defTriangleCfg.cfg.enable   = _triangCfg.enabled;
    defTriangleCfg.cfg.shift    =  _triangCfg.shift;
    defTriangleCfg.cfg.CDCoffset = _triangCfg.cdcOffset;

    for(i=0; i<_skCfg.numOfPatches;i++)
    {
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[i].indexNv, eoprot_tag_sk_skin_cmmnds_boardscfg);

        // get min and max address
        uint8_t minAddr = 16;
        uint8_t maxAddr = 0;

        for(k=0; k<_skCfg.patchInfoList[i].cardAddrList.size(); k++)
        {
            int adr = _skCfg.patchInfoList[i].cardAddrList[k];

            if(isCANaddressValid(adr))
            {
                if(_skCfg.patchInfoList[i].cardAddrList[k] <  minAddr)
                    minAddr = _skCfg.patchInfoList[i].cardAddrList[k];

                if(_skCfg.patchInfoList[i].cardAddrList[k] >  maxAddr)
                    maxAddr = _skCfg.patchInfoList[i].cardAddrList[k];
            }
        }



        // we send the config to the whole patch, hence 0xffff
        defBoardCfg.candestination[0] = defBoardCfg.candestination[1] = 0xffff;

        if(false == res->setRemoteValue(protoid, &defBoardCfg))
        {
            yError() << "EmbObjSkin::init(): in skin BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << " Error in send default board config for patch #"<<  i;
            return false;
        }

    }
    SystemClock::delaySystem(0.010);

    for(i=0; i<_skCfg.numOfPatches;i++)
    {
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[i].indexNv, eoprot_tag_sk_skin_cmmnds_trianglescfg);


        // we send the config to the whole patch, hence 0xffff
        defTriangleCfg.candestination[0] = defTriangleCfg.candestination[1] = 0xffff;

        if(false == res->setRemoteValue(protoid, &defTriangleCfg))
        {
            yError() << "EmbObjSkin::init(): in skin BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << " Error in send default triangle config for patch # "<<  i;
            return false;
        }

    }

    opened = true;

    return true;
}

bool EmbObjSkin::initialised()
{
    return opened;
}

eth::iethresType_t EmbObjSkin::type()
{
    return eth::iethres_skin;
}

#undef DEBUG_PRINT_RX_STATS
#if defined(DEBUG_PRINT_RX_STATS)
static uint32_t receivedpatches[2][16] = {0};
static uint32_t counterpa = 0;
#endif

bool EmbObjSkin::update(eOprotID32_t id32, double timestamp, void *rxdata)
{
    uint8_t           msgtype = 0;
    uint8_t           i, triangle = 0;
    static int error = 0;
    int p;
    EOarray* arrayof = (EOarray*)rxdata;
    uint8_t sizeofarray = eo_array_Size(arrayof);

    eOprotIndex_t indexpatch = eoprot_ID2index(id32);

    for(p=0; p<_skCfg.numOfPatches; p++)
    {
        if(_skCfg.patchInfoList[p].indexNv == indexpatch)
            break;
    }
    if(p >= _skCfg.numOfPatches)
    {
        yError() << "EmbObjSkin::update(): skin of BOARD" << res->getProperties().boardnameString << "IP" << res->getProperties().ipv4addrString << ": received data of patch with nvindex= " << indexpatch;
        return false;
    }

   // yDebug() << "received data from " << patchInfoList[p].idPatch << "port";

    errors.resize(sizeofarray);

    for(i=0; i<sizeofarray; i++)
    {       
        eOsk_candata_t *candata = (eOsk_candata_t*) eo_array_At(arrayof, i);

        if(NULL == candata)
        {
            break;
        }

        uint16_t canframeid11 = EOSK_CANDATA_INFO2IDCAN(candata->info);
        uint8_t  canframesize = EOSK_CANDATA_INFO2SIZE(candata->info);
        uint8_t *canframedata = candata->data;

        uint8_t mtbId = 255; // unknown mtb card addr
        uint8_t cardAddr = 0;
        uint8_t valid = 0;
        uint8_t skinClass;

        if(_newCfg)
            skinClass = ICUBCANPROTO_CLASS_PERIODIC_SKIN;
        else
            skinClass = ICUBCANPROTO_CLASS_PERIODIC_ANALOGSENSOR;



        valid = (((canframeid11 & 0x0f00) >> 8) == skinClass) ? 1 : 0;

        if(valid)
        {
            cardAddr = (canframeid11 & 0x00f0) >> 4;
            //get index of start of data of board with addr cardId.
            for (size_t cId_index = 0; cId_index< _skCfg.patchInfoList[p].cardAddrList.size(); cId_index++)
            {
                if(_skCfg.patchInfoList[p].cardAddrList[cId_index] == cardAddr)
                {
                    mtbId = cId_index;
                    if(_skCfg.numOfPatches==2 && p==0)
                        mtbId +=  _skCfg.patchInfoList[1].cardAddrList.size(); //add max num of boards on patch number 2 because they are sorted in decreasing order by can addr
                    break;
                }
            }

            if(mtbId == 255)
            {
                //yError() << "Unknown cardId from skin\n";
                return false;
            }

            //printf("mtbId=%d\n", mtbId);
            triangle = (canframeid11 & 0x000f);
            msgtype = (int) canframedata[0];

            int index=16*12*mtbId + triangle*12;

            // marco.accame: added lock to avoid concurrent access to this->skindata. i lock at triangle resolution ...
            mutex.wait();

            if (msgtype == 0x40)
            {
#if defined(DEBUG_PRINT_RX_STATS)
                receivedpatches[p][cardAddr]++;
                counterpa ++;
#endif
                // Message head
                for(int k = 0; k < 7; k++)
                {
                    skindata[index + k] = canframedata[k + 1];
                }
            }
            else if (msgtype == 0xC0)
            {
                // Message tail
                for(int k = 0; k < 5; k++)
                {
                    skindata[index + k + 7] = canframedata[k + 1];
                }

                // Skin diagnostics
                if (_brdCfg.useDiagnostic)  // if user requests to check the diagnostic
                {
                    if (canframesize == 8)
                    {
                        // Skin diagnostics is active
                        _isDiagnosticPresent = true;

                        // Get error code head and tail
                        short head = canframedata[6];
                        short tail = canframedata[7];
                        int fullMsg = (head << 8) | (tail & 0xFF);

                        // Store error message
                        errors[i].net = indexpatch;
                        errors[i].board = cardAddr;
                        errors[i].sensor = triangle;
                        errors[i].error = fullMsg;

                        if (fullMsg != SkinErrorCode::StatusOK)
                        {
                            yError() << "embObjSkin error code: " <<
                                        "BOARD: " << res->getProperties().boardnameString  <<
                                        "IP:"     << res->getProperties().ipv4addrString <<
                                        "canDeviceNum: " << errors[i].net <<
                                        "board: " <<  errors[i].board <<
                                        "sensor: " << errors[i].sensor <<
                                        "error: " << iCub::skin::diagnostics::printErrorCode(errors[i].error).c_str();
                        }
                    }
                    else
                    {
                        _isDiagnosticPresent = false;
                    }
                }
            }
            mutex.post();
        }
        else if(canframeid11 == 0x100)
        {
            /* Can frame with id =0x100 contains Debug info. SO I skip it.*/
            return true;
        }
        else
        {
            if(error == 0)
                yError() << "EMS: " << res->getProperties().ipv4addrString << " Unknown Message received from skin (" << i<<"/"<< sizeofarray <<"): frameID=" << canframeid11<< " len="<<canframesize << "canframe.data="<<canframedata[0] << " " <<canframedata[1] << " " <<canframedata[2] << " " <<canframedata[3] <<"\n" ;
            error++;
            if (error == 10000)
                error = 0;
        }
    }

#if defined(DEBUG_PRINT_RX_STATS)
    if(counterpa >= 10000)
    {
        counterpa = 0;
        yDebug("pa = {{%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d} {%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d}}",
                receivedpatches[0][0], receivedpatches[0][1], receivedpatches[0][2], receivedpatches[0][3], receivedpatches[0][4], receivedpatches[0][5], receivedpatches[0][6], receivedpatches[0][7],
                receivedpatches[0][8], receivedpatches[0][9], receivedpatches[0][10], receivedpatches[0][11], receivedpatches[0][12], receivedpatches[0][13], receivedpatches[0][14], receivedpatches[0][15],
                receivedpatches[1][0], receivedpatches[1][1], receivedpatches[1][2], receivedpatches[1][3], receivedpatches[1][4], receivedpatches[1][5], receivedpatches[1][6], receivedpatches[1][7],
                receivedpatches[1][8], receivedpatches[1][9], receivedpatches[1][10], receivedpatches[1][11], receivedpatches[1][12], receivedpatches[1][13], receivedpatches[1][14], receivedpatches[1][15]
                );

    }
#endif

    return true;
}

/* *********************************************************************************************************************** */
/* ******* Diagnose skin errors.                                            ********************************************** */
//bool EmbObjSkin::diagnoseSkin(void) {
//    using iCub::skin::diagnostics::DetectedError;   // FG: Skin diagnostics errors
//    using yarp::sig::Vector;

//    if (useDiagnostics) {
//        // Write errors to port
//        for (size_t i = 0; i < errors.size(); ++i) {
//            Vector &out = portSkinDiagnosticsOut.prepare();
//            out.clear();

//            out.push_back(errors[i].net);
//            out.push_back(errors[i].board);
//            out.push_back(errors[i].sensor);
//            out.push_back(errors[i].error);

//            portSkinDiagnosticsOut.write(true);
//        }
//    }

//    return true;
//}
/* *********************************************************************************************************************** */
// eof


