// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2026  Mesh Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino, Jacopo Losi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

// system includes
#include <iostream>
#include <string.h>

// Ace & Yarp includes
#include <yarp/os/Time.h>
#include <yarp/os/NetType.h>
#include <yarp/conf/environment.h>

// iCub includes
#include "embObjSkin.h"
#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolSK.h"

#include "EoSkin.h"
#include "iCubCanProtocol.h"


#include <ethResource.h>
#include "../embObjLib/hostTransceiver.hpp"

#include "EOnv.h"
#include "EoCommon.h"

using namespace std;
using namespace iCub::skin::diagnostics;

namespace {
    YARP_LOG_COMPONENT(EMBOBJSKIN, "yarp.dev.embObjSkin")
    // this is the default value for no load condition for skin taxels, meaning 0% pressure, 
    // and it is by definition equal to 240 in decimal. 
    // It is used as initialization value since it is passed by configuration file too on the noLoad parameter, 
    // but it is not necessarily the final no load value during runtime, since it can be changed by configuration file. 
    // So this constant is mainly the pre-config/default initialization value, not necessarily the final runtime no-load value.
    constexpr double defaultSkinNoLoadValue{0xf0}; 

    bool isCANaddressValid(int adr)
    {
        return ((adr>0) && (adr<15));
    }

     // adding these constants under the same enum since they are all related to skin hardware characteristics.
    enum { 
        EMBSK_SIZE_INFO = 128,
        SPECIAL_TRIANGLE_CFG_MAX_NUM = 20,
        MAX_NUM_OF_PATCHES_PER_ETH_BOARD = 2, // this is the maximum number of patches that can be associated to a single eth board. This is a limit that we can set since it is not possible to have more than 2 patches per board (since each patch corresponds to a can port and each board has 2 can ports)
        SKIN_TAXELS_PER_TRIANGLE = 12, // this is the number of taxels per triangle, it is a fixed value since it depends on the hardware design of the skin, and it is not expected to change in the future.
        TRIANGLES_PER_CAN_BOARD = 16 // this is the number of triangles per can board, it is a fixed value since it depends on the hardware design of the skin, and it is not expected to change in the future. Each can board has 16 triangles, each triangle has 12 taxels, so each can board has 192 taxels.
    };
}


std::optional<size_t> SkinPatchInfo::checkCardAddrIsInList(int cardAddr)
{
    auto it = std::find(cardAddrList.begin(), cardAddrList.end(), cardAddr);
    if (it == cardAddrList.end())
        return std::nullopt;
    return static_cast<size_t>(std::distance(cardAddrList.begin(), it));
}

EmbObjSkin::EmbObjSkin() :  _isDiagnosticPresent(false)
{
    res         = NULL;
    ethManager  = NULL;
    opened     = false;
    sensorsNum  = 0;
    _cumulativeTaxels = 0;
    _skCfg.numOfPatches = 0;
    _skCfg.totalCardsNum = 0;

    std::string tmp = yarp::conf::environment::get_string("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
        verbosewhenok = (bool)(yarp::conf::numeric::from_string(tmp, 0U));
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

std::string EmbObjSkin::boardInfo() const
{
    if(nullptr == res)
        return {"[res not yet available]"};
    return std::string("BOARD ") + res->getProperties().boardnameString + " IP " + res->getProperties().ipv4addrString;
}


bool EmbObjSkin::initWithSpecialConfig(yarp::os::Searchable& config)
{
    eOprotID32_t    protoid;
    std::vector<SpecialSkinBoardCfgParam> boardCfgList(_skCfg.totalCardsNum);
    unsigned int    numofcfg;
    unsigned int    j = 0, p = 0;

    if(!_newCfg)
    {
        return true; //if we use old style config then return
    }

    //-----------------------------------------------------------------------------------------------------
    //------------ read special cfg board --------------------------------------------------------------

    numofcfg = _skCfg.totalCardsNum;//set size of my vector boardCfgList;
    //in output the function return number of special board cfg are in file xml
    // TODO: add documentation regarding usage of special configuration in xml file
    bool ret = _cfgReader.readSpecialBoardCfg(config, boardCfgList.data(), &numofcfg);

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
            yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": patch" << boardCfgList[j].patch << "not exists";
            return false;
        }
        //now p is the index of patch.

        eOcanport_t canport = _skCfg.patchInfoList[p].canport;

        //check if card address are in patch
        std::optional<size_t> boardIdx;
        for(int a=boardCfgList[j].boardAddrStart; a<=boardCfgList[j].boardAddrEnd; a++)
        {
            boardIdx = _skCfg.patchInfoList[p].checkCardAddrIsInList(a);
            if(!boardIdx)
            {
                yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << "card with address" << a << "is not present in patch" << _skCfg.patchInfoList[p].idPatch;
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
        for (size_t sensorId = 0; sensorId < TRIANGLES_PER_CAN_BOARD; sensorId++)
        {
            size_t index = TRIANGLES_PER_CAN_BOARD * SKIN_TAXELS_PER_TRIANGLE * (*boardIdx) + sensorId * SKIN_TAXELS_PER_TRIANGLE;
            yCTrace(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": index is" << index << "value is:" << boardCfgList[j].cfg.noLoad;

            // Message head
            for(int k = 0; k < SKIN_TAXELS_PER_TRIANGLE; k++)
            {
                yCTrace(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": size is:" << skindata.size() << "index is" << (index+k) << "value is:" << boardCfgList[j].cfg.noLoad;
                if((index+k) >= skindata.size())
                {
                    yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": index too big";
                }
                skindata[index + k] = boardCfgList[j].cfg.noLoad;
            }
        }

        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[p].indexNv, eoprot_tag_sk_skin_cmmnds_boardscfg);

        if(false == res->setRemoteValue(protoid, &bcfg))
        {
            yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << "Error in send special board config for mtb with addr from" << boardCfgList[j].boardAddrStart << "to addr" << boardCfgList[j].boardAddrEnd;
            return false;
        }

    } //end for for each special board cfg

    SystemClock::delaySystem(0.010); // 10 ms (m.a.a-delay: before it was 0.01)

    //-----------------------------------------------------------------------------------------------------
    //------------ read special cfg triangle --------------------------------------------------------------
    std::vector<SpecialSkinTriangleCfgParam> triangleCfg(SPECIAL_TRIANGLE_CFG_MAX_NUM);
    numofcfg = SPECIAL_TRIANGLE_CFG_MAX_NUM;    //set size of my vector triangleCfg;
                                                //in output the function return number of special triangle cfg are in file xml
    ret =  _cfgReader.readSpecialTriangleCfg(config, triangleCfg.data(), &numofcfg);
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
            yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ":patch" << triangleCfg[j].patch << "not exists";
            return false;
        }
        //now p is index patch

        eOcanport_t canport = _skCfg.patchInfoList[p].canport;

        //check if bcfg.boardAddr is in my patches list
        if(!_skCfg.patchInfoList[p].checkCardAddrIsInList(triangleCfg[j].boardAddr))
        {
            yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << "card with address" << triangleCfg[j].boardAddr << "is not present in patch" << _skCfg.patchInfoList[p].idPatch;
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


        SystemClock::delaySystem(0.010);

        if(false == res->setRemoteValue(protoid, &tcfg))
        {
            yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << "Error in send special triangle config for board CAN" << canport+1 << ":" << triangleCfg[j].boardAddr;
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
    // A card is the number of each entry skinCanAddrsPatchX inside the patches group --> i.e. the number of CAN addresses found in config file. We should change the name of this variable with something more meaningful.
    _skCfg.totalCardsNum = 0;
    _cumulativeTaxels = 0;


    servConfigSkin_t skinconfig;
    parser->parseService(config, skinconfig);

    bPatches = config.findGroup("patches", "skin patches connected to this device");
    if(bPatches.isNull())
    {
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << "patches group is missing";
        return(false);
    }

    _skCfg.numOfPatches=0;
    for (int i=1; i<=MAX_NUM_OF_PATCHES_PER_ETH_BOARD; i++)
    {
        std::string skinCanAddrsPatchx = "skinCanAddrsPatch";
        skinCanAddrsPatchx.append(std::to_string(i));
        if (bPatches.check(skinCanAddrsPatchx))
        {
           _skCfg.numOfPatches++; //we increase the number of patches per each instance of skinCanAddrsPatchX found in config file. 
           bPatchList.addInt32(i);
        }
    }

    _skCfg.patchInfoList.clear();
    _skCfg.patchInfoList.resize(_skCfg.numOfPatches);
    for(int j=1; j<_skCfg.numOfPatches+1; j++)
    {
        int id = bPatchList.get(j-1).asInt32();
        if((id!=1) && (id!=2))
        {
            yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << "expecting at most 2 patches";
            return false;
        }
        _skCfg.patchInfoList[j-1].idPatch = id;
        _skCfg.patchInfoList[j-1].indexNv = convertIdPatch2IndexNv(id);
        _skCfg.patchInfoList[j-1].canport = (1 == id) ? eOcanport1 : eOcanport2;
    }


    for(int i=0; i<_skCfg.numOfPatches; i++)
    {
        char tmp[80];
        int id = _skCfg.patchInfoList[i].idPatch;
        snprintf(tmp, sizeof(tmp), "skinCanAddrsPatch%d", id);

        xtmp = bPatches.findGroup(tmp);
        if(xtmp.isNull())
        {
            yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << "does not find " << tmp << "in xml file";
            return false;
        }

        _skCfg.patchInfoList[i].cardAddrList.resize(xtmp.size()-1);
        _skCfg.patchInfoList[i].taxelsSize = _skCfg.patchInfoList[i].cardAddrList.size() * TRIANGLES_PER_CAN_BOARD * SKIN_TAXELS_PER_TRIANGLE;
        _skCfg.patchInfoList[i].taxelsOffset = _cumulativeTaxels;
        _cumulativeTaxels += _skCfg.patchInfoList[i].taxelsSize;
        for(int j=1; j<xtmp.size(); j++)
        {
            int addr = xtmp.get(j).asInt32();
            _skCfg.totalCardsNum++;
            _skCfg.patchInfoList[i].cardAddrList[j-1] = addr;
        }
    }
    // impose the number of sensors (triangles found in config file)
    sensorsNum = _cumulativeTaxels;
    // sensorsNum is the total number of taxels/sensors on this ETH board.

    // resize the skindata holder
    mtx.lock();

    this->skindata.resize(sensorsNum);
    int ttt = this->skindata.size();
    for (int i=0; i < ttt; i++)
    {
        this->skindata[i]=defaultSkinNoLoadValue; // skindata contains data for all the cards. 240 is the max value for a taxel, i.e. 0% pressure on it.
    }

    mtx.unlock();

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
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": Cannot have so many skin patches. detected" << ethservice.configuration.data.sk.skin.numofpatches << "max is" << eomn_serv_skin_maxpatches;
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
        for (int triangleId = 0; triangleId < TRIANGLES_PER_CAN_BOARD; triangleId++)
        {
            int index = TRIANGLES_PER_CAN_BOARD * SKIN_TAXELS_PER_TRIANGLE * board_idx + triangleId * SKIN_TAXELS_PER_TRIANGLE;

            // Message head
            for (size_t k = 0; k < SKIN_TAXELS_PER_TRIANGLE; k++)
            {
                if((index+k) >= skindata.size())
                    yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": index too big";
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
        yCFatal(EMBOBJSKIN) << __YFUNCTION__ << ": fails to instantiate ethManager";
        return false;
    }

    if(false == ethManager->verifyEthBoardInfo(config, ipv4addr, boardIPstring, boardName))
    {
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": object TheEthManager fails in parsing ETH properties from xml file";
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
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ": fails in fromConfig():" << boardInfo() << ": cannot proceed any further";
        cleanup();
        return false;
    }



    // -- instantiate EthResource etc.

    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": fails because could not instantiate the ethResource for BOARD w/ IP =" << boardIPstring << "... unable to continue";
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
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ": error in ethResources::serviceVerifyActivate() for" << boardInfo();
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
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ": fails to start skin service for" << boardInfo() << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yCDebug(EMBOBJSKIN) << __YFUNCTION__ << ": correctly starts skin service of" << boardInfo();
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

yarp::dev::ReturnValue EmbObjSkin::getNrOfSkinPatches(size_t &n) const
{
    yCDebug(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": n. of enabled skin patches" << _skCfg.numOfPatches;
    n = _skCfg.numOfPatches; //should return the number of patches defined as the parameter `skinCanAddrsPatch1` in the hw configuration file (thus for our boards 1 or 2)
    return yarp::dev::ReturnValue_ok;
}

MAS_status EmbObjSkin::getSkinPatchStatus(size_t sens_index) const
{
    if(sens_index >= _skCfg.numOfPatches)
    {
        return MAS_ERROR;
    }

    return MAS_OK;
}

yarp::dev::ReturnValue EmbObjSkin::getSkinPatchName(size_t sens_index, std::string &name) const
{
    if(sens_index >= _skCfg.numOfPatches)
    {
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": index out of range. Requested:" << sens_index << "max:" << _skCfg.numOfPatches;
        return yarp::dev::ReturnValue::return_code::return_value_error_generic;
    }

    name = "skin_patch_" + std::to_string(_skCfg.patchInfoList[sens_index].idPatch);
    yCDebug(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << name;
    return yarp::dev::ReturnValue_ok;
}

yarp::dev::ReturnValue EmbObjSkin::getSkinPatchMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if(sens_index >= _skCfg.numOfPatches)
    {
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": index out of range. Requested:" << sens_index << "max:" << _skCfg.numOfPatches;
        return yarp::dev::ReturnValue::return_code::return_value_error_generic;
    }

    const auto& patch = _skCfg.patchInfoList[sens_index];
    out.resize(patch.taxelsSize);
    std::lock_guard<std::mutex> lck(mtx);
    for (size_t i = 0; i < patch.taxelsSize; i++)
    {
        out[i] = skindata[patch.taxelsOffset + i];
    }
    timestamp = yarp::os::Time::now();

    return yarp::dev::ReturnValue_ok;
}

size_t EmbObjSkin::getSkinPatchSize(size_t sens_index) const
{
    if(sens_index >= _skCfg.numOfPatches)
    {
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": index out of range. Requested:" << sens_index << "max:" << _skCfg.numOfPatches;
        return 0;
    }
    return _skCfg.patchInfoList[sens_index].taxelsSize; //should return the number of taxels, that is the number of triangles (16) per card, times the number of taxels per triangle (12), times the number of cards per patch found in config file.
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
            yCError(EMBOBJSKIN) << __YFUNCTION__ << ": unable to start skin for" << boardInfo() << "on port" << _skCfg.patchInfoList[i].idPatch;
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
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": fails to add its variables to regulars: cannot proceed any further";
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
                yCDebug(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": added" << id32v.size() << "regular rops";
            char nvinfo[128];
            for (size_t r = 0; r<id32v.size(); r++)
            {
                uint32_t id32 = id32v.at(r);
                eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
                yCDebug(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": added regular rop for" << nvinfo;
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
            yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << "Error in send default board config for patch #" << i;
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
            yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << "Error in send default triangle config for patch #" << i;
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
        yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": received data of patch with nvindex=" << indexpatch;
        return false;
    }

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
            size_t localBoardIndex = 0;
            bool foundCardAddr = false;
            for (size_t cId_index = 0; cId_index< _skCfg.patchInfoList[p].cardAddrList.size(); cId_index++)
            {
                if(_skCfg.patchInfoList[p].cardAddrList[cId_index] == cardAddr)
                {
                    localBoardIndex = cId_index;
                    foundCardAddr = true;
                    break;
                }
            }

            if(!foundCardAddr)
            {
                yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << ": Unknown cardId from skin";
                return false;
            }

            triangle = (canframeid11 & 0x000f);
            msgtype = (int) canframedata[0];

            const size_t patchBase = _skCfg.patchInfoList[p].taxelsOffset;
            const size_t boardBase = localBoardIndex * TRIANGLES_PER_CAN_BOARD * SKIN_TAXELS_PER_TRIANGLE;
            const size_t index = patchBase + boardBase + triangle * SKIN_TAXELS_PER_TRIANGLE;

            // marco.accame: added lock to avoid concurrent access to this->skindata. i lock at triangle resolution ...
            mtx.lock();

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
                            yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() <<
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
            mtx.unlock();
        }
        else if(canframeid11 == 0x100)
        {
            /* Can frame with id =0x100 contains Debug info. SO I skip it.*/
            return true;
        }
        else
        {
            if(error == 0)
                yCError(EMBOBJSKIN) << __YFUNCTION__ << ":" << boardInfo() << "Unknown Message received from skin (" << i << "/" << sizeofarray << "): frameID=" << canframeid11 << " len=" << canframesize << "canframe.data=" << canframedata[0] << " " << canframedata[1] << " " << canframedata[2] << " " << canframedata[3];
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
// eof
