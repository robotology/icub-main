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

// embObj  includes
#include "EOnv_hid.h"   // TODO needed??
#include <embObjSkin.h>

#include "EoSkin.h"
// #include <ethManager.h>
#include <ethResource.h>
#include "../embObjLib/hostTransceiver.hpp"


using namespace std;

EmbObjSkin::EmbObjSkin() :  mutex(1)
{
    res         = NULL;
    ethManager  = NULL;
    initted     = false;
    sensorsNum  = 0;
    numOfPatches = 0;
    totalCardsNum = 0;
    memset(info, 0x00, (size_t)SIZE_INFO);
};
bool EmbObjSkin::initWithSpecialConfig(yarp::os::Searchable& config)
{
    int             totConfigSize = 0;
    int             configSize;
    Bottle          bNumOfset;
    int             numOfSets;
    eOprotID32_t    protoid;

    if(!_newCfg)
    {
        return true; //if we use old style config then return
    }

    //-----------------------------------------------------------------------------------------------------
    //------------ read special cfg board --------------------------------------------------------------

    Bottle boardCfgSpecialGroup = config.findGroup("specialCfgBoards", "Special configuration for skin boards");
    if(!boardCfgSpecialGroup.isNull()) //not mandatory field
    {
        eOsk_cmd_boardsCfg_t bcfg;
        totConfigSize = 0;
        configSize = sizeof(eOsk_cmd_boardsCfg_t);

        bNumOfset = boardCfgSpecialGroup.findGroup("numOfSets", "numer of special sets of triangles");
        if(bNumOfset.isNull())
        {
            yError() << "skin " << _fId.name << "numOfSet is missed from specialCfgBoards";
            return(false);
        }
        numOfSets =  bNumOfset.get(1).asInt();
        //verifico se i triangoli appertengono ad una board nella mia lista
        //leggo i dati da cfg e li metto in triangCfgSpecial
        //iniviali a ems
        for(int j=1;j<=numOfSets;j++)
        {
            char tmp[80];
            sprintf(tmp, "boardSetCfg%d", j);

            Bottle &xtmp = boardCfgSpecialGroup.findGroup(tmp);
            if(xtmp.isNull())
            {
                yError() << "skin of board num " << _fId.boardNum << "doesn't find " << tmp << "in specialCfgBoards group in xml file";
                return false;
            }

            int patch           = xtmp.get(1).asInt();
            bcfg.addrstart      = xtmp.get(2).asInt();
            bcfg.addrend        = xtmp.get(3).asInt();
            bcfg.cfg.period     = xtmp.get(4).asInt();
            bcfg.cfg.skintype   = xtmp.get(5).asInt();
            bcfg.cfg.noload     = xtmp.get(6).asInt();

//            //VALE: solo per debug:
//            yError() << "patch" << patch;
//            yError() << "bcfg.addrstart" << bcfg.addrstart;
//            yError() << "bcfg.addrend" << bcfg.addrend;
//            yError() << "bcfg.cfg.skintype" << bcfg.cfg.skintype;
//            yError() << "bcfg.cfg.period" << bcfg.cfg.period;
//            yError() << "bcfg.cfg.noload" << bcfg.cfg.noload;

            //Note: here i don't check if addresses have been configured in patches list.
            //if a message is send to a board not present, it does nothing
            protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, patch, eoprot_tag_sk_skin_cmd_boardscfg);

            if( !(EOK_HOSTTRANSCEIVER_capacityofropframeoccasionals >= (totConfigSize += configSize)) )
            {
                yDebug() << "skin board "<< _fId.boardNum<< " too many stuff to be sent at once... splitting in more messages";
                Time::delay(0.01);
                totConfigSize = 0;
            }

            if(! res->addSetMessage(protoid, (uint8_t*)&bcfg))
            {
                yError() << "skin board "<< _fId.boardNum << " Error in send special board config for mtb with addr from"<<  bcfg.addrstart << " to addr " << bcfg.addrend;
                return false;
            }

            totConfigSize += configSize;
        }
    }
//    else
//    {
//        //VALE solo per debug()
//        yDebug() << "non ho trovato specialCfgBoards";
//
//    }

    //-----------------------------------------------------------------------------------------------------
    //------------ read special cfg triangle --------------------------------------------------------------
    Bottle triangleCfgSpecialGroup = config.findGroup("specialCfgTriangles", "Special configuration for skin triangles");
    if(!triangleCfgSpecialGroup.isNull()) //not mandatory field
    {
        eOsk_cmd_trianglesCfg_t tcfg;
        totConfigSize = 0;
        configSize = sizeof(eOsk_cmd_trianglesCfg_t);

        bNumOfset = triangleCfgSpecialGroup.findGroup("numOfSets", "number of special sets of triangles");
        if(bNumOfset.isNull())
        {
            yError() << "skin " << _fId.name << "numOfSet is missed from SpecialCfgTriangles";
            return(false);
        }
        numOfSets =  bNumOfset.get(1).asInt();
        //verifico se i triangoli appertengono ad una board nella mia lista
        //leggo i dati da cfg e li metto in triangCfgSpecial
        //iniviali a ems
        for(int j=1;j<=numOfSets;j++)
        {
            char tmp[80];
            sprintf(tmp, "triangleSetCfg%d", j);

            Bottle &xtmp = triangleCfgSpecialGroup.findGroup(tmp);
            if(xtmp.isNull())
            {
                yError() << "skin of board num " << _fId.boardNum << "doesn't find " << tmp << "in SpecialCfgTriangles group in xml file";
                return false;
            }

            int patch            = xtmp.get(1).asInt();
            tcfg.boardaddr      = xtmp.get(2).asInt();
            tcfg.idstart        = xtmp.get(3).asInt();
            tcfg.idend          = xtmp.get(4).asInt();
            tcfg.cfg.enable     = xtmp.get(5).asInt();
            tcfg.cfg.shift      = xtmp.get(6).asInt();
            tcfg.cfg.CDCoffset  = xtmp.get(7).asInt();

//            //VALE: solo per debug:
//            yError() << "tcfg.boardaddr" << tcfg.boardaddr;
//            yError() << "tcfg.idstart" << tcfg.idstart;
//            yError() << "tcfg.idend" << tcfg.idend;
//            yError() << "tcfg.cfg.enable" << tcfg.cfg.enable;
//            yError() << "tcfg.cfg.shift" << tcfg.cfg.shift;
//            yError() << "tcfg.cfg.CDCoffset" << tcfg.cfg.CDCoffset;


            //check if bcfg.boardAddr is in my patches list
           bool boardFound = false;
           for(int k=0; k</*cardIdPerPatch[patch].size()*/15 && !boardFound; k++)
           {
               if(cardIdPerPatch[patch][k] == tcfg.boardaddr)
                   boardFound = true;
           }
           if(!boardFound)
           {
               yError() << "skin " << _fId.name << tmp << "with addr not configured in patches";
               continue;
           }
            protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, patch, eoprot_tag_sk_skin_cmd_trianglescfg);

            if( ! (EOK_HOSTTRANSCEIVER_capacityofropframeoccasionals >= (totConfigSize += configSize)) )
            {
                 yDebug() << "skin board "<< _fId.boardNum<< " too many stuff to be sent at once... splitting in more messages";
                Time::delay(0.01);
                totConfigSize = 0;
            }

            if(! res->addSetMessage(protoid, (uint8_t*)&tcfg))
            {
                yError() << "skin board "<< _fId.boardNum << " Error in send default triangle config for mtb "<<  tcfg.boardaddr;
                return false;
            }
            totConfigSize += configSize;
        }
    }
//    else
//    {
//        //VALE solo per debug()
//        yDebug() << "non ho trovato SpecialCfgTriangles";
//
//    }

    return true;
}
bool EmbObjSkin::fromConfig(yarp::os::Searchable& config)
{
    Bottle bPatches, xtmp;
    //reset total num of cards
    totalCardsNum = 0;


    bPatches = config.findGroup("patches", "skin patches connected to this device");
    if(bPatches.isNull())
    {
        yError() << "skin board num " << _fId.boardNum << "patches group is missed!";
        return(false);
    }

    xtmp = bPatches.findGroup("numOfPatches");
    if(xtmp.isNull())
    {
        yError() << "skin board num " << _fId.boardNum << "numOfPatches is missed!";
        return(false);
    }

    numOfPatches = xtmp.get(1).asInt();

    cardIdPerPatch.clear();
    cardIdPerPatch.resize(numOfPatches);

    for(int i=0; i<numOfPatches; i++)
    {
        char tmp[80];
        sprintf(tmp, "skinCanAddrsPatch%d", i+1);

        xtmp = bPatches.findGroup(tmp);
        if(xtmp.isNull())
        {
            yError() << "skin of board num " << _fId.boardNum << "doesn't find " << tmp << "in xml file";
            return false;
        }

        cardIdPerPatch[i].resize(xtmp.size()-1);

        for(int j=1; j<xtmp.size(); j++)
        {
            int id = xtmp.get(j).asInt();
            totalCardsNum++;
            cardIdPerPatch[i][j-1] = id;
        }
    }

//    yError() << "totalCardsNum=" << totalCardsNum;
//    for(int i=0; i<cardIdPerPatch.size(); i++)
//    {
//        for(int j=0; j<cardIdPerPatch[i].size(); j++)
//        {
//            yError() << " elem num " << j << "of patch " << i << "is " << cardIdPerPatch[i][j];
//        }
//    }


    /*read skin boards default configuration*/
    //first of all init _brdCfg field with default values,
    //then read cfg file: if there isn't default cfg I use mine else use read cfg.
    _brdCfg.setDefaultValues();
    Bottle boardCfgDefGroup =config.findGroup("defaultCfgBoard", "Default configuration for skin boards");
    if (boardCfgDefGroup.isNull()==false)
    {
        Bottle xtmp;
        xtmp = boardCfgDefGroup.findGroup("period");
        if (xtmp.isNull())
        {
            yError() << "skin of board num " << _fId.boardNum << "doesn't find period in defaultCfgBoard group in xml file";
            return false;
        }
        _brdCfg.period = xtmp.get(1).asInt();

        xtmp = boardCfgDefGroup.findGroup("skinType");
        if (xtmp.isNull())
        {
            yError() << "skin of board num " << _fId.boardNum << "doesn't find skinType in defaultCfgBoard group in xml file";
            return false;
        }

        _brdCfg.skinType= xtmp.get(1).asInt();

        xtmp = boardCfgDefGroup.findGroup("noLoad");
        if (xtmp.isNull())
        {
            yError() << "skin of board num " << _fId.boardNum << "doesn't find noLoad in defaultCfgBoard group in xml file";
            return false;
        }
        _brdCfg.noLoad= xtmp.get(1).asInt();

        xtmp = boardCfgDefGroup.findGroup("diagnostic");
        if (xtmp.isNull())
        {
            yError() << "skin of board num " << _fId.boardNum << "doesn't find diagnostic in defaultCfgBoard group in xml file";
            return false;
        }
    }

    /*read skin triangle default configuration*/
    //first of all init _triangCfg field with default values,
    //then read cfg file: if there isn't default cfg I use mine else use read cfg.
    _triangCfg.setDefaultValues();
    Bottle triangleCfgDefGroup =config.findGroup("defaultCfgTriangle", "Default configuration for skin triangle");
    if (triangleCfgDefGroup.isNull()==false)
    {
        Bottle xtmp;
        xtmp = triangleCfgDefGroup.findGroup("enabled");
        if (xtmp.isNull())
        {
            yError() << "skin of board num " << _fId.boardNum << "doesn't find enabled in defaultCfgTriangle group in xml file";
            return false;
        }
        _triangCfg.enabled = xtmp.get(1).asBool();

        xtmp = triangleCfgDefGroup.findGroup("shift");
        if (xtmp.isNull())
        {
            yError() << "skin of board num " << _fId.boardNum << "doesn't find shift in defaultCfgTriangle group in xml file";
            return false;
        }

        _triangCfg.shift = xtmp.get(1).asInt();

        xtmp = triangleCfgDefGroup.findGroup("cdcOffset");
        if (xtmp.isNull())
        {
            yError() << "skin of board num " << _fId.boardNum << "doesn't find cdcOffset in defaultCfgTriangle group in xml file";
            return false;
        }

        _triangCfg.cdcOffset = xtmp.get(1).asInt();
    }

    if((boardCfgDefGroup.isNull()==false)&& (triangleCfgDefGroup.isNull()==false))
    {
        _newCfg = true;
        yWarning() << "skin configuration uses new version!!!";
    }
    else
    {
        _newCfg = false;
        yWarning() << "skin configuration uses old version!!!";
    }


//    //VALE: stampo i valori letti per debug
//    yError() << "_brdCfg.period" << _brdCfg.period;
//    yError() << "_brdCfg.skinType" << _brdCfg.skinType;
//    yError() << "_brdCfg.noLoad" << _brdCfg.noLoad;
//    yError() << "_brdCfg.diagnostic" << _brdCfg.diagnostic;
//    yError() << "_triangCfg.enabled"<< _triangCfg.enabled;
//    yError() << "_triangCfg.shitft"<< _triangCfg.shift;
//    yError() << "_triangCfg.cdcOffset"<< _triangCfg.cdcOffset;
//


    return true;
}

bool EmbObjSkin::open(yarp::os::Searchable& config)
{
    std::string str;
    if(config.findGroup("GENERAL").find("Verbose").asInt())
        str=config.toString().c_str();
    else
        str="\n";
    yTrace() << str;

    Bottle          groupEth, parameter;

    int      port;

    // Get both PC104 and EMS ip addresses and port from config file
    groupEth  = Bottle(config.findGroup("ETH"));
    Bottle parameter1( groupEth.find("PC104IpAddress").asString() );
    port      = groupEth.find("CmdPort").asInt();              // .get(1).asInt();
    sprintf(_fId.PC104ipAddr.string, "%s", parameter1.toString().c_str(), port);
    _fId.PC104ipAddr.port = port;

    Bottle parameter2( groupEth.find("IpAddress").asString() );    // .findGroup("IpAddress");
    sprintf(_fId.EMSipAddr.string, "%s", parameter2.toString().c_str());
    _fId.EMSipAddr.port = port;

    sscanf(_fId.EMSipAddr.string,"\"%d.%d.%d.%d", &_fId.EMSipAddr.ip1, &_fId.EMSipAddr.ip2, &_fId.EMSipAddr.ip3, &_fId.EMSipAddr.ip4);
    sscanf(_fId.PC104ipAddr.string,"\"%d.%d.%d.%d", &_fId.PC104ipAddr.ip1, &_fId.PC104ipAddr.ip2, &_fId.PC104ipAddr.ip3, &_fId.PC104ipAddr.ip4);

    sprintf(_fId.EMSipAddr.string,"%u.%u.%u.%u:%u", _fId.EMSipAddr.ip1, _fId.EMSipAddr.ip2, _fId.EMSipAddr.ip3, _fId.EMSipAddr.ip4, _fId.EMSipAddr.port);
    sprintf(_fId.PC104ipAddr.string,"%u.%u.%u.%u:%u", _fId.PC104ipAddr.ip1, _fId.PC104ipAddr.ip2, _fId.PC104ipAddr.ip3, _fId.PC104ipAddr.ip4, _fId.PC104ipAddr.port);

    // Check input parameters
    bool correct=true;

    sprintf(info, "EmbObjSkin - referred to EMS: %s", _fId.EMSipAddr.string);

    if (!correct)
    {
        std::cerr<<"Error: insufficient parameters to EmbObjSkin\n";
        return false;
    }

    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "Unable to instantiate ethManager";
        return false;
    }

    // fill FEAT_ID data
    _fId.type = Skin;

    _fId.boardNum  = 255;
    Value val =config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
    {
        _fId.boardNum =val.asInt();
    }
    else
    {
        yError() << "skin: No board number found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
        return false;
    }
    _fId.ep = eoprot_endpoint_skin;

    //N.B.: use a dynamic_cast to extract correct interface when using this pointer
    _fId.handle = (this);

    /* Once I'm ok, ask for resources, through the _fId struct I'll give the ip addr, port and
    *  and boradNum to the ethManagerin order to create the ethResource requested.
    * I'll Get back the very same sturct filled with other data useful for future handling
    * like the EPvector and EPhash_function */
    res = ethManager->requestResource(&_fId);
    if(NULL == res)
    {
        yError() << "EMS device not instantiated... unable to continue";
        return false;
    }

    if(!isEpManagedByBoard())
    {
        yError() << "EMS "<< _fId.boardNum << "is not connected to skin";
        return false;
    }

    if(!this->fromConfig(config))
    {
        return false;
    }
    //resize data vector with number of triangle found in config file
    sensorsNum=16*12*totalCardsNum;     // max num of card
    data.resize(sensorsNum);
    int ttt = data.size();
    for (int i=0; i < ttt; i++)
        data[i]=(double)255;

    init();
    initWithSpecialConfig(config);
    start();

    res->goToRun();
    yTrace() << "EmbObj Skin for board " << _fId.boardNum << " correctly instatiated\n";
    return true;
}


bool EmbObjSkin::isEpManagedByBoard()
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, 0, eoprot_tag_sk_skin_config_sigmode);

    EOnv nv;
    if(NULL == res->getNVhandler(protoid, &nv))
    {
        return false;
    }
    return true;
}


bool EmbObjSkin::close()
{
    int ret = ethManager->releaseResource(_fId);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
    return true;
}

void EmbObjSkin::setId(FEAT_ID &id)
{
    _fId=id;
}

int EmbObjSkin::read(yarp::sig::Vector &out)
{
    mutex.wait();
    out=data;  //old - this needs the running thread
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
//#warning "create a ROP to start/initialize the MTB, if needed"

//	int 							j=0;
//	eOmc_joint_config_t				a;
//	uint16_t						sizze;
//
//	eOnvID_t nvid = eo_cfg_nvsEP_sk_NVID_Get(endpoint_sk_emsboard_leftlowerarm, 0x00, skinNVindex_sconfig__sigmode);
//	res->load_occasional_rop(eo_ropcode_set, endpoint_sk_emsboard_leftlowerarm, nvid);

    return true;
}

int EmbObjSkin::calibrateChannel(int ch, double v)
{
    //NOT YET IMPLEMENTED
    return calibrateSensor();
}

int EmbObjSkin::calibrateSensor(const yarp::sig::Vector& v)
{
    //data=v;
    return 0;
}

int EmbObjSkin::calibrateChannel(int ch)
{
    return 0;
}

bool EmbObjSkin::start()
{
    eOprotID32_t      protoid;
    uint8_t           dat = 1;
    bool              ret = true;
    int               i;
yDebug() << "sono in start";
    for(i=0; i<numOfPatches && (ret);i++)
    {
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, i, eoprot_tag_sk_skin_config_sigmode);
        yDebug() << "start skin proto id  = " << protoid;
        ret = res->addSetMessage(protoid, &dat);
    }

    if(!ret)
        yError() << "error in start skin for board " << _fId.boardNum << " on port " << i;

    return ret;
}

bool EmbObjSkin::init()
{
//    char str[128];
    int j = 0;

    EOnv                        *cnv;
    eOmn_ropsigcfg_command_t    *ropsigcfgassign;
    EOarray                     *array;
    eOropSIGcfg_t               sigcfg;
    EOnv                        *nvRoot;
    EOnv                        nvtmp;
    eOprotID32_t                protoid;

yDebug() << "entro nella init";
    //
    //	config regulars
    //
    EOnv nvtmp_ropsigcfgassign;
    eOprotID32_t protoid_ropsigcfgassign = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_cmmnds_ropsigcfg );

    yDebug() << "protoid_ropsigcfgassign=" << protoid_ropsigcfgassign;
    cnv = res->getNVhandler(protoid_ropsigcfgassign, &nvtmp_ropsigcfgassign);
    ropsigcfgassign = (eOmn_ropsigcfg_command_t*) cnv->ram;
    array = (EOarray*) &ropsigcfgassign->array;
    eo_array_Reset(array);
    array->head.capacity = NUMOFROPSIGCFG;
    array->head.itemsize = sizeof(eOropSIGcfg_t);
    ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;

    for(int i=0; i<numOfPatches; i++)
    {
        protoid = eoprot_ID_get((eOprotEndpoint_t)_fId.ep, eoprot_entity_sk_skin, i, eoprot_tag_sk_skin_status_arrayof10canframes);
        sigcfg.id32 = protoid;
        eo_array_PushBack(array, &sigcfg);
    }

    // Send message
    if( !res->addSetMessage(protoid_ropsigcfgassign, (uint8_t *) ropsigcfgassign) )
    {
        yError() << "while setting rop sig cfg";
    }


    //send default board and triangle configuration
    if(!_newCfg)
    {
        yDebug() << "init esco";
        return true;
    }
    eOsk_cmd_boardsCfg_t  defBoardCfg;
    eOsk_cmd_trianglesCfg_t defTriangleCfg;
    int                     i,k;

    defBoardCfg.cfg.skintype    = _brdCfg.skinType;
    defBoardCfg.cfg.period      = _brdCfg.period;
    defBoardCfg.cfg.noload      = _brdCfg.noLoad;

    defTriangleCfg.idstart      = 0;
    defTriangleCfg.idend        = 15;
    defTriangleCfg.cfg.enable   = _triangCfg.enabled;
    defTriangleCfg.cfg.shift    =  _triangCfg.shift;
    defTriangleCfg.cfg.CDCoffset = _triangCfg.cdcOffset;

    for(i=0; i<numOfPatches;i++)
    {
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, i, eoprot_tag_sk_skin_cmd_boardscfg);

        //get min and max address
        uint8_t minAddr = 16;
        uint8_t maxAddr = 0;

        for(k=0; k<cardIdPerPatch[i].size(); k++)
        {
            if(cardIdPerPatch[i][k] <  minAddr)
                minAddr = cardIdPerPatch[i][k];

            if(cardIdPerPatch[i][k] >  maxAddr)
                maxAddr = cardIdPerPatch[i][k];
        }

        defBoardCfg.addrstart = minAddr;
        defBoardCfg.addrend = maxAddr;

        if(!res->addSetMessage(protoid, (uint8_t*)&defBoardCfg))
        {
            yError() <<"skin board "<< _fId.boardNum << " Error in send default board config for mtb with addr from "<<  defBoardCfg.addrstart << "to " << defBoardCfg.addrend;
            return false;
        }

    }
    int totConfigSize = 0;
    int configSize = sizeof(eOsk_cmd_trianglesCfg_t);

    for(i=0; i<numOfPatches;i++)
    {
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, i, eoprot_tag_sk_skin_cmd_trianglescfg);

        for(k=0; k</*cardIdPerPatch[i].size()*/15; k++)
        {
            if( ! (EOK_HOSTTRANSCEIVER_capacityofropframeoccasionals >= (totConfigSize += configSize)) )
            {
                 yDebug() << "skin board " << _fId.boardNum << " too many stuff to be sent at once... splitting in more messages";
                Time::delay(0.01);
                totConfigSize = 0;
            }
            defTriangleCfg.boardaddr = cardIdPerPatch[i][k];
            if(! res->addSetMessage(protoid, (uint8_t*)&defTriangleCfg))
            {
                yError() << "skin board "<< _fId.boardNum<< " Error in send default triangle config for mtb "<<  defTriangleCfg.boardaddr;
                return false;
            }
            totConfigSize += configSize;
        }
    }

    return true;
}

bool EmbObjSkin::fillData(void *raw_skin_data, eOnvID32_t id32)
{
    uint8_t           msgtype = 0;
    uint8_t           i, triangle = 0;
    EOarray_of_10canframes 	*sk_array = (EOarray_of_10canframes*) raw_skin_data;
    static int error = 0;

    eOprotIndex_t patch = eoprot_ID2index(id32);
    if(patch >= cardIdPerPatch.size())
    {
        yError() << "skin of board num " << _fId.boardNum << ": received data of patch num " << patch << "but i have only " << cardIdPerPatch.size() << " patches";
    }


    for(i=0; i<sk_array->head.size; i++)
    {
        eOutil_canframe_t *canframe;
        //uint8_t  j; 
        uint8_t mtbId =255; //unknown mtb card addr
        uint8_t  cardAddr, valid = 0;

        canframe = (eOutil_canframe_t*) &sk_array->data[i*sizeof(eOutil_canframe_t)];
        valid = (((canframe->id & 0x0F00) >> 8) == 3) ? 1 : 0;

        if(valid)
        {
            cardAddr = (canframe->id & 0x00f0) >> 4;
            //get index of start of data of board with addr cardId.
            for(int cId_index = 0; cId_index< cardIdPerPatch[patch].size(); cId_index++)
            {
                if(cardIdPerPatch[patch][cId_index] == cardAddr)
                {
                    mtbId = cId_index;
                    break;
                }
            }

            if(mtbId == 255)
            {
                //yError() << "Unknown cardId from skin\n";
                return false;
            }

            //printf("mtbId=%d\n", mtbId);
            triangle = (canframe->id & 0x000f);
            msgtype= ((canframe->data[0])& 0x80);

            int index=16*12*mtbId + triangle*12;

            //yError() << "skin fill data: mtbid" << mtbId<< " triangle " << triangle << "  msgtype" << msgtype;
            if (msgtype)
            {
                for(int k=0; k<5; k++)
                {
                    data[index+k+7]=canframe->data[k+1];
                    // yError() << "fill data " << data[index+k+7];
                }
            }
            else
            {
                for(int k=0; k<7; k++)
                {
                    this->data[index+k]=canframe->data[k+1];
                    // yError() << "fill data " << data[index+k];
                }
            }
        }
        else if(canframe->id == 0x100)
        {
            /* Can frame with id =0x100 contains Debug info. SO I skip it.*/
            return true;
        }
        else
        {
            if(error == 0)
                yError() << "EMS: " << res->boardNum << " Unknown Message received from skin (" << i<<"/"<< sk_array->head.size<<"): frameID=" << canframe->id << " len="<<canframe->size << "data="<<canframe->data[0] << " " <<canframe->data[1] << " " <<canframe->data[2] << " " <<canframe->data[3] <<"\n" ;
            error++;
            if (error == 10000)
                error = 0;
        }
    }
    return true;
}

// eof


