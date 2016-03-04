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


using namespace std;
using namespace iCub::skin::diagnostics;


int SkinPatchInfo::checkCardAddrIsInList(int cardAddr)
{
    for(int i=0; i< cardAddrList.size(); i++)
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
    memset(info, 0x00, sizeof(info));

    ConstString tmp = NetworkBase::getEnvironment("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
        verbosewhenok = (bool)NetType::toInt(tmp);
    }
    else
    {
        verbosewhenok = false;
    }
}


EmbObjSkin::~EmbObjSkin() { }


bool EmbObjSkin::initWithSpecialConfig(yarp::os::Searchable& config)
{
    Bottle          bNumOfset;
    int             numOfSets;
    eOprotID32_t    protoid;
    int             p, j;
    SpecialSkinBoardCfgParam* boardCfgList = new SpecialSkinBoardCfgParam[_skCfg.totalCardsNum]; //please add cleanup!
    int             numofcfg;

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
            yError() << "skin of board num " << _fId.boardNumber << ": patch " << boardCfgList[j].patch << "not exists";
            return false;
        }
        //now p is the index of patch.

        //check if card address are in patch
        int boardIdx = -1;
        for(int a=boardCfgList[j].boardAddrStart; a<=boardCfgList[j].boardAddrEnd; a++)
        {
            boardIdx = _skCfg.patchInfoList[p].checkCardAddrIsInList(a);
            if(-1 == boardIdx)
            {
                yError() << "skin of board num " << _fId.boardNumber << " card with address " << a << "is not present in patch " << _skCfg.patchInfoList[p].idPatch;
                return(false);
            }
        }

        //prepare data to send to ems
        eOsk_cmd_boardsCfg_t bcfg;
        bcfg.addrstart = boardCfgList[j].boardAddrStart;
        bcfg.addrend = boardCfgList[j].boardAddrEnd;
        bcfg.cfg.skintype = boardCfgList[j].cfg.skinType;
        bcfg.cfg.period = boardCfgList[j].cfg.period;
        bcfg.cfg.noload = boardCfgList[j].cfg.noLoad;

        // Init the data vector with special config values from "noLoad" param in config file.
        // This is to have a correct initilization for the data sent through yarp port
        for (int sensorId = 0; sensorId < 16; sensorId++)
        {
            int index = 16*12*boardIdx + sensorId*12;

            // Message head
            for(int k = 0; k < 12; k++)
            {
//                yDebug() << "readNewSpecialConfiguration size is: " << data.size() << " index is " << (index+k) << " value is: " << boardCfgList[j].cfg.noLoad;
                if((index+k) >= data.size())
                    yError() << "readNewSpecialConfiguration: index too big";
                data[index + k] = boardCfgList[j].cfg.noLoad;
            }
        }
//        //uncomment for debug only
//        yDebug() << "\n Special board cfg num " << j;
//        boardCfgList[j].debugPrint();

        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[p].indexNv, eoprot_tag_sk_skin_cmmnds_boardscfg);


        if(! res->addSetMessage(protoid, (uint8_t*)&bcfg))
        {
            yError() << "skin board "<< _fId.boardNumber << " Error in send special board config for mtb with addr from"<<  bcfg.addrstart << " to addr " << bcfg.addrend;
            return false;
        }

    } //end for for each special board cfg

    Time::delay(0.010); // 10 ms (m.a.a-delay: before it was 0.01)

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
            yError() << "skin of board num " << _fId.boardNumber << ": patch " << triangleCfg[j].patch << "not exists";
            return false;
        }
        //now p is index patch

        //check if bcfg.boardAddr is in my patches list
        if(-1 == _skCfg.patchInfoList[p].checkCardAddrIsInList(triangleCfg[j].boardAddr))
        {
            yError() << "skin of board num " << _fId.boardNumber <<  " card with address " << triangleCfg[j].boardAddr << "is not present in patch " << _skCfg.patchInfoList[p].idPatch;
            return(false);
        }

        //prepare data to send to ems
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[p].indexNv, eoprot_tag_sk_skin_cmmnds_trianglescfg);


        eOsk_cmd_trianglesCfg_t tcfg = {0};
        tcfg.boardaddr = triangleCfg[j].boardAddr;
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
        Time::delay(0.010);

        if(! res->addSetMessage(protoid, (uint8_t*)&tcfg))
        {
            yError() << "skin board "<< _fId.boardNumber << " Error in send default triangle config for mtb "<<  tcfg.boardaddr;
            return false;
        }
    }

    Time::delay(0.010); // 10 ms (m.a.a-delay: before it was 0.01)

    return true;
}


bool EmbObjSkin::fromConfig(yarp::os::Searchable& config)
{
    Bottle bPatches, bPatchList, xtmp;
    //reset total num of cards
    _skCfg.totalCardsNum = 0;


    bPatches = config.findGroup("patches", "skin patches connected to this device");
    if(bPatches.isNull())
    {
        yError() << "skin board num " << _fId.boardNumber << "patches group is missed!";
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
            yError() << "skin board num " << _fId.boardNumber << "ems expected only patch num 1 or 2";
            return false;
        }
        _skCfg.patchInfoList[j-1].idPatch = id;
        _skCfg.patchInfoList[j-1].indexNv = convertIdPatch2IndexNv(id);
    }

//    //uncomment for debug only
//    yError() << "numOfPatches=" << _skCfg.numOfPatches;
//    for(int j=0; j<_skCfg.numOfPatches; j++ )
//    {
//        yError() << " patchInfoList[" << j << "]: patch=" << _skCfg.patchInfoList[j].idPatch << "indexNv=" << _skCfg.patchInfoList[j].indexNv;
//    }


    for(int i=0; i<_skCfg.numOfPatches; i++)
    {
        char tmp[80];
        int id = _skCfg.patchInfoList[i].idPatch;
        snprintf(tmp, sizeof(tmp), "skinCanAddrsPatch%d", id);

        xtmp = bPatches.findGroup(tmp);
        if(xtmp.isNull())
        {
            yError() << "skin of board num " << _fId.boardNumber << "doesn't find " << tmp << "in xml file";
            return false;
        }

        _skCfg.patchInfoList[i].cardAddrList.resize(xtmp.size()-1);

        for(int j=1; j<xtmp.size(); j++)
        {
            int addr = xtmp.get(j).asInt();
            _skCfg.totalCardsNum++;
            _skCfg. patchInfoList[i].cardAddrList[j-1] = addr;
        }
    }

    // resize data vector with number of triangles found in config file
    sensorsNum = 16*12*_skCfg.totalCardsNum;     // max num of card

    mutex.wait();

    this->data.resize(sensorsNum);
    int ttt = this->data.size();
    for (int i=0; i < ttt; i++)
    {
        this->data[i]=(double)240;
    }

    mutex.post();

//    //uncomment for debug only
//    yError() << "totalCardsNum=" << _skCfg.totalCardsNum;
//    for(int i=0; i<_skCfg.patchInfoList.size(); i++)
//    {
//        for(int j=0; j<_skCfg.patchInfoList[i].cardAddrList.size(); j++)
//        {
//            yError() << " elem num " << j << "of patch " <<_skCfg.patchInfoList[i].idPatch << "is " << _skCfg.patchInfoList[i].cardAddrList[j];
//        }
//    }

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
            for(int k = 0; k < 12; k++)
            {
//                yDebug() << "EO readNewConfiguration (default) size is: " << data.size()
//                         << " index is " << (index+k) << " value is: " << _brdCfg.noLoad;
                if((index+k) >= data.size())
                    yError() << "readNewConfiguration: index too big";
                data[index + k] = _brdCfg.noLoad;
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
    std::string str;
    if(config.findGroup("GENERAL").find("verbose").asBool())
        str=config.toString().c_str();
    else
        str="\n";
    yTrace() << str;


    Bottle   groupEth, parameter;
    int      port;

    Bottle groupTransceiver = Bottle(config.findGroup("TRANSCEIVER"));
    if(groupTransceiver.isNull())
    {
        yError() << "EmbObjSkin::open() can't find TRANSCEIVER group in xml config files";
        return false;
    }

    Bottle groupProtocol = Bottle(config.findGroup("PROTOCOL"));
    if(groupProtocol.isNull())
    {
        yError() << "EmbObjSkin::open() can't find PROTOCOL group in config files";
        return false;
    }

    // Get both PC104 and EMS ip addresses and port from config file
    groupEth  = Bottle(config.findGroup("ETH"));
    Bottle parameter1( groupEth.find("PC104IpAddress").asString() );
    port      = groupEth.find("CmdPort").asInt();              // .get(1).asInt();
    snprintf(_fId.pc104IPaddr.string, sizeof(_fId.pc104IPaddr.string), "%s", parameter1.toString().c_str());
    _fId.pc104IPaddr.port = port;

    Bottle parameter2( groupEth.find("IpAddress").asString() );    // .findGroup("IpAddress");
    snprintf(_fId.boardIPaddr.string, sizeof(_fId.boardIPaddr.string), "%s", parameter2.toString().c_str());
    _fId.boardIPaddr.port = port;

    sscanf(_fId.boardIPaddr.string,"\"%d.%d.%d.%d", &_fId.boardIPaddr.ip1, &_fId.boardIPaddr.ip2, &_fId.boardIPaddr.ip3, &_fId.boardIPaddr.ip4);
    sscanf(_fId.pc104IPaddr.string,"\"%d.%d.%d.%d", &_fId.pc104IPaddr.ip1, &_fId.pc104IPaddr.ip2, &_fId.pc104IPaddr.ip3, &_fId.pc104IPaddr.ip4);

    snprintf(_fId.boardIPaddr.string, sizeof(_fId.boardIPaddr.string), "%u.%u.%u.%u:%u", _fId.boardIPaddr.ip1, _fId.boardIPaddr.ip2, _fId.boardIPaddr.ip3, _fId.boardIPaddr.ip4, _fId.boardIPaddr.port);
    snprintf(_fId.pc104IPaddr.string, sizeof(_fId.pc104IPaddr.string), "%u.%u.%u.%u:%u", _fId.pc104IPaddr.ip1, _fId.pc104IPaddr.ip2, _fId.pc104IPaddr.ip3, _fId.pc104IPaddr.ip4, _fId.pc104IPaddr.port);

    // Check input parameters
    bool correct=true;

    snprintf(info, sizeof(info), "EmbObjSkin - referred to EMS: %s", _fId.boardIPaddr.string);

    if (!correct)
    {
        std::cerr<<"Error: insufficient parameters to EmbObjSkin\n";
        return false;
    }

    ethManager = TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "embObjskin::open() cannot instantiate ethManager";
        return false;
    }


    Value val = config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
    {
        _fId.boardNumber = val.asInt();
    }
    else
    {
        yError() << "skin: No board number found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
        return false;
    }

    _fId.boardName[0] = '\0';
    Value *valName;
    if(config.findGroup("ETH").check("Name", valName, "Board name"))
    {
        if(valName->isString())
        {
            memset(_fId.boardName, 0, BOARDNAME_MAXSIZE);
            snprintf(_fId.boardName, BOARDNAME_MAXSIZE, "%s", valName->asString().c_str());
        }
        else
        {
            yError () << "embObjMotionControl: EMS Board name is not valid";
        }
    }
    _fId.endpoint = eoprot_endpoint_skin;
    _fId.entity = eoprot_entity_sk_skin;
    _fId.type = ethFeatType_Skin;

    // N.B.: use a dynamic_cast to extract correct interface when using this pointer
    _fId.interface = this;


    /* Once I'm ok, ask for resources, through the _fId struct I'll give the ip addr, port and
    *  and boradNum to the ethManagerin order to create the ethResource requested.
    * I'll Get back the very same sturct filled with other data useful for future handling
    * like the EPvector and EPhash_function */
    res = ethManager->requestResource(config, groupTransceiver, groupProtocol, _fId);
    if(NULL == res)
    {
        yError() << "embObjSkin::open() fails because could not instantiate the ethResource board" << _fId.boardNumber << " ... unable to continue";
        return false;
    }

//    if(false == res->isEPmanaged(eoprot_endpoint_skin))
//    {
//        yError() << "embObjSkin::open() detected that EMS "<< _fId.boardNumber << " does not support skin";
//        cleanup();
//        return false;
//    }


    if(false == res->verifyBoard(groupProtocol))
    {
        yError() << "embObjSkin::init() fails in function verifyBoard() for board " << _fId.boardNumber << ": CANNOT PROCEED ANY FURTHER";
        cleanup();
        return false;
    }

    if(!res->verifyEPprotocol(groupProtocol, eoprot_endpoint_skin))
    {
        yError() << "embObjSkin::init() fails in function verifyEPprotocol() for board "<< _fId.boardNumber << ": the board does not have the same eoprot_endpoint_management and/or eoprot_endpoint_skin protocol version: DO A FW UPGRADE";
        cleanup();
        return false;
    }

    //    // marco.accame on 04 sept 2014: we could add a verification about the entities of skin ... MAYBE in the future
    //
    //    uint8_t numberofskins = eoprot_entity_numberof_get(featIdBoardNum2nvBoardNum(_fId.boardNumber), eoprot_endpoint_skin, eoprot_entity_sk_skin);
    //    if(false == res->verifyENTITYnumber(groupProtocol, eoprot_endpoint_skin, eoprot_entity_sk_skin, numberofskins))
    //    {   // JUST AN EXAMPLE OF HOW TO DO IT FOR THE SKIN.
    //        yError() << "embObjSkin::init() fails in function verifyENTITYnumber() for board "<< _fId.boardNumber << " and entity eoprot_entity_sk_skin: VERIFY their number in board, and in XML files";
    //        return false;
    //    }


    if(!this->fromConfig(config))
    {
        yError() << "embObjSkin::init() fails in function fromConfig() for board " << _fId.boardNumber << ": CANNOT PROCEED ANY FURTHER";
        cleanup();
        return false;
    }

    char name[80];
    sprintf(name, "embObjSkin on EMS %d", _fId.boardNumber);
    _cfgReader.setName(name);



    if(!init())
        return false;

    /* Following delay is necessary in order to give enough time to skin boards to configure all its triangles */
    Time::delay(0.5);

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

    if(false == res->goToRun(eoprot_endpoint_skin, eoprot_entity_sk_skin))
    {
        yError() << "embObjSkin::open() fails to start control loop of board" << _fId.boardNumber << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjSkin::open() correctly activated control loop of BOARD" << _fId.boardNumber;
        }
    }

    return true;
}


bool EmbObjSkin::isEpManagedByBoard()
{
    if(eobool_true == eoprot_endpoint_configured_is(res->get_protBRDnumber(), eoprot_endpoint_skin))
    {
        return true;
    }

    return false;
}

void EmbObjSkin::cleanup(void)
{
    if(ethManager == NULL) return;

    int ret = ethManager->releaseResource(_fId);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
}

bool EmbObjSkin::close()
{
    cleanup();
    return true;
}

#if 0
void EmbObjSkin::setId(ethFeature_t &id)
{
    _fId=id;
}
#endif


int EmbObjSkin::read(yarp::sig::Vector &out)
{
    mutex.wait();
    out = this->data;  //old - this needs the running thread
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
//        yWarning()<< "EmbObjSkin::start() detected that skin for board " << _fId.boardNumber << "uses new signal mode";
    }
    else
    {
        dat = eosk_sigmode_signal_oldway;
//        yWarning()<< "EmbObjSkin::start() detected  that skin for board " << _fId.boardNumber << "used old signal mode";
    }

    for(i=0; i<_skCfg.numOfPatches;i++)
    {
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[i].indexNv, eoprot_tag_sk_skin_config_sigmode);
        ret = res->addSetMessage(protoid, &dat);
        if(!ret)
        {
            yError() << "unable to start skin for board " << _fId.boardNumber << " on port " <<  _skCfg.patchInfoList[i].idPatch;
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


    // configure the regulars
    if(false == res->addRegulars(id32v, true))
    {
        yError() << "EmbObjSkin::configPeriodicMessage() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }
    else
    {
        if(verbosewhenok)
        {
            yDebug() << "embObjSkin::configPeriodicMessage() added" << id32v.size() << "regular rops to BOARD" << res->get_protBRDnumber()+1;
            char nvinfo[128];
            for(int r=0; r<id32v.size(); r++)
            {
                uint32_t id32 = id32v.at(r);
                eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
                yDebug() << "\t it added regular rop for" << nvinfo;
            }
        }
    }
    Time::delay(0.005);  // 5 ms (m.a.a-delay: before it was 0)

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
    int                     i,k;

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
            if(_skCfg.patchInfoList[i].cardAddrList[k] <  minAddr)
                minAddr = _skCfg.patchInfoList[i].cardAddrList[k];

            if(_skCfg.patchInfoList[i].cardAddrList[k] >  maxAddr)
                maxAddr = _skCfg.patchInfoList[i].cardAddrList[k];
        }

        defBoardCfg.addrstart = minAddr;
        defBoardCfg.addrend = maxAddr;

        if(!res->addSetMessage(protoid, (uint8_t*)&defBoardCfg))
        {
            yError() <<"skin board "<< _fId.boardNumber << " Error in send default board config for mtb with addr from "<<  defBoardCfg.addrstart << "to " << defBoardCfg.addrend;
            return false;
        }

    }
    Time::delay(0.010);

    for(i=0; i<_skCfg.numOfPatches;i++)
    {
        protoid = eoprot_ID_get(eoprot_endpoint_skin, eoprot_entity_sk_skin, _skCfg.patchInfoList[i].indexNv, eoprot_tag_sk_skin_cmmnds_trianglescfg);

        for(k=0; k<_skCfg.patchInfoList[i].cardAddrList.size(); k++)
        {
            defTriangleCfg.boardaddr = _skCfg.patchInfoList[i].cardAddrList[k];
            if(! res->addSetMessage(protoid, (uint8_t*)&defTriangleCfg))
            {
                yError() << "skin board "<< _fId.boardNumber<< " Error in send default triangle config for mtb "<<  defTriangleCfg.boardaddr;
                return false;
            }
        }
    }

    opened = true;

    return true;
}

bool EmbObjSkin::initialised()
{
    return opened;
}

iethresType_t EmbObjSkin::type()
{
    return iethres_skin;
}

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
        yError() << "skin of board num " << _fId.boardNumber << ": received data of patch with nvindex= " << indexpatch;
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
            for(int cId_index = 0; cId_index< _skCfg.patchInfoList[p].cardAddrList.size(); cId_index++)
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

            // marco.accame: added lock to avoid cincurrent access to this->data. i lock at triangle resolution ...
            mutex.wait();

            if (msgtype == 0x40)
            {
                // Message head
                for(int k = 0; k < 7; k++)
                {
                    data[index + k] = canframedata[k + 1];
                }
            }
            else if (msgtype == 0xC0)
            {
                // Message tail
                for(int k = 0; k < 5; k++)
                {
                    data[index + k + 7] = canframedata[k + 1];
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
                                        "BOARD: " << res->getName()  <<
                                        "IP:"     << res->getIPv4string() <<
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
                yError() << "EMS: " << res->getIPv4string() << " Unknown Message received from skin (" << i<<"/"<< sizeofarray <<"): frameID=" << canframeid11<< " len="<<canframesize << "canframe.data="<<canframedata[0] << " " <<canframedata[1] << " " <<canframedata[2] << " " <<canframedata[3] <<"\n" ;
            error++;
            if (error == 10000)
                error = 0;
        }
    }
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


