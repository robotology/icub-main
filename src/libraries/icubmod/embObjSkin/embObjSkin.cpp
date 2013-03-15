// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <embObjSkin.h>
#include "EOnv_hid.h"
#ifdef _SETPOINT_TEST_
#include "EOYtheSystem.h"
#endif
#include <yarp/os/Time.h>
#include <iostream>
#include <string.h>

// const int CAN_DRIVER_BUFFER_SIZE=2047;

#define SKIN_DEBUG 0

using namespace std;

bool EmbObjSkin::open(yarp::os::Searchable& config)
{
    std::string str=config.toString().c_str();
    yTrace() << str;

    // Check input parameters
    int i;
    bool correct=true;
    //correct &= config.check("Period");

    memset(info, 0x00, (size_t)SIZE_INFO);
    Bottle xtmp, xtmp2;
    ACE_TCHAR address[64];
    xtmp = Bottle(config.findGroup("ETH"));
    xtmp2 = xtmp.findGroup("IpAddress");
    //correct &= xtmp.check("IpAddress");

    strcpy(address, xtmp2.get(1).asString().c_str());
    sprintf(info, "EmbObjSkin - referred to EMS: %s", address);

    if (!correct)
    {
        std::cerr<<"Error: insufficient parameters to EmbObjSkin\n";
        return false;
    }

    //
    //	CONFIGURATION
    //

    // open ethResource, if needed

//	Property prop;
//	ACE_TCHAR tmp[126];
//	string str=config.toString().c_str();
//	xtmp = Bottle(config.findGroup("FEATURES"));
//	prop.fromString(str.c_str());
//	prop.unput("device");
//	prop.unput("subdevice");
//	// look for Ethernet device driver to use and put it into the "device" field.
//	Value &device=xtmp.find("device");
//	prop.put("device", device.asString().c_str());

    ethResCreator *resList = ethResCreator::instance();
    res = resList->getResource(config);

    if(NULL == res)
    {
        yError() << "EMS device not instantiated... unable to continue";
        return false;
    }

// 	int period=config.find("Period").asInt();
// 	setRate(period);

    Bottle ids=config.findGroup("SkinCanIds").tail();

    if (ids.size()>1)
    {
        cerr<<"Warning: EmbObjSkin id list contains more than one entry -> devices will be merged. "<<endl;
    }
    for (int i=0; i<ids.size(); i++)
    {
        int id = ids.get(i).asInt();
        cardId.push_back (id);
#if SKIN_DEBUG
        fprintf(stderr, "Id reading from %d\n", id);
#endif
    }


    //elements are:
    // sensorsNum=16*12*cardId.size();  // orig
    sensorsNum=16*12*7;		// max num of card
    data.resize(sensorsNum);

    int ttt = data.size();
    for (int i=0; i < ttt; i++)
        data[i]=(double)255;

    // fill FEAT_ID data
    memset(&_fId, 0x00, sizeof(FEAT_ID) );
    _fId.ep = 255;
    _fId.type = Skin;
    std::string FeatId = config.find("FeatId").asString().c_str();
    strcpy(_fId.name, FeatId.c_str());

    _fId.boardNum  = 255;
    Value val =config.findGroup("ETH").check("Ems",Value(1), "Board number");
    if(val.isInt())
        _fId.boardNum =val.asInt();
    else
        printf("No board number found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    switch(_fId.boardNum)
    {
    case 2:
        _fId.ep = endpoint_sk_emsboard_leftlowerarm;
        yDebug() << "Opening eoSkin class for left lower arm (eb2)\n";
        break;
    case 4:
        _fId.ep = endpoint_sk_emsboard_rightlowerarm;
        yDebug() << "Opening eoSkin class for right lower arm (eb4)\n";
        break;
    default:
        yError() << "Requested non-existing Skin endpoint for board" << _fId.boardNum;
        return false;
    }

    //N.B.: use a dynamic_cast to extract correct interface when using this pointer
    _fId.handle = (this);
    // Save eo data of this board/EP
    res->transceiver->getHostData(&_fId.EPvector, &_fId.EPhash_function);

    resList->addLUTelement(_fId);

    //RateThread::start();

    init();
    return true;
}

bool EmbObjSkin::close()
{
// 	RateThread::stop();
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
#warning "create a ROP to start/initialize the MTB, if needed"
//	int 							j=0;
//	eOmc_joint_config_t				a;
//	uint16_t						sizze;
//
//	eOnvID_t nvid = eo_cfg_nvsEP_sk_NVID_Get(endpoint_sk_emsboard_leftlowerarm, 0x00, skinNVindex_sconfig__sigmode);
//	res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_sk_emsboard_leftlowerarm, nvid);

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

bool EmbObjSkin::init()
{
    char str[128];
    int j = 0;

    EOnv 						*cnv;
    eOmn_ropsigcfg_command_t 	*ropsigcfgassign;
    EOarray						*array;
    eOropSIGcfg_t 				sigcfg;
    eOcfg_nvsEP_mn_commNumber_t dummy = 0;
    eOnvID_t 					nvid;
    EOnv 						*nvRoot;

    EOnv nvtmp;

#ifdef _SETPOINT_TEST_
    eoy_sys_Initialise(NULL, NULL, NULL);
#endif

    eOcfg_nvsEP_sk_endpoint_t ep = (eOcfg_nvsEP_sk_endpoint_t) _fId.ep;
    nvid = eo_cfg_nvsEP_sk_NVID_Get((eOcfg_nvsEP_sk_endpoint_t)ep, dummy, skinNVindex_sconfig__sigmode);
    nvRoot = res->transceiver->getNVhandler(ep, nvid, &nvtmp);
    if(NULL == nvRoot)
    {
        printf("\n>>> ERROR \ntransceiver->getNVhandler returned NULL!!\n");
        return false;
    }
    uint8_t dat = 1;
    if( eores_OK != eo_nv_Set(nvRoot, &dat, eobool_true, eo_nv_upd_dontdo))
    {
        printf("\n>>> ERROR \neo_nv_Set !!\n");
        return false;
    }
    // tell agent to prepare a rop to send
    res->transceiver->load_occasional_rop(eo_ropcode_set, ep, nvid);

    //
    //	config regulars
    //
    EOnv nvtmp_ropsigcfgassign;
    eOnvID_t nvid_ropsigcfgassign = eo_cfg_nvsEP_mn_comm_NVID_Get(endpoint_mn_comm, dummy, commNVindex__ropsigcfgcommand);
    cnv = res->transceiver->getNVhandler(endpoint_mn_comm, nvid_ropsigcfgassign, &nvtmp_ropsigcfgassign);
    ropsigcfgassign = (eOmn_ropsigcfg_command_t*) cnv->loc;
    array = (EOarray*) &ropsigcfgassign->array;
    eo_array_Reset(array);
    array->head.capacity = NUMOFROPSIGCFG;
    array->head.itemsize = sizeof(eOropSIGcfg_t);
    ropsigcfgassign->cmmnd = ropsigcfg_cmd_append;

    sigcfg.ep = _fId.ep;
    nvid = eo_cfg_nvsEP_sk_NVID_Get((eOcfg_nvsEP_sk_endpoint_t)sigcfg.ep, 0, skinNVindex_sstatus__arrayof10canframe);
    sigcfg.id = nvid;
    sigcfg.plustime = 0;
    eo_array_PushBack(array, &sigcfg);
    res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_comm, nvid_ropsigcfgassign);


    res->goToRun();

    /*EOnv nvtmp_go2state;
    eOnvID_t nvid_go2state 		= eo_cfg_nvsEP_mn_appl_NVID_Get(endpoint_mn_appl, dummy, applNVindex_cmmnds__go2state);
    EOnv 	*nv_p 				= res->transceiver->getNVhandler(endpoint_mn_appl, nvid_go2state, &nvtmp_go2state);
    eOmn_appl_state_t  desired 	= applstate_running;

    if( eores_OK != eo_nv_Set(nv_p, &desired, eobool_true, eo_nv_upd_dontdo))
    printf("error!!");
    // tell agent to prepare a rop to send
    res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_appl, nvid_go2state);    */

    return true;
}

/*
void EmbObjSkin::run()
{
	mutex.wait();

	mutex.post();
}
*/

bool EmbObjSkin::fillData(void *raw_skin_data)
{
    uint8_t 				msgtype = 0;
    uint8_t 				i, triangle = 0;
    EOarray_of_10canframes 	*sk_array = (EOarray_of_10canframes*) raw_skin_data;
    //	yarp::sig::Vector 		&pv = outPort.prepare();
    static int error = 0;

    for(i=0; i<sk_array->head.size; i++)
    {
        eOutil_canframe_t *canframe;
        uint8_t  j, mtbId =0;
        uint8_t  cardId, valid = 0;

        canframe = (eOutil_canframe_t*) &sk_array->data[i*sizeof(eOutil_canframe_t)];
        valid = (((canframe->id & 0x0F00) >> 8) == 3) ? 1 : 0;

        if(valid)
        {
            cardId = (canframe->id & 0x00f0) >> 4;
            switch (cardId)
            {
            case 14:
                mtbId = 0;
                break;
            case 13:
                mtbId = 1;
                break;
            case 12:
                mtbId = 2;
                break;
            case 11:
                mtbId = 3;
                break;
            case 10:
                mtbId = 4;
                break;
            case 9:
                mtbId = 5;
                break;
            case 8:
                mtbId = 6;
                break;
            default:
                yError() << "Unknown cardId from skin\n";
                return false;
                break;
            }
            triangle = (canframe->id & 0x000f);
            msgtype= ((canframe->data[0])& 0x80);

            int index=16*12*mtbId + triangle*12;

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
        		yError() << "Unknown Message received from skin: frameID=" << canframe->id << "\n" ;
        	error++;
        	if (error == 10000)
        		error = 0;
        }
    }
    return true;  // bool?
}

