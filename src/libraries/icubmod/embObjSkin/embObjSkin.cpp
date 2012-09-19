// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <embObjSkin.h>
#include "EOnv_hid.h"

#include <yarp/os/Time.h>
#include <iostream>
#include <string.h>

const int CAN_DRIVER_BUFFER_SIZE=2047;

#define SKIN_DEBUG 0

using namespace std;

#include "debugging.h"

bool EmbObjSkin::open(yarp::os::Searchable& config)
{
	// Debug info
	// AC_YARP_INFO(Logger::get(), "EmbObjSkin::open", Logger::get().log_files.f3);

	printf("EmbObjSkin param %s\n", config.toString().c_str());

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

	Property prop;
	ACE_TCHAR tmp[126];
	string str=config.toString().c_str();
	xtmp = Bottle(config.findGroup("FEATURES"));
	prop.fromString(str.c_str());
	prop.unput("device");
	prop.unput("subdevice");
	// look for Ethernet device driver to use and put it into the "device" field.
	Value &device=xtmp.find("device");
	prop.put("device", device.asString().c_str());

	ethResCreator *resList = ethResCreator::instance();
	res = resList->getResource(prop);


	int period=config.find("Period").asInt();
	setRate(period);

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
	cout << "FeatId = " << FeatId << endl;
	strcpy(_fId.name, FeatId.c_str());

#warning "Fix this"
	if( 0 == strcmp("left_arm", _fId.name) )
		_fId.ep = endpoint_sk_emsboard_leftlowerarm;		// fare in maniera più sicura

	if( 0 == strcmp("right_arm", _fId.name) )
		_fId.ep = endpoint_sk_emsboard_rightlowerarm;


	if(255 == _fId.ep)
	{
		printf("\n ERROR: MotionControl endpoint not found!!!\n");
		return false;
	}

	_fId.handle = dynamic_cast<IiCubFeature*> (this);		//weird behaviour
	// Save eo data of this board/EP
	res->transceiver->getHostData(&_fId.EPvector, &_fId.EPhash_function);

	resList->addLUTelement(_fId);

	//RateThread::start();

	init();
	return true;
}

bool EmbObjSkin::close()
{
	RateThread::stop();
//	if (pCanBufferFactory)
//	{
//		destroyBuffer(inBuffer);
//		destroyBuffer(outBuffer);
//	}
//	driver.close();
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
	// AC_YARP_INFO(Logger::get(),"EmbObjSkin::calibrateSensor", Logger::get().log_files.f3);

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

	eOcfg_nvsEP_sk_endpoint_t ep = (eOcfg_nvsEP_sk_endpoint_t) _fId.ep;
	nvid = eo_cfg_nvsEP_sk_NVID_Get((eOcfg_nvsEP_sk_endpoint_t)ep, dummy, skinNVindex_sconfig__sigmode);
	nvRoot = res->transceiver->getNVhandler(ep, nvid);
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
	eOnvID_t nvid_ropsigcfgassign = eo_cfg_nvsEP_mn_comm_NVID_Get(endpoint_mn_comm, dummy, commNVindex__ropsigcfgcommand);
	cnv = res->transceiver->getNVhandler(endpoint_mn_comm, nvid_ropsigcfgassign);
	ropsigcfgassign = (eOmn_ropsigcfg_command_t*) cnv->loc;
	array = (EOarray*) &ropsigcfgassign->array;
	eo_array_Reset(array);
	array->head.capacity = NUMOFROPSIGCFG;
	array->head.itemsize = sizeof(eOropSIGcfg_t);
	ropsigcfgassign->cmmnd = ropsigcfg_cmd_assign;

	sigcfg.ep = _fId.ep;
	nvid = eo_cfg_nvsEP_sk_NVID_Get((eOcfg_nvsEP_sk_endpoint_t)sigcfg.ep, 0, skinNVindex_sstatus__arrayof10canframe);
	sigcfg.id = nvid;
	sigcfg.plustime = 0;
	eo_array_PushBack(array, &sigcfg);
	res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_comm, nvid_ropsigcfgassign);


	eOnvID_t nvid_go2state 		= eo_cfg_nvsEP_mn_appl_NVID_Get(endpoint_mn_appl, dummy, applNVindex_cmmnds__go2state);
	EOnv 	*nv_p 				= res->transceiver->getNVhandler(endpoint_mn_appl, nvid_go2state);
	eOmn_appl_state_t  desired 	= applstate_running;

	if( eores_OK != eo_nv_Set(nv_p, &desired, eobool_true, eo_nv_upd_dontdo))
		printf("error!!");
	// tell agent to prepare a rop to send
	res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_appl, nvid_go2state);

	return true;
}

void EmbObjSkin::run()
{	
	mutex.wait();

	mutex.post();
}

bool EmbObjSkin::fillData(char *raw_skin_data)
{
	uint8_t 				msgtype = 0;
	uint8_t 				i, triangle = 0;
	EOarray_of_10canframes 	*sk_array = (EOarray_of_10canframes*) raw_skin_data;
	//	yarp::sig::Vector 		&pv = outPort.prepare();

	print_debug(AC_debug_file, "\n--- ARRAY SIZE = %d  ---- \n", sk_array->head.size);

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
					printf("Unexpected value %d\n", cardId);
					return false;
					break;
			}
			triangle = (canframe->id & 0x000f);
			msgtype= ((canframe->data[0])& 0x80);
			print_debug(AC_debug_file, "\n data id 0x%04X, 0x", canframe->id);

			int index=16*12*mtbId + triangle*12;

			print_debug(AC_debug_file,"%0X ", canframe->data[0]);
			if (msgtype)
			{
				for(int k=0;k<5;k++)
				{
					data[index+k+7]=canframe->data[k+1];
					print_debug(AC_debug_file, "%0X ", canframe->data[k+1]);
				}
			}
			else
			{
				for(int k=0;k<7;k++)
				{
					this->data[index+k]=canframe->data[k+1];
					print_debug(AC_debug_file, "%0X ", canframe->data[k+1]);
				}
			}
		}
		else
		{
			print_debug(AC_debug_file, "Unknown Message\n");
		}
	}
	return true;  // bool?
}

void EmbObjSkin::threadRelease()
{
#if SKIN_DEBUG
	printf("SkinPrototype Thread releasing...\n");	
	printf("... done.\n");
#endif
}

