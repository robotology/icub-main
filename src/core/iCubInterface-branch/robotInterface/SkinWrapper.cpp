/*
 * SkinWrapper.cpp
 *
 *  Created on: May 18, 2012
 *      Author: icub
 */


#include "RobotInterfaceRemap.h"
#include "extractPath.h"

#include <sstream>

#include <yarp/os/Thread.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Os.h>

#include "ControlBoardWrapper.h"
#include "ControlBoardWrapper2.h"

#include <iCub/FactoryInterface.h>

// Logger interface
#include <yarp/os/Log.h>
#include <yarp/os/impl/Logger.h>

// should not include this stuff...
#include "EoSkin.h"
#include "eOcfg_nvsEP_sk.h"

#undef _PRINT_DEBUG_

#ifdef _PRINT_DEBUG_
#define printDebug(args...)		printf(args)
#else
#define printDebug
#endif

SkinPartEntry::SkinPartEntry()
{
	// AC_YARP_INFO(Logger::get(),"SkinPartEntry::SkinPartEntry()", Logger::get().log_files.f3);
	analogServer=0;
	analog=0;
}

SkinPartEntry::~SkinPartEntry()
{
	// AC_YARP_INFO(Logger::get(),"SkinPartEntry::~SkinPartEntry() ", Logger::get().log_files.f3);
}

void SkinPartEntry::calibrate()
{
	if (analog)
	{
		analog->calibrateSensor();
	}
}

bool SkinPartEntry::open(yarp::os::Property &deviceP, yarp::os::Property &partP)
{
	// AC_YARP_INFO(Logger::get(),"SkinPartEntry::open(...) - id " + String(id.c_str()), Logger::get().log_files.f3);
	printf("SkinPartEntry param 1 (deviceP) = %s\n", deviceP.toString().c_str());
	printf("SkinPartEntry param 2 (partP)   = %s\n", partP.toString().c_str());

	bool correct=true;
	correct=correct&&partP.check("device");
	correct=correct&&partP.check("robot");
	//correct=correct&&partP.check("FeatId");
	//correct=correct&&partP.check("canbusdevice");
	//correct=correct&&partP.check("ports");		// list of the ports where to send the tactile data
	//
	//    Bottle xtmp, xtmp2;
	//
	//    bool correct=true;
	//
	//    correct=correct&&robotOptions.check("GENERAL");
	//
	//    if(correct)
	//    	xtmp = Bottle(robotOptions.findGroup("GENERAL"));
	//
	//    correct=correct&&xtmp.check("PC104IpAddress");
	//
	//    if(correct)
	//    	xtmp2 = xtmp.findGroup("PC104IpAddress");
	//
	//    partOptions.put("PC104IpAddress", xtmp2.get(1).asString().c_str());

	if (!correct)
		return false;

	int period=20;
	if (partP.check("period"))
	{
		period=partP.find("period").asInt();
	}
	else
	{
		std::cout<<"Warning: part "<<id<<" using default period ("<<period<<")\n";
	}

	// Open the device
	std::string devicename=partP.find("device").asString().c_str();
	deviceP.put("device", devicename.c_str());

	std::string canbusdevice=partP.find("canbusdevice").asString().c_str();
	deviceP.put("canbusdevice", canbusdevice.c_str());

	driver.open(deviceP);
	if (!driver.isValid())
		return false;

	driver.view(analog);
	driver.view(hook);
	RateThread *thread;
	driver.view(thread);

	if (!analog)
	{
		std::cerr<<"Error: part "<<id<<" device " << devicename << " does not implement analog interface"<<endl;
		driver.close();
		return false;
	}

	// Read the list of ports
	std::string robotName=partP.find("robot").asString().c_str();
	std::string root_name;
	root_name+="/";
	root_name+=robotName;
	root_name+="/skin/";

	std::vector<AnalogPortEntry> skinPorts;
	if(!partP.check("ports")){
		// if there is no "ports" section take the name of the "skin" group as the only port name
		skinPorts.resize( (size_t)1);
		skinPorts[0].offset = 0;
		skinPorts[0].length = -1;
		skinPorts[0].port_name = root_name + this->id;
	}
	else{
		Bottle *ports=partP.find("ports").asList();

		if (!partP.check("total_taxels", "number of taxels of the part"))
			return false;
		int total_taxels=partP.find("total_taxels").asInt();
		int nports=ports->size();
		int totalT = 0;
		skinPorts.resize(nports);

		for(int k=0;k<ports->size();k++)
		{
			Bottle parameters=partP.findGroup(ports->get(k).asString().c_str());

			if (parameters.size()!=5)
			{
				cerr<<"Error: check skin port parameters in part description"<<endl;
				cerr<<"--> I was expecting "<<ports->get(k).asString().c_str() << " followed by four integers"<<endl;
				return false;
			}

			int wBase=parameters.get(1).asInt();
			int wTop=parameters.get(2).asInt();
			int base=parameters.get(3).asInt();
			int top=parameters.get(4).asInt();

			cout<<"--> "<<wBase<<" "<<wTop<<" "<<base<<" "<<top<<endl;

			//check consistenty
			if(wTop-wBase != top-base){
				cerr<<"Error: check skin port parameters in part description"<<endl;
				cerr<<"Numbers of mapped taxels do not match.\n";
				return false;
			}
			int taxels=top-base+1;

			skinPorts[k].length = taxels;
			skinPorts[k].offset = wBase;
			skinPorts[k].port_name = root_name+string(ports->get(k).asString().c_str());

			totalT+=taxels;
		}

		if (totalT!=total_taxels)
		{
			cerr<<"Error total number of mapped taxels does not correspond to total taxels"<<endl;
			return false;
		}
	}

	// 7 = cardId.size()
	wholeData.resize((size_t)16*12*7);
	for (int i=0; i<wholeData.size(); i++)
		wholeData[i]=255;

	fId.type = Skin;
	std::string FeatId = deviceP.find("FeatId").asString().c_str();
	cout << "FeatId = " << FeatId << endl;
	strcpy(fId.name, FeatId.c_str());

	if( 0 == strcmp("left_arm", fId.name) )
		fId.ep = endpoint_sk_emsboard_leftlowerarm;

	if( 0 == strcmp("right_arm", fId.name) )
			fId.ep = endpoint_sk_emsboard_rightlowerarm;

	hook->setId(fId);
	thread->start();

	analogServer = new AnalogServer(skinPorts);
	analogServer->setRate(period);
	analogServer->attach(analog);
	analogServer->start();

	return true;
}

void SkinPartEntry::close()
{
	// AC_YARP_INFO(Logger::get(),"SkinPartEntry::close() - id " + String(id.c_str()), Logger::get().log_files.f3);
	std::cout<<"Closing skin part "<< id << endl;
	if (analogServer)
	{
		analogServer->stop();
		delete analogServer;
	}
	if (analog)
		analog=0;

	driver.close();
}

Vector * SkinPartEntry::getData()
{
	return &wholeData;
}

bool SkinPartEntry::pushData(yarp::sig::Vector &in)
{
	hook->pushData(in);
	return false;
}


bool SkinPartEntry::fillData(char *data)
{
	uint8_t 				msgtype = 0;
	uint8_t 				i, triangle = 0;
	EOarray_of_10canframes 	*sk_array = (EOarray_of_10canframes*) data;
	//	yarp::sig::Vector 		&pv = outPort.prepare();


	printDebug("\n--- ARRAY SIZE = %d  ---- \n", sk_array->head.size);

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
			printDebug("\n data id 0x%04X, 0x", canframe->id);

			int index=16*12*mtbId + triangle*12;

			printDebug("%0X ", canframe->data[0]);
			if (msgtype)
			{
				for(int k=0;k<5;k++)
				{
					wholeData[index+k+7]=canframe->data[k+1];
					printDebug("%0X ", canframe->data[k+1]);
				}
			}
			else
			{
				for(int k=0;k<7;k++)
				{
					wholeData[index+k]=canframe->data[k+1];
					printDebug("%0X ", canframe->data[k+1]);
				}
			}
		}
		else
		{
			printDebug("Unknown Message\n");
		}
	}
}

SkinPartEntry *SkinParts::find(const string &pName)
{
	SkinPartsIt it=begin();
	for(;it!=end(); it++)
	{
		if ((*it)->id==pName)
		{
			return (*it);
		}
	}

	return 0;
}

//Vector * SkinParts::findus2(FEAT_ID *id )
//{
//	SkinPartsIt it=begin();
//	for(;it!=end(); it++)
//	{
//		if( ((*it)->fId.type == id->type) && (strcmp((*it)->fId.name, id->name) == 0) )
//		{
//			return (*it)->getData(); //&(*it);
//		}
//	}
//	return NULL;
//}

IiCubFeature * SkinParts::findus(FEAT_ID *id )
{
	IiCubFeature *ret;
//	id->type = Skin;
//	memset(id->name, 0x00, sizeof(id->name) );
	SkinPartsIt it=begin();
	uint8_t id1;
	FeatureType f1;

	for(;it!=end(); it++)
	{
		id1=((*it)->fId.ep);
		f1 = ((*it)->fId.type);
		//if( ((*it)->fId.type == id->type) && (strcmp((*it)->fId.name, id->name) == 0) && ((*it)->fId.ep) == id->ep)
		if( ((*it)->fId.type == id->type) && ((*it)->fId.ep) == id->ep)
		{
			ret = (*it);
			return ret;
		}
	}
	return NULL;
}

void SkinParts::close()
{
	// AC_YARP_INFO(Logger::get(),"SkinParts::close()", Logger::get().log_files.f3);
	SkinPartsIt it;
	for(it=this->begin(); it!=this->end(); it++)
	{
		(*it)->close();
	}
}
