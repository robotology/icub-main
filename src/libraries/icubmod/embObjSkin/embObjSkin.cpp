// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2010 RobotCub Consortium
// Authors: Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <embObjSkin.h>

#include <yarp/os/Time.h>
#include <iostream>
#include <string.h>

const int CAN_DRIVER_BUFFER_SIZE=2047;

#define SKIN_DEBUG 0

using namespace std;


#ifdef _AC_
	#include "/usr/local/src/robot/iCub/pc104/device-drivers/cfw002/src/LinuxDriver/API/libcfw002.h"
#else
	#include "libcfw002.h"
#endif


Cfw2CanMessage::Cfw2CanMessage()
{
	msg=0;
}

Cfw2CanMessage::~Cfw2CanMessage()
{
}

CanMessage &Cfw2CanMessage::operator=(const CanMessage &l)
{
    const Cfw2CanMessage &tmp=dynamic_cast<const Cfw2CanMessage &>(l);
    memcpy(msg, tmp.msg, sizeof(CFWCAN_MSG));
    return *this;
}


unsigned int Cfw2CanMessage::getId() const
{
	return msg->id;
}

unsigned char Cfw2CanMessage::getLen() const
{
	return msg->len;
}

void Cfw2CanMessage::setLen(unsigned char len)
{
	msg->len = len;
}

void Cfw2CanMessage::setId(unsigned int id)
{
	msg->id = id;
}

const unsigned char *Cfw2CanMessage::getData() const
{
	return msg->data;
}

unsigned char *Cfw2CanMessage::getData()
{
	return msg->data;
}

unsigned char *Cfw2CanMessage::getPointer()
{
	return (unsigned char *) msg;
}

const unsigned char *Cfw2CanMessage::getPointer() const
{
	return (const unsigned char *) msg;
}

void Cfw2CanMessage::setBuffer(unsigned char *b)
{
	if (b != 0)
		msg = (CFWCAN_MSG *) (b);
}

CanBuffer EmbObjSkin::createBuffer(int elem)
   {
       CanBuffer ret;
       CFWCAN_MSG *storage=new CFWCAN_MSG[elem];
       CanMessage **messages=new CanMessage *[elem];
       Cfw2CanMessage *tmp=new Cfw2CanMessage[elem];

       memset(storage, 0, sizeof(CFWCAN_MSG)*elem);

       for(int k=0;k<elem;k++)
           {
               messages[k]=&tmp[k];
               messages[k]->setBuffer((unsigned char *)(&storage[k]));
           }

       ret.resize(messages, elem);
       return ret;
   }

 void EmbObjSkin::destroyBuffer(CanBuffer &buffer)
   {
       CanMessage **m=buffer.getPointer();
       CFWCAN_MSG *storage=0;
       Cfw2CanMessage *msgs=0;

       if (m==0)
           {
               fprintf(stderr, "Warning trying to detroy non valid buffer\n");
               return;
           }

       storage=reinterpret_cast<CFWCAN_MSG *>(m[0]->getPointer());
       msgs=dynamic_cast<Cfw2CanMessage *>(m[0]);

       if ((msgs==0)||(storage==0))
           {
               fprintf(stderr, "Warning, troubles destroying memory\n");
               return;
           }

       delete [] storage;
       delete [] msgs;
       delete [] m;
   }

bool EmbObjSkin::open(yarp::os::Searchable& config)
{
	// Debug info
	YARP_INFO(Logger::get(), "EmbObjSkin::open", Logger::get().log_files.f3);

	printf("EmbObjSkin param %s\n", config.toString().c_str());

	// Check input parameters
	int i;
	bool correct=true;
	//correct &= config.check("Period");



	memset(info, 0x00, SIZE_INFO);
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


//	Property prop;
//
//	prop.put("device", config.find("canbusdevice").asString().c_str());
//	prop.put("CanTxTimeout", 500);
//	prop.put("CanRxTimeout", 500);
//	prop.put("CanDeviceNum", config.find("CanDeviceNum").asInt());
//	prop.put("CanMyAddress", 0);
//	prop.put("CanTxQueueSize", CAN_DRIVER_BUFFER_SIZE);
//	prop.put("CanRxQueueSize", CAN_DRIVER_BUFFER_SIZE);

//	driver.open(prop);
//	if (!driver.isValid())
//	{
//		fprintf(stderr, "Error opening PolyDriver check parameters\n");
//		return false;
//	}


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

	res = ethResCreator::getResource(prop);



#if 1

	int period=config.find("Period").asInt();
	setRate(period);

	Bottle ids=config.findGroup("SkinCanIds").tail();

	if (ids.size()>1)
	{
		cerr<<"Warning: SkinPrototype id list contains more than one entry -> devices will be merged. "<<endl;
	}
	for (int i=0; i<ids.size(); i++)
	{
		int id = ids.get(i).asInt();
		cardId.push_back (id);
#if SKIN_DEBUG
		fprintf(stderr, "Id reading from %d\n", id);
#endif
	}

//	pCanBus=0;
//	pCanBufferFactory=0;
//
//
//	driver.view(pCanBus);	// ??
//	if (!pCanBus)
//	{
//		fprintf(stderr, "Error opening /ecan device not available\n");
//		return false;
//	}
//
//	driver.view(pCanBufferFactory);
//	pCanBus->canSetBaudRate(0); //default 1MB/s
//
// 		configure HW can filter
//	for (int i=0; i<cardId.size(); i++)
//		for (int id=0; id<16; ++id)
//		{
//			pCanBus->canIdAdd(0x300+(cardId[i]<<4)+id);
//		}

	outBuffer=createBuffer(CAN_DRIVER_BUFFER_SIZE);
	inBuffer=createBuffer(CAN_DRIVER_BUFFER_SIZE);
#endif
	//elements are:
	sensorsNum=16*12*cardId.size();
	data.resize(sensorsNum);

	RateThread::start();
	return true;
}

bool EmbObjSkin::close()
{
	RateThread::stop();
//	if (pCanBufferFactory)
	{
		destroyBuffer(inBuffer);
		destroyBuffer(outBuffer);
	}
//	driver.close();
	return true;
}

int EmbObjSkin::read(yarp::sig::Vector &out)
{
	mutex.wait();
	YARP_INFO(Logger::get(),"EmbObjSkin::read", Logger::get().log_files.f3);

	EOarray_of_10canframes			skinData;
	uint16_t						size;
	EOnv							*nv = NULL;


	eOnvID_t nvid = eo_cfg_nvsEP_sk_NVID_Get(endpoint_sk_emsboard_rightlowerarm, 0x00, skinNVindex_sstatus__arrayof10canframe);

	res->transceiver->getNVvalue(nv, (uint8_t *)&skinData, &size);

	unsigned int canMessages=skinData.head.size;

	for (unsigned int i=0; i<canMessages; i++)
	{
		//CanMessage &msg=inBuffer[i];

		eOutil_canframe_t &msg = *(eOutil_canframe_t*)&skinData.data[i];
		unsigned int msgid=msg.id;
		unsigned int id;
		unsigned int sensorId;
		id=(msgid & 0x00f0)>>4;
		sensorId=msgid&0x000f;

		unsigned int type=msg.data[0]&0x80;
		int len=msg.size;

		for (int i=0; i<cardId.size(); i++)
		{
			if (id==cardId[i])
			{
				int index=16*12*i + sensorId*12;

				if (type)
				{
					for(int k=0;k<5;k++)
						data[index+k+7]=msg.data[k+1];
				}
				else
				{
					for(int k=0;k<7;k++)
						data[index+k]=msg.data[k+1];
				}
				//    else
				//        {
				//            std::cerr<<"Error: skin received malformed message\n";
				//        }
			}
		}
	}
///////////////////////////////////////

//	out=data;  //old - this needs the running thread
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
	YARP_INFO(Logger::get(),"EmbObjSkin::calibrateSensor", Logger::get().log_files.f3);

	for (int i=0; i<cardId.size(); i++)
	{
#if SKIN_DEBUG
		printf("SkinPrototype:: calibrating boardId: %d\n",cardId[i]);
#endif

		unsigned int canMessages=0;
		unsigned id = 0x200 + cardId[i];

		CanMessage &msg=outBuffer[0];
		msg.setId(id);
		msg.getData()[0]=0x4C; // message type
		msg.getData()[1]=0x01; 
		msg.getData()[2]=0x01; 
		msg.getData()[3]=0x01;
		msg.getData()[4]=0;
		msg.getData()[5]=0x22;
		msg.getData()[6]=0;
		msg.getData()[7]=0;
		msg.setLen(8);
		canMessages=0;
		// create a ROP to start/initialize the MTB, if needed
//		pCanBus->canWrite(outBuffer, 1, &canMessages);
	}


	int 							j=0;
	eOmc_joint_config_t				a;
	uint16_t						sizze;

	eOnvID_t nvid = eo_cfg_nvsEP_sk_NVID_Get(endpoint_sk_emsboard_leftlowerarm, 0x00, skinNVindex_sconfig__sigmode);
	res->transceiver->load_occasional_rop(eo_ropcode_set, endpoint_sk_emsboard_leftlowerarm, nvid);

	return true;
}

int EmbObjSkin::calibrateChannel(int ch, double v)
{
	//NOT YET IMPLEMENTED
	return calibrateSensor();
}

int EmbObjSkin::calibrateSensor(const yarp::sig::Vector& v)
{
	return 0;
}

int EmbObjSkin::calibrateChannel(int ch)
{
	return 0;
}

bool EmbObjSkin::threadInit()
{
	for (int i=0; i<cardId.size(); i++)
	{
#if SKIN_DEBUG
		printf("SkinPrototype:: thread initialising boardId:%d\n",cardId[i]);
#endif

		unsigned int canMessages=0;
		unsigned id = 0x200 + cardId[i];

		CanMessage &msg=outBuffer[0];
		msg.setId(id);
		msg.getData()[0]=0x4C; // message type
		msg.getData()[1]=0x01; 
		msg.getData()[2]=0x01; 
		msg.getData()[3]=0x01;
		msg.getData()[4]=0;
		msg.getData()[5]=0x22;
		msg.getData()[6]=0;
		msg.getData()[7]=0;
		msg.setLen(8);
		canMessages=0;
		// create a ROP to start/initialize the MTB, if needed
//		pCanBus->canWrite(outBuffer, 1, &canMessages);
	}

	return true;
}

void EmbObjSkin::run()
{	
	mutex.wait();
	YARP_INFO(Logger::get(),"EmbObjSkin::run", Logger::get().log_files.f3);
	/*
	unsigned int canMessages=0;

	// read nv from NvsCfg DataBase
	//bool res=pCanBus->canRead(inBuffer,CAN_DRIVER_BUFFER_SIZE,&canMessages);
	if (!res)
	{
		std::cerr<<"canRead failed\n";
	}

	for (unsigned int i=0; i<canMessages; i++)
	{
		CanMessage &msg=inBuffer[i];

		unsigned int msgid=msg.getId();
		unsigned int id;
		unsigned int sensorId;
		id=(msgid & 0x00f0)>>4;
		sensorId=msgid&0x000f;

		unsigned int type=msg.getData()[0]&0x80;
		int len=msg.getLen();

		for (int i=0; i<cardId.size(); i++)
		{
			if (id==cardId[i])
			{
				int index=16*12*i + sensorId*12;

				if (type)
				{
					for(int k=0;k<5;k++)
						data[index+k+7]=msg.getData()[k+1];
				}
				else
				{
					for(int k=0;k<7;k++)
						data[index+k]=msg.getData()[k+1];
				}
				//    else
				//        {
				//            std::cerr<<"Error: skin received malformed message\n";
				//        }
			}
		}
	}
*/
	mutex.post();
}

void EmbObjSkin::threadRelease()
{
#if SKIN_DEBUG
	printf("SkinPrototype Thread releasing...\n");	
	printf("... done.\n");
#endif
}

