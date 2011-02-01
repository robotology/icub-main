// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include <yarp/os/Time.h>
#include "iCubInterfaceGuiServer.h"

class JointRemapper
{
public:
    JointRemapper(){}
    ~JointRemapper(){}

    void push(int d0,int d1,int j0)
    {
        mapD0.push_back(d0);
        mapD1.push_back(d1);
        mapJ0.push_back(j0);
    }

    int getJoint(int pos)
    {
        pos*=2;

        for (int i=0; i<(int)mapD0.size(); ++i)
        {
            if (pos>=mapD0[i] && pos<=mapD1[i])
            {
                return mapJ0[i]+pos-mapD0[i];
            }
        }

        return -1;
    }

protected:
    std::vector<int> mapD0,mapD1,mapJ0;
};

void iCubInterfaceGuiServer::config(std::string& PATH,yarp::os::Property &robot)
{
    std::vector<JointRemapper> jointRmp;

    yarp::os::ConstString robotName=robot.find("name").asString();

    yarp::os::Bottle general=robot.findGroup("GENERAL");
    yarp::os::Bottle *parts=general.find("parts").asList();

    for (int t=0; t<parts->size(); ++t)
    {
        yarp::os::ConstString partName=parts->get(t).asString();
        yarp::os::Bottle part=robot.findGroup(partName.c_str());

        yarp::os::Bottle *networks=part.find("networks").asList();

        for (int n=0; n<networks->size(); ++n)
        {
            std::string netName(networks->get(n).asString().c_str());

            yarp::os::Bottle jMap=part.findGroup(netName.c_str());

            int j0=jMap.get(1).asInt();
            int d0=jMap.get(3).asInt();
            int d1=jMap.get(4).asInt();

            yarp::os::Bottle net=robot.findGroup(netName.c_str());

            bool bExists=false;

            for (unsigned int i=0; i<mNetworks.size(); ++i)
            {
                if (mNetworks[i]->mName==netName)
                {
                    jointRmp[i].push(d0,d1,j0);
                    bExists=true;
                    break;
                }
            }

            if (!bExists)
            {
                std::string file(net.find("file").asString().c_str());
                std::string device(net.find("canbusdevice").asString().c_str());
                mNetworks.push_back(new iCubNetwork(netName,file,device));

                jointRmp.push_back(JointRemapper());
                jointRmp.back().push(d0,d1,j0);
            }
        }
    }

    // we have now the networks list

    for (unsigned int n=0; n<mNetworks.size(); ++n)
    {
        yarp::os::Property netConf;
        netConf.fromConfigFile((PATH+mNetworks[n]->mFile).c_str());
        yarp::os::Bottle canConf=netConf.findGroup("CAN");
        
        mNetworks[n]->setID(canConf.find("CanDeviceNum").asInt());
    
        yarp::os::Bottle devices=canConf.findGroup("CanAddresses");
        devices=devices.tail();
    
        for (int d=0; d<devices.size(); ++d)
        {
            int joint=jointRmp[n].getJoint(d);
            mNetworks[n]->addBoard(new iCubBLLBoard(devices.get(d).asInt(),joint,joint+1));
        }
    }

    // ANALOGS
    yarp::os::Bottle *analogs=general.find("analog").asList();

    if (analogs)
    {       
        for (int a=0; a<analogs->size(); ++a)
        {
            std::string name=analogs->get(a).asString().c_str();

            std::string netID=robot.findGroup(name.c_str()).find("network").asString().c_str();

            for (int n=0; n<(int)mNetworks.size(); ++n)
            {
                if (*mNetworks[n]==netID)
                {
                    yarp::os::Property analogConf;
                    analogConf.fromConfigFile((PATH+mNetworks[n]->mFile).c_str());

                    yarp::os::Bottle *deviceIDs=robot.findGroup(name.c_str()).find("deviceId").asList();
                    for (int d=0; d<deviceIDs->size(); ++d)
                    {
                        std::string dev=deviceIDs->get(d).asString().c_str();
                        yarp::os::Bottle devConf=analogConf.findGroup(dev.c_str());
                        
                        int CanAddress=devConf.find("CanAddress").asInt();
                        int Channels=devConf.find("Channels").asInt();

                        mNetworks[n]->addBoard(new iCubAnalogBoard(CanAddress,Channels));
                    }
                    break;
                }
            }
        }
    }
  
    // skin
    yarp::os::Bottle *skinParts=general.find("skinParts").asList();
    if (skinParts)
    {       
        int nskin=skinParts->size();
        
        for (int n=0; n<nskin; ++n)
        {
            std::string skinName=skinParts->get(n).asString().c_str();

            yarp::os::Property skinOptions;
            skinOptions.fromString(robot.findGroup(skinName.c_str()).toString());

            if (skinOptions.check("file"))
            {
                std::string file=PATH+(skinOptions.find("file").asString()).c_str();

                yarp::os::Property skin;
                skin.fromConfigFile(file.c_str());

                std::string device(skinOptions.find("canbusdevice").asString().c_str());
                
                mNetworks.push_back(new iCubNetwork(skinName,file,device));
                mNetworks.back()->setID(skin.find("CanDeviceNum").asInt());
            }
        }
    }
    
    ///////////////////////////////////////
    mPort.open("/icubinterfacegui/server");
}

void iCubInterfaceGuiServer::run()
{
    yarp::os::Bottle msg;
    yarp::os::Bottle rpl;
	askStop=false;

    while (!askStop)
    {
        if (mPort.read(msg,true))
        {
            yarp::os::ConstString cmd=msg.get(0).asString();

            if (cmd=="GET_CONF")
            {
                mMutex.wait();
                rpl=toBottle(true);
                mMutex.post();
                mPort.reply(rpl);
            }
            else if (cmd=="GET_DATA")
            {
                mMutex.wait();
                rpl=toBottle();
                mPort.reply(rpl);
                mMutex.post();
            }
        }
        else
        {
            yarp::os::Time::delay(1.0);
        }
    } 
}

bool iCubInterfaceGuiServer::log(const std::string &key,const yarp::os::Value &data)
{
    mMutex.wait();
    for (unsigned int n=0; n<(int)mNetworks.size(); ++n)
    {
        if (mNetworks[n]->findAndWrite(key,data))
        {
            mMutex.post();
            return true;
        }
    }
    mMutex.post();
    return false;
}
/*
yarp::dev::LoggerDataRef* iCubInterfaceGuiServer::getDataReference(const std::string &key)
{
    yarp::dev::LoggerDataRef* pRef=NULL;

    for (unsigned int n=0; n<(int)mNetworks.size(); ++n)
    {
        if ((pRef=mNetworks[n]->getDataReference(key)))
        {
            pRef->setMutex(&mMutex);
            return pRef;
        }
    }

    return NULL;
}
*/
yarp::os::Bottle iCubInterfaceGuiServer::toBottle(bool bConfig)
{
    yarp::os::Bottle bot;

    bot.addString("DATA");

    for (int n=0; n<(int)mNetworks.size(); ++n)
    {   
        yarp::os::Bottle net=mNetworks[n]->toBottle(bConfig);

        if (net.size())
        {
            yarp::os::Bottle &addList=bot.addList();
            addList.addInt(n);
            addList.append(net);
        }
    }

    return bot;
}

/*
bool iCubInterfaceGuiServer::findAndRead(std::string address,yarp::os::Value* data)
{
    mMutex.wait();
    for (unsigned int n=0; n<(int)mNetworks.size(); ++n)
    {
        if (mNetworks[n]->findAndRead(address,data))
        {
            mMutex.post();
            return true;
        }
    }
    mMutex.post();
    return false;
}
*/
