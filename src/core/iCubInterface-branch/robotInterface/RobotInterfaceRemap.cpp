// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
#include "debugging.h"
#include <yarp/os/Log.h>
#include <yarp/os/impl/Logger.h>


using namespace yarp::os::impl;
using namespace yarp::dev;
using namespace yarp::os;
using namespace std;

inline void printNoDeviceFound(const char *str)
{
    printf("==========\n");
    printf("Warning: skipping device %s, the automatic procedure failed to detect associated id on the CAN network.\n", str);
    printf("Possible causes could be:\n");
    printf("- The corresponding CAN device is not working\n");
    printf("- No valid descriptor was found on the firmware. Run the canLoader app and ");
    printf("set the 'additional info' string in the firmware so that it starts with %s (case sensitive).\n", str);
}

RobotNetworkEntry *RobotNetwork::find(const string &id)
{
    RobotNetworkIt it=begin();
    while(it!=end())
    {
        if ((*it)->id==id)
            return (*it);
        it++;
    }
    return 0;
}

RobotPartEntry::RobotPartEntry()
{
    // AC_YARP_INFO(Logger::get(),"RobotPartEntry::RobotPartEntry()", Logger::get().log_files.f3);
    iwrapper=0;
}

RobotPartEntry::~RobotPartEntry()
{
    // AC_YARP_INFO(Logger::get(),"RobotPartEntry::~RobotPartEntry()", Logger::get().log_files.f3);
    close();
}

bool RobotPartEntry::open(Property &p)
{
    // AC_YARP_INFO(Logger::get(),"RobotPartEntry::open()", Logger::get().log_files.f3);

    driver.open(p);

    if (!driver.isValid())
        return false;

    driver.view(iwrapper);

    if (iwrapper)
        return true;
    else
        return false;
}

void RobotPartEntry::close()
{
    // AC_YARP_INFO(Logger::get(),"RobotPartEntry::close() ", Logger::get().log_files.f3);
    if (driver.isValid())
    {
        iwrapper=0;
        driver.close();
    }
}

RobotPartEntry *RobotParts::find(const string &pName)
{
    RobotPartIt it=begin();
    for(;it!=end(); it++)
    {
        if ((*it)->id==pName)
            {
                return (*it);
            }
    }

    return 0;
}



// implementation of the RobotInterfaceRemap class

RobotInterfaceRemap::RobotInterfaceRemap()
{
    gyro_i = 0;

    initialized = false;

    isParking=false;
    isCalibrating=false;
    abortF=false;

    #ifdef _USE_INTERFACEGUI
    mServerLogger=NULL;
    #endif
}

RobotInterfaceRemap::~RobotInterfaceRemap()
{
    #ifdef _USE_INTERFACEGUI
    if (mServerLogger)
    {
        iCubInterfaceGuiServer *deleting=mServerLogger;
        mServerLogger=NULL;

        deleting->stop();
        delete deleting;
    }
    #endif
}

void RobotInterfaceRemap::park(bool wait)
{
    // AC_YARP_INFO(Logger::get(),"RobotInterfaceRemap::park(bool wait)", Logger::get().log_files.f3);

    if (abortF)
        return;

    isParking=true;

    RobotNetworkIt it=networks.begin();
    while(it!=networks.end())
    {
        (*it)->startPark();
        it++;
    }

    it=networks.begin();
    if (wait)
    {
        while(it!=networks.end())
        {
            (*it)->joinPark();
            it++;
        }
    }

    isParking=false;
}

void RobotInterfaceRemap::calibrate(bool wait)
{
    if (abortF)
        return;

    isCalibrating=true;

    RobotNetworkIt it=networks.begin();
    while(it!=networks.end())
    {
        (*it)->startCalib();
        it++;
    }

    it=networks.begin();
    if(wait)
    {
        while(it!=networks.end())
        {
            (*it)->joinCalib();
            it++;
        }
    }

    isCalibrating=false;
}

bool RobotInterfaceRemap::initialize(const std::string &inifile)
{
    // AC_YARP_INFO(Logger::get(),"RobotInterfaceRemap::initialize(...)", Logger::get().log_files.f3);
    std::string filename;
    std::string portname;

    std::string PATH;

    PATH=extractPath(inifile.c_str());

    Property robotOptions;
    printf("Read robot description from %s\n", inifile.c_str());
    robotOptions.fromConfigFile(inifile.c_str());

    if (robotOptions.findGroup("GENERAL").check("automaticIds"))
    {
        Value &v=robotOptions.findGroup("GENERAL").find("automaticIds");

        //try to read as list
        Bottle *pNetList=v.asList();

        Bottle netList;
        if (pNetList==0)
            netList.addString(v.asString().c_str());     //read as single string
        else
            netList=*pNetList;

        std::cout<<"Starting automatic detection of can ids"<<endl;
        std::cout<<"Warning: error messages during this procedure can be ignored"<<endl;

        for(int m=0; m<netList.size() ;m++)
        {
            std::string device=netList.get(m).asString().c_str();
            std::cout<<"I'm probing the icub can network on "<<device<< " this might take a few seconds..." << endl;

            ICUB_CAN_IDS tmpIds=idDisc.discover(device.c_str());
            can_ids.push(device, tmpIds);

            int found=tmpIds.size();
            std::cout<<"Found " << found << " networks" << endl;
            ICUB_CAN_IDS::CanIdIterator it=tmpIds.canIdList.begin();
            while(it!=tmpIds.canIdList.end())
            {
                cout << "On " << device << " network " << (*it).key << " has id " << (*it).id << endl;
                it++;
            }

            automaticIds=true;
        }

        std::cout<<"Terminated automatic detection of can ids"<<endl;
    }
    else
    {
        printf("Using can ids from ini file.\n");
        automaticIds=false;
    }

    if (robotOptions.check("fileformat"))
    {
        float format=static_cast<float>(robotOptions.find("fileformat").asDouble());
        if (format==2.0)
            return initialize20(inifile);
        else
        {
            std::cerr<<"Configuration file unrecognized format\n";
            return false;
        }
    }
    else
        return initialize10(inifile);

}

bool RobotInterfaceRemap::initCart(const::string &file)
{
    // AC_YARP_INFO(Logger::get(),"RobotInterfaceRemap::initCart(...)", Logger::get().log_files.f3);
    Property options;
    options.fromConfigFile(file.c_str());

    int nDrivers=options.findGroup("GENERAL").find("NumberOfDrivers").asInt();

    std::cout<<"Initializing controller with parts: ";
    yarp::dev::PolyDriverList plist;
    bool valid=true;
    for(int k=0; k<nDrivers; k++)
    {
        std::string drGroup;
        std::ostringstream tmpStr;
        tmpStr << k;
        drGroup=std::string("DRIVER_").append(tmpStr.str());
        std::string part=options.findGroup(drGroup.c_str()).find("Key").asString().c_str();

        std::cout<<part;
        std::cout<<",";

        RobotPartEntry *robPart=parts.find(part);
        if (robPart)
        {
             plist.push(&robPart->driver, part.c_str());
        }
        else
        {
            valid=false;
        }
    }

    std::cout<<"\n";

    if (valid)
    {
        CartesianController *cart=new CartesianController;
        options.put("device", "cartesiancontrollerserver");
        if (cart->driver.open(options))
            {
                cart->driver.view(cart->iwrapper);
                cart->iwrapper->attachAll(plist);
                cartesianControllers.push_back(cart);
            }
        else
            {
                std::cerr<<"Sorry, could not open cartesian controller\n";
                return false;
            }
    }
    else
    {
        std::cerr<<"Sorry, could not detect all devices required by controller\n";
        return false;
    }

    return true;
}

bool RobotInterfaceRemap::finiCart()
{
    // AC_YARP_INFO(Logger::get(),"RobotInterfaceRemap::finiCart(...)", Logger::get().log_files.f3);
    CartesianControllersIt it=cartesianControllers.begin();

    while(it!=cartesianControllers.end())
    {
        (*it)->close();
        it++;
    }
    return true;
}

bool RobotInterfaceRemap::initialize10(const std::string &inifile)
{
    // AC_YARP_INFO(Logger::get(),"RobotInterfaceRemap::initialize10(...)", Logger::get().log_files.f3);
    fprintf(stderr, "Going to initialize the robot with a file\n");

    std::string PATH;
    PATH=extractPath(inifile.c_str());

    Property robotOptions;
    fprintf(stderr, "Read robot description from %s\n", inifile.c_str());
    robotOptions.fromConfigFile(inifile.c_str());

    robotName=robotOptions.findGroup("GENERAL").find("name").asString().c_str();
    Bottle *pNets=robotOptions.findGroup("GENERAL").find("networks").asList();
    Bottle nets;
    if (pNets==0)
    {
        // This is to maintain compatibility with old ini files
        fprintf(stderr, "Warning parsing %s could not find a valid network description\n",inifile.c_str());
        fprintf(stderr, "Assuming old style ini file\n");
        nets.fromString("HEAD RIGHT_ARM LEFT_ARM LEGS");
    }
    else
    {
        nets=*pNets;
    }

    int nnets=nets.size();
    std::cout<<"Using " <<nnets <<" networks"<<endl;

    //std::cout<<robotOptions.toString()<<endl;

    int n=0;
    for(n=0;n<nnets;n++)
    {
        std::string netid=nets.get(n).asString().c_str();
        RobotNetworkEntry *netEntry;
        if (robotOptions.check(netid.c_str()))
        {
            netEntry=new RobotNetworkEntry;

            Property tmpProp;
            tmpProp.fromString(robotOptions.findGroup(netid.c_str()).toString());

            if (!instantiateNetwork(PATH, tmpProp, *netEntry))
            {
                std::cerr<< "Troubles instantiating "<< netid.c_str() << endl;
                delete netEntry;
            }
            else
            {
                netEntry->id=netid;
                networks.push_back(netEntry);

                Bottle *partsList=robotOptions.findGroup(netid.c_str()).find("parts").asList();
                if (partsList!=0)
                {
                    for(int p=0;p<partsList->size();p++)
                    {
                        Property tmpProp;
                        tmpProp.fromString(robotOptions.findGroup(partsList->get(p).asString()).toString());
                        tmpProp.put("device", "controlboardwrapper");
                        std::string prefix=robotName;
                        prefix+="/";
                        prefix+=partsList->get(p).asString();
                        tmpProp.put("name", prefix.c_str());
                        std::cout<<"-->"<<tmpProp.toString()<<endl;

                        RobotPartEntry *partEntry=new RobotPartEntry;
                        partEntry->id=partsList->get(p).asString().c_str();

                        if (partEntry->open(tmpProp))
                        {
                            PolyDriverList p;
                            p.push(&netEntry->driver, "");
                            partEntry->iwrapper->attachAll(p);
                            parts.push_back(partEntry);
                        }
                    }
                }
                else
                {
                    cout<<"No part list specified exporting whole device"<<endl;
                    Property tmpProp;
                    std::string prefix=robotName;
                    prefix+="/";
                    prefix+=netid.c_str();

                    //this is to maintain compatibility with old ini file/code
                    for(unsigned int k=0;k<prefix.length();k++)
                    {
                        prefix[k]=tolower(prefix[k]);
                    }
                    ///

                    tmpProp.put("name", prefix.c_str());
                    tmpProp.put("device", "controlboardwrapper");
                    std::cout<<"--> " << tmpProp.toString()<<endl;
                    RobotPartEntry *partEntry=new RobotPartEntry;
                    partEntry->id=netid;

                    if (partEntry->open(tmpProp))
                        {
                            PolyDriverList p;
                            p.push(&netEntry->driver, netEntry->id.c_str());
                            partEntry->iwrapper->attachAll(p);
                            parts.push_back(partEntry);
                        }
                }
            }
        }
        else
        {
            std::cout<<"not found, skipping"<<endl;
            netEntry=0;
        }
    }

    fprintf(stderr, "RobotInterface::now opening inertial\n");
    if (robotOptions.check("INERTIAL"))
    {
        Property tmpProp;
        //copy parameters verbatim from relative section
        tmpProp.fromString(robotOptions.findGroup("INERTIAL").toString());
        fprintf(stderr, "RobotInterface:: inertial sensor is in the conf file\n");
        if (!instantiateInertial(PATH, tmpProp))
            fprintf(stderr, "RobotInterface::warning troubles instantiating inertial sensor\n");
    }
    else
        fprintf(stderr, "RobotInterface::no inertial sensor defined in the config file\n");

    std::cout<<"Starting robot calibration!"<<endl;
    calibrate();
    std::cout<<"Finished robot calibration!"<<endl;

    return true;
}

bool RobotInterfaceRemap::initialize20(const std::string &inifile)
{
    fprintf(stderr, "Initialization from file, new version 2.0\n");

    std::string PATH;
    PATH=extractPath(inifile.c_str());

    Property robotOptions;  // will holf icub.ini in formatted form
    fprintf(stderr, "Read robot description from %s\n", inifile.c_str());
    robotOptions.fromConfigFile(inifile.c_str());

    string str=robotOptions.toString().c_str();
    // AC_YARP_INFO(Logger::get(), "RobotInterfaceRemap::initialize20", Logger::get().log_files.f3);

    robotName=robotOptions.findGroup("GENERAL").find("name").asString().c_str();
    Bottle *reqParts=robotOptions.findGroup("GENERAL").find("parts").asList();
    int nparts=0;
    if (reqParts!=0)
        nparts=reqParts->size();

    string PC104IpAddress=robotOptions.findGroup("GENERAL").find("PC104IpAddress").asString().c_str();
    printf("\n\n>> PC104IpAddress : %s\n\n", PC104IpAddress.c_str());
    std::cout << "******************************************************************" << endl;
    std::cout << "--> Creating MotionControl Devices                               *" <<endl;
    std::cout << "******************************************************************" << endl;
    std::cout<<"Found " << nparts <<" parts"<<endl;

    int n=0;
    for(n=0;n<nparts;n++)
    {
        std::string partid=reqParts->get(n).asString().c_str();
        std::cout<<"--> Processing "<<partid<<endl;

        // AC_YARP_INFO(Logger::get(), String("--> Processing part: ") + String(reqParts->get(n).asString()), Logger::get().log_files.f3);

        RobotPartEntry *partEntry=new RobotPartEntry;
        partEntry->id=partid;

        Bottle *nets=robotOptions.findGroup(partid.c_str()).find("networks").asList();
        if (nets==0)
        {
            std::cerr<< "Error, missing network list in inifile for part " << partid << endl;
            std::cerr<< "Should be something like networks (net1 net2)"<<endl;
            return false;
        }

        for(int n=0;n<nets->size();n++)
        {
            std::string netId=nets->get(n).asString().c_str();
            RobotNetworkEntry *net=networks.find(netId);

            if (!net)
            {
                //create it and push into networks
                net=new RobotNetworkEntry;
                net->id=netId;
                networks.push_back(net);
            }

            //add to list of networks for part
            partEntry->push(net);
        }

        //add to list of parts
        parts.push_back(partEntry);
    }

    //now go through list of networks and initialize it

    RobotNetworkIt netit=networks.begin();
    while(netit!=networks.end())
    {
        RobotNetworkEntry *netEntry=(*netit);
        std::string netid=netEntry->id;

        Property tmpProp;
        tmpProp.fromString(robotOptions.findGroup(netid.c_str()).toString());


        tmpProp.put("PC104IpAddress", PC104IpAddress.c_str());
        tmpProp.put("FeatId", netEntry->id.c_str());

        string str=tmpProp.toString().c_str();
        // AC_YARP_INFO(Logger::get(), tmpProp.toString().c_str(), Logger::get().log_files.f3);


        cout << "Instantiating network " << netid.c_str() << "...";
        if (!instantiateNetwork(PATH, tmpProp, *netEntry))
        {
            cerr << endl << "ERROR: troubles instantiating " << netid.c_str() << endl;
        }
        else
            cout << "Network "<< netid.c_str() << " instantiated correctly"<< endl;

        netit++;
    }

    std::cout<<"--> Starting robot calibration!  -- dummy"<<endl;
    calibrate();
    std::cout<<"Finished robot calibration!"<<endl;

    //now iterate through list of parts to see if all networks have been created correctly
    std::cout << "******************************************************************" << endl;
    std::cout << "--> Now I will go through the list of parts to create the wrappers" <<endl;
    std::cout << "******************************************************************" << endl;

    RobotPartIt partit=parts.begin();

    while(partit!=parts.end())
    {
        RobotPartEntry *tmp=*partit;

        using namespace std;
        printf("§§§§§§§§§§ robotOptions INFOS!!\n");
        //string str2=robotOptions.toString().c_str();
        cout << str << endl << endl;

        //create the wrappers
        Property tmpProp;
        //copy parameters verbatim from relative section
        tmpProp.fromString(robotOptions.findGroup(tmp->id.c_str()).toString());

        //add device name
        tmpProp.put("device", "controlboardwrapper2");
        //append robot name
        std::string prefix=robotName;
        prefix+="/";
        prefix+=tmp->id.c_str();
        tmpProp.put("name", prefix.c_str());
        //open wrapper
        std::cout<<"Opening wrapper for " << tmp->id << endl;
        //std::cout<<"Parameter list:"<<endl;
        //std::cout<<tmpProp.toString();
        //std::cout<<endl;

        if (!tmp->open(tmpProp))
        {
            partit++;
            continue;
        }

        RobotNetworkIt netIt=tmp->networks.begin();

        //now attach all network devices
        PolyDriverList polylist;
        while(netIt!=tmp->networks.end())
        {
            RobotNetworkEntry *net=(*netIt);

            if (net->isValid())
            {
/*                iCubDeviceInterface        *res;
                net->driver.view(res);
                PolyDriver   *res2 = res->motionCtrl;
                std::cout<<"Attaching " << net->id.c_str() << endl;
                polylist.push(res2, net->id.c_str());
                //tmp->wrapper.attach(&net->driver, net->id.c_str());
*/
//             orig
                std::cout<<"Attaching " << net->id.c_str() << endl;
                polylist.push(&net->driver, net->id.c_str());
                //tmp->wrapper.attach(&net->driver, net->id.c_str());

            }
            else
            {
                std::cout<<"Skipping " << net->id.c_str() << " for part " << tmp->id << endl;
            }
            netIt++;
        }

        tmp->iwrapper->attachAll(polylist);
        std::cout<<endl;
        partit++;
    }

    fprintf(stderr, "RobotInterface::now opening inertial\n");
    if (robotOptions.check("INERTIAL"))
    {
        Property tmpProp;
        //copy parameters verbatim from relative section
        tmpProp.fromString(robotOptions.findGroup("INERTIAL").toString());
        fprintf(stderr, "RobotInterface:: inertial sensor is in the conf file\n");
        if (!instantiateInertial(PATH, tmpProp))
            fprintf(stderr, "RobotInterface::warning troubles instantiating inertial sensor\n");
    }
    else
    {
        fprintf(stderr, "RobotInterface::no inertial sensor defined in the config file\n");
    }



    // now go thourgh list of networks and create analog interface
    Bottle *analogNets=robotOptions.findGroup("GENERAL").find("analog").asList();
    std::cout<<"--> Checking if I need to create analog wrappers"<<std::endl;
    if (analogNets)
    {
        int nanalog=analogNets->size();

        int n=0;
        for(n=0;n<nanalog;n++)
        {
            std::string analogid=analogNets->get(n).asString().c_str();

            std::string netid=robotOptions.findGroup(analogid.c_str()).find("network").asString().c_str();
            int period=20;
            if (robotOptions.findGroup(analogid.c_str()).check("period"))
                period = robotOptions.findGroup(analogid.c_str()).find("period").asInt();
            else
                std::cout<<"Warning: could not find period using default value ("<<period<<")\n";

            Bottle *analogIds=robotOptions.findGroup(analogid.c_str()).find("deviceId").asList();
            if (analogIds!=0)
                for(int k=0;k<analogIds->size();k++)
                {
                    std::string deviceId=analogIds->get(k).asString().c_str();
                    std::cout<<"Instantiating analog device " << deviceId << " on "<< netid << endl;

                    RobotNetworkEntry *selectedNet=networks.find(netid);
                    if (selectedNet==0)
                    {
                        std::cerr<<"Sorry "<<netid<<" has not been instantiated, skipping"<<endl;
                    }
                    else
                    {
                        DeviceDriver *dTmp;
                        yarp::dev::IFactoryInterface *iFactory;
                        selectedNet->driver.view(iFactory);

                        if (iFactory==0)
                        {
                            std::cout<<"CanBus device does not support iFactory interface\n";
                        }
                        else
                        {
                            yarp::os::Property prop;
                            prop.put("device", "analog");
                            prop.put("deviceid", deviceId.c_str());
                            dTmp=iFactory->createDevice(prop);

                            IAnalogSensor *iTmp=dynamic_cast<IAnalogSensor *>(dTmp);

                            selectedNet->analogSensors.push_back(iTmp);
                            if (iTmp)
                            {
                                std::string name;
                                name+="/";
                                name+=robotName;
                                name+="/";
                                name+=deviceId.c_str();
                                name+="/analog:o";

                                AnalogServer *tmp=new AnalogServer(name.c_str());
                                tmp->setRate(period);
                                tmp->attach(iTmp);
                                tmp->start();

                                selectedNet->analogServers.push_back(tmp);
                            }
                        }
                    }
                }
        }
    }
    else
    {
        std::cout<<"No analog wrappers requested\n";
    }

    // Creating skin Parts
     std::cout << "******************************************************************" << endl;
     std::cout << "--> Creating skin Part                                            " <<endl;
     std::cout << "******************************************************************" << endl;
     fflush(stdout);

    Bottle *skinParts=robotOptions.findGroup("GENERAL").find("skinParts").asList();
    std::cout<<"--> Checking if I need to create skin parts"<<std::endl;
    if (skinParts)
    {
        int nskin=skinParts->size();
        cout<< "I have found " << nskin << " parts\n";
        int n=0;
        for (n=0;n<nskin;n++)
        {
            std::string partId=skinParts->get(n).asString().c_str();
            std::cout<<"Opening " << partId << "\n";

            Property partOptions;
            partOptions.fromString(robotOptions.findGroup(partId.c_str()).toString());
            partOptions.put("robot", robotName.c_str());

            Bottle xtmp, xtmp2;

            bool correct=true;

            correct=correct&&robotOptions.check("GENERAL");

            if(correct)
                xtmp = Bottle(robotOptions.findGroup("GENERAL"));

            correct=correct&&xtmp.check("PC104IpAddress");

            if(correct)
                xtmp2 = xtmp.findGroup("PC104IpAddress");

            SkinPartEntry *tmp=new SkinPartEntry;
            tmp->setId(partId);

            if (partOptions.check("file"))
            {
                std::string filename=PATH+partOptions.find("file").asString().c_str();

                Property deviceParams;
                deviceParams.fromConfigFile(filename.c_str());
                deviceParams.put("PC104IpAddress", xtmp2.get(1).asString().c_str());
                deviceParams.put("FeatId",partId.c_str());

                if (tmp->open(deviceParams, partOptions))
                {
                    skinparts.push_back(tmp);
                }
                else
                {
                    std::cerr<<"Error instantiating skin part " << partId << "check parameters"<<endl;
                    delete tmp;
                }
            }
        }
    }

    #ifdef _USE_INTERFACEGUI
    mServerLogger=new iCubInterfaceGuiServer;
    mServerLogger->config(PATH,robotOptions);


    for (RobotNetworkIt netIt=networks.begin(); netIt!=networks.end(); ++netIt)
    {
        IClientLogger* pCL=NULL;
        (*netIt)->driver.view(pCL);

        if (pCL)
        {
            pCL->setServerLogger(mServerLogger);
        }
    }

    for (SkinPartsIt skinIt=skinparts.begin(); skinIt!=skinparts.end(); ++skinIt)
    {
        IClientLogger* pCL=NULL;
        (*skinIt)->driver.view(pCL);

        if (pCL)
        {
            pCL->setServerLogger(mServerLogger);
        }
    }

    mServerLogger->start();

    #endif

    return true;
}

bool RobotInterfaceRemap::instantiateNetwork(std::string &path, Property &robotOptions, RobotNetworkEntry &net)
{
    std::string file=robotOptions.find("file").asString().c_str();
    std::string fullFilename;
    fullFilename=path;

    string str("\n\n ------------------------------ \n RobotInterfaceRemap::instantiateNetwork(...)\n");
    str += robotOptions.toString().c_str();
    // AC_YARP_INFO(Logger::get(), str.c_str(), Logger::get().log_files.f3);

    if (fullFilename.length()!=0 && fullFilename[fullFilename.length()-1]!='/')
    {
        fullFilename.append("/",1);
    }
    fullFilename.append(file.c_str(), file.length());

    Property deviceParameters;
    deviceParameters.fromConfigFile(fullFilename.c_str());

    Value &device=robotOptions.find("device");
    Value &subdevice=robotOptions.find("subdevice");
    Value &candevice=robotOptions.find("canbusdevice");
    Value &PC104IpAddress=robotOptions.find("PC104IpAddress");
    Value &featId=robotOptions.find("FeatId");

    deviceParameters.put("robotName",robotName.c_str());
    deviceParameters.put("device", device);
    deviceParameters.put("subdevice", subdevice);
    deviceParameters.put("canbusdevice",candevice);
    deviceParameters.put("PC104IpAddress",PC104IpAddress);
    deviceParameters.put("FeatId",featId);


    printf("\n\ndevice dev: %s\n", device.asString().c_str());
    printf("subdevice dev: %s\n", subdevice.asString().c_str());
    printf("candevice dev: %s\n", candevice.asString().c_str());
    printf("PC104IpAddress dev: %s\n", PC104IpAddress.asString().c_str());

    ICUB_CAN_IDS *ids=can_ids.find(candevice.asString().c_str());

    int networkN=deviceParameters.findGroup("CAN").find("CanDeviceNum").asInt();
    if (ids)
    {
        string netid=deviceParameters.findGroup("CAN").find("NetworkId").asString().c_str();

        //std::cerr<<"Netid:"<<netid<<endl;
        //overwriting net id
        networkN=(*ids)[netid];
        if (networkN!=-1)
        {
            forceNetworkId(deviceParameters, networkN);
        }
        else
        {
            std::cerr<<"Error: requested automatic ids but no id was found for network " << netid << endl;
            return false;
        }
    }

    printf("can device: %s\n", candevice.asString().c_str() );
    if (robotOptions.check("calibrator"))
    {
        Value &calibrator=robotOptions.find("calibrator");
        Property pTemp;
        pTemp.fromString(deviceParameters.toString());
        pTemp.put("device",calibrator.toString());
        net.calibrator.open(pTemp);
        std::cout<<"Opened calibrator "; fflush(stdout);
    }

    if (robotOptions.check("verbose"))
        deviceParameters.put("verbose", 1);

    std::cout<<"Opening network " << networkN << " on device " << device.asString().c_str() << "... "; fflush(stdout);

    ICalibrator *icalibrator = 0x00;
    IControlCalibration2 *iCrtlCalib =0x00;

    //
    //  this function will be called once for each "Feature" the ems will expose, i.e. one for MC, one for AS, one for skin
    //  they'll be on three different RobotNetworkEntry
    //


#if 1   // old style

    net.driver.open(deviceParameters);

    if (!net.driver.isValid())
    {
        std::cout<<"failed!"<<endl;
        return false;
    }
    std::cout<<"done!"<<endl;



    //acquire calibration int
    net.driver.view(iCrtlCalib);

    //save interface for later use
    net.iCalib=iCrtlCalib;

    //acquire calibrator int
    net.calibrator.view(icalibrator);

    //set calibrator
    net.iCalib->setCalibrator(icalibrator);
#else

    // new style
    //acquire ethResources handler
    /* iCubDeviceInterface *t = 0x00;
    net.driver->view(t);
    net.device = t;

    //acquire calibrator handler

    net.device->getControlCalibration(&iCrtlCalib);
    net.iCalib=iCrtlCalib;

    net.calibrator.view(icalibrator);
    net.iCalib->setCalibrator(icalibrator);
    */

    // new new style
    //acquire ethResources handler
//    iCubDeviceInterface *t = 0x00;

//    iCubDeviceInterface *tmpDevice;
//    net.manager.open(deviceParameters);
//
//    if (!net.manager.isValid())
//    {
//        std::cout<<"failed!"<<endl;
//        return false;
//    }
//    std::cout<<"done!"<<endl;
//    net.manager.view(tmpDevice);

    //acquire calibrator handler
//    tmpDevice->getControlCalibration(&iCrtlCalib);
//    net.iCalib=iCrtlCalib;

//    net.calibrator.view(icalibrator);
//    net.iCalib->setCalibrator(icalibrator);


    //
    // Open different features
    //

    Property prop;
    ACE_TCHAR tmp[126];
    //Searchable config;
    str=deviceParameters.toString().c_str();
    Bottle xtmp = Bottle(deviceParameters.findGroup("FEATURES"));
    prop.fromString(str.c_str());
    prop.unput("device");
    prop.unput("subdevice");
    // look for Ethernet device driver to use and put it into the "device" field.
    Value &motionControl=xtmp.find("motionControl");
    strcpy(tmp, motionControl.asString().c_str());
    prop.put("device", motionControl.asString().c_str());

    net.driver.open(prop);
    if (!net.driver.isValid())
    {
        std::cout<<"failed!"<<endl;
        return false;
    }
    std::cout<<"done!"<<endl;

    Property prop;
    ACE_TCHAR tmp[126];
    //Searchable config;
    str=deviceParameters.toString().c_str();
    Bottle xtmp = Bottle(deviceParameters.findGroup("FEATURES"));
    prop.fromString(str.c_str());
    prop.unput("device");
    prop.unput("subdevice");
    // look for Ethernet device driver to use and put it into the "device" field.
    Value &motionControl=xtmp.find("motionControl");
    strcpy(tmp, motionControl.asString().c_str());
    prop.put("device", motionControl.asString().c_str());

    net.driver.open(prop);
    if (!net.driver.isValid())
    {
        std::cout<<"failed!"<<endl;
        return false;
    }
    std::cout<<"done!"<<endl;

    Property prop;
    ACE_TCHAR tmp[126];
    //Searchable config;
    str=deviceParameters.toString().c_str();
    Bottle xtmp = Bottle(deviceParameters.findGroup("FEATURES"));
    prop.fromString(str.c_str());
    prop.unput("device");
    prop.unput("subdevice");
    // look for Ethernet device driver to use and put it into the "device" field.
    Value &motionControl=xtmp.find("motionControl");
    strcpy(tmp, motionControl.asString().c_str());
    prop.put("device", motionControl.asString().c_str());

    net.driver.open(prop);
    if (!net.driver.isValid())
    {
        std::cout<<"failed!"<<endl;
        return false;
    }
    std::cout<<"done!"<<endl;


#endif

    return true;
}

bool RobotInterfaceRemap::instantiateInertial(Property &options)
{
    // AC_YARP_INFO(Logger::get(),"RobotInterfaceRemap::instantiateInertial(...) 1st version", Logger::get().log_files.f3);
    fprintf(stderr, "Instantiating an INERTIAL device\n");

    const char *conf = yarp::os::getenv("ICUB_ROOT");
    robotName=options.findGroup("GENERAL").find("name").asString().c_str();

    //////////// inertial
    Value &device=options.findGroup("INERTIAL").find("device");
    Value &subdevice=options.findGroup("INERTIAL").find("subdevice");
    Value &inifile=options.findGroup("INERTIAL").find("file");

    std::string fullFilename;
    std::string portName;
    fullFilename+=conf;
    fullFilename+="/conf/";
    fullFilename+=inifile.asString().c_str();

    Property p;
    p.fromConfigFile(fullFilename.c_str());

    p.put("device", device);
    p.put("subdevice", subdevice);

    portName+="/";
    portName+=robotName.c_str();
    portName+="/inertial";
    p.put("name", portName.c_str());

    // create a device for the arm
    gyro.open(p);
    if (!gyro.isValid())
    {
        return false;
    }

    bool ok = gyro.view(gyro_i);

    return ok;
}

bool RobotInterfaceRemap::instantiateInertial(const std::string &path, Property &options)
{
    // AC_YARP_INFO(Logger::get(),"RobotInterfaceRemap::instantiateInertial(...) 2nd version", Logger::get().log_files.f3);
    //    std::cout<<"Path: "<<path<<" Property list: "<<options.toString().c_str();
    std::string file=options.find("file").asString().c_str();
    std::string device=options.find("device").asString().c_str();
    std::string subdevice=options.find("subdevice").asString().c_str();

    std::string fullFilename;
    fullFilename=path;
    if (fullFilename.length()!=0 && fullFilename[fullFilename.length()-1]!='/')
    {
        fullFilename.append("/",1);
    }
    fullFilename.append(file.c_str(), file.length());

    Property deviceParameters;
    deviceParameters.fromConfigFile(fullFilename.c_str());

    std::string portName;
    portName+="/";
    portName+=robotName.c_str();
    portName+="/inertial";
    deviceParameters.put("name", portName.c_str());

    deviceParameters.put("device", device.c_str());
    deviceParameters.put("subdevice", subdevice.c_str());

    // create a device for the arm
    gyro.open(deviceParameters);
    if (!gyro.isValid())
    {
        return false;
    }

    bool ok = gyro.view(gyro_i);

    return ok;
}

bool RobotInterfaceRemap::detachWrappers()
{
    RobotPartEntry *tmpPart;
    int n=parts.size();
    while(n--)
    {
        tmpPart=parts.back();
        // std::cerr<<"Detaching "<<tmpPart->id<<endl;

        // std::cerr<<"Done detaching " << tmpPart->id<<endl;
        if (tmpPart->iwrapper!=0)
        {
            tmpPart->iwrapper->detachAll();
            tmpPart->close();
        }

        delete tmpPart;
        // std::cerr<<"Deleted object";
        parts.pop_back();
    }
    return true;
}

bool RobotInterfaceRemap::closeNetworks()
{
    // AC_YARP_INFO(Logger::get(),"RobotInterfaceRemap::closeNetworks()", Logger::get().log_files.f3);
    RobotNetworkEntry *tmpNet;
    int n=networks.size();

    while(n--)
    {
        tmpNet=networks.back();
        tmpNet->close();
        std::cout<<"Closed network: " << tmpNet->id << endl;
        delete tmpNet;
        networks.pop_back();
    }

    #ifdef _USE_INTERFACEGUI
    if (mServerLogger)
    {
        iCubInterfaceGuiServer *deleting=mServerLogger;
        mServerLogger=NULL;

        deleting->stop();
        delete deleting;
    }
    #endif

    skinparts.close();

    if (!gyro.isValid())
        gyro.close();

    initialized = false;
    return true;
}

// check if automatically discovered network id matches the one
// in the Property, substitute it if necessary.
bool RobotInterfaceRemap::forceNetworkId(yarp::os::Property& op, int autoN)
{
    bool ret=true;
    if (autoN==-1)
        return false;

    Bottle& can = op.findGroup("CAN");
    int networkN=can.find("CanDeviceNum").asInt();
    Bottle &n=can.addList();
    char tmp[80];
    sprintf(tmp, "CanForcedDeviceNum %d", autoN);
    n.fromString(tmp);

    return ret;
}


void RobotInterfaceRemap::abort()
{
    if (isParking)
    {
        RobotNetworkIt it=networks.begin();
        while(it!=networks.end())
        {
            (*it)->abortPark();
            it++;
        }
    }
    if (isCalibrating)
    {
        RobotNetworkIt it=networks.begin();
        while(it!=networks.end())
        {
            (*it)->abortCalibration();
            it++;
        }
    }

    abortF=true;
}
