// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "RobotInterfaceRemap.h"
#include "extractPath.h"

#include <yarp/os/Thread.h>

#include "ControlBoardWrapper.h"
#include "ControlBoardWrapper2.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;
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
    wrapper=0;
}

RobotPartEntry::~RobotPartEntry()
{

}

RobotPartEntry *RobotParts::find(const string &pName)
{
    RobotPartIt it=begin();
    for(;it!=end(); it++)
    {
        if ((*it)->id==pName)
            return (*it);
    }

    return false;
}


// implementation of the RobotInterfaceRemap class

RobotInterfaceRemap::RobotInterfaceRemap() 
{
    gyro_i = 0; 

    initialized = false; 

    isParking=false;
    isCalibrating=false;
    abortF=false;
}  

RobotInterfaceRemap::~RobotInterfaceRemap()
{

}

void RobotInterfaceRemap::park(bool wait)
{
    std::cout<<"RobotInterfaceRemap::park\n";

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

bool RobotInterfaceRemap::initialize10(const std::string &inifile)
{
    ACE_OS::fprintf(stderr, "Going to initialize the robot with a file\n");

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
                        std::string prefix=robotName;
                        prefix+="/";
                        prefix+=partsList->get(p).asString();
                        tmpProp.put("name", prefix.c_str());
                        //std::cout<<tmpProp.toString()<<endl;

                        RobotPartEntry *partEntry=new RobotPartEntry;
                        partEntry->id=partsList->get(p).asString().c_str();

                        ControlBoardWrapper *wp=new ControlBoardWrapper;
                        if (wp->open(tmpProp))
                        {
                            PolyDriverList p;
                            p.push(&netEntry->driver, "");
                            wp->attachAll(p);
                            partEntry->wrapper=wp;
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
                    //std::cout<<tmpProp.toString()<<endl;
                    RobotPartEntry *partEntry=new RobotPartEntry;
                    partEntry->id=netid;
                    ControlBoardWrapper *wp=new ControlBoardWrapper;

                    if (wp->open(tmpProp))
                    {
                        PolyDriverList p;
                        p.push(&netEntry->driver, netEntry->id.c_str());
                        wp->attachAll(p);

                        partEntry->wrapper=wp;
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
        fprintf(stderr, "RobotInterface:: inertial sensor is in the conf file\n");
        if (!instantiateInertial(robotOptions))
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
    ACE_OS::fprintf(stderr, "Initialization from file, new version 2.0\n");

    std::string PATH;
    PATH=extractPath(inifile.c_str());

    Property robotOptions;
    fprintf(stderr, "Read robot description from %s\n", inifile.c_str());
    robotOptions.fromConfigFile(inifile.c_str());

    robotName=robotOptions.findGroup("GENERAL").find("name").asString().c_str();
    Bottle *reqParts=robotOptions.findGroup("GENERAL").find("parts").asList();
    if (reqParts==0)
    {
        // This is to maintain compatibility with old ini files
        std::cerr<<"Warning parsing " << inifile << " could not find a \"parts\" description\n";
        return false;
    }

    int nparts=reqParts->size();
    std::cout<<"Found " << nparts <<" parts"<<endl;

    //std::cout<<robotOptions.toString()<<endl;

    int n=0;
    for(n=0;n<nparts;n++)
    {
        std::string partid=reqParts->get(n).asString().c_str();
        std::cout<<"Processing "<<partid<<endl;

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

        cout << "Instantiating network " << netid.c_str() << "...";
        if (!instantiateNetwork(PATH, tmpProp, *netEntry))
        {
            cerr << endl << "ERROR: troubles instantiating " << netid.c_str() << endl;
        }
        else
            cout << "Network "<< netid.c_str() << " instantiated correctly"<< endl;

        netit++;
    }

    //now iterate through list of parts to see if all networks have been created correctly
    std::cout<<"Now I will go through the list of parts to create the wrappers"<<endl;

    RobotPartIt partit=parts.begin();

    while(partit!=parts.end())
    {
        RobotPartEntry *tmp=*partit;    

        //create the wrappers
        Property tmpProp;
        //copy parameters verbatim from relative section
        tmpProp.fromString(robotOptions.findGroup(tmp->id.c_str()).toString());

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

        ControlBoardWrapper2 *wp=new ControlBoardWrapper2;

        if (!wp->open(tmpProp))
        {
            delete wp;
            partit++;
            continue;
        }

        tmp->wrapper=wp;

        RobotNetworkIt netIt=tmp->networks.begin();

        //now attach all network devices
        PolyDriverList polylist;
        while(netIt!=tmp->networks.end())
        {
            RobotNetworkEntry *net=(*netIt);

            if (net->isValid())
            {
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

        tmp->wrapper->attachAll(polylist);

        std::cout<<endl;
        partit++;
    }

    fprintf(stderr, "RobotInterface::now opening inertial\n");
    if (robotOptions.check("INERTIAL")) 
    {
        fprintf(stderr, "RobotInterface:: inertial sensor is in the conf file\n");
        if (!instantiateInertial(robotOptions))
            fprintf(stderr, "RobotInterface::warning troubles instantiating inertial sensor\n");
    }
    else
        fprintf(stderr, "RobotInterface::no inertial sensor defined in the config file\n");

    // now go thourgh list of networks and create analog interface
    Bottle *analogNets=robotOptions.findGroup("ANALOG").find("networks").asList();
    if (analogNets)
    {
        int period;
        period=robotOptions.findGroup("ANALOG").find("period").asInt();
        if (period<5)
        {
            std::cerr<<"Sorry found invalid period value for ANALOG group, using default instead 20 [ms]"<<endl;
            period=20;
        }
        
        int nanalog=analogNets->size();

        int n=0;
        for(n=0;n<nanalog;n++)
        {
            std::string netid=analogNets->get(n).asString().c_str();
            std::cout<<"Instantiating analog device on "<< netid << endl;
        
            RobotNetworkEntry *selectedNet=networks.find(netid);
            if (selectedNet==0)
            {
                std::cerr<<"Sorry "<<netid<<" has not been instantiated, skipping"<<endl;
            }
            else
            {
                IGenericSensor *iTmp;
                selectedNet->driver.view(iTmp);
                selectedNet->iAnalog=iTmp;
                if (selectedNet->iAnalog)
                {
                    std::string name;
                    name+="/";
                    name+=robotName;
                    name+="/";
                    name+=selectedNet->id;
                    name+="/analog:o";

                    selectedNet->iAnalogServer=new ServerGenericSensor(name.c_str());
                    selectedNet->iAnalogServer->setRate(period);
                    selectedNet->iAnalogServer->attach(iTmp);
                    selectedNet->iAnalogServer->start();
                }
            }
         }
    }

    std::cout<<"Starting robot calibration!"<<endl;
    calibrate();
    std::cout<<"Finished robot calibration!"<<endl;

    return true;
}

bool RobotInterfaceRemap::instantiateNetwork(std::string &path, Property &robotOptions, RobotNetworkEntry &net)
{
    std::string file=robotOptions.find("file").asString().c_str();
    std::string fullFilename;
    fullFilename=path;
    if (fullFilename[fullFilename.length()-1]!='/')
        fullFilename.append("/",1);

    fullFilename.append(file.c_str(), file.length());

    Property deviceParameters;
    deviceParameters.fromConfigFile(fullFilename.c_str());

    Value &device=robotOptions.find("device");
    Value &subdevice=robotOptions.find("subdevice");
    Value &candevice=robotOptions.find("canbusdevice");

    deviceParameters.put("device", device);
    deviceParameters.put("subdevice", subdevice);
    deviceParameters.put("canbusdevice",candevice);

    ICUB_CAN_IDS *ids=can_ids.find(candevice.asString().c_str());

    int networkN=deviceParameters.findGroup("CAN").find("CanDeviceNum").asInt();
    if (ids)
    {
        string netid=deviceParameters.findGroup("CAN").find("NetworkId").asString().c_str();

        //        std::cerr<<"Netid:"<<netid<<endl;
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

    if (robotOptions.check("calibrator"))
    {
        Value &calibrator=robotOptions.find("calibrator");
        Property pTemp;
        pTemp.fromString(deviceParameters.toString());
        pTemp.put("device",calibrator.toString());
        net.calibrator.open(pTemp);
    }

    if (robotOptions.check("verbose")) 
        deviceParameters.put("verbose", 1);

    std::cout<<"Opening network " << networkN << " on device " << device.asString().c_str() << "... ";

    net.driver.open(deviceParameters);

    if (!net.driver.isValid())
    {
        std::cout<<"failed!"<<endl;
        return false;
    }
    std::cout<<"done!"<<endl;

    ICalibrator *icalibrator;
    IControlCalibration2 *icalib;
    //acquire calibrator int
    net.calibrator.view(icalibrator);
    //acquire calibration int
    net.driver.view(icalib);

    //save interface for later use
    net.iCalib=icalib;
    //set calibrator
    net.iCalib->setCalibrator(icalibrator);

    return true;
} 

bool RobotInterfaceRemap::instantiateInertial(Property &options)
{
    ACE_OS::fprintf(stderr, "Instantiating an INERTIAL device\n");

    const char *conf = ACE_OS::getenv("ICUB_ROOT");
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

bool RobotInterfaceRemap::finalize()
{ 
    RobotPartEntry *tmpPart;
    int n=parts.size();
    while(n--)
    {
        tmpPart=parts.back();
        // std::cerr<<"Detaching "<<tmpPart->id<<endl;

        // std::cerr<<"Done detaching " << tmpPart->id<<endl;
        if (tmpPart->wrapper!=0)
        {
            tmpPart->wrapper->detachAll();
            delete tmpPart->wrapper;
        }

        delete tmpPart;
        // std::cerr<<"Deleted object";
        parts.pop_back();
    }

    RobotNetworkEntry *tmpNet;
    n=networks.size();

    while(n--)
    {
        tmpNet=networks.back();
        tmpNet->close();
        std::cout<<"Closed network: " << tmpNet->id << endl;
        delete tmpNet;
        networks.pop_back();
    }

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
