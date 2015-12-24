// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2014 RobotCub Consortium
 * Author: Alberto Cardellino
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <DebugInterfaceWrapper.h>
#include <RpcMsgHandler.h>
#include <WrappedDevice.h>
#include <iostream>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace iCub::Wrapper::iDebug;

inline void appendTimeStamp(Bottle &bot, Stamp &st)
{
    int count=st.getCount();
    double time=st.getTime();
    bot.addVocab(VOCAB_TIMESTAMP);
    bot.addInt(count);
    bot.addDouble(time);
}

// d for the driver factory.
DriverCreator *createDebugInterfaceWrapper()
{
    return new DriverCreatorOf<DebugInterfaceWrapper>("debugWrapper",
                                                      "debugWrapper",
                                                      "DebugInterfaceWrapper");
}

DebugInterfaceWrapper::DebugInterfaceWrapper() : rpcMsgHandler(this)
{
    subDeviceOwned = NULL;
    controlledJoints = 0;
    _verbose = false;
}

bool DebugInterfaceWrapper::open(Searchable& config)
{
    Property prop;
    prop.fromString(config.toString().c_str());

//    std::cout << "DebugInterface Wrapper params are \n\t" << config.toString().c_str();
    _verbose = (prop.check("verbose","if present, give detailed output"));
    if (_verbose)
        cout<<"running with verbose output\n";

//    std::cout<<"Using DebugInterfaceWrapper\n";
    if (prop.check("subdevice", "Do I need one?"))
    {
        std::cout << "\nFound " << prop.find("subdevice").asString() << " subdevice, opening it\n";
        if(! openAndAttachSubDevice(prop))
        {
            printf("Error while opening subdevice\n");
            return false;
        }
    }
    else
    {
        if(!openDeferredAttach(prop))
            return false;
    }


    std::string rootName = prop.check("rootName",Value("/"), "starting '/' if needed.").asString().c_str();
    partName = prop.check("name",Value("controlboard"), "prefix for port names").asString().c_str();

    rootName+= (partName);

    // open the port
    if(!rpc_p.open((rootName+"/debug/rpc:i").c_str()) )
    {
        std::cout << "Error, DebugInterfaceWrapper was not able to open port with name " << (rootName+"/debug/rpc:i").c_str() << std::endl;
    }
    rpc_buffer.attach(rpc_p);                   // attach a buffer ti the port
    rpcMsgHandler.attach(rpc_buffer);           // set a callack class to the port (to do only after succesfull attach?)

    return true;
}


// Default usage
// Open the wrapper only, the attach method needs to be called before using it
bool DebugInterfaceWrapper::openDeferredAttach(Property& prop)
{
    int base_device = 0;
    int top_device  = 0;
    if (!prop.check("networks", "list of networks merged by this wrapper"))
        return false;

    Bottle *nets=prop.find("networks").asList();
    if(nets==0)
    {
        cerr<<"Error parsing parameters: \"networks\" should be followed by a list\n";
        return false;
    }

    if (!prop.check("joints", "number of joints of the part"))
        return false;

    controlledJoints = prop.find("joints").asInt();

    int nsubdevices = nets->size();

    device.lut.resize(controlledJoints);
    device.subdevices.resize(nsubdevices);

    // configure the devices
    int totalJ = 0;
    for( int k = 0; k < nets->size(); k++)
    {
        Bottle parameters;
        int wBase;
        int wTop;

        parameters = prop.findGroup(nets->get(k).asString().c_str());

        // cout<<"Net is "<< nets->get(k).asString().c_str()<<"\n";
        //cout<<parameters.toString().c_str();

        if(parameters.size() == 2)
        {
            Bottle *bot = parameters.get(1).asList();
            Bottle tmpBot;
            if(bot == NULL)
            {
                // probably data are not passed in the correct way, try to read them as a string.
                ConstString bString(parameters.get(1).asString());
                tmpBot.fromString(bString);

                if(tmpBot.size() != 4)
                {
                    cerr << "Error: check network parameters in part description" << endl;
                    cerr << "--> I was expecting "<<nets->get(k).asString().c_str() << " followed by a list of four integers in parenthesis" << endl;
                    cerr << "Got: "<< parameters.toString().c_str() << "\n";
                    return false;
                }
                else
                {
                    bot = &tmpBot;
                }
            }

            // If I came here, bot is correct
            wBase = bot->get(0).asInt();
            wTop  = bot->get(1).asInt();
            base_device = bot->get(2).asInt();
            top_device  = bot->get(3).asInt();
        }
        else if (parameters.size()==5)
        {
            // cout<<"Parameter networks use deprecated syntax\n";
            wBase=parameters.get(1).asInt();
            wTop=parameters.get(2).asInt();
            base_device = parameters.get(3).asInt();
            top_device  = parameters.get(4).asInt();
        }
        else
        {
            cerr << "Error: check network parameters in part description" << endl;
            cerr << "--> I was expecting " << nets->get(k).asString().c_str() << " followed by a list of four integers in parenthesis" << endl;
            cerr << "Got: "<< parameters.toString().c_str() << "\n";
            return false;
        }

        SubDevice *tmpDevice = device.getSubdevice(k);

        int axes = top_device - base_device + 1;
        if (!tmpDevice->configure(base_device, top_device, axes, nets->get(k).asString().c_str()) )
        {
            cerr << "configure of subdevice ret false" << endl;
            return false;
        }

        for(int wj = wBase, devj=base_device; wj <= wTop; wj++, devj++)
        {
            device.lut[wj].deviceEntry = k;
            device.lut[wj].deviceJoint = devj;
        }

        totalJ += axes;
    }

    if (totalJ != controlledJoints)
    {
        cerr<<"Error total number of mapped joints ("<< totalJ <<") does not correspond to part joints (" << controlledJoints << ")" << endl;
        return false;
    }

    prop.put("rootName", "/");
    return true;
}

// For the simulator, if a subdevice parameter is given to the wrapper, it will
// open it and and attach to immediatly.
bool DebugInterfaceWrapper::openAndAttachSubDevice(Property& prop)
{
    Property p;
    int base_device = 0;
    int top_device  = 0;
    subDeviceOwned = new PolyDriver;
    p.fromString(prop.toString().c_str());

    p.setMonitor(prop.getMonitor(), "subdevice"); // pass on any monitoring


    p.unput("device");
    p.put("device",prop.find("subdevice").asString());

    // if error occour during open, quit here.
    printf("opening DebugInterfaceWrapper subdevice\n");
    subDeviceOwned->open(p);

    if (!subDeviceOwned->isValid())
    {
        printf("opening DebugInterfaceWrapper subdevice... FAILED\n");
        return false;
    }

    printf("opening DebugInterfaceWrapper subdevice... done\n");

    // getting parameters in simStyle
    if (!p.check("GENERAL","section for general motor control parameters"))
    {
        fprintf(stderr, "Cannot understand configuration parameters\n");
        return false;
    }

    controlledJoints = p.findGroup("GENERAL").check("TotalJoints",Value(1), "Number of total joints").asInt();
    device.lut.resize(controlledJoints);
    device.subdevices.resize(1);

    printf("Attaching DebugInterfaceWrapper to subdevice\n");

    // configure the device
    base_device = 0;
    top_device  = controlledJoints-1;

    SubDevice *tmpDevice=device.getSubdevice(0);

    std::string subDevName ((partName + "_" + prop.find("subdevice").asString().c_str()));
    if (!tmpDevice->configure(base_device, top_device, controlledJoints, subDevName) )
    {
        cerr<<"configure of subdevice ret false"<<endl;
        return false;
    }

    for(int j=0; j<controlledJoints; j++)
    {
        device.lut[j].deviceEntry = 0;
        device.lut[j].deviceJoint = j;
    }

    if (!device.subdevices[0].attach(subDeviceOwned, subDevName))
        return false;

    // initialization.
    rpcMsgHandler.initialize();

    prop.put("rootName", "");
    return true;
}


bool DebugInterfaceWrapper::attachAll(const PolyDriverList &polylist)
{
    for(int p=0;p<polylist.size();p++)
    {
        // find appropriate entry in list of subdevices and attach to it
        unsigned int k = 0;
        for(k=0; k<device.subdevices.size(); k++)
        {
            std::string tmpKey=polylist[p]->key.c_str();
            if (device.subdevices[k].id_subDev == tmpKey)
            {
                if (!device.subdevices[k].attach(polylist[p]->poly, tmpKey))
                    return false;
            }
        }
    }

    //check if all devices are attached to the driver
    bool ready=true;
    for(unsigned int k=0; k<device.subdevices.size(); k++)
    {
        if (!device.subdevices[k].isAttached())
        {
            ready=false;
        }
    }

    if (!ready)
        return false;

    // initialization.
    rpcMsgHandler.initialize();

    return true;
}



bool DebugInterfaceWrapper::detachAll()
{
    int devices=device.subdevices.size();
    for(int k=0;k<devices;k++)
        device.getSubdevice(k)->detach();

    return true;
}

bool DebugInterfaceWrapper::close()
{
    // close the port connection here!
    rpc_p.close();
    if(subDeviceOwned != NULL)
    {
        delete subDeviceOwned;
        subDeviceOwned = NULL;
    }
    return true;
}


// Here for help the RpcMsgHandler
bool DebugInterfaceWrapper::getAxes(int &val)
{
    val = controlledJoints;
    return true;
}

// Debug Interface
bool DebugInterfaceWrapper::getParameter(int j, unsigned int type, double *t)
{
    int subIndex = device.lut[j].deviceEntry;
    int subJoint = device.lut[j].deviceJoint;

    SubDevice *p=device.getSubdevice(subIndex);
    if (!p)
        return false;

    return p->iDbg_subDev->getParameter(subJoint, type, t);
}

bool DebugInterfaceWrapper::setParameter(int j, unsigned int type, double t)
{
    int subIndex = device.lut[j].deviceEntry;
    int subJoint = device.lut[j].deviceJoint;

    SubDevice *p=device.getSubdevice(subIndex);
    if (!p)
        return false;

    return p->iDbg_subDev->setParameter(subJoint, type, t);
}

bool DebugInterfaceWrapper::getDebugParameter(int j, unsigned int index, double *t)
{
    int subIndex = device.lut[j].deviceEntry;
    int subJoint = device.lut[j].deviceJoint;

    SubDevice *p=device.getSubdevice(subIndex);
    if (!p)
        return false;

    return p->iDbg_subDev->getDebugParameter(subJoint, index, t);
}

bool DebugInterfaceWrapper::setDebugParameter(int j, unsigned int index, double t)
{
    int subIndex = device.lut[j].deviceEntry;
    int subJoint = device.lut[j].deviceJoint;

    SubDevice *p=device.getSubdevice(subIndex);
    if (!p)
        return false;

    return p->iDbg_subDev->setDebugParameter(subJoint, index, t);
}
