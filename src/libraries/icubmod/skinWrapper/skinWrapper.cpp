/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "skinWrapper.h"

skinWrapper::skinWrapper()
{
    printf("\n\n\nskinWrapper constructor\n\n\n");
    analogServer=0;
    analog=0;
}

skinWrapper::~skinWrapper() { }

void skinWrapper::calibrate()
{
    if (analog)
    {
        analog->calibrateSensor();
    }
}

// convertire in un wrapper vero con metodo attach!!!! 
bool skinWrapper::open(yarp::os::Searchable &inputParams)
{
    printf("skinWrapper param 1 (params) = %s\n", inputParams.toString().c_str());

    string str=inputParams.toString().c_str();
    
    Property params;
    params.fromString(inputParams.toString().c_str());
    bool correct=true;
    correct=correct&&params.check("device");
    if(params.check("robot") )
    	printf("'robot' param found %s\n", params.find("robot").asString().c_str() );

    if(params.check("name") )
    	printf("'name' param found %s\n", params.find("robot").asString().c_str() );

    if(params.check("robotName") )
    	printf("'robotName' param found %s\n", params.find("robot").asString().c_str() );


    printf("returning\n");
    fflush(stdout);
    return true;

    correct=correct&&params.check("name");
    //correct=correct&&params.check("FeatId");
    //correct=correct&&params.check("canbusdevice");
    //correct=correct&&params.check("ports");        // list of the ports where to send the tactile data
    //
    //    Bottle xtmp, xtmp2;
    //
    //    bool correct=true;
    //
    //    correct=correct&&robotOptions.check("GENERAL");
    //
    //    if(correct)
    //        xtmp = Bottle(robotOptions.findGroup("GENERAL"));
    //
    //    correct=correct&&xtmp.check("PC104IpAddress");
    //
    //    if(correct)
    //        xtmp2 = xtmp.findGroup("PC104IpAddress");
    //
    //    partOptions.put("PC104IpAddress", xtmp2.get(1).asString().c_str());

    if (!correct)
    {
      yError() << "Some parameters are missing!!\n";
        return false;
    }

    int period=20;
    if (params.check("period"))
    {
        period=params.find("period").asInt();
    }
    else
    {
        std::cout<<"Warning: part "<<id<<" using default period ("<<period<<")\n";
    }

    // Open the device
    std::string devicename=params.find("device").asString().c_str();
    params.put("device", devicename.c_str());

    std::string canbusdevice=params.find("canbusdevice").asString().c_str();
    params.put("canbusdevice", canbusdevice.c_str());

    driver.open(params);
    if (!driver.isValid())
        return false;

    driver.view(analog);

    RateThread *thread;
    driver.view(thread);

    if (!analog)
    {
        std::cerr<<"Error: part "<<id<<" device " << devicename << " does not implement analog interface"<<endl;
        driver.close();
        return false;
    }

    // Read the list of ports
    std::string robotName=params.find("robot").asString().c_str();
    std::string root_name;
    root_name+="/";
    root_name+=robotName;
    root_name+="/skin/";

    std::vector<AnalogPortEntry> skinPorts;
    if(!params.check("ports")){
        // if there is no "ports" section take the name of the "skin" group as the only port name
        skinPorts.resize( (size_t)1);
        skinPorts[0].offset = 0;
        skinPorts[0].length = -1;
        skinPorts[0].port_name = root_name + this->id;
    }
    else{
        Bottle *ports=params.find("ports").asList();

        if (!params.check("total_taxels", "number of taxels of the part"))
            return false;
        int total_taxels=params.find("total_taxels").asInt();
        int nports=ports->size();
        int totalT = 0;
        skinPorts.resize(nports);

        for(int k=0;k<ports->size();k++)
        {
            Bottle parameters=params.findGroup(ports->get(k).asString().c_str());

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

    analogServer = new AnalogServer(skinPorts);
    analogServer->setRate(period);
    analogServer->attach(analog);
    analogServer->start();

    return true;
}

bool skinWrapper::close()
{
    std::cout<<"Closing skin part "<< id << endl;
    if (analogServer)
    {
        analogServer->stop();
        delete analogServer;
    }
    if (analog)
        analog=0;

    driver.close();
    return true;
}


//skinWrapper *SkinParts::find(const string &pName)
//{
//    SkinPartsIt it=begin();
//    for(;it!=end(); it++)
//    {
//        if ((*it)->id==pName)
//        {
//            return (*it);
//        }
//    }
//
//    return 0;
//}
//
//
//
//void SkinParts::close()
//{
//    // AC_YARP_INFO(Logger::get(),"SkinParts::close()", Logger::get().log_files.f3);
//    SkinPartsIt it;
//    for(it=this->begin(); it!=this->end(); it++)
//    {
//        (*it)->close();
//    }
//}
