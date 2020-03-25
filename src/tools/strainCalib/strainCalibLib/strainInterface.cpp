// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/



#include "strainInterface.h"

//#include "driver.h"
//#include "downloader.h"
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <stdlib.h> //added for abs
#include <string.h>




using namespace yarp::dev;
using namespace yarp::os;
using namespace std;


//strainInterface::strainInterface()
//{
//    downloader.set_verbose(false);
//    config.load_default();
//}

//strainInterface::~strainInterface()
//{

//}


bool strainInterface::open(const Config &cfg)
{
    if(true == opened)
    {
        close();
    }

    config = cfg;

    if(true == downloader.connected)
    {
        downloader.stopdriver();
        yError() << "strainInterface::open(): failure because ... Driver Stopped because it was running .... very strange i should have called a strainInterface::close()";
        return false;
    }


    yarp::os::Property params;

    params.put("device", config.get_networkstring().c_str());
    if(Network::ETH == config.network)
    {
        unsigned int localAddr = (10<<24)|(0<<16)|(1<<8)|104;   // 10.0.1.104
        unsigned int remoteAddr = (10<<24)|(0<<16)|(1<<8)|1;    // 10.0.1.1

        params.put("local", int( localAddr));
        params.put("remote",int(remoteAddr));
        params.put("canid", config.get_canbus());

    }
    else
    {
        int networkId = 0;
        params.put("canDeviceNum", networkId);
        params.put("canTxQueue", 64);
        params.put("canRxQueue", 64);
        params.put("canTxTimeout", 2000);
        params.put("canRxTimeout", 2000);
    }

    //try to connect to the driver
    int ret = downloader.initdriver(params);

    if (0 != ret)
    {
        if(-2 == ret)
        {
            yDebug("Init ETH driver: The ETH board has just jumped to eUpdater\n Connect again");
        }
        else
        {
            yError("Init driver failed: Hardware busy or not connected?!");
        }
        return false;
    }

    yDebug("Driver Connected");

    opened = true;

    return true;
}


bool strainInterface::close()
{
    downloader.strain_stop_sampling(config.get_canbus(), config.get_canaddress());
    downloader.stopdriver();
    config.load_default();
    opened = false;

    return true;
}


bool strainInterface::print(const vector<cDownloader::strain_value_t> &values, FILE *fp)
{
    if(false == opened)
    {
        yError() << "strainInterface::print(): you must call strainInterface::open() first";
        return false;
    }

    // if fp is NULL, i dont print to file. the format is the one accepted for calib of strain

    // printf("Acquired %d samples. Ready!	\n", static_cast<int>(values.size()));

    for(size_t n=0; n<values.size(); n++)
    {
        unsigned short unsigned_gaugeData[6] = {0};
        signed short signed_gaugeData[6] = {0};

        for(size_t c=0; c<6; c++)
        {
            unsigned_gaugeData[c] = values[values.size()-1-n].channel[c];
            signed_gaugeData[c] = unsigned_gaugeData[c]-0x7fff;
        }

        if(NULL != fp)
        {
            fprintf(fp,"%d %d %d %d %d %d %d\n",static_cast<int>(n),signed_gaugeData[0],signed_gaugeData[1],signed_gaugeData[2],signed_gaugeData[3],signed_gaugeData[4],signed_gaugeData[5]);
        }
        yDebug("samples:%3d [0]:%+6d [1]:%+6d [2]:%+6d [3]:%+6d [4]:%+6d [5]:%+6d\r\n",static_cast<int>(n),signed_gaugeData[0],signed_gaugeData[1],signed_gaugeData[2],signed_gaugeData[3],signed_gaugeData[4],signed_gaugeData[5]);

    }

    return true;
}


bool strainInterface::get(const unsigned int number, vector<cDownloader::strain_value_t> &values)
{
    if(false == opened)
    {
        yError() << "strainInterface::get(): you must call strainInterface::open() first";
        return false;
    }

    double t0 = yarp::os::SystemClock::nowSystem();

    yDebug() << "strainInterface::get(): is acquiring" << number << "from the 6 channels. Please wait ...";

    const bool calibmode = false;
    downloader.strain_acquire_start(config.get_canbus(), config.get_canaddress(), config.get_txrate(), calibmode);
    downloader.strain_acquire_get(config.get_canbus(), config.get_canaddress(), values, number);
    //yarp::os::SystemClock::delaySystem(0.100); i used it to test the flush operation of strain_acquire_stop()....
    downloader.strain_acquire_stop(config.get_canbus(), config.get_canaddress());

    double t1 = yarp::os::SystemClock::nowSystem();


    yDebug() << "strainInterface::get() has succesfully acquired" << values.size() << "values of the 6 channels in" << (t1-t0) << "seconds";
    yDebug() << "the values are:";
    for(int i=0; i<values.size(); i++)
    {
        yDebug() << "#" << i+1 << "=" << values[i].channel[0] << values[i].channel[1] << values[i].channel[2] << values[i].channel[3] <<
                                         values[i].channel[4] << values[i].channel[5] << values[i].valid;
    }

    return true;
}


// end of file


