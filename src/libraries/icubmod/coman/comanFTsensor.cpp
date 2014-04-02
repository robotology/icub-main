// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
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

/// general purpose stuff.

#include <yarp/os/Time.h>
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <ace/config.h>
#include <ace/Log_Msg.h>

#include <string>
#include <iostream>
#include <string.h>


/// specific to this device driver.
#include "comanFTsensor.h"


#ifdef WIN32
#pragma warning(once:4355)
#endif

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    yWarning() << std::string(txt) << " not yet implemented for comanFTsensor\n";
    return false;
}

//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
static inline bool validate(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;  // size includes also the name of the parameter
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        fprintf(stderr, "%s not found\n", key1.c_str());
        return false;
    }

    if( tmp.size() != size)
    {
        fprintf(stderr, "%s incorrect number of entries\n", key1.c_str());
        return false;
    }

    out=tmp;

    return true;
}

bool comanFTsensor::fromConfig(yarp::os::Searchable &_config)
{
    Bottle xtmp;

    if(_config.check("device") )
    {
        setDeviceId(_config.find("device").asString() );
    }
    else
    {
        printf("No device name found.");
        std::cout << "Params are: " << _config.toString();
    }

    // Analog Sensor stuff
    Bottle config = _config.findGroup("ANALOG");
    if (!validate(config, xtmp, "numberOfSensors","Number of FT boards", 1))
    {
        yError() << "comanFTsensor: Boards number not found or not valid, quitting\n";
        _channels = 0;
        numberOfBoards = 0;
        return false;
    }
    else
    {    // get starts from 1 because the parameter name is counted as index 0
        numberOfBoards = xtmp.get(1).asInt();
    }

    FTmap = new int[numberOfBoards];
    scaleFactor=new double[numberOfBoards];

    std::cout << "\n\n  num of boards is " << numberOfBoards << "\n\n";
    if (!validate(config, xtmp, "newtonsToSensor","Scaling factor for FT sensors", numberOfBoards))
    {
        yError() << "FTSensor " << getDeviceId() << ": newtonsToSensor param was not found";
        return false;
    }
    else
    {
        for (int i=0; i<numberOfBoards; i++)
        	scaleFactor[i]=xtmp.get(1+i).asDouble();
    }

    if (!validate(config, xtmp, "FTmap","mapping of FTsensor unique Ids and a sequence number", numberOfBoards))
    {
        yWarning() << "comanFTsensor: FT sensor map not found or not correct, check your config file. Quitting";
        for(int i=0; i < numberOfBoards; i++)
        {
            FTmap[i] = 0;
            std::cout << "Ftmap[" << i << "] is " << FTmap[i];
        }
        return false;
    }
    else
    {    // get starts from 1 because the parameter name is counted as index 0
        for(int i=0; i<numberOfBoards; i++)
        {
        	FTmap[i] = xtmp.get(1+i).asInt();
            std::cout << "Ftmap[" << i << "] is " << FTmap[i];
        }
    }

    if (!validate(config, xtmp, "channels","number of channels per FT sensor", 1))
    {
        yWarning() <<  "comanFTsensor: Using default value = 6 (3 forces plus 3 torques)";
        _channels = 6;
    }
    else
    {    // get starts from 1 because the parameter name is counted as index 0
        _channels = xtmp.get(1).asInt();
    }

    if (!validate(config, xtmp, "UseCalibration","Calibration parameters if needed", 1))
    {
        yWarning() <<  "comanFTsensor: Using default value = 0 (Don't use calibration)";
        _useCalibration = 0;
    }
    else
    {    // get starts from 1 because the parameter name is counted as index 0
        _useCalibration = xtmp.get(1).asInt();
    }
    return true;
};

bool comanFTsensor::close()
{
    return _comanHandler->deInstance();
    delete [] scaleFactor;
    delete [] FTmap;
}

comanFTsensor::comanFTsensor()
{
    yTrace();
    _boards_ctrl = NULL;
    _useCalibration=0;
    _channels=0;
    scaleFactor=0;

    status=IAnalogSensor::AS_OK;
}

comanFTsensor::~comanFTsensor()
{
    if(FTmap != NULL)
        delete[] FTmap;
    if (scaleFactor != NULL)
        delete[] scaleFactor;
}

FtBoard * comanFTsensor::getFTpointer(int j)
{
    return _fts[j+1];
}

bool comanFTsensor::open(yarp::os::Searchable &config)
{
    _comanHandler = comanDevicesHandler::instance();

    if(_comanHandler == NULL)
    {
        yError() << "unable to create a new Coman Device Handler class!";
        return false;
    }

    if(!_comanHandler->open(config))
    {
        yError() << "Unable to initialize Coman Device Handler class... probably np boards were found. Check log.";
        return false;
    }
    _boards_ctrl = _comanHandler->getBoard_ctrl_p();

    if(_boards_ctrl == NULL)
    {
        yError() << "unable to create a new Boards_ctrl class!";
        return false;
    }

    Property prop;
    std::string str=config.toString().c_str();
    yTrace() << str;

    if(!fromConfig(config))
        return false;

    prop.fromString(str.c_str());


    // TODO fix this!
#warning "<><> TODO: This is a copy of the mcs map. Verify that things will never change after this copy or use a pointer (better) <><>"
    _fts = _boards_ctrl->get_fts_map();
    return true;
}


/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/
int comanFTsensor::read(yarp::sig::Vector &out)
{
    // This method gives data to the analogServer

    mutex.wait();
    status = AS_OK;

    // TODO
    int sensor_idx = 0;
    out.resize(numberOfBoards * _channels);

    for(int idx = 0; idx < numberOfBoards; idx++)
    {
        FtBoard *ftSensor = NULL;
        sensor_idx = FTmap[idx];
        ftSensor = _fts[sensor_idx]; // sensor_ids = boardId
        if( NULL == ftSensor)
        {
            //yError() << "Trying to read data from a non-existing FT sensor " << sensor_idx << ", check your config file";
            for(int tmp_idx = 0; tmp_idx < _channels; tmp_idx++)
                out[idx*_channels + tmp_idx] = 0;

//          Debug purpose, this way the output will be the boardId, to check if it is
//          trying to read from the right board
            out[idx*_channels + 0] = (double) sensor_idx;
            out[idx*_channels + 1] = (double) sensor_idx;
            out[idx*_channels + 2] = (double) sensor_idx;
            out[idx*_channels + 3] = (double) sensor_idx;
            out[idx*_channels + 4] = (double) sensor_idx;
            out[idx*_channels + 5] = (double) sensor_idx;
            continue;
        }


        ts_bc_data_t bc_data;
        ft_bc_data_t &data = bc_data.raw_bc_data.ft_bc_data;

        ftSensor->get_bc_data(bc_data);

        // ci sono 6 valori da leggere per ogni FT, per ora!!
        out[idx*_channels + 0] = (double)data.fx / scaleFactor[idx];
        out[idx*_channels + 1] = (double)data.fy / scaleFactor[idx];
        out[idx*_channels + 2] = (double)data.fz / scaleFactor[idx];
        out[idx*_channels + 3] = (double)data.tx / scaleFactor[idx];
        out[idx*_channels + 4] = (double)data.ty / scaleFactor[idx];
        out[idx*_channels + 5] = (double)data.tz / scaleFactor[idx];
    }
    mutex.post();

    return status;
}

int comanFTsensor::getState(int ch)
{
    printf("getstate\n");
    return AS_OK;
}

int comanFTsensor::getChannels()
{
    yTrace();
//     McBoard *joint_p = getFTpointer(j);  // TODO su quale sensore?
//     int8_t channels;
//     if( joint_p->getItem(GET_ANALOG_INPUTS,   NULL, 0, REPLY_ANALOG_INPUTS, &channels, sizeof(channels)) )
//         yError() << "Error while getting number of channels";
//     int ret = (int) channels;
//     return ret;
//    return 0;

     return _channels;
}

int comanFTsensor::calibrateSensor()
{
    return AS_OK;
}

int comanFTsensor::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int comanFTsensor::calibrateChannel(int ch)
{
    return AS_OK;
}

int comanFTsensor::calibrateChannel(int ch, double v)
{
    return AS_OK;
}
