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

    scaleFactor=new double[numberOfBoards];
    int i=0;
    for (i=0; i<numberOfBoards; i++) scaleFactor[i]=1;

    FTmap = new int[numberOfBoards];

    if (!validate(config, xtmp, "FTmap","mapping of FTsensor unique Ids and a sequence number", numberOfBoards))
    {
        yWarning() << "comanFTsensor: FT sensor map not found or not correct, check your config file. Quitting";
        for(int i=0; i < numberOfBoards; i++)
            FTmap[i] = 0;
        return false;
    }
    else
    {    // get starts from 1 because the parameter name is counted as index 0
        for(int i=0; i<numberOfBoards; i++)  
        FTmap[i] = xtmp.get(1+i).asInt();
        yDebug() << "FTmap is " << FTmap;
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


comanFTsensor::comanFTsensor()
{
    yTrace();
    boards_ctrl = NULL;
    _useCalibration=0;
    _channels=0;
    scaleFactor=0;

    status=IAnalogSensor::AS_OK;
}

comanFTsensor::~comanFTsensor()
{
    if(FTmap != NULL)
        delete FTmap;
    if (scaleFactor != NULL)
        delete scaleFactor;
}

bool comanFTsensor::open(yarp::os::Searchable &config)
{
    boards_ctrl = Boards_ctrl::instance();
    Property prop;
    std::string str=config.toString().c_str();
    yTrace() << str;

    if(!fromConfig(config))
        return false;

    prop.fromString(str.c_str());

    boards_ctrl = Boards_ctrl::instance();
    if(boards_ctrl == NULL)
    {
        yError() << "unable to open Boards_ctrl class!";
        return false;
    }
    boards_ctrl->open(config);
    // TODO fix this!
#warning "<><> TODO: This is a copy of the mcs map. Verify that things will never change after this copy or use a pointer (better) <><>"
    _fts = boards_ctrl->get_fts_map();
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
    // Che TIPO di dato è (int, double...)??? Chi lo decide???
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
            continue;
            //return AS_ERROR;
        }

        ft_bc_data_t data;

        ftSensor->get_bc_data(&data);


        // ci sono 6 valori da leggere per ogni FT, per ora!!
        out[idx*_channels + 0] = data.fx;
        out[idx*_channels + 1] = data.fy;
        out[idx*_channels + 2] = data.fz;
        out[idx*_channels + 3] = data.tx;
        out[idx*_channels + 4] = data.ty;
        out[idx*_channels + 5] = data.tz;
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
