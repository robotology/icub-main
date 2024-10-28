// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2024 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Jacopo Losi, Valentina Gaggero
 * email:   jacopo.losi@iit.it, valentina.gaggero@iit.it
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

// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _ICUB_FAKERAWVALUESPUBLISHER_H_
#define _ICUB_FAKERAWVALUESPUBLISHER_H_

// system std include
#include <map>

// yarp includes
#include <yarp/dev/DeviceDriver.h>

#include <iCub/IRawValuesPublisher.h>

// ParamParser generated classes
#include "FakeRawValuesPublisher_ParamsParser.h"

using namespace iCub;
class FakeRawValuesPublisher :
    public iCub::debugLibrary::IRawValuesPublisher,
    public yarp::dev::DeviceDriver,
    public FakeRawValuesPublisher_ParamsParser
{
    public:
        FakeRawValuesPublisher();
        ~FakeRawValuesPublisher()  = default;

        // IRawValuesPublisher
        virtual bool getRawDataMap(std::map<std::string, std::vector<std::int32_t>> &map) override;
        virtual bool getRawData(std::string key, std::vector<std::int32_t> &data) override;
        virtual bool getKeys(std::vector<std::string> &keys) override;
        virtual int  getNumberOfKeys() override;
        virtual bool getMetadataMap(rawValuesKeyMetadataMap &metamap) override;
        virtual bool getKeyMetadata(std::string key, rawValuesKeyMetadata &meta) override;


    private:
        /* Internal methods */
        bool getRawData_core(std::string key, std::vector<std::int32_t> &data);
        
        /* DevideDriver methods */
        bool open(yarp::os::Searchable& config) override;
        bool close() override;

        int m_numberOfJomos = 0;
        int m_sawtoothThreshold = 0;
        std::string m_rawValuesVectorTag = "";
        std::vector<std::int32_t> m_rawDataAuxVector;
        int m_sawthootTestVal = 0;

        std::map<std::string, rawValuesKeyMetadata> m_rawValuesMetadataMap;
};

#endif //_ICUB_FAKERAWVALUESPUBLISHER_H_

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------
