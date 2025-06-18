/*
 * SPDX-FileCopyrightText: 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _ICUB_IRAWVALUESPUBLISHER_H_
#define _ICUB_IRAWVALUESPUBLISHER_H_

#include <iCub/rawValuesDataVectorsMap.h>
#include <iCub/rawValuesKeyMetadata.h>
#include <iCub/rawValuesKeyMetadataMap.h>
#include <iCub/RawValuesPublisherMetadata.h>

namespace iCub
{
    namespace debugLibrary
    {
        class IRawValuesPublisher
        {
            public:
                /* Fill the passed map with the raw values coming from the low level
                * @param map for raw values
                * @return true if the local raw value can be taken otherwise false 
                */
                virtual bool getRawDataMap(std::map<std::string, std::vector<std::int32_t>> &map) = 0;
                
                /* Fill the passed vector with the raw values coming from the low level 
                * given the specific key associated to a vector of raw data
                * @param key to be found in the local map of metadata
                * @param data vector of raw values to be filled
                * @return true if the key exists in the local metadata map and if the raw values can be found 
                * otherwise false 
                */
                virtual bool getRawData(std::string key, std::vector<std::int32_t> &data) = 0;
                
                /* Fill the passed vector with the keys saved in the local map of metadata
                * @param keys vector of strings which will be filled with the keys of the metadata map
                * @return true if there's no error in pushing back the keys in the vector
                */
                virtual bool getKeys(std::vector<std::string> &keys) = 0;

                /* @return the number of keys, i.e. entries, saved in the local map of metadata 
                */
                virtual int  getNumberOfKeys() = 0;

                /* Copy the local map of metadata to the one passed as argument
                * @param metamap map of metadata to be filled
                * @return true if the local map is not empty and if the copy ends correctly
                * otherwise return false
                */
                virtual bool getMetadataMap(rawValuesKeyMetadataMap &metamap) = 0;
                
                /* Given the key of a specific entry fill the metadata with the one
                * present at the requested entry of the local map
                * @param key for getting the specific entry in the local map
                * @param meta the metadata to be filled with the data and the requested entry
                * @return true if the find and copy ends fine otherwise false
                */
                virtual bool getKeyMetadata(std::string key, rawValuesKeyMetadata &meta) = 0;

                /* Fill the passed vector with the name of the axes saved 
                * at the requested key entry in the local map of metadata
                * @param key for getting the specific entry in the local map
                * @param axesNames vector of strings which will be filled with the axesNames of the metadata map
                * @return true if there's no error in pushing back the axesNames in the vector
                */
                virtual bool getAxesNames(std::string key, std::vector<std::string> &axesNames) = 0;
        };
    }
}

#endif //_ICUB_IRAWVALUESPUBLISHER_H_

