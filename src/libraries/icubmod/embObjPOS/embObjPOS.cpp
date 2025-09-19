
/*
 * Copyright (C) 2020 iCub Tech - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
*/


// general purpose stuff.
#include <string>
#include <iostream>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Yarp Includes
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/NetType.h>

// external libraries includes
#include <ace/config.h>
#include <ace/Log_Msg.h>


// specific to this device driver.
#include <embObjPOS.h>
#include <ethManager.h>
#include "EoAnalogSensors.h"
#include "EOnv_hid.h"
#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolAS.h"


#ifdef WIN32
#pragma warning(once:4355)
#endif

namespace {
    YARP_LOG_COMPONENT(EMBOBJPOS, "yarp.dev.embObjPOS")
}

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;



bool embObjPOS::fromConfig(yarp::os::Searchable &config, servConfigPOS_t &serviceConfig)
{
    ServiceParser parser;
    if(false == parser.parseService(config, serviceConfig))
    {
        yError() << m_PDdevice.getBoardInfo() << ": missing some configuration parameter. Check logs and your config file.";
        return false;
    }

    return true;
}


embObjPOS::embObjPOS(): m_PDdevice("embObjPOS") 
{ 
}


embObjPOS::~embObjPOS()
{
    close();
}


bool embObjPOS::initialised()
{
    return m_PDdevice.isOpen();
}


bool embObjPOS::open(yarp::os::Searchable &config)
{
    // 1) prepare eth service verifing if the eth manager is available and parsing info about the eth board.

    yCInfo(EMBOBJPOS) << "embObjPOS::open(): preparing ETH resource";

    if(! m_PDdevice.prerareEthService(config, this))
        return false;

    yCInfo(EMBOBJPOS) << "embObjPOS::open(): browsing xml files which describe the service";

    // 2) read stuff from config file
    serviceConfig = {};
    if(!fromConfig(config, serviceConfig))
    {
        yCError(EMBOBJPOS) << "embObjPOS missing some configuration parameter. Check logs and your config file.";
        return false;
    }

    // 3) prepare data vector
    {
        // for now resize to max number of pos sensors 
        //--> then we will resize to the number of enabled sensors 
        //--> we can eventually use here reserve to max number and resize to enabled number
        m_data.resize(eOas_pos_data_maxnumber, 0.0); 
    }


    yCInfo(EMBOBJPOS) << "embObjPOS::open(): verify the presence of the board and if its protocol version is correct";

    // 4) verify analog sensor protocol and then verify-Activate the POS service
    if(!m_PDdevice.res->verifyEPprotocol(eoprot_endpoint_analogsensors))
    {
        cleanup();
        return false;
    }


    yCInfo(EMBOBJPOS) << "embObjPOS::open(): verify and activate the POS service";

    const eOmn_serv_parameter_t* servparam = &serviceConfig.ethservice;

    if(!m_PDdevice.res->serviceVerifyActivate(eomn_serv_category_pos, servparam, 5.0))
    {
        yCError(EMBOBJPOS) << m_PDdevice.getBoardInfo() << "open() has an error in call of ethResources::serviceVerifyActivate()";
        cleanup();
        return false;
    }


    yCInfo(EMBOBJPOS) << "embObjPOS::open(): configure the POS service";

    if(false == sendConfig2boards(serviceConfig))
    {
        cleanup();
        return false;
    }

    yCInfo(EMBOBJPOS) << "embObjPOS::open(): impose the network variable which the ETH bord must stream up";

    // Set variable to be signaled
    if(false == initRegulars())
    {
        cleanup();
        return false;
    }


    yCInfo(EMBOBJPOS) << "embObjPOS::open(): start the POS service";

    if(!m_PDdevice.res->serviceStart(eomn_serv_category_pos))
    {
        yCError(EMBOBJPOS) << m_PDdevice.getBoardInfo() << "open() fails to start as service.... cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(m_PDdevice.isVerbose())
        {
            yCDebug(EMBOBJPOS) << m_PDdevice.getBoardInfo() << "open() correctly starts service";
        }
    }

    yCInfo(EMBOBJPOS) << "embObjPOS::open(): start streaming of POS data";

    sendStart2boards();

    m_PDdevice.setOpen(true);

    return true;
}


bool embObjPOS::sendConfig2boards(servConfigPOS_t &serviceConfig)
{
    eOprotID32_t id32 = eo_prot_ID32dummy;

    eOas_pos_config_t cfg;
    cfg.datarate  = serviceConfig.acquisitionrate;
    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_pos, 0, eoprot_tag_as_pos_config);

    if(false == m_PDdevice.res->setcheckRemoteValue(id32, &cfg, 10, 0.010, 0.050))
    {
        yCError(EMBOBJPOS) << m_PDdevice.getBoardInfo() << "FATAL error in sendConfig2boards() while try to configure datarate=" << cfg.datarate;
        return false;
    }

    if(m_PDdevice.isVerbose())
    {
        yCDebug(EMBOBJPOS) << m_PDdevice.getBoardInfo() << ": sendConfig2boards() correctly configured boards with datarate=" << cfg.datarate;
    }

    return true;

}

bool embObjPOS::sendStart2boards(void)
{
    eOprotID32_t id32 = eo_prot_ID32dummy;

    uint8_t enable=1;

    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_pos, 0, eoprot_tag_as_pos_cmmnds_enable);

    if(false == m_PDdevice.res->setcheckRemoteValue(id32, &enable, 10, 0.010, 0.050))
    {
        yCError(EMBOBJPOS) << m_PDdevice.getBoardInfo() << "FATAL error in sendStart2boards() while try to enable the boards transmission";
        return false;
    }

    if(m_PDdevice.isVerbose())
    {
        yCDebug(EMBOBJPOS) << m_PDdevice.getBoardInfo() << ": sendStart2boards() correctly enabled the boards transmission";
    }

    return true;

}


bool embObjPOS::initRegulars(void)
{
    // configure regular rops

    vector<eOprotID32_t> id32v(0);
    eOprotID32_t id32 = eo_prot_ID32dummy;

    // we need to choose the id32 to put inside the vector

    id32 = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_pos, 0, eoprot_tag_as_pos_status);;

    // and put it inside vector

    id32v.push_back(id32);

    // now we send the vector

    if(false == m_PDdevice.res->serviceSetRegulars(eomn_serv_category_pos, id32v))
    {
        yCError(EMBOBJPOS) << m_PDdevice.getBoardInfo() << "initRegulars() fails to add its variables to regulars: cannot proceed any further";
        return false;
    }

    if(m_PDdevice.isVerbose())
    {
        yCDebug(EMBOBJPOS) << m_PDdevice.getBoardInfo() << "initRegulars() added" << id32v.size() << "regular rops ";
        char nvinfo[128];
        for (size_t r = 0; r<id32v.size(); r++)
        {
            uint32_t item = id32v.at(r);
            eoprot_ID2information(item, nvinfo, sizeof(nvinfo));
            yCDebug(EMBOBJPOS) << "\t it added regular rop for" << nvinfo;
        }
    }
    
    return true;
}

void embObjPOS::helper_remapperFromSensorToDataIndex(const uint32_t sensorId, uint32_t &dataIndex) const
{
    /**
     * This is the typedef enum present in EoBoards.h 
     * that defines the index in the data vector of each finger position read by the POS service.
     * We can use this function to remap the sensorId (which is the idList[] element) to the corresponding index in m_data[] vector.
     * therefore we can get from the m_data[] vector the position of the sensor corresponding to the desired finger.
     * This remapping is needed bacause the sensorId is always in a sequence ranges from 0 to n-1, where n is the number of enabled sensors,
     * which is 4 for open-close and 2 for abduction, while the index in m_data[] vector is not in a sequence from 0 to n-1,
     * but it is defined by the eObrd_portpos_t enum, which has some values not in sequence, e.g. the abductions.
     * We copy here the enum for clarity:
     * typedef enum
        {
            eobrd_portpos_hand_thumb_oc         = 0,
            eobrd_portpos_hand_index_oc         = 1,
            eobrd_portpos_hand_middle_oc        = 2,
            eobrd_portpos_hand_ring_pinky_oc    = 3,
            eobrd_portpos_hand_thumb_add        = 4,
            eobrd_portpos_hand_tbd              = 5,
            eobrd_portpos_hand_index_add        = 6,    
            
            eobrd_portpos_none                  = 31,    // as ... eobrd_port_none
            eobrd_portpos_unknown               = 30     // as ... eobrd_port_unknown
        } eObrd_portpos_t;
    */
    
    // First we need to understand if the sensorId is related to open-close or abduction.
    // We can do this by checking the size of idList[] vector, which is 4 for open-close and 2 for abduction.
    // We need to use a temporary variable to store the dataIndex, because it is passed by reference.
    // and the enum has a different type than uint32_t.

    eObrd_portpos_t portPosIndex = eobrd_portpos_unknown; // default value in case of error
    if (serviceConfig.idList.size() == 4) // open-close
    {
        switch (sensorId)
        {
            case 0:
                portPosIndex = eobrd_portpos_hand_thumb_oc;
                break;
            case 1:
                portPosIndex = eobrd_portpos_hand_index_oc;
                break;
            case 2:
                portPosIndex = eobrd_portpos_hand_middle_oc;
                break;
            case 3:
                portPosIndex = eobrd_portpos_hand_ring_pinky_oc;
                break;
            default:
                portPosIndex = eobrd_portpos_unknown; // error
                break;
        }
    }
    else if (serviceConfig.idList.size() == 2) // abduction
    {
       switch (sensorId)
        {
            case 0:
                portPosIndex = eobrd_portpos_hand_thumb_add;
                break;
            case 1:
                portPosIndex = eobrd_portpos_hand_index_add;
                break;
            default:
                portPosIndex = eobrd_portpos_unknown; // error
                break;
        }
    }
    else
    {
        portPosIndex = eobrd_portpos_unknown; // error
    }

    dataIndex = static_cast<uint32_t>(portPosIndex);
}

size_t embObjPOS::getNrOfEncoderArrays() const
{
    // this should return the number of sensors we wanna have per control boards
    // e.g. currently POS service can manage up to 4 position sensors (2 for open/close and 2 for abduction)
    // so we are returning the number of enabled sensors, i.e. 2 or 4, depending on the device
    // differently, we can choose another approach, i.e. return 1, as the encoder array is a single entity, 
    // which is actually the control board associated with the opened serivice
    // and then getEncoderArraySize() should return 2 or 4, i.es the number of enabled sensors
    yCDebug(EMBOBJPOS) << m_PDdevice.getBoardInfo() << "embObjPOS::getNrOfEncoderArrays called at timestamp=" << yarp::os::Time::now() << "returning" << serviceConfig.idList.size();
    return serviceConfig.idList.size();
}

yarp::dev::MAS_status embObjPOS::getEncoderArrayStatus(size_t sens_index) const
{
    //TODO: maybe we should add a mutex for each method which write data to be thread safe
    if (sens_index >= serviceConfig.idList.size()) return yarp::dev::MAS_UNKNOWN; //REV-VALE:this is wrong. I guess it is a copy-paste error. It should be 4 or the max size of sensor we can manage. Maybe in fw-shared there is already a constant for this. defined.
    return yarp::dev::MAS_OK;
}

bool embObjPOS::getEncoderArrayName(size_t sens_index, std::string &name) const
{
    //TODO: maybe we should add a mutex for each method which write data to be thread safe
    if (sens_index >= serviceConfig.idList.size()) return false; 
    name = serviceConfig.idList[sens_index];
    return true;
}

bool embObjPOS::getEncoderArrayMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    // we cannot receice an index higher that the number of enabled sensors
    if (sens_index >= serviceConfig.idList.size()) return false; 
    timestamp = yarp::os::Time::now();
    // we need to remap the sens_index, which is the index in idList[], to the corresponding index in m_data[] vector
    uint32_t dataIndex = 0;
    helper_remapperFromSensorToDataIndex(static_cast<uint32_t>(sens_index), dataIndex);
    out = m_data[dataIndex]; //TODO: this should be a vector of 2 or 4 elements, depending on the device, if related to abduction or open-close. Each number is the position of the actuator.
    return true;
}

size_t embObjPOS::getEncoderArraySize(size_t sens_index) const
{
    if (sens_index >= serviceConfig.idList.size()) return 0;
    return 1; //TODO: this should return the size of data at index sens_index, which should actually be 1.
}

eth::iethresType_t embObjPOS::type()
{
    return eth::iethres_analogpos;
}


bool embObjPOS::update(eOprotID32_t id32, double timestamp, void* rxdata)
{
    // called by feat_manage_analogsensors_data() which is called by:
    // eoprot_fun_UPDT_as_pos_status
    char nvinfo[128] = {};
    eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
    if(!m_PDdevice.isOpen())
        return false;

    eOas_pos_status_t *pos_st_ptr  = (eOas_pos_status_t*)rxdata;
    uint32_t sizeOfData = pos_st_ptr->arrayofdata.head.size;

    if(sizeOfData>m_data.size())
        yCWarning(EMBOBJPOS) << m_PDdevice.getBoardInfo() << "In update function I received more data than I had been configured to store.My size is " << m_data.size() << "while I received " << sizeOfData << "values!";

    std::lock_guard<std::mutex> lock(m_mutex);

    for(uint32_t i=0; i<sizeOfData; i++)
    {
        eOas_pos_data_t posdata = pos_st_ptr->arrayofdata.data[i];
        //pos value is in deci-degree
        m_data[i]= posdata.value*0.1;
    }

    return true;
}

bool embObjPOS::close()
{
    cleanup();
    return true;
}

void embObjPOS::cleanup(void)
{
    m_PDdevice.cleanup(static_cast<eth::IethResource*> (this));
}

// eof

