/*
 * Copyright (C) Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "diagnosticLowLevelFormatter.h"
#include "diagnosticLowLevelFormatter_hid.h"



using namespace Diagnostic::LowLevel;
using namespace Diagnostic;


InfoFormatter::InfoFormatter(eth::TheEthManager* ethManager, eOmn_info_basic_t* infobasic, uint8_t * extra, const EOnv* nv, const eOropdescriptor_t* rd) :
        m_ethManager(ethManager),m_infobasic(infobasic), m_extra(extra), m_nv(nv), m_rd(rd)
{;}


bool InfoFormatter::getDiagnosticInfo(EmbeddedInfo &info)
{
    //==> firts of all fill all the common info to all messages and then parser the parameter depending on the specific message
    
    AuxEmbeddedInfo dnginfo;

    //1. fill all the common info to all messages
    dnginfo.sourceBoardIpAddr = eo_nv_GetIP(m_nv);
    dnginfo.baseInfo.sourceBoardName = m_ethManager->getName(eo_nv_GetIP(m_nv));
    getTimeOfInfo(dnginfo.baseInfo.timeOfInfo);
    getSourceOfMessage(dnginfo.baseInfo);
    getSeverityOfError(dnginfo.baseInfo);
    
    dnginfo.baseMessage = std::string(eoerror_code2string(m_infobasic->properties.code));
    
    eOmn_info_extraformat_t extraf  = static_cast<eOmn_info_extraformat_t>EOMN_INFO_PROPERTIES_FLAGS_get_extraformat(m_infobasic->properties.flags);
    dnginfo.extraMessage.clear();

    if(eomn_info_extraformat_verbal == extraf)
    {
        dnginfo.extraMessage.append((NULL == m_extra) ? ("no extra info despite we are in verbal mode") : ((const char *)m_extra));
    }
    else
    {
        dnginfo.extraMessage.append(".");
    }


    ipv4ToString(dnginfo.baseInfo);
    dnginfo.param64 = m_infobasic->properties.par64; 
    dnginfo.param16 = m_infobasic->properties.par16;
    dnginfo.errorCode = m_infobasic->properties.code;

    EntityNameProvider entityNameProvider{dnginfo.sourceBoardIpAddr, m_ethManager};

    //2. create the parser related to the error cod family
    std::unique_ptr<DefaultParser> parser_ptr;
 
    eOerror_category_t category = eoerror_code2category(m_infobasic->properties.code);
    switch(category)
    {
        case eoerror_category_Config: parser_ptr = std::make_unique<ConfigParser>(dnginfo, entityNameProvider); break;
        
        case eoerror_category_MotionControl: parser_ptr = std::make_unique<MotionControlParser>(dnginfo, entityNameProvider); break;

        case eoerror_category_HardWare: parser_ptr = std::make_unique<HwErrorParser>(dnginfo, entityNameProvider); break;

        case eoerror_category_System: parser_ptr = std::make_unique<SysParser>(dnginfo, entityNameProvider); break;

        case eoerror_category_ETHmonitor: parser_ptr = std::make_unique<EthMonitorParser>(dnginfo, entityNameProvider); break;

        case eoerror_category_Skin: parser_ptr = std::make_unique<SkinParser>(dnginfo, entityNameProvider); break;

        case eoerror_category_InertialSensor: parser_ptr = std::make_unique<InertialSensorParser>(dnginfo, entityNameProvider); break;

        case eoerror_category_AnalogSensor: parser_ptr = std::make_unique<AnalogSensorParser>(dnginfo, entityNameProvider); break;

        default:                      parser_ptr = std::make_unique<DefaultParser>(dnginfo, entityNameProvider); break;
    };

    parser_ptr->parseInfo();

    //3. return the parsered info
    info = dnginfo.baseInfo;

    return true;

}

void InfoFormatter::getSourceOfMessage(EmbeddedInfo &info)
{
    const char * const sourcenames[] =
    {
        "LOCAL",
        "CAN1",
        "CAN2",
        "UNKNOWN"
    };
    
    eOmn_info_source_t source = static_cast<eOmn_info_source_t>EOMN_INFO_PROPERTIES_FLAGS_get_source(m_infobasic->properties.flags);

    info.sourceCANBoardAddr = EOMN_INFO_PROPERTIES_FLAGS_get_address(m_infobasic->properties.flags);

    info.sourceCANPortStr = std::string(((source > eomn_info_source_can2) ? (sourcenames[3]) : (sourcenames[source])));

}



void InfoFormatter::getTimeOfInfo(TimeOfInfo &timeOfInfo)
{
    timeOfInfo.sec  = m_infobasic->timestamp / 1000000;
    timeOfInfo.msec = (m_infobasic->timestamp % 1000000) / 1000;
    timeOfInfo.usec = m_infobasic->timestamp % 1000;
}


void InfoFormatter::ipv4ToString(EmbeddedInfo &info)
{
    char ipinfo[20] = {0};
    eo_common_ipv4addr_to_string(eo_nv_GetIP(m_nv), ipinfo, sizeof(ipinfo));
    info.sourceBoardIpAddrStr.clear();
    info.sourceBoardIpAddrStr.append(ipinfo);
}

void InfoFormatter::getSeverityOfError(EmbeddedInfo &info)
{
    eOmn_info_type_t type = static_cast<eOmn_info_type_t>EOMN_INFO_PROPERTIES_FLAGS_get_type(m_infobasic->properties.flags);
    switch(type)
    {
        case eomn_info_type_info: {info.severity =  SeverityOfError::info;return;}
        case eomn_info_type_debug: {info.severity =  SeverityOfError::debug;return;}
        case eomn_info_type_warning: {info.severity =  SeverityOfError::warning;return;}
        case eomn_info_type_error: {info.severity =  SeverityOfError::error;return;}
        case eomn_info_type_fatal: {info.severity =  SeverityOfError::fatal;return;}

        
    };
}

/**************************************************************************************************************************/
/******************************************        EntityNameProvider       ***************************************************/
/**************************************************************************************************************************/
EntityNameProvider::EntityNameProvider(eOipv4addr_t boardAddr, eth::TheEthManager* ethManager):m_ethManager(ethManager)
{
       m_MC_ethRes = m_ethManager->getInterface(boardAddr, eth::iethresType_t::iethres_motioncontrol);
}

bool EntityNameProvider::getAxisName(uint32_t entityId, std::string &axisName)
{
    if(m_MC_ethRes == nullptr)
    {
        axisName = "N/A";
        return false;
    }
    
    return (m_MC_ethRes->getEntityName(entityId, axisName));
}

bool EntityNameProvider::getEncoderTypeName(uint32_t jomoId, eOmc_position_t pos, std::string &encoderTypeName)
{
    if (m_MC_ethRes == nullptr)  
    {
        encoderTypeName = "N/A";
        return false;
    }

    return (m_MC_ethRes->getEncoderTypeName(jomoId, pos, encoderTypeName));
}