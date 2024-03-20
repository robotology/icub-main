/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef __diagnosticLowLevelFormatter_hid_h__
#define __diagnosticLowLevelFormatter_hid_h__

#include <string>
#include <string_view>
#include <memory>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>
#include "diagnosticLowLevelFormatter.h"
#include "EoError.h"

#include "IethResource.h"


namespace Diagnostic {
    namespace LowLevel {
        class DefaultParser;
        class ConfigParser;
        class MotionControlParser;
        class SkinParser;
        class HwErrorParser;
        class SysParser;
        class EthMonitorParser;
        class InertialSensorParser;
        class AnalogSensorParser;
        class AuxEmbeddedInfo;
        class EntityNameProvider;
    }
}

constexpr int diagstr_lenght = 64;
typedef char diagstr[diagstr_lenght];

//In this class the Info formatter and the parsers collect the embedded diagnostic Info.
//Only the info contained in Diagnostic::EmbeddedInfo  baseInfo are usefull for the user level.
class Diagnostic::LowLevel::AuxEmbeddedInfo
{
public:
    Diagnostic::EmbeddedInfo  baseInfo;
    eOipv4addr_t              sourceBoardIpAddr;  // is the ipv4 address, in eOipv4addr_t,  of the board that sends the diagnostic information 
    eOerror_code_t            errorCode; // this is the error code sent by the board;
    std::string               baseMessage; //this is the base message string without any parameter
    std::string               extraMessage;// in some case, the board can send an extra string
    uint64_t                  param64; // the 64 bits paramter sent by the board
    uint16_t                  param16; //the 16 bits parameter sent by the board

public:
    void printMessage();
};


//This class has the goal to provide the name of an entity (axis, sensors, etc) to the parsers.
class Diagnostic::LowLevel::EntityNameProvider
{
public:
    EntityNameProvider(eOipv4addr_t boardAddr, eth::TheEthManager* ethManager);
    EntityNameProvider() = delete;
    EntityNameProvider(const Diagnostic::LowLevel::EntityNameProvider &EntityNameProvider){};
    ~EntityNameProvider(){;};
    EntityNameProvider(const Diagnostic::LowLevel::EntityNameProvider &&EntityNameProvider){};
private:
    eth::TheEthManager* m_ethManager;
    eth::IethResource* m_MC_ethRes;
public:
    bool getAxisName(uint32_t entityId, std::string &axisName);
};



class Diagnostic::LowLevel::DefaultParser
{

public:
    DefaultParser(Diagnostic::LowLevel::AuxEmbeddedInfo &dnginfo, Diagnostic::LowLevel::EntityNameProvider &entityNameProvider);
    DefaultParser() = delete;
    DefaultParser(const Diagnostic::LowLevel::DefaultParser &parser)= delete;
    ~DefaultParser(){};
    DefaultParser(Diagnostic::LowLevel::DefaultParser &&parser)= delete;
    virtual void parseInfo(); 

protected:
    Diagnostic::LowLevel::AuxEmbeddedInfo &m_dnginfo;
    Diagnostic::LowLevel::EntityNameProvider &m_entityNameProvider;
    
    void printBaseInfo();
};

class Diagnostic::LowLevel::ConfigParser : public Diagnostic::LowLevel::DefaultParser
{
public:
    ConfigParser(Diagnostic::LowLevel::AuxEmbeddedInfo &dnginfo, Diagnostic::LowLevel::EntityNameProvider &entityNameProvider);
    ConfigParser() = delete;
    ConfigParser(const Diagnostic::LowLevel::ConfigParser &parser)= delete;
    ~ConfigParser(){};
    ConfigParser(Diagnostic::LowLevel::ConfigParser &&parser)=delete;

    void parseInfo();
};


class Diagnostic::LowLevel::MotionControlParser : public Diagnostic::LowLevel::DefaultParser
{
public:
    MotionControlParser(Diagnostic::LowLevel::AuxEmbeddedInfo &dnginfo, Diagnostic::LowLevel::EntityNameProvider &entityNameProvider);
    MotionControlParser() = delete;
    MotionControlParser(const Diagnostic::LowLevel::MotionControlParser &parser)= delete;
    ~MotionControlParser(){};
    MotionControlParser(Diagnostic::LowLevel::MotionControlParser &&parser)=delete;

    void parseInfo();


};

class Diagnostic::LowLevel::SkinParser : public Diagnostic::LowLevel::DefaultParser
{
public:
    SkinParser(Diagnostic::LowLevel::AuxEmbeddedInfo &dnginfo, Diagnostic::LowLevel::EntityNameProvider &entityNameProvider);
    SkinParser() = delete;
    SkinParser(const Diagnostic::LowLevel::SkinParser &parser) = delete;
    ~SkinParser(){};
    SkinParser(Diagnostic::LowLevel::SkinParser &&parser) = delete;

    void parseInfo();
};

class Diagnostic::LowLevel::HwErrorParser : public Diagnostic::LowLevel::DefaultParser
{
public:
    HwErrorParser(Diagnostic::LowLevel::AuxEmbeddedInfo &dnginfo, Diagnostic::LowLevel::EntityNameProvider &entityNameProvider);
    HwErrorParser() = delete;
    HwErrorParser(const Diagnostic::LowLevel::HwErrorParser &parser)= delete;
    ~HwErrorParser(){};
    HwErrorParser(Diagnostic::LowLevel::HwErrorParser &&parser)=delete;

    void parseInfo();


};

class Diagnostic::LowLevel::SysParser : public Diagnostic::LowLevel::DefaultParser
{
public:
    SysParser(Diagnostic::LowLevel::AuxEmbeddedInfo &dnginfo, Diagnostic::LowLevel::EntityNameProvider &entityNameProvider);
    SysParser() = delete;
    SysParser(const Diagnostic::LowLevel::SysParser &parser)= delete;
    ~SysParser(){};
    SysParser(Diagnostic::LowLevel::SysParser &&parser)=delete;

    void parseInfo();

private:
void canMask2canBoardsStr(uint16_t canmask, diagstr canboardsstr);
void getCanMonitorInfo(eOmn_serv_category_t &serv_category, diagstr boardsOnCan1, diagstr boardsOnCan2);
void getCanMonitorInfoWithTime(eOmn_serv_category_t &serv_category, diagstr boardsOnCan1, diagstr boardsOnCan2, uint32_t &time);
};


class Diagnostic::LowLevel::EthMonitorParser : public Diagnostic::LowLevel::DefaultParser
{
public:
    EthMonitorParser(Diagnostic::LowLevel::AuxEmbeddedInfo &dnginfo, Diagnostic::LowLevel::EntityNameProvider &entityNameProvider);
    EthMonitorParser() = delete;
    EthMonitorParser(const Diagnostic::LowLevel::EthMonitorParser &parser)= delete;
    ~EthMonitorParser(){};
    EthMonitorParser(Diagnostic::LowLevel::EthMonitorParser &&parser)=delete;

    void parseInfo();

};

class Diagnostic::LowLevel::InertialSensorParser : public Diagnostic::LowLevel::DefaultParser
{
public:
    InertialSensorParser(Diagnostic::LowLevel::AuxEmbeddedInfo &dnginfo, Diagnostic::LowLevel::EntityNameProvider &entityNameProvider);
    InertialSensorParser() = delete;
    InertialSensorParser(const Diagnostic::LowLevel::InertialSensorParser &parser) = delete;
    ~InertialSensorParser(){};
    InertialSensorParser(Diagnostic::LowLevel::InertialSensorParser &&parser) = delete;

    void parseInfo();
};

class Diagnostic::LowLevel::AnalogSensorParser : public Diagnostic::LowLevel::DefaultParser
{
public:
    AnalogSensorParser(Diagnostic::LowLevel::AuxEmbeddedInfo &dnginfo, Diagnostic::LowLevel::EntityNameProvider &entityNameProvider);
    AnalogSensorParser() = delete;
    AnalogSensorParser(const Diagnostic::LowLevel::AnalogSensorParser &parser) = delete;
    ~AnalogSensorParser(){};
    AnalogSensorParser(Diagnostic::LowLevel::AnalogSensorParser &&parser) = delete;

    void parseInfo();
};



#endif //__diagnosticLowLevelFormatter_hid_h__
