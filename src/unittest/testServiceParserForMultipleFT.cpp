/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <iostream>
#include <vector>

#include "serviceParser.h"
#include <yarp/os/ResourceFinder.h>

using namespace testing;
using ::testing::_;
using ::testing::InvokeArgument;
using ::testing::Matcher;

class ServiceParser_mock: public ServiceParser
{
    public:
        using ServiceParser::checkSpecificForMultipleFT;
        using ServiceParser::checkPropertyCanBoards;
        using ServiceParser::checkPropertySensors;
        using ServiceParser::checkSettings;
        using ServiceParser::checkServiceType;
        using ServiceParser::checkCanMonitor;
        
        ServiceParser_mock():ServiceParser(){};
};

TEST(General, base_positive_001)
{
    yarp::os::Bottle bottle;
    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_strain;
    bool error{false};
    bool ret=serviceParser.checkSpecificForMultipleFT(bottle,type,error);

	EXPECT_TRUE(ret);
}

TEST(General, base_positive_002)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(FT_SETTINGS (useCalibration false))");
  
    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    bool error{false};
    bool ret=serviceParser.checkSpecificForMultipleFT(bottle,type,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
}

TEST(General, check_property_canboards_positive_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2) (PROTOCOL (major 2) (minor 0) ) (FIRMWARE (major 2) (minor 0) (build 7) ) )");
  
    ServiceParser_mock serviceParser;
    bool error{false};
    bool ret=serviceParser.checkPropertyCanBoards(bottle,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
}

TEST(General, check_property_canboards_negative_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2) (PROTOCOL (xxxx 2) (minor 0) ) (FIRMWARE (major 2) (minor 0) (build 7) ) )");
  
    ServiceParser_mock serviceParser;
    bool error{false};
    bool ret=serviceParser.checkPropertyCanBoards(bottle,error);

	EXPECT_FALSE(ret);
    EXPECT_FALSE(error);
}

TEST(General, check_property_sensors_positive_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id test_ft_sensor) (type eoas_strain) (location CAN2:13) )");
  
    ServiceParser_mock serviceParser;
    bool error{false};
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    bool ret=serviceParser.checkPropertySensors(bottle,type,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    EXPECT_EQ(1,serviceParser.as_service.properties.sensors.size());
}

TEST(General, check_property_sensors_positive_002)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id test_ft_sensor test1_ft_sensor) (type eoas_strain eoas_strain) (location CAN2:13 CAN2:14) )");
  
    ServiceParser_mock serviceParser;
    bool error{false};
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    bool ret=serviceParser.checkPropertySensors(bottle,type,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    EXPECT_EQ(2,serviceParser.as_service.properties.sensors.size());
}

TEST(General, check_settings_positive_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(SETTINGS (acquisitionRate 10) (enabledSensors fakeId) (temperature-acquisitionRate 1000) )");
  
    ServiceParser_mock serviceParser;
    bool error{false};

    servAnalogSensor_t toAdd={"fakeId",eoas_ft,{1,2},eobrd_strain2};
    serviceParser.as_service.properties.sensors.push_back(toAdd);
    
    bool ret=serviceParser.checkSettings(bottle,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(1,serviceParser.as_service.settings.enabledsensors.size());
    EXPECT_EQ("fakeId",serviceParser.as_service.settings.enabledsensors.at(0));
}

TEST(General, check_settings_positive_002)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(SETTINGS (acquisitionRate 10 20) (enabledSensors fakeId fakeId1) (temperature-acquisitionRate 1000 999) )");
  
    ServiceParser_mock serviceParser;
    bool error{false};

    servAnalogSensor_t toAdd={"fakeId",eoas_ft,{1,2},eobrd_strain2};
    serviceParser.as_service.properties.sensors.push_back(toAdd);
    toAdd={"fakeId1",eoas_ft,{1,2},eobrd_strain2};
    serviceParser.as_service.properties.sensors.push_back(toAdd);

    bool ret=serviceParser.checkSettings(bottle,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(2,serviceParser.as_service.settings.enabledsensors.size());
    EXPECT_EQ("fakeId",serviceParser.as_service.settings.enabledsensors.at(0));
    EXPECT_EQ("fakeId1",serviceParser.as_service.settings.enabledsensors.at(1));
}

TEST(General, check_checkservicetype_positive_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(type eomn_serv_AS_ft)"); 
    bool error{false};
    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    
    bool ret=serviceParser.checkServiceType(bottle,type,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
}

TEST(General, check_checkservicetype_positive_002)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(type eomn_serv)"); 
    bool error{false};
    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    
    bool ret=serviceParser.checkServiceType(bottle,type,error);

	EXPECT_FALSE(ret);
    EXPECT_TRUE(error);
}

TEST(General, check_checkservicetype_negative_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(type eomn_serv_AS_strain)"); 
    bool error{false};
    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    
    bool ret=serviceParser.checkServiceType(bottle,type,error);

	EXPECT_FALSE(ret);
    EXPECT_FALSE(error);
}

TEST(General, check_checkCanMonitor_positive_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(checkrate 100)(reportmode ALL)(periodicreportrate 200)"); 
    bool error{false};
    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    
    bool ret=serviceParser.checkServiceType(bottle,type,error);

	EXPECT_TRUE(ret); 

    eObrd_canmonitor_cfg_t tmp={100,eobrd_canmonitor_reportmode_ALL,200};  
    EXPECT_EQ(tmp,as_service.properties.config.data.as.ft.canmonitorconfig);
}

TEST(General, check_checkCanMonitor_negative_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(checkrate 100)(reportmode ALL)"); 
    bool error{false};
    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    
    bool ret=serviceParser.checkServiceType(bottle,type,error);

	EXPECT_FALSE(ret); 
}

TEST(General, check_checkCanMonitor_negative_002)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(checkrate 100)(reportmode xxx)(periodicreportrate 200)"); 
    bool error{false};
    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    
    bool ret=serviceParser.checkServiceType(bottle,type,error);

	EXPECT_FALSE(ret); 
}