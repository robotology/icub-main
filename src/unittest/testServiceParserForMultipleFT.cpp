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

#include "serviceParserMultipleFt.h"
#include <yarp/os/ResourceFinder.h>

using namespace testing;
using ::testing::_;
using ::testing::InvokeArgument;
using ::testing::Matcher;

class ServiceParser_mock : public ServiceParserMultipleFt
{
public:
    using ServiceParserMultipleFt::checkCanMonitor;
    using ServiceParserMultipleFt::checkPropertyCanBoards;
    using ServiceParserMultipleFt::checkPropertySensors;
    using ServiceParserMultipleFt::checkServiceType;
    using ServiceParserMultipleFt::checkSettings;
    using ServiceParserMultipleFt::ftInfo_;

    ServiceParser_mock() : ServiceParserMultipleFt(){};
};

TEST(General, check_settings_positive_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(SETTINGS (acquisitionRate 10) (enabledSensors fakeId) (temperature-acquisitionRate 1000) (useCalibration true) )");

    ServiceParser_mock serviceParser;
    bool error{false};

    bool ret = serviceParser.checkSettings(bottle, error);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 1000, true, "", "", 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_settings_positive_002)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(SETTINGS (acquisitionRate 10 20) (enabledSensors fakeId fakeId1) (temperature-acquisitionRate 1000 999) (useCalibration true false) )");

    ServiceParser_mock serviceParser;
    bool error{false};

    bool ret = serviceParser.checkSettings(bottle, error);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 1000, true, "", "", 0, 0, 0, 0, 0}}, {"fakeId1", {20, 999, false, "", "", 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
    EXPECT_EQ(expected.at("fakeId1"), serviceParser.ftInfo_.at("fakeId1"));
}

TEST(General, check_property_sensors_positive_001)
{
    ServiceParser_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId) (board strain2) (location CAN2:13) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 1000, true, "", "", 0, 0, 0, 0, 0}}};
    bool error{false};
    
    bool ret=serviceParser.checkPropertySensors(bottle,error);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 1000, true, "strain2", "CAN2:13", 0, 0, 0, 0, 0}}};

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_sensors_positive_002)
{
    ServiceParser_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId fakeId1) (board strain2 strain2) (location CAN2:13 CAN2:14) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 1000, true, "", "", 0, 0, 0, 0, 0}}, {"fakeId1", {20, 999, false, "", "", 0, 0, 0, 0, 0}}};
    bool error{false};
    
    bool ret=serviceParser.checkPropertySensors(bottle,error);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 1000, true, "strain2", "CAN2:13", 0, 0, 0, 0, 0}}, {"fakeId1", {20, 999, false, "strain2", "CAN2:14", 0, 0, 0, 0, 0}}};

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
    EXPECT_EQ(expected.at("fakeId1"), serviceParser.ftInfo_.at("fakeId1"));
}

TEST(General, check_property_sensors_positive_003)
{
    ServiceParser_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId fakeId1) (board strain2 strain2) (location CAN2:13 CAN2:14) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 1000, true, "", "", 0, 0, 0, 0, 0}}};
    bool error{false};
    
    bool ret=serviceParser.checkPropertySensors(bottle,error);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 1000, true, "strain2", "CAN2:13", 0, 0, 0, 0, 0}}};

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_sensors_negative_001)
{
    ServiceParser_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId) (board strain2) (location CAN2:14) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 1000, true, "", "", 0, 0, 0, 0, 0}}};
    bool error{false};
    
    bool ret=serviceParser.checkPropertySensors(bottle,error);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 1000, true, "strain2", "CAN2:13", 0, 0, 0, 0, 0}}};

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_NE(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_canboards_positive_001)
{
    ServiceParser_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 1000, true, "strain2", "CAN2:13", 0, 0, 0, 0, 0}}};
    bool error{false};
    
    bool ret=serviceParser.checkPropertyCanBoards(bottle,error);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 1000, true, "strain2", "CAN2:13", 2, 3, 4, 5, 6}}};

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_canboards_positive_002)
{
    ServiceParser_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 1000, true, "strain2", "CAN2:13", 0, 0, 0, 0, 0}},{"fakeId1", {10, 1000, true, "strain", "CAN2:13", 0, 0, 0, 0, 0}}};
    bool error{false};
    
    bool ret=serviceParser.checkPropertyCanBoards(bottle,error);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 1000, true, "strain2", "CAN2:13", 2, 3, 4, 5, 6}},{"fakeId1", {10, 1000, true, "strain", "CAN2:13", 0, 0, 0, 0, 0}}};

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
    EXPECT_EQ(expected.at("fakeId1"), serviceParser.ftInfo_.at("fakeId1"));
}

TEST(General, check_property_canboards_positive_003)
{
    ServiceParser_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2 strain) (PROTOCOL (major 2 7) (minor 3 8) ) (FIRMWARE (major 4 9) (minor 5 10) (build 6 11) ) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 1000, true, "strain2", "CAN2:13", 0, 0, 0, 0, 0}},{"fakeId1", {10, 1000, true, "strain", "CAN2:13", 0, 0, 0, 0, 0}}};
    bool error{false};
    
    bool ret=serviceParser.checkPropertyCanBoards(bottle,error);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 1000, true, "strain2", "CAN2:13", 2, 3, 4, 5, 6}},{"fakeId1", {10, 1000, true, "strain", "CAN2:13", 7, 8, 9, 10, 11}}};

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
    EXPECT_EQ(expected.at("fakeId1"), serviceParser.ftInfo_.at("fakeId1"));
}

#if 0
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

#endif