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
#include "testUtils.h"

using namespace testing;
using ::testing::_;
using ::testing::InvokeArgument;
using ::testing::Matcher;

TEST(General, check_settings_positive_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(SETTINGS (ftPeriod 10) (enabledSensors fakeId) (temperaturePeriod 100) (useCalibration true) )");

    ServiceParserMultipleFt_mock serviceParser;

    bool ret = serviceParser.checkSettings(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_settings_positive_002)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(SETTINGS (ftPeriod 10 20) (enabledSensors fakeId fakeId1) (temperaturePeriod 100 200) (useCalibration true false) )");

    ServiceParserMultipleFt_mock serviceParser;

    bool ret = serviceParser.checkSettings(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0}},
                                                            {"fakeId1", {20, 200, eoas_ft_mode_raw, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
    EXPECT_EQ(expected.at("fakeId1"), serviceParser.ftInfo_.at("fakeId1"));
}

TEST(General, check_settings_negative_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString(
        "(SETTINGS (ftPeriod 10 20 30 40 50) (enabledSensors fakeId0 fakeId1 fakeId2 fakeId13 fakeId4 ) (temperaturePeriod 100 200 998 997 996) (useCalibration true false true true true) )");

    ServiceParserMultipleFt_mock serviceParser;

    bool ret = serviceParser.checkSettings(bottle);

    EXPECT_FALSE(ret);
}

TEST(General, check_settings_negative_002)
{
    yarp::os::Bottle bottle;
    bottle.fromString(
        "(SETTINGS (ftPeriod 10 20 30 40) (enabledSensors fakeId0 fakeId1 fakeId2 fakeId13 fakeId4 ) (temperaturePeriod 100 200 998 997 996) (useCalibration true false true true true) )");

    ServiceParserMultipleFt_mock serviceParser;

    bool ret = serviceParser.checkSettings(bottle);

    EXPECT_FALSE(ret);
}

TEST(General, check_settings_negative_003)
{
    yarp::os::Bottle bottle;
    bottle.fromString(
        "(SETTINGS (ftPeriod 10 20 30 40 50) (enabledSensors fakeId0 fakeId1 fakeId2 fakeId13 fakeId4 ) (temperaturePeriod 100 200 998 997) (useCalibration true false true true true) )");

    ServiceParserMultipleFt_mock serviceParser;

    bool ret = serviceParser.checkSettings(bottle);

    EXPECT_FALSE(ret);
}

TEST(General, check_settings_negative_004)
{
    yarp::os::Bottle bottle;
    bottle.fromString(
        "(SETTINGS (ftPeriod 10 20 30 40 50) (enabledSensors fakeId0 fakeId1 fakeId2 fakeId13 fakeId4 ) (temperaturePeriod 100 200 998 997 996) (useCalibration true false true true ) )");

    ServiceParserMultipleFt_mock serviceParser;

    bool ret = serviceParser.checkSettings(bottle);

    EXPECT_FALSE(ret);
}

TEST(General, check_property_sensors_positive_001)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId) (board strain2) (location CAN2:13) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertySensors(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_sensors_positive_002)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId fakeId1) (board strain2 strain2) (location CAN2:13 CAN2:14) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 2, 13, 0, 0, 0, 0, 0}}, {"fakeId1", {20, 200, eoas_ft_mode_raw, eobrd_unknown, 2, 14, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertySensors(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}},
                                                            {"fakeId1", {20, 200, eoas_ft_mode_raw, eobrd_strain2, 2, 14, 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
    EXPECT_EQ(expected.at("fakeId1"), serviceParser.ftInfo_.at("fakeId1"));
}

TEST(General, check_property_sensors_positive_003)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId fakeId1) (board strain2 strain2) (location CAN2:13 CAN2:14) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertySensors(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_sensors_positive_004)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId) (board eobrd_strain2) (location CAN2:13) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertySensors(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_sensors_positive_005)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId) (board eobrd_strain) (location CAN2:13) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertySensors(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain, 2, 13, 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_sensors_negative_001)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId) (board strain2) (location CAN2:14) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertySensors(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_NE(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_sensors_negative_002)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId) (board strain2) (location CANx:15) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertySensors(bottle);

    EXPECT_FALSE(ret);
}

TEST(General, check_property_sensors_negative_003)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId fakeId1) (board strain2 strain2) (location CAN2:13 CAN2:14) )");
    serviceParser.ftInfo_ = {{"notexistent", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 2, 13, 0, 0, 0, 0, 0}}, {"fakeId1", {20, 200, eoas_ft_mode_raw, eobrd_unknown, 2, 14, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertySensors(bottle);

    EXPECT_FALSE(ret);
}

TEST(General, check_property_sensors_negative_004)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(SENSORS (id fakeId fakeId1) (board wrongBoard strain2) (location CAN2:13 CAN2:14) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 2, 13, 0, 0, 0, 0, 0}}, {"fakeId1", {20, 200, eoas_ft_mode_raw, eobrd_unknown, 2, 14, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertySensors(bottle);

    EXPECT_FALSE(ret);
}

TEST(General, check_property_canboards_positive_001)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertyCanBoards(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 2, 3, 4, 5, 6}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_canboards_positive_002)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}, {"fakeId1", {10, 100, eoas_ft_mode_calibrated, eobrd_strain, 2, 13, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertyCanBoards(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 2, 3, 4, 5, 6}},
                                                            {"fakeId1", {10, 100, eoas_ft_mode_calibrated, eobrd_strain, 2, 13, 0, 0, 0, 0, 0}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
    EXPECT_EQ(expected.at("fakeId1"), serviceParser.ftInfo_.at("fakeId1"));
}

TEST(General, check_property_canboards_positive_003)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2 strain) (PROTOCOL (major 2 7) (minor 3 8) ) (FIRMWARE (major 4 9) (minor 5 10) (build 6 11) ) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}, {"fakeId1", {10, 100, eoas_ft_mode_calibrated, eobrd_strain, 2, 13, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertyCanBoards(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 2, 3, 4, 5, 6}},
                                                            {"fakeId1", {10, 100, eoas_ft_mode_calibrated, eobrd_strain, 2, 13, 7, 8, 9, 10, 11}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
    EXPECT_EQ(expected.at("fakeId1"), serviceParser.ftInfo_.at("fakeId1"));
}

TEST(General, check_property_canboards_positive_004)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type eobrd_strain2) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertyCanBoards(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 2, 3, 4, 5, 6}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
}

TEST(General, check_property_canboards_negative_001)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2 strain) (PROTOCOL (major 2 7) (minor 3 8) ) (FIRMWARE (major 4 9) (minor 5 10) (build 6 11) ) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}, {"fakeId1", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 2, 13, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertyCanBoards(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 2, 3, 4, 5, 6}},
                                                            {"fakeId1", {10, 100, eoas_ft_mode_calibrated, eobrd_strain, 2, 13, 7, 8, 9, 10, 11}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
    EXPECT_NE(expected.at("fakeId1"), serviceParser.ftInfo_.at("fakeId1"));
}

TEST(General, check_property_canboards_negative_002)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2 strain) (PROTOCOL (major 2 7) (minor 3 8) ) (FIRMWARE (major 4 9) (minor 5 10) (build 6 20) ) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}, {"fakeId1", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 2, 13, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertyCanBoards(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 2, 3, 4, 5, 6}},
                                                            {"fakeId1", {10, 100, eoas_ft_mode_calibrated, eobrd_strain, 2, 13, 7, 8, 9, 10, 11}}};

    EXPECT_TRUE(ret);

    ASSERT_EQ(expected.size(), serviceParser.ftInfo_.size());
    EXPECT_EQ(expected.at("fakeId"), serviceParser.ftInfo_.at("fakeId"));
    EXPECT_NE(expected.at("fakeId1"), serviceParser.ftInfo_.at("fakeId1"));
}

TEST(General, check_property_canboards_negative_003)
{
    ServiceParserMultipleFt_mock serviceParser;
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type wrongboardname strain) (PROTOCOL (major 2 7) (minor 3 8) ) (FIRMWARE (major 4 9) (minor 5 10) (build 6 20) ) )");
    serviceParser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 0, 0, 0, 0, 0}}, {"fakeId1", {10, 100, eoas_ft_mode_calibrated, eobrd_unknown, 2, 13, 0, 0, 0, 0, 0}}};

    bool ret = serviceParser.checkPropertyCanBoards(bottle);

    std::map<std::string /*sensor id*/, FtInfo> expected = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 2, 3, 4, 5, 6}},
                                                            {"fakeId1", {10, 100, eoas_ft_mode_calibrated, eobrd_strain, 2, 13, 7, 8, 9, 10, 11}}};

    EXPECT_FALSE(ret);
}

TEST(General, check_checkCanMonitor_positive_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(CANMONITOR (checkPeriod 101) (ratePeriod 200) (reportMode ALL) )");

    ServiceParserMultipleFt_mock serviceParser;

    bool ret = serviceParser.checkCanMonitor(bottle);

    EXPECT_TRUE(ret);

    eObrd_canmonitor_cfg_t expected = {101, eobrd_canmonitor_reportmode_ALL, 200};
    EXPECT_EQ(expected, serviceParser.canMonitor_);
}

TEST(General, check_checkCanMonitor_negative_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(CANMONITOR (checkPeriod 101) (ratePeriod 200) (reportMode xxx) )");

    ServiceParserMultipleFt_mock serviceParser;

    bool ret = serviceParser.checkCanMonitor(bottle);

    EXPECT_FALSE(ret);
}

TEST(General, check_checkCanMonitor_negative_002)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(CANMONITOR (checkPeriod 300) (ratePeriod 200) (reportMode ALL) )");

    ServiceParserMultipleFt_mock serviceParser;

    bool ret = serviceParser.checkCanMonitor(bottle);

    EXPECT_FALSE(ret);
}

TEST(General, check_checkCanMonitor_negative_003)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(CANMONITOR (checkPeriod 200) (ratePeriod 200000) (reportMode ALL) )");

    ServiceParserMultipleFt_mock serviceParser;

    bool ret = serviceParser.checkCanMonitor(bottle);

    EXPECT_FALSE(ret);
}

TEST(General, toEomn_positive_001)
{
    FtInfo info = {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 1, 13, 4, 5, 1, 2, 3};

    eOas_ft_sensordescriptor_t out;
    bool res = info.toEomn(out);

    eOas_ft_sensordescriptor_t expected = {{eobrd_strain2, {1, 2, 3}, {4, 5}}, {0, 13, eobrd_caninsideindex_none}, 0};

    EXPECT_TRUE(res);
    EXPECT_EQ(expected, out);
}

TEST(General, toEomn_positive_002)
{
    FtInfo info = {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 2, 13, 4, 5, 1, 2, 3};

    eOas_ft_sensordescriptor_t out;
    bool res = info.toEomn(out);

    eOas_ft_sensordescriptor_t expected = {{eobrd_strain2, {1, 2, 3}, {4, 5}}, {1, 13, eobrd_caninsideindex_none}, 0};

    EXPECT_TRUE(res);
    EXPECT_EQ(expected, out);
}

TEST(General, toEomn_negative_001)
{
    FtInfo info = {10, 100, eoas_ft_mode_calibrated, eobrd_strain2, 3, 13, 4, 5, 1, 2, 3};

    eOas_ft_sensordescriptor_t out;
    bool res = info.toEomn(out);

    EXPECT_FALSE(res);
}

TEST(General, checkBoardType_positive_001)
{
    ServiceParserMultipleFt_mock serviceParser;

    eObrd_type_t expected=eobrd_strain2;

    EXPECT_EQ(expected,serviceParser.checkBoardType("strain2"));
}

TEST(General, checkBoardType_positive_002)
{
    ServiceParserMultipleFt_mock serviceParser;

    eObrd_type_t expected=eobrd_strain;

    EXPECT_EQ(expected,serviceParser.checkBoardType("strain"));
}


TEST(General, checkBoardType_positive_003)
{
    ServiceParserMultipleFt_mock serviceParser;

    eObrd_type_t expected=eobrd_strain2;

    EXPECT_EQ(expected,serviceParser.checkBoardType("eobrd_strain2"));
}


TEST(General, checkBoardType_positive_004)
{
    ServiceParserMultipleFt_mock serviceParser;

    eObrd_type_t expected=eobrd_strain;

    EXPECT_EQ(expected,serviceParser.checkBoardType("eobrd_strain"));
}

TEST(General, checkBoardType_negative_001)
{
    ServiceParserMultipleFt_mock serviceParser;

    eObrd_type_t expected=eobrd_unknown;

    EXPECT_EQ(expected,serviceParser.checkBoardType("amcbldc"));
}

TEST(General, checkBoardType_negative_002)
{
    ServiceParserMultipleFt_mock serviceParser;

    eObrd_type_t expected=eobrd_unknown;

    EXPECT_EQ(expected,serviceParser.checkBoardType(""));
}