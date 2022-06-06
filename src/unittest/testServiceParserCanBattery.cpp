/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <yarp/os/ResourceFinder.h>

#include <iostream>
#include <vector>

#include "EoManagement.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "serviceParserCanBattery.h"
#include "testUtils.h"

using ::testing::_;
using ::testing::An;
using ::testing::Eq;
using ::testing::Gt;
using ::testing::InSequence;
using ::testing::InvokeArgument;
using ::testing::Matcher;
using ::testing::Pointee;
using ::testing::Return;
using ::testing::TypedEq;
using testing::internal::operator==;

TEST(ServiceParserCanBattery, check_settings_positive_001)
{
	yarp::os::Bottle bottle;
	bottle.fromString("(SETTINGS (acquisitionRate 100) (enabledSensors fakeId) )");

	ServiceParserCanBattery_mock serviceParser;

	bool ret = serviceParser.checkSettings(bottle);

	std::map<std::string, BatteryInfo> expected = {{"fakeId", {100, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0}}};

	EXPECT_TRUE(ret);

	EXPECT_EQ(expected.at("fakeId"), serviceParser.batteryInfo_);
}

TEST(ServiceParserCanBattery, check_settings_negative_001)
{
	yarp::os::Bottle bottle;
	bottle.fromString("(SETTINGS (acquisitionRate 100) (enabledSensors) )");

	ServiceParserMultipleFt_mock serviceParser;

	bool ret = serviceParser.checkSettings(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_settings_negative_002)
{
	yarp::os::Bottle bottle;
	bottle.fromString("(SETTINGS (acquisitionRate) (enabledSensors fakeId) )");

	ServiceParserMultipleFt_mock serviceParser;

	bool ret = serviceParser.checkSettings(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_settings_negative_003)
{
	yarp::os::Bottle bottle;
	bottle.fromString("(XXX (acquisitionRate 100) (enabledSensors fakeId) )");

	ServiceParserMultipleFt_mock serviceParser;

	bool ret = serviceParser.checkSettings(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_settings_negative_004)
{
	yarp::os::Bottle bottle;
	bottle.fromString("(SETTINGS (xxx 100) (enabledSensors fakeId) )");

	ServiceParserMultipleFt_mock serviceParser;

	bool ret = serviceParser.checkSettings(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_settings_negative_005)
{
	yarp::os::Bottle bottle;
	bottle.fromString("(SETTINGS (acquisitionRate) (XXX fakeId) )");

	ServiceParserMultipleFt_mock serviceParser;

	bool ret = serviceParser.checkSettings(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_property_sensors_positive_001)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(SENSORS (id fakeId) (board bms) (location CAN2:13) )");
	serviceParser.batteryInfo_ = {100, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertySensors(bottle);

	BatteryInfo expected = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	EXPECT_TRUE(ret);

	EXPECT_EQ(expected, serviceParser.batteryInfo_);
}

TEST(ServiceParserCanBattery, check_property_sensors_negative_001)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(SENSORS (id fakeId) (xxx bms) (location CAN2:13) )");
	serviceParser.batteryInfo_ = {100, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertySensors(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_property_sensors_negative_002)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(SENSORS (id fakeId) (board bms) (xxx CAN2:13) )");
	serviceParser.batteryInfo_ = {100, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertySensors(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_property_canboards_positive_001)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(CANBOARDS (type bms) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
	serviceParser.batteryInfo_ = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertyCanBoards(bottle);

	BatteryInfo expected = {100, eobrd_bms, 2, 13, 2, 3, 4, 5, 6};

	EXPECT_TRUE(ret);

	EXPECT_EQ(expected, serviceParser.batteryInfo_);
}

TEST(ServiceParserCanBattery, check_property_canboards_negative_001)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(CANBOARDS (xxx bms) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
	serviceParser.batteryInfo_ = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertyCanBoards(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_property_canboards_negative_002)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(CANBOARDS (type bms) (XXX (major 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
	serviceParser.batteryInfo_ = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertyCanBoards(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_property_canboards_negative_003)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(CANBOARDS (type bms) (PROTOCOL (xx 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
	serviceParser.batteryInfo_ = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertyCanBoards(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_property_canboards_negative_004)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(CANBOARDS (type bms) (PROTOCOL (major 2) (xx 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
	serviceParser.batteryInfo_ = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertyCanBoards(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_property_canboards_negative_005)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(CANBOARDS (type bms) (PROTOCOL (major 2) (minor 3) ) (xx (major 4) (minor 5) (build 6) ) )");
	serviceParser.batteryInfo_ = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertyCanBoards(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_property_canboards_negative_006)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(CANBOARDS (type bms) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (xx 4) (minor 5) (build 6) ) )");
	serviceParser.batteryInfo_ = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertyCanBoards(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_property_canboards_negative_007)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(CANBOARDS (type bms) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (major 4) (xx 5) (build 6) ) )");
	serviceParser.batteryInfo_ = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertyCanBoards(bottle);

	EXPECT_FALSE(ret);
}
TEST(ServiceParserCanBattery, check_property_canboards_negative_008)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(CANBOARDS (type bms) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (xx 6) ) )");
	serviceParser.batteryInfo_ = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertyCanBoards(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, check_property_canboards_negative_009)
{
	ServiceParserCanBattery_mock serviceParser;
	yarp::os::Bottle bottle;
	bottle.fromString("(CANBOARDS (type xx) (PROTOCOL (major 2) (minor 3) ) (FIRMWARE (major 4) (minor 5) (build 6) ) )");
	serviceParser.batteryInfo_ = {100, eobrd_bms, 2, 13, 0, 0, 0, 0, 0};

	bool ret = serviceParser.checkPropertyCanBoards(bottle);

	EXPECT_FALSE(ret);
}

TEST(ServiceParserCanBattery, checkBoardType_positive_001) {
	ServiceParserCanBattery_mock serviceParser;

	eObrd_type_t expected = eobrd_bms;

	EXPECT_EQ(expected, serviceParser.checkBoardType("bms"));
}

TEST(ServiceParserCanBattery, checkBoardType_negative_001) {
	ServiceParserCanBattery_mock serviceParser;

	eObrd_type_t expected = eobrd_bms;

	EXPECT_NE(expected, serviceParser.checkBoardType("eobrd_strain2"));
}

TEST(ServiceParserCanBattery, checkBoardType_negative_002) {
	ServiceParserCanBattery_mock serviceParser;

	eObrd_type_t expected = eobrd_bms;

	EXPECT_NE(expected, serviceParser.checkBoardType(""));
}


TEST(General, toEomn_positive_001) {
	BatteryInfo info = {100, eobrd_bms, 1, 13, 4, 5, 1, 2, 3};

	eOas_battery_sensordescriptor_t out;
	bool res = info.toEomn(out);

	eOas_battery_sensordescriptor_t expected = {{eobrd_bms, {1, 2, 3}, {4, 5}}, {eOcanport1, 13, 2}};

	EXPECT_TRUE(res);
	EXPECT_EQ(expected, out);
}

TEST(General, toEomn_negative_001) {
	BatteryInfo info = {100, eobrd_bms, 5, 13, 4, 5, 1, 2, 3};

	eOas_battery_sensordescriptor_t out;
	bool res = info.toEomn(out);

	EXPECT_FALSE(res);
}
