/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <ethResource.h>

#include "EoProtocolAS.h"
#include "testUtils.h"
#include "embObjBattery.h"

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

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

class embObjCanBatterysensor_Mock : public yarp::dev::embObjBattery
{
   public:
	using yarp::dev::embObjBattery::canBatteryData_;
	using yarp::dev::embObjBattery::initRegulars;
	using yarp::dev::embObjBattery::sendConfig2boards;
	using yarp::dev::embObjBattery::sendStart2boards;
	using yarp::dev::embObjBattery::embObjBattery::calculateBoardTime;
	using yarp::dev::embObjBattery::embObjBattery::update;

	embObjCanBatterysensor_Mock(std::shared_ptr<yarp::dev::embObjDevPrivData> device) : yarp::dev::embObjBattery(device){};
	embObjCanBatterysensor_Mock() : yarp::dev::embObjBattery(){};
	MOCK_METHOD(double, calculateBoardTime, (eOabstime_t), (override));
};

class embObjDevPrivData_Mock : public yarp::dev::embObjDevPrivData
{
   public:
	MOCK_METHOD(bool, isOpen, (), (const, override));

	embObjDevPrivData_Mock(std::string name) : yarp::dev::embObjDevPrivData(name){};
};

class EthResource_Mock : public eth::EthResource
{
   public:
	MOCK_METHOD(bool, setcheckRemoteValue, (const eOprotID32_t, void *, const unsigned int, const double, const double), (override));
	MOCK_METHOD(bool, serviceSetRegulars, (eOmn_serv_category_t, vector<eOprotID32_t> &, double), (override));
};

TEST(CanBatterysensor, sendConfig2boards_simple_positive_001)
{
	// Setup
	embObjCanBatterysensor_Mock device;
	ServiceParserCanBattery_mock parser;
	parser.batteryInfo_ = {100, eobrd_bms, 0, 0, 0, 0, 0, 0, 0};

	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_battery_config);

	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, _, 10, 0.010, 0.050)).WillOnce(Return(true));

	// Test
	bool ret = device.sendConfig2boards(parser, &deviceRes);

	EXPECT_TRUE(ret);
}

TEST(CanBatterysensor, sendConfig2boards_simple_negative_001)
{
	// Setup
	embObjCanBatterysensor_Mock device;
	ServiceParserCanBattery_mock parser;
	parser.batteryInfo_ = {100, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0};
	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_battery_config);

	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, _, 10, 0.010, 0.050)).WillOnce(Return(false));

	// Test
	bool ret = device.sendConfig2boards(parser, &deviceRes);
	EXPECT_FALSE(ret);
}

TEST(CanBatterysensor, sendStart2boards_simple_positive_001)
{
	// Setup
	embObjCanBatterysensor_Mock device;
	ServiceParserCanBattery_mock parser;
	parser.batteryInfo_ = {100, eobrd_bms, 0, 0, 0, 0, 0, 0, 0};
	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_ft_cmmnds_enable);
	bool enable = 1;

	// EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, Pointee(enable), 10, 0.010, 0.050)).WillOnce(Return(true));
	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, _, 10, 0.010, 0.050)).WillOnce(Return(true));

	// Test
	bool ret = device.sendStart2boards(parser, &deviceRes);
	EXPECT_TRUE(ret);
}

TEST(CanBatterysensor, sendStart2boards_simple_negative_001)
{
	// Setup
	embObjCanBatterysensor_Mock device;
	ServiceParserCanBattery_mock parser;
	parser.batteryInfo_ = {100, eobrd_unknown, 0, 0, 0, 0, 0, 0, 0};
	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_ft_cmmnds_enable);
	bool enable = 1;

	// EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, Pointee(enable), 10, 0.010, 0.050)).WillOnce(Return(true));
	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, _, 10, 0.010, 0.050)).WillOnce(Return(false));

	// Test
	bool ret = device.sendStart2boards(parser, &deviceRes);
	EXPECT_FALSE(ret);
}

TEST(CanBatterysensor, serviceSetRegulars_simple_positive_001)
{
	// Setup
	embObjCanBatterysensor_Mock device;
	ServiceParserCanBattery_mock parser;
	parser.batteryInfo_ = {100, eobrd_bms, 0, 0, 0, 0, 0, 0, 0};
	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_battery_status_timedvalue);
	vector<eOprotID32_t> ids = {id32First};

	EXPECT_CALL(deviceRes, serviceSetRegulars(eomn_serv_category_battery, ids, _)).WillOnce(Return(true));

	// Test
	bool ret = device.initRegulars(parser, &deviceRes);
	EXPECT_TRUE(ret);
}

TEST(CanBatterysensor, serviceSetRegulars_simple_negative_001)
{
	// Setup
	embObjCanBatterysensor_Mock device;
	ServiceParserCanBattery_mock parser;
	parser.batteryInfo_ = {100, eobrd_bms, 0, 0, 0, 0, 0, 0, 0};
	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_battery_status_timedvalue);
	vector<eOprotID32_t> ids = {id32First};

	EXPECT_CALL(deviceRes, serviceSetRegulars(eomn_serv_category_battery, ids, _)).WillOnce(Return(false));

	// Test
	bool ret = device.initRegulars(parser, &deviceRes);
	EXPECT_FALSE(ret);
}

TEST(CanBatterysensor, update_simple_positive_001)
{
	// Setup
	yarp::os::Network::init();
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjCanBatterysensor_Mock device(privateData);
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_ft_status_timedvalue);
	eOas_battery_timedvalue_t data = {0 /*age*/, 1, 2, 3, 4, 5};
	CanBatteryData expected = {1, 2, 3, 4, 5, 7, ""};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));
	EXPECT_CALL(device, calculateBoardTime(_)).WillRepeatedly(Return(7));

	// Test
	bool ret = device.update(id32First, 1, (void *)&data);
	EXPECT_TRUE(ret);
	EXPECT_EQ(expected, device.canBatteryData_);
}

TEST(CanBatterysensor, update_simple_negative_001)
{
	// Setup
	yarp::os::Network::init();
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjCanBatterysensor_Mock device(privateData);
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_battery, 0, eoprot_tag_as_ft_status_timedvalue);
	eOas_battery_timedvalue_t data = {0 /*age*/, 1, 2, 3, 4, 9};
	CanBatteryData expected = {1, 2, 3, 4, 5, 7, ""};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));
	EXPECT_CALL(device, calculateBoardTime(_)).WillRepeatedly(Return(7));

	// Test
	bool ret = device.update(id32First, 1, (void *)&data);
	EXPECT_TRUE(ret);
	EXPECT_NE(expected, device.canBatteryData_);
}

TEST(CanBatterysensor, getBatteryVoltage_positive_001)
{
	// Setup
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjCanBatterysensor_Mock device(privateData);

	device.canBatteryData_ = {1, 2, 3, 4, 5, 6, ""};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	// Test
	double data;
	bool ret = device.getBatteryVoltage(data);

	double expected = 2;
	EXPECT_EQ(expected, data);
}

TEST(CanBatterysensor, calculateBoardTime_positive_001)
{
	// Setup
	yarp::os::Network::init();
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjCanBatterysensor_Mock device(privateData);

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	// Test
	double data;
	double timestamp;
	double first = device.embObjBattery::calculateBoardTime(1000000);
	double second = device.embObjBattery::calculateBoardTime(2000000);
	double third = device.embObjBattery::calculateBoardTime(3000000);
	double diff = second - first;

	EXPECT_TRUE(1.0 == diff);

	diff = third - first;

	EXPECT_TRUE(2.0 == diff);
}

TEST(CanBatterysensor, type)
{
	// Setup
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjCanBatterysensor_Mock device(privateData);

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	EXPECT_TRUE(device.type() == eth::iethres_analogbattery);
}
