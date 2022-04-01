/*
 * Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Luca Tricerri
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <embObjMultipleFTsensors.h>
#include <ethResource.h>
#include "EoProtocolAS.h"

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

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

class embObjMultipleFTsensor_Mock : public yarp::dev::embObjMultipleFTsensors
{
   public:
	using yarp::dev::embObjMultipleFTsensors::ftSensorsData_;
	using yarp::dev::embObjMultipleFTsensors::initRegulars;
	using yarp::dev::embObjMultipleFTsensors::sendConfig2boards;
	using yarp::dev::embObjMultipleFTsensors::sendStart2boards;
	using yarp::dev::embObjMultipleFTsensors::temperaturesensordata_;
	using yarp::dev::embObjMultipleFTsensors::embObjMultipleFTsensors::getSixAxisForceTorqueSensorMeasure;
	using yarp::dev::embObjMultipleFTsensors::embObjMultipleFTsensors::update;

	embObjMultipleFTsensor_Mock(std::shared_ptr<yarp::dev::embObjDevPrivData> device) : yarp::dev::embObjMultipleFTsensors(device){};
	embObjMultipleFTsensor_Mock() : yarp::dev::embObjMultipleFTsensors(){};
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

TEST(MultiplembObjMultipleFTsensor, sendConfig2boards_simple_positive_001)
{
	// Setup
	embObjMultipleFTsensor_Mock device;
	ServiceParserMultipleFt_mock parser;
	parser.ftInfo_ = {{"fakeId", {10, 200, eoas_ft_mode_calibrated, "", 0, 0, 0, 0, 0, 0, 0}}};
	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 0, eoprot_tag_as_ft_config);

	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, _, 10, 0.010, 0.050)).WillOnce(Return(true));

	// Test
	bool ret = device.sendConfig2boards(parser, &deviceRes);

	EXPECT_TRUE(ret);
}

TEST(MultiplembObjMultipleFTsensor, sendConfig2boards_double_sensor_positive_001)
{
	// Setup
	embObjMultipleFTsensor_Mock device;
	ServiceParserMultipleFt_mock parser;
	parser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, "", 0, 0, 0, 0, 0, 0, 0}}, {"fakeId2", {10, 100, eoas_ft_mode_calibrated, "", 0, 0, 0, 0, 0, 0, 0}}};
	EthResource_Mock deviceRes;
	eOas_ft_config_t cfg = {eoas_ft_mode_calibrated, 10, 0, 100};
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 0, eoprot_tag_as_ft_config);
	uint32_t id32Second = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 1, eoprot_tag_as_ft_config);

	// EXPECT_CALL(deviceRes, setcheckRemoteValue(34013184, Pointee(cfg), 10, 0.010, 0.050)).WillOnce(Return(true));
	// EXPECT_CALL(deviceRes, setcheckRemoteValue(34013440, Pointee(cfg), 10, 0.010, 0.050)).WillOnce(Return(true));
	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, _, 10, 0.010, 0.050)).WillOnce(Return(true));
	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32Second, _, 10, 0.010, 0.050)).WillOnce(Return(true));

	bool ret = device.sendConfig2boards(parser, &deviceRes);

	EXPECT_TRUE(ret);
}

TEST(MultiplembObjMultipleFTsensor, sendConfig2boards_simple_negative_001)
{
	// Setup
	embObjMultipleFTsensor_Mock device;
	ServiceParserMultipleFt_mock parser;
	parser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, "", 0, 0, 0, 0, 0, 0, 0}}};
	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 0, eoprot_tag_as_ft_config);

	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, _, 10, 0.010, 0.050)).WillOnce(Return(false));

	// Test
	bool ret = device.sendConfig2boards(parser, &deviceRes);
	EXPECT_FALSE(ret);
}

TEST(MultiplembObjMultipleFTsensor, sendStart2boards_simple_positive_001)
{
	// Setup
	embObjMultipleFTsensor_Mock device;
	ServiceParserMultipleFt_mock parser;
	parser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, "", 0, 0, 0, 0, 0, 0, 0}}};
	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 0, eoprot_tag_as_ft_cmmnds_enable);
	bool enable = 1;

	// EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, Pointee(enable), 10, 0.010, 0.050)).WillOnce(Return(true));
	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, _, 10, 0.010, 0.050)).WillOnce(Return(true));

	// Test
	bool ret = device.sendStart2boards(parser, &deviceRes);
	EXPECT_TRUE(ret);
}

TEST(MultiplembObjMultipleFTsensor, sendStart2boards_double_positive_001)
{
	// Setup
	embObjMultipleFTsensor_Mock device;
	ServiceParserMultipleFt_mock parser;
	parser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, "", 0, 0, 0, 0, 0, 0, 0}}, {"fakeId1", {10, 100, eoas_ft_mode_calibrated, "", 0, 0, 0, 0, 0, 0, 0}}};
	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 0, eoprot_tag_as_ft_cmmnds_enable);
	uint32_t id32Second = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 1, eoprot_tag_as_ft_cmmnds_enable);
	bool enable = 1;

	// EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, Pointee(enable), 10, 0.010, 0.050)).WillOnce(Return(true));
	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32First, _, 10, 0.010, 0.050)).WillOnce(Return(true));
	EXPECT_CALL(deviceRes, setcheckRemoteValue(id32Second, _, 10, 0.010, 0.050)).WillOnce(Return(true));

	// Test
	bool ret = device.sendStart2boards(parser, &deviceRes);
	EXPECT_TRUE(ret);
}

TEST(MultiplembObjMultipleFTsensor, serviceSetRegulars_simple_positive_001)
{
	// Setup
	embObjMultipleFTsensor_Mock device;
	ServiceParserMultipleFt_mock parser;
	parser.ftInfo_ = {{"fakeId", {10, 100, eoas_ft_mode_calibrated, "", 0, 0, 0, 0, 0, 0, 0}}, {"fakeId1", {10, 100, eoas_ft_mode_calibrated, "", 0, 0, 0, 0, 0, 0, 0}}};
	EthResource_Mock deviceRes;
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 0, eoprot_tag_as_ft_status_timedvalue);
	uint32_t id32Second = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 1, eoprot_tag_as_ft_status_timedvalue);
	vector<eOprotID32_t> ids = {id32First, id32Second};

	EXPECT_CALL(deviceRes, serviceSetRegulars(eomn_serv_category_ft, ids, _)).WillOnce(Return(true));

	// Test
	bool ret = device.initRegulars(parser, &deviceRes);
	EXPECT_TRUE(ret);
}

TEST(MultiplembObjMultipleFTsensor, update_simple_positive_001)
{
	// Setup
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjMultipleFTsensor_Mock device(privateData);
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 0, eoprot_tag_as_ft_status_timedvalue);
	eOas_ft_timedvalue_t data = {100, 1, 2, 3, {5, 6, 7, 8, 9, 10}};
	yarp::sig::Vector expected = {5, 6, 7, 8, 9, 10};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	// Test
	bool ret = device.update(id32First, 1, (void *)&data);
	EXPECT_TRUE(ret);
	EXPECT_EQ(expected, device.ftSensorsData_[0].data_);
	EXPECT_EQ(3, device.temperaturesensordata_[0].data_);
}

TEST(MultiplembObjMultipleFTsensor, update_simple_positive_002)
{
	// Setup
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjMultipleFTsensor_Mock device(privateData);
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 1, eoprot_tag_as_ft_status_timedvalue);
	eOas_ft_timedvalue_t data = {100, 1, 2, 3, {5, 6, 7, 8, 9, 10}};
	yarp::sig::Vector expected = {5, 6, 7, 8, 9, 10};
	yarp::sig::Vector expectedEmpty = {0, 0, 0, 0, 0, 0};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	// Test
	bool ret = device.update(id32First, 1, (void *)&data);
	EXPECT_TRUE(ret);
	EXPECT_EQ(expectedEmpty, device.ftSensorsData_[0].data_);
	EXPECT_EQ(expected, device.ftSensorsData_[1].data_);
	EXPECT_EQ(0, device.temperaturesensordata_[0].data_);
	EXPECT_EQ(3, device.temperaturesensordata_[1].data_);
}

TEST(MultiplembObjMultipleFTsensor, update_negative_001)
{
	// Setup
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjMultipleFTsensor_Mock device(privateData);
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 4, eoprot_tag_as_ft_status_timedvalue);
	eOas_ft_timedvalue_t data = {100, 1, 2, 3, {5, 6, 7, 8, 9, 10}};
	yarp::sig::Vector expected = {5, 6, 7, 8, 9, 10};
	yarp::sig::Vector expectedEmpty = {0, 0, 0, 0, 0, 0};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	// Test
	bool ret = device.update(id32First, 1, (void *)&data);
	EXPECT_FALSE(ret);
}

TEST(MultiplembObjMultipleFTsensor, update_negative_002)
{
	// Setup
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjMultipleFTsensor_Mock device(privateData);
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_pos, 3, eoprot_tag_as_ft_status_timedvalue);
	eOas_ft_timedvalue_t data = {100, 1, 2, 3, {5, 6, 7, 8, 9, 10}};
	yarp::sig::Vector expected = {5, 6, 7, 8, 9, 10};
	yarp::sig::Vector expectedEmpty = {0, 0, 0, 0, 0, 0};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	// Test
	bool ret = device.update(id32First, 3, (void *)&data);
	EXPECT_FALSE(ret);
}

TEST(MultiplembObjMultipleFTsensor, update_negative_003)
{
	// Setup
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjMultipleFTsensor_Mock device(privateData);
	uint32_t id32First = eoprot_ID_get(eoprot_endpoint_analogsensors, eoprot_entity_as_ft, 3, eoprot_tag_as_ft_status);
	eOas_ft_timedvalue_t data = {100, 1, 2, 3, {5, 6, 7, 8, 9, 10}};
	yarp::sig::Vector expected = {5, 6, 7, 8, 9, 10};
	yarp::sig::Vector expectedEmpty = {0, 0, 0, 0, 0, 0};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	// Test
	bool ret = device.update(id32First, 3, (void *)&data);
	EXPECT_FALSE(ret);
}

TEST(MultiplembObjMultipleFTsensor, getNrOfSixAxisForceTorqueSensors_positive_001)
{
	// Setup
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjMultipleFTsensor_Mock device(privateData);

	device.ftSensorsData_ = {{{0}, {{1, 2, 3, 4, 5, 6}, 99.49}}};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	// Test
	yarp::sig::Vector data;
	double timestamp;
	bool ret = device.getSixAxisForceTorqueSensorMeasure(0, data, timestamp);

	yarp::sig::Vector expected = {1, 2, 3, 4, 5, 6};
	EXPECT_EQ(expected, data);
    EXPECT_EQ( 99.49, timestamp);
}

TEST(MultiplembObjMultipleFTsensor, getNrOfSixAxisForceTorqueSensors_double_positive_001)
{
	// Setup
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjMultipleFTsensor_Mock device(privateData);

	device.ftSensorsData_ = {{{0}, {{1, 2, 3, 4, 5, 6}, 99.49}},{{1}, {{10, 20, 30, 40, 50, 60}, 99.49}}};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	// Test
	yarp::sig::Vector data;
	double timestamp;
	bool ret = device.getSixAxisForceTorqueSensorMeasure(1, data, timestamp);

	yarp::sig::Vector expected = {10, 20, 30, 40, 50, 60};
	EXPECT_EQ(expected, data);
    EXPECT_EQ( 99.49, timestamp);
}

TEST(MultiplembObjMultipleFTsensor, getNrOfSgetTemperatureSensorMeasure_positive_001)
{
	// Setup
	std::shared_ptr<embObjDevPrivData_Mock> privateData = std::make_shared<embObjDevPrivData_Mock>("test");
	embObjMultipleFTsensor_Mock device(privateData);

	device.temperaturesensordata_ = {{{0}, {34, 99.49}}};

	EXPECT_CALL(*privateData, isOpen()).WillRepeatedly(Return(true));

	// Test
	double data;
	double timestamp;
	bool ret = device.getTemperatureSensorMeasure(0, data, timestamp);

	double expected = 34;
	EXPECT_EQ(expected, data);
    EXPECT_EQ( 99.49, timestamp);
}