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
        using ServiceParser::CheckSpecificForMultipleFT;
        using ServiceParser::CheckPropertyCanBoards;
        using ServiceParser::CheckPropertySensors;
        using ServiceParser::CheckSettings;
        ServiceParser_mock():ServiceParser(){};
};

TEST(General, base_positive_001)
{
    yarp::os::Bottle bottle;
    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_strain;
    bool error{false};
    bool ret=serviceParser.CheckSpecificForMultipleFT(bottle,type,error);

	EXPECT_TRUE(ret);
}

TEST(General, base_positive_002)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(FT_SETTINGS (useCalibration false))");
  
    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    bool error{false};
    bool ret=serviceParser.CheckSpecificForMultipleFT(bottle,type,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
}

TEST(General, check_property_canboards_positive_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2) (PROTOCOL (major 2) (minor 0) ) (FIRMWARE (major 2) (minor 0) (build 7) ) )");
  
    ServiceParser_mock serviceParser;
    bool error{false};
    bool ret=serviceParser.CheckPropertyCanBoards(bottle,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
}

TEST(General, check_property_canboards_negative_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(CANBOARDS (type strain2) (PROTOCOL (xxxx 2) (minor 0) ) (FIRMWARE (major 2) (minor 0) (build 7) ) )");
  
    ServiceParser_mock serviceParser;
    bool error{false};
    bool ret=serviceParser.CheckPropertyCanBoards(bottle,error);

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
    bool ret=serviceParser.CheckPropertySensors(bottle,type,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
}

TEST(General, check_settings_positive_001)
{
    yarp::os::Bottle bottle;
    bottle.fromString("(SETTINGS (acquisitionRate 10) (enabledSensors fakeId) (temperature-acquisitionRate 1000) )");
  
    ServiceParser_mock serviceParser;
    bool error{false};

    servAnalogSensor_t toAdd={"fakeId",eoas_ft,{1,2},eobrd_strain2};
    serviceParser.as_service.properties.sensors.push_back(toAdd);
    
    bool ret=serviceParser.CheckSettings(bottle,error);

	EXPECT_TRUE(ret);
    EXPECT_FALSE(error);
}