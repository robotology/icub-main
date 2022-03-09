#include "gmock/gmock.h"
#include "gtest/gtest.h"

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
        ServiceParser_mock():ServiceParser(){};
};

TEST(General, base_positive_001)
{
    yarp::os::ResourceFinder& rf(yarp::os::ResourceFinder::getResourceFinderSingleton());
    rf.setDefaultConfigFile("./config.xml");
    yarp::os::Bottle bottle(rf.findGroup("SERVICE"));

    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_strain;
    bool error{false};
    bool ret=serviceParser.CheckSpecificForMultipleFT(bottle,type,error);

	EXPECT_TRUE(ret);
}

TEST(General, base_positive_002)
{
    yarp::os::ResourceFinder& rf(yarp::os::ResourceFinder::getResourceFinderSingleton());
    rf.setDefaultConfigFile("/home/triccyx/Documents/icub-main/build/bin/config.xml");
    std::string tmp=rf.toString();
    yarp::os::Bottle bottle(rf.findGroup("SERVICE"));


    ServiceParser_mock serviceParser;
    eOmn_serv_type_t type=eomn_serv_AS_ft;
    bool error{false};
    bool ret=serviceParser.CheckSpecificForMultipleFT(bottle,type,error);

	EXPECT_FALSE(ret);
}
