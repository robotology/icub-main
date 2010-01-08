#include <yarp/os/ResourceFinder.h>

#include "TestSet.h"
#include "TestPart.h"
#include "TestMotorInterface.h"

int main(int argc,char* argv[])
{
    yarp::os::ResourceFinder rf;
    //rf.setVerbose();
    rf.setDefaultContext("tutorials/icubtest");
    rf.setDefaultConfigFile("test.ini");
    rf.configure("ICUB_ROOT",argc,argv);
    
    yarp::os::Bottle references=rf.findGroup("REFERENCES");

    yarp::os::Value robot=references.find("robot");
    if (!robot.isNull())
    {
        printf("robot=%s\n",robot.asString().c_str());
        iCubMotorDriver::SetRobot(robot.asString());
    }

    iCubTestSet ts(references);

    yarp::os::Bottle test_set=rf.findGroup("TESTS").tail();

    for (int t=0; t<test_set.size(); ++t)
    {
        yarp::os::impl::String file_name(test_set.get(t).toString());
        printf("%s\n",file_name.c_str());

        yarp::os::ResourceFinder test_rf;
        test_rf.setDefaultContext("tutorials/icubtest");
        test_rf.setDefaultConfigFile(file_name.c_str());
        test_rf.configure("ICUB_ROOT",argc,argv);

        ts.AddTest(new iCubTestPart(test_rf));
    }

    ts.run();
    ts.PrintReport();

    return 0;
}
