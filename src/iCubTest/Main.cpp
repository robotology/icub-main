#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>

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
        iCubDriver::SetRobot(robot.asString());
    }

    iCubTestSet ts(references);

    yarp::os::Bottle testSet=rf.findGroup("TESTS").tail();

    for (int t=0; t<testSet.size(); ++t)
    {       
        yarp::os::Bottle test(testSet.get(t).toString());
        std::string testType(test.get(0).asString());
        std::string fileName(test.get(1).asString());

        printf("*%s\n",testType.c_str());
        printf("*%s\n",fileName.c_str());

        yarp::os::ResourceFinder testRf;
        testRf.setDefaultContext("tutorials/icubtest");
        testRf.setDefaultConfigFile(fileName.c_str());
        testRf.configure("ICUB_ROOT",argc,argv);

        if (testType=="iCubTestPart")
        {
            ts.AddTest(new iCubTestPart(testRf));
        }
        else if (testType=="iCubTestCamera")
        {
            //ts.AddTest(new iCubTestCamera(testRf));
        }
        else if (testType=="iCubTestInertial")
        {
            //ts.AddTest(new iCubTestInertial(testRf));
        }
        else if (testType=="iCubTestForceTorque")
        {
            //ts.AddTest(new iCubTestForceTorque(testRf));
        }
    }

    ts.run();
    ts.PrintReport();

    return 0;
}
