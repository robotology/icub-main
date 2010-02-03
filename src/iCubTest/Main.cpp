// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Scalzo
 * email: alessandro.scalzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
*
@ingroup icub_module
\defgroup icub_iCubTest iCubTest

iCubTest can be used to test the health of an iCub robotic platform. 

\section intro_sec Description

iCubTest supplies automation to iCub robotic platforms testing activity. A test can be made of several subtests 
involving the iCub parts and components (i.e. motors, encoders, inertial unit, cameras, force-torque sensors, ...).
The subtests are executed in sequential order, and the results are saved in a .XML test report. A new iCubMyTest 
procedure can be created by deriving it from the base class iCubTest and implementing the virtual function run() 
and the constructor.

For example, a subtest can set target position angles for the iCub joints and then check if the robotic platform can
reach them measuring position errors and taking into account timeouts and other errors.

iCubMyTest can add entries to the main test .XML report. The iCubMyTestReportEntry class will be derived from the base 
class iCubTestReportEntry implementing the virtual functions print() and printStdio() and the constructor.

\section lib_sec Libraries

YARP_OS
YARP_sig
YARP_dev
YARP_math

\section parameters_sec Parameters
Provide a comprehensive list of the parameters you can pass to the module. For example:

--from mytest.ini: configuration file to use
--robot /icub01: robot name
--user "Alice Cooper": people executing the test
--outfile myReport.xml: report file name
--comment "...": textual comment

\section portsa_sec Ports Accessed

part: torso, head, left_arm, right_arm, left_leg, right_leg.

/icubXX/part/command:i <br>
/icubXX/part/rpc:i <br>
/icubXX/part/state:o

\section portsc_sec Ports Created

part: torso, head, left_arm, right_arm, left_leg, right_leg.

/iCubTest/part/command:o <br>
/iCubTest/part/rpc:o <br>
/iCubTest/part/state:i

\section out_data_sec Output Data Files

iCubTest produces a test report file in XML format.

Example:

\code
<report>
	<user>Alice Cooper</user>
	<comment>Testing left and right arms.</comment>
	<success>NO</success>
	<failures-total>4</failures-total>
	<test>
		<description>Set each joint position to 15.0 degrees</description>
		<references>
			<datetime>Wed Jan 20 16:32:39 2010</datetime>
			<part>00001</part>
		</references>
		<outcome>FAILURE</outcome>
		<failures>2</failures>
		<output>
			<name>Joint 0 position</name>
			<result>SUCCESS</result>
			<target>15.000000</target>
			<value>14.036245</value>
			<rangemin>12.000000</rangemin>
			<rangemax>18.000000</rangemax>
		</output>
		<output>
			<name>Joint 1 position</name>
			<result>FAILED: value out of range</result>
			<target>-30.000000</target>
			<value>-0.000000</value>
			<rangemin>-27.000000</rangemin>
			<rangemax>-33.000000</rangemax>
		</output>
            ...
            ...
            ...
    </test>
    <test>
        ...
        ...
        ...
    </test>
    ...
    ...
    ...
</report>
\endcode
 
\section conf_file_sec Configuration Files

iCubTest needs a main configuration and one configuration file for each subtest.

The main configuration file consists of the following sections:

The file consists in a few sections:
\code
[REFERENCES]
user "Alice Cooper"
comment "Testing left and right arms."
outfile /home/user/TestReport3.xml
robot /icub01
[TESTS]
iCubTestMotors test_left_arm.ini
iCubTestMotors test_right_arm.ini
\endcode

\e robot robot's name
\e user people executing the test
\e outfile report file name
\e comment textual comment

\e iCubTestMotors test type 
\e test_left_arm.ini test configuration file

\section tested_os_sec Tested OS

Linux and Windows.

\section example_sec Example Instantiation of the Module

iCubTest --from mytest.ini

\author Alessandro Scalzo

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/iCubTest/main.cpp.
**/


#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>

#include "TestSet.h"
#include "TestMotors.h"
#include "DriverInterface.h"

int main(int argc,char* argv[])
{
    // do not forget to initialize yarp! --nat.
    yarp::os::Network yarp;

    yarp::os::ResourceFinder rf;
    //rf.setVerbose();
    rf.setDefaultContext("tutorials/iCubTest");
    rf.setDefaultConfigFile("test.ini");
    rf.setDefault("robot","/icub");

    rf.configure("ICUB_ROOT",argc,argv);

    yarp::os::Bottle references=rf.findGroup("REFERENCES");

    if (references.check("robot"))
    {
        iCubDriver::setRobot(references.find("robot").asString());
    }

    // create the test set
    iCubTestSet ts(references);

    // add tests to the test set
    yarp::os::Bottle testSet=rf.findGroup("TESTS").tail();

    for (int t=0; t<testSet.size(); ++t)
    {       
        yarp::os::Bottle test(testSet.get(t).toString());
        std::string testType(test.get(0).asString());
        std::string fileName(test.get(1).asString());

        yarp::os::ResourceFinder testRf;
        testRf.setDefaultContext("tutorials/iCubTest");
        testRf.setDefaultConfigFile(fileName.c_str());
        testRf.configure("ICUB_ROOT",argc,argv);

        if (testType=="iCubTestMotors")
        {
            ts.addTest(new iCubTestMotors(testRf));
        }
        else if (testType=="iCubTestCamera")
        {
            //ts.AddTest(new iCubTestCamera(testRf));
            fprintf(stderr,"iCubTestCamera not yet implemented\n");
        }
        else if (testType=="iCubTestInertial")
        {
            //ts.AddTest(new iCubTestInertial(testRf));
            fprintf(stderr,"iCubTestInertial not yet implemented\n");
        }
        else if (testType=="iCubTestForceTorque")
        {
            //ts.AddTest(new iCubTestForceTorque(testRf));
            fprintf(stderr,"iCubTestForceTorque not yet implemented\n");
        }
    }

    // execute tests
    int numFailures=ts.run();
    
    // save test reports in XML format
    ts.printReport();
    
    // return number of failed tests
    return numFailures;
}
