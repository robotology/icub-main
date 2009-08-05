// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 *
 * \defgroup icub_cableLengthGuard cableLengthGuard 
 *
 * A basic module for checking whether the shoulder
 * cables have reached their length limit.
 *
 * \section intro_sec Description
 * 
 * This modules takes care of the fact that 
 * the shoulder cables pose additional constraints
 * on the shoulder joint positions. Software joint 
 * limits are usually imposed at the firmware level.
 * These software limits are expressed as:
 *
 *  min0 <= joint0 <= max0
 *
 *  min1 <= joint1 <= max1
 *
 *  min2 <= joint2 <= max2
 *
 * The values of min* and max* are specified in
 * the iCubInterface configuration file 
 * (e.g. app/iCubGenova01/conf/icub.ini) and can
 * be accessed via the getLimits() function.
 * The cable lengths impose additional constraints
 * on the joints. These additional constrints cannot
 * be expressed in the form above. Specifically,
 * the cable lengths impose contraints of the following
 * type:
 *
 * <ul>
 * <li> Ci0 * joint0 + Ci1 * joint1 + Ci2 * joint2 + di >= 0  i=0, 1, 2 ...
 * </ul>
 *
 * In matrix notation  we represent the constriaints as C*theta >= d
 * The module cableLengthGuard can be used to enforce
 * contraints of this form. The module has two actions:
 * 
 * <ul>
 * <li>(1) If during a movement the robot enters a configuration (theta)
 * which violates the constraints, the movement execution is
 * aborted and the robot is driven to a safe configuration.
 * 
 * <li>(2) If the a commanded movement aims at a configuration (theta_final)
 * which violates the constraints, the final position is 
 * aborted and a new position (theta_new) compatible with
 * the constraints is commanded. In particular theta_new is
 * computed as follows:
 * 
 *
 * min ||theta_new - theta_final||^2     s.t. C*theta_new >= d 
 *
 * In other words theta_new is compatible with the constraints
 * and as close as possible to the original target.
 * </ul> 
 * Remarkably the strategy (2) is only adopted if theta_final is
 * available (i.e. the command port is connected). Otherwise
 * only the strategy (1) is used.
 *
 * \section libraries_sec Libraries
 * This module requires GSL and an additional GSL package named
 * CQP for quadratic programming (see http://www.gnu.org/software/gsl)
 * 
 * \section parameters_sec Parameters
 * The module requires three parameters. The first one (--robot)
 * specifies the robot name (e.g. --robot icub). The second
 * (--part) specifies the used part (e.g. --part left_arm).
 * The thrid argument (--command) specifies the port name
 * that will be opened in order to receive the position commands.
 * Command positions has to be in the remote_control_board 
 * format (e.g. 'set pos 0 10.0' and 'set poss (0.0 10.0 ... 20.0)').
 * This information can be easily accessed by connecting 
 * the specified command port with the remote control board ports
 * (e.g. when trying to filter the commands coming from the 
 * robotMotorGui client a connection should be added from the 
 * /icub/gui/left_arm/command:o and the --command port). Commands
 * from the remote control baord rpc port (e.g. /icub/gui/left_arm/rpc:o)
 * are tricky to be filtered and require a rpcSniffer module
 * \ref icub_rpcSniffer.
 *
 * Moreover the modules requires a config file whose name
 * is determined by the used robot and part (e.g. --robot 
 * icub and --part head will require a config file named
 * config_icub_head.ini). The config file has the following
 * format: 
 *
 * \code
 * nJoints      n                //number of joints to be constrained
 *
 * joints       (i1 i2 i3...in)  //index of the used joint
 *
 * nConstr      k                //number of constraints (i.e. rows in C)
 *
 * d0           d0               //first element in d
 *
 * .
 * .
 * .
 *
 * dk           dk               //k-th element in d
 *
 * C0           (C00 ... C0n)    //first row in C
 *
 * .
 * .
 * .
 * 
 * Ck           (Ck0 ... Ckn)    //k-th row in C
 *
 * \endcode
 *
 * \section portsa_sec Ports Accessed
 * For each part initalized (e.g. right_arm):
 * <ul>
 * <li> /icub/right_arm/rpc:i 
 * <li> /icub/right_arm/command:i
 * <li> /icub/gui/right_arm/state:i
 * </ul>
 *
 * \section portsc_sec Ports Created
 * For each part initalized (e.g. right_arm):
 * <ul>
 * <li> /icub/cableChecker/right_arm/rpc:o
 * <li> /icub/cableChecker/right_arm/command:o
 * <li> /icub/cableChecker/right_arm/state:o
 * <li> an input command port whose name is spcified in the argument --command
 * </ul>
 *
 * \author Francesco Nori
 *
 * Copyright (C) 2008 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/cableLengthGuard/main.cpp.
 */


#include "cableLengthGuard.h"
//
int getNumberUsedJoints(Property p, int &n)
{
    if (!p.check("nJoints"))
        {
            fprintf(stderr, "Missing option 'nJoints' in given config file\n");
            return 0;
        }
    Value& nJoints = p.find("nJoints");
    n = nJoints.asInt();
    return 1;
}

int getUsedJointsMap(Property p, int n, int* thetaMap)
{
    if (!p.check("joints"))
        {
            fprintf(stderr, "Missing option 'joints' in given config file\n");
            return 0;
        }
    Value& joints =  p.find("joints");
    Bottle *pJoints = joints.asList();
    if (pJoints->size()!=n)
        {
            fprintf(stderr, "The 'nJoints' and 'joints' params are incompatible");
            return 0;
        }

    for (int i = 0; i < n; i++)
    {
        thetaMap[i] = pJoints->get(i).asInt();
        //fprintf(stderr, "%d ", thetaMap[i]);
    }
    return 1;
}

int getNumberConstraints(Property p, int &n)
{
    if (!p.check("nConstr"))
        {
            fprintf(stderr, "Missing option 'nConstr' in given config file\n");
            return 0;
        }
    Value& nJoints = p.find("nConstr");
    n = nJoints.asInt();
    return 1;
}

int getConstraintsVector(Property p,Vector &d, int r)
{
    
    char row[80]="";
    for (int i = 0; i < r; i++)
        {
            char rowName[80]="d";
            sprintf(row, "%d", i);
            strcat(rowName, row);
            if (!p.check(rowName))
                {
                    fprintf(stderr, "Missing option %s in given config file\n", rowName);
                    return 0;
                }
        }
    d.resize(r);
    for (int i=0; i<r; i++)
        {
            char rowName[80]="d";
            sprintf(row, "%d", i);
            strcat(rowName, row);
            d(i) = p.find(rowName).asDouble();
        }
    return 1;
}

int getConstraintsMatrix(Property p,Matrix &C, int r, int c)
{
    
    char row[80]="";
    for (int i = 0; i < r; i++)
        {
            char rowName[80]="C";
            sprintf(row, "%d", i);
            strcat(rowName, row);
            if (!p.check(rowName))
                {
                    fprintf(stderr, "Missing option %s in given config file\n", rowName);
                    return 0;
                }
            else
                {
                    Bottle* pRow = p.find(rowName).asList();
                    if (pRow->size()!=c)
                        {
                            fprintf(stderr, "The %s param in the config file has not the right dimension", rowName);
                            return 0;
                        }
                }
        }
    C.resize(r,c);
    for (int i=0; i<r; i++)
        for (int j=0; j<c; j++)
        {
            char rowName[80]="C";
            sprintf(row, "%d", i);
            strcat(rowName, row);
            Bottle* pRow = p.find(rowName).asList();
            //fprintf(stderr, "Getting (%d,%d) = %.1f", i, j, pRow->get(j).asDouble());
            C(i,j) = pRow->get(j).asDouble();
        }
    return 1;
}

int main(int argc, char *argv[]) 
{
    // get command line options
    Property options;
    options.fromCommand(argc, argv);
    if (!options.check("robot") 
        || !options.check("part")
        || !options.check("command") ) {
        ACE_OS::printf("Missing either --robot or --part or --command options\n");
        return 0;
    }

    Network::init();
	Time::turboBoost();
    
    yarp::String name((size_t)1024);
    Value& robot = options.find("robot");
    Value& part = options.find("part");
    Value& commandPort = options.find("command");

    // get command file options
    Property fileOptions;
    char configFileRobotPart[120] = "";
    strcat(configFileRobotPart, "config_");
    strcat(configFileRobotPart, robot.asString().c_str());
    strcat(configFileRobotPart, "_");
    strcat(configFileRobotPart, part.asString().c_str());
    strcat(configFileRobotPart, ".ini");
    if(!fileOptions.fromConfigFile(configFileRobotPart))
        {
            fprintf(stderr, "Couldn't find a file named %s.\nThis file is mandatory!\n", configFileRobotPart);
            return 1;
        }
    
    int nJoints;
    if (getNumberUsedJoints(fileOptions, nJoints) == 0)
        return 1;

    int* thetaMap = new int[nJoints];
    if (getUsedJointsMap(fileOptions, nJoints, thetaMap) == 0)
        return 1;

    int nConstr;
    if (getNumberConstraints(fileOptions, nConstr) == 0)
        return 1;
        
    Vector d;
    if (getConstraintsVector(fileOptions, d, nConstr)==0)
        return 1;
    fprintf(stderr, "%s\n", d.toString().c_str());

    Matrix C;
    if (getConstraintsMatrix(fileOptions, C, nConstr, nJoints)==0)
        return 1;
    fprintf(stderr, "%s\n", C.toString().c_str());

    //if (!fileOptions.check("d"))
    //    {
    //        fprintf(stderr, "Missing option 'd' in given config file\n");
    //        return 1;
    //    }
    //Value& d =  fileOptions.find("d");


    Property ddOptions;
    ddOptions.put("device", "remote_controlboard");
    
    ACE_OS::sprintf(&name[0], "/%s/cableChecker/%s", robot.asString().c_str(), part.asString().c_str());
    ddOptions.put("local", name.c_str());

    ACE_OS::sprintf(&name[0], "/%s/%s", robot.asString().c_str(), part.asString().c_str());
    ddOptions.put("remote", name.c_str());
    

	fprintf(stderr, "%s", ddOptions.toString().c_str());

    
    // create a device 
    PolyDriver dd(ddOptions);
    if (!dd.isValid()) {
        ACE_OS::printf("Device not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 1;
    }

    IPositionControl *ipos;
    int nJnts;
    if (dd.view(ipos))
        {
            ipos->getAxes(&nJnts);
            fprintf(stderr, "Number of axes is: %d \n", nJnts);
        }
    else
        {
            fprintf(stderr, "Unable to access the number of axes \n");
            return 1;
        }
    
    collectorPort *lastCommand = new collectorPort(nJnts);
    cableLengthGuard *cableChecker = new cableLengthGuard(&dd, lastCommand, C, d, 500);

    cableChecker->setThetaMap(thetaMap);

    //Start the port listener
    ACE_OS::sprintf(&name[0], "%s", commandPort.asString().c_str());
    lastCommand -> open(name.c_str());
    lastCommand -> start();

    //Start the cable length checker
    Time::delay(1);
    cableChecker->start();

    char cmd[80];
    bool quit=false;
    while (!quit) 
        {
            ACE_OS::printf("Type 'quit+enter' to exit the program\n");
            scanf("%s", cmd);
            if (strcmp(cmd, "quit")==0)
                quit=true;
        }
    fprintf(stderr, "Stopping cableChcker\n");
    cableChecker->stop();
    
    fprintf(stderr, "Closing port sniffer\n");
    lastCommand->close();
    Time::delay(1);
    fprintf(stderr, "Stopping port sniffer\n");
    lastCommand->stop();

    fprintf(stderr, "Deleting port sniffer class\n");
    delete lastCommand;
    fprintf(stderr, "Deleting cableCheck class\n");
    delete cableChecker;

    //finally close the dd
    fprintf(stderr, "Closing the device driver\n");
    dd.close();
    fprintf(stderr, "Device driver closed\n");

    delete[] thetaMap;

    Network::fini();
    return 0;
}
