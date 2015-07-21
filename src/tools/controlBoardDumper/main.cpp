// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Francesco Nori
 * email:  francesco.nori@iit.it
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
 * @ingroup icub_tools
 *
 * \defgroup icub_controlBoardDumper controlBoardDumper
 *
 * A basic module for collecting data (position, current, 
 * applied voltage, pid errors, ...) from a controlBoard
 * and sending them trough a yarp port. These data can then
 * be dumped to a file using yarpdatadumper. Time stamp is
 * forwarded if present in the collected data. If not present
 * a time stamp is added. <b> Remark: </b> all collected data
 * are obtained by a remoteControlBoard connected with a controlBoard.
 * Therefore, collected data needs to be available to the controlBoard.
 * When using the iCub canbus protocol this is not necessary guaranteed
 * for all data: data are available only if the corresponding broadcast
 * is enabled. This can be easily done by seeting the proper entry in 
 * the controlBoard configuration file (e.g. icub_right_arm_safe.ini):
 *      
 * <ul>
 * <li> broadcast_pos: if 1 enables getEncoders (joint position)
 * <li> broadcast_vel_acc: if 1 enables getEncoderSpeeds (joint velocity)
 *      and getEncoderAccelerations (joint acceleration)
 * <li> broadcast_pid: if 1 enables getOutputs (voltage give to the motor)
 * <li> broadcast_pid_err: if 1 enables getPositionErrors and getTorqueErrors (pid tracking error)
 * <li> broadcast_current: if 1 enables getCurrents (current given to the motor)
 * </ul>
 *
 * \section intro_sec Description
 * 
 * Data from a control board can be easily accessed trough
 * suitable interfaces (IPositionControl, IAmplifierControl, 
 * IPidControl ...) and corresponding functions (getEncoders,
 * getCurrrents, getOutputs, ...). The module (controlBoardDumper)
 * accesses all these data and make them available on the 
 * network opening suitable yarp ports. The data are collected
 * at a fixed frequency by a suitable thread whose frequency can 
 * be changed. 
 *
 * \section libraries_sec Libraries
 * Yarp libraries.
 * 
 * 
 * \section parameters_sec Parameters
 * The module uses the ResourceFinder class as a way to retrieve its
 * own configuration. In particular, the default config file is
 * controlBoardDumper.ini and it is assumed to be in the app/dumpControlBoardData/conf
 * directory. The file structure is the following:
 *
 * \code
 *
 * robot        icub             //the robot name [default: icub]
 *
 * part         p                //the robot part to be dumped [default: head]
 *
 * rate         r                //[ms] the sampling time for collected data [default: 500]
 *
 * joints       (i1 i2 i3...iN)  //indexes of the joints to be read [default: (0 1)]
 *
 * dataToDump   (data1...dataN)  //type of data to be collected [default: all possible data, except debug interface (getRotorxxx)]
 *
 * dataToDumpAll                 // will dump all data, including the debug interface (getRotorxxx)
 *
 * logToFile                     //if present, this options creates a log file for each data port
 *
 * \endcode
 * 
 * If no such file can be found, the application is started
 * with the default parameters. The first parameter ('robot' with
 * default value 'icub') specifies the robot name. The second 
 * parameter ('part' with default 'head') specifies the used part.
 * The third parameter ('rate' with default '500') specifies the
 * acquisition rate.
 * The parameter dataToDump can assume the following values:
 * <ul>
 * <li> getEncoders             (joint position)
 * <li> getEncoderSpeeds        (joint velocity)
 * <li> getEncoderAccelerations (joint acceleration)
 * <li> getPositionErrors       (difference between desired and actual position)
 * <li> getTorqueErrors         (difference between desired and measured torque, if available)
 * <li> getOutputs              (voltage (PWM) given to the motor)
 * <li> getCurrents             (current given to the motor)
 * <li> getTorques              (joint torques, if available)
 * <li> getPosPidReferences     (last position referenece)
 * <li> getTrqPidReferences     (last torque reference)
 * <li> getRotorPositions       (hires rotor position, if debug interface is available)
 * <li> getRotorSpeeds          (hires rotor velocity, if debug interface is available)
 * <li> getRotorAccelerations   (hires rotor acceleration, if debug interface is available)
 * <li> getControlModes         (joint control mode)
 * <li> getInteractionModes     (joint interaction mode)
 * </ul>
 * Other data can be easily added by modyfing the classes in the 
 * file genericControlBoardDumper.cpp which contains a class named
 * GetData with a method getData which can be used to instantiate
 * a thread controlBoardDumper which does not depend on the specific
 * interface (e.g. IPositionControl) or function (e.g. getEncoders).
 *
 * Please note that for dumping the getRototxxx data the debugInterface is required. This means the robot must instantiate the
 * debugInterfaceWrapper in order to have remote access to those data.
 *
 * The module can also be executed from a command line with the following format:
 * \code
 *
 * controlBoardDumper --robot r --part p --rate r --joints "(i1 i2 i3...iN)"
 *
 * \endcode
 * for example:
 * \code
 *
 * controlBoardDumper --robot icub --part head --rate 20 --joints "(0 1 2)"
 * \endcode
 *
 * In this case all the following standard data will be dumped:
 * <li> getEncoders
 * <li> getEncoderSpeeds
 * <li> getEncoderAccelerations
 * <li> getPositionErrors
 * <li> getTorqueErrors
 * <li> getOutputs
 * <li> getCurrents
 * <li> getTorques
 *
 * To dump only specific data you can use the --dataToDump option
 *
 * \code
 * controlBoardDumper --robot r --part p --rate r --joints "(i1 i2 i3...iN)" --dataToDump "(data1...dataN)"
 * \endcode
 *
 * and the example is:
 * \code
 *
 * controlBoardDumper --robot icub --part head --rate 20 --joints "(0 1 2)" --dataToDump "(getCurrents getOutputs)"
 *
 * \endcode
 *
 * To dump all the data, including the debug ones, you can use the dataToDumpAll option
 * controlBoardDumper --robot r --part p --rate r --joints "(i1 i2 i3...iN)" --dataToDumpAll
 *
 * \endcode
 * and the example is:
 * \code
 *
 * controlBoardDumper --robot icub --part head --rate 20 --joints "(0 1 2)" --dataToDumpAll
 *
 * \endcode
 *
 * \section portsa_sec Ports Accessed
 * For each part initalized (e.g. head):
 * <ul>
 * <li> /icub/head/rpc:i
 * <li> /icub/head/command:i
 * <li> /icub/head/state:i
 * </ul>
 *
 * \section portsc_sec Ports Created
 * For each part initalized (e.g. head):
 * <ul>
 * <li> /robot/controlBoardDumper/part/data e.g. /icub/controlBoardDumper/head/getEncoders
 * </ul>
 *
 * \author Francesco Nori
 *
 * Copyright (C) 2008 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/controlBoardDumper/main.cpp.
 */


#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Module.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include "dumperThread.h"
#include <string>

#define NUMBER_OF_AVAILABLE_STANDARD_DATA_TO_DUMP 16
#define NUMBER_OF_AVAILABLE_DEBUG_DATA_TO_DUMP 3


//
void getRate(Property p, int &r)
{
    if (!p.check("rate"))
        r = 500;
    else
    {
        Value& rate = p.find("rate");
        r = rate.asInt();
    }
}

int getNumberUsedJoints(Property p, int &n)
{
    if (!p.check("joints"))
    {
        yError("Missing option 'joints' in given config file\n");
        return 0;
    }
    Value& joints =  p.find("joints");
    Bottle *pJoints = joints.asList();
    if (pJoints == 0)
    {
         yError("Error in option 'joints'\n");
         return 0;
    }
    n = pJoints->size();

    return 1;
}

int getUsedJointsMap(Property p, int n, int* thetaMap)
{
    if (!p.check("joints"))
        {
            yError("Missing option 'joints' in given config file\n");
            return 0;
        }
    Value& joints =  p.find("joints");
    Bottle *pJoints = joints.asList();
    if (pJoints == 0)
    {
        yError("Error in option 'joints'\n");
        return 0;
    }
    if (pJoints->size()!=n)
    {
        yError("The 'nJoints' and 'joints' params are incompatible");
        return 0;
    }

    for (int i = 0; i < n; i++)
    {
        thetaMap[i] = pJoints->get(i).asInt();
    }
    return 1;
}

int getNumberConstraints(Property p, int &n)
{
    if (!p.check("nConstr"))
    {
        yError("Missing option 'nConstr' in given config file\n");
        return 0;
    }
    Value& nJoints = p.find("nConstr");
    n = nJoints.asInt();
    return 1;
}

int getNumberDataToDump(Property p, int &n)
{
    if (!p.check("dataToDump"))
    {
        yError("Missing option 'dataToDump' in given config file\n");
        return 0;
    }
    Value& list = p.find("dataToDump");
    Bottle *pList = list.asList();
    if (pList == 0)
    {
            yError("Error in option 'dataToDump'\n");
            return 0;
    }
    n = pList->size();
    return 1;
}

int getDataToDump(Property p, std::string *listOfData, int n, bool *needDebug)
{

    std::string availableDataToDump[NUMBER_OF_AVAILABLE_STANDARD_DATA_TO_DUMP];
    std::string availableDebugDataToDump[NUMBER_OF_AVAILABLE_DEBUG_DATA_TO_DUMP];

    int j;

    // standard
    availableDataToDump[0]  = ConstString("getEncoders");
    availableDataToDump[1]  = ConstString("getEncoderSpeeds");
    availableDataToDump[2]  = ConstString("getEncoderAccelerations");
    availableDataToDump[3]  = ConstString("getPositionErrors");
    availableDataToDump[4]  = ConstString("getOutputs");
    availableDataToDump[5]  = ConstString("getCurrents");
    availableDataToDump[6]  = ConstString("getTorques");
    availableDataToDump[7]  = ConstString("getTorqueErrors");
    availableDataToDump[8]  = ConstString("getPosPidReferences");
    availableDataToDump[9]  = ConstString("getTrqPidReferences");
    availableDataToDump[10] = ConstString("getControlModes");
    availableDataToDump[11] = ConstString("getInteractionModes");
    availableDataToDump[12] = ConstString("getMotorEncoders");
    availableDataToDump[13] = ConstString("getMotorSpeeds");
    availableDataToDump[14] = ConstString("getMotorAccelerations");
    availableDataToDump[15] = ConstString("getTemperature");

    // debug
    availableDebugDataToDump[0] = ConstString("getRotorPositions");
    availableDebugDataToDump[1] = ConstString("getRotorSpeeds");
    availableDebugDataToDump[2] = ConstString("getRotorAccelerations");

    if (!p.check("dataToDump"))
    {
        yError("Missing option 'dataToDump' in given config file\n");
        return 0;
    }

    Value& list = p.find("dataToDump");
    Bottle *pList = list.asList();

    for (int i = 0; i < n; i++)
    {
        listOfData[i] = pList->get(i).toString();
        // check if the requested data is a standard one
        for(j = 0; j< NUMBER_OF_AVAILABLE_STANDARD_DATA_TO_DUMP; j++)
        {
            if(listOfData[i] == (availableDataToDump[j]))
                break;
        }
        // check if the requested data is a debug one
        for(j = 0; j< NUMBER_OF_AVAILABLE_DEBUG_DATA_TO_DUMP; j++)
        {
            if(listOfData[i] == (availableDebugDataToDump[j]))
            {
                *needDebug = true;
                break;
            }
        }
        // if not found
        if(j == (NUMBER_OF_AVAILABLE_STANDARD_DATA_TO_DUMP + NUMBER_OF_AVAILABLE_DEBUG_DATA_TO_DUMP))
        {
            yError("Illegal values for 'dataToDump': %s does not exist!\n",listOfData[i].c_str());
            return 0;
        }
    }
   return 1;
}


class DumpModule: public Module
{
private:
    Property options;
    Property fileOptions;
    Property ddBoardOptions;
    Property ddDebugOptions;
    ConstString name;
    ConstString configFileRobotPart;
    ConstString portPrefix;

    PolyDriver ddBoard;
    PolyDriver ddDebug;
    bool useDebugClient;

    //get the joints to be dumped
    int rate;
    int nJoints;
    int* thetaMap;
    std::string *dataToDump;  
    int nData;

    boardDumperThread *myDumper;

    //time stamp
    IPreciselyTimed *istmp;
    //encoders
    IEncoders *ienc;
    GetSpeeds myGetSpeeds;
    GetAccs myGetAccs;
    GetEncs myGetEncs;
    //motor
    IMotor *imot;
    GetTemps myGetTemps;
    //motor encoders
    IMotorEncoders *imotenc;
    GetMotSpeeds myGetMotorSpeeds;
    GetMotAccs myGetMotorAccs;
    GetMotEncs myGetMotorEncs;
    //pid
    IPidControl *ipid;
    GetPosErrs myGetPosErrs;
    GetPidRefs myGetPidRefs;
    GetOuts myGetOuts;
    //amp
    IAmplifierControl *iamp;
    GetCurrs myGetCurrs;
    //torques
    ITorqueControl  *itrq;
    GetTrqs    myGetTrqs;
    GetTrqRefs myGetTrqRefs;
    GetTrqErrs myGetTrqErrs;
    //debug
    IDebugInterface *idbg;
    GetRotorPosition     myGetRotorPoss;
    GetRotorSpeed        myGetRotorVels;
    GetRotorAcceleration myGetRotorAccs;
    //modes
    IControlMode2       *icmod;
    IInteractionMode    *iimod;
    GetControlModes     myGetControlModes;
    GetInteractionModes myGetInteractionModes;

public:
    DumpModule() : useDebugClient(false)
    { 
        istmp=0;
        ienc=0;
        imotenc=0;
        ipid=0;
        iamp=0;
        itrq=0;
        idbg=0;
        icmod=0;
        iimod=0;
    }

    virtual bool open(Searchable &s)
    {

        Time::turboBoost();

        // get command line options
        options.fromString(s.toString());
        if (!options.check("robot") || !options.check("part")) 
        {
            printf("Missing either --robot or --part options. Quitting!\n");
            return false;
        }

        std::string dumpername; 
        // get dumepr name
        if (options.check("name"))
        {
            dumpername = options.find("name").asString();
            dumpername += "/";
        }
        else
        {
            dumpername = "/controlBoardDumper/";
        }

        Value& robot = options.find("robot");
        Value& part = options.find("part");
        configFileRobotPart = "config_";
        configFileRobotPart = configFileRobotPart + robot.asString();
        configFileRobotPart = configFileRobotPart + "_";
        configFileRobotPart = configFileRobotPart + part.asString();
        configFileRobotPart = configFileRobotPart + ".ini";

        //get the joints to be dumped
        getRate(options, rate);

        if (getNumberUsedJoints(options, nJoints) == 0)
            return false;

        thetaMap = new int[nJoints];
        if (getUsedJointsMap(options, nJoints, thetaMap) == 0)
            return false;

        //get the type of data to be dumped
        if (getNumberDataToDump(options, nData) == 0)
            return false;

        dataToDump = new std::string[nData];
        if (getDataToDump(options, dataToDump, nData, &useDebugClient) == 0)
            return false;        


        // Printing configuration to the user
        yInfo("Running with the following configuration:\n");
        yInfo("Selected rate is: %d\n", rate);
        yInfo("Data selected to be dumped are:\n");
        for (int i = 0; i < nData; i++)
            yInfo("\t%s \n", dataToDump[i].c_str());    

        //open remote control board
        ddBoardOptions.put("device", "remote_controlboard");
        ddDebugOptions.put("device", "debugInterfaceClient");
    
        ConstString localPortName = name;
        ConstString localDebugPortName = name;
        localPortName = localPortName + dumpername.c_str();
        localDebugPortName = localPortName + "debug/";
        //localPortName = localPortName + robot.asString();
        localPortName = localPortName + part.asString();
        localDebugPortName = localDebugPortName + part.asString();
        ddBoardOptions.put("local", localPortName.c_str());
        ddDebugOptions.put("local", localDebugPortName.c_str());

        ConstString remotePortName = "/";
        ConstString remoteDebugPortName;
        remotePortName = remotePortName + robot.asString();
        remotePortName = remotePortName + "/";
        remotePortName = remotePortName + part.asString();
        ddBoardOptions.put("remote", remotePortName.c_str());

        remoteDebugPortName = remotePortName + "/debug";
        ddDebugOptions.put("remote", remoteDebugPortName.c_str());
    
        // create a device 
        ddBoard.open(ddBoardOptions);
        if (!ddBoard.isValid()) {
            printf("Device not available.\n");
            Network::fini();
            return false;
        }

        if (useDebugClient )
        {
            ddDebug.open(ddDebugOptions);
            if (!ddDebug.isValid())
            {
                yError("\n-----------------------------------------\n");
                yError("Debug Interface is mandatory to run this module with the '--dataToDumpAll' option or to dump any of the getRotorxxx data.\n");
                yError("Please Verify the following 2 conditions are satisfied:\n\n");
                yError("1) Check 'debugInterfaceClient' is available using 'yarpdev --list' command\n");
//                yError("%s", Drivers::factory().toString().c_str());

                std::string deviceList, myDev;
                deviceList.clear();
                deviceList.append(Drivers::factory().toString().c_str());
                myDev = "debugInterfaceClient";
                if(deviceList.find(myDev) != std::string::npos)
                    yError("\t--> Seems OK\n");
                else
                    yError("\t--> Seems NOT OK. The device was not found, please activate the compilation using the corrisponding CMake flag.\n");

                yError("\n2) Check if the robot has the 'debugInterfaceWrapper' device up and running. \n You should see from 'yarp name list' output, a port called\n");
                yError("\t/robotName/part_name/debug/rpc:i\n If not, fix the robot configuration files to instantiate the 'debugInterfaceWrapper' device.\n");
                yError("\nQuickFix: If you set the --dataToDumpAll and do not need the advanced debug feature (getRotorxxx) just remove this option. See help for more information.\n");
                yError("------------- END ERROR MESSAGE ---------------\n\n");
                Network::fini();
                return false;
            }
        }

        bool logToFile = false;
        if (options.check("logToFile")) logToFile = true;

        portPrefix= dumpername.c_str() + part.asString() + "/";
        //boardDumperThread *myDumper = new boardDumperThread(&dd, rate, portPrefix, dataToDump[0]);
        //myDumper->setThetaMap(thetaMap, nJoints);

        myDumper = new boardDumperThread[nData];

        for (int i = 0; i < nData; i++)
            {
                if (dataToDump[i] == "getEncoders" )
                {
                    if (ddBoard.view(ienc))
                        {
                            yInfo("Initializing a getEncs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetEncs.setInterface(ienc);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getEncoders::The time stamp initalization interfaces was successfull! \n");
                                myGetEncs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetEncs);
                        }
                }
                else if (dataToDump[i] == "getEncoderSpeeds")
                {
                    if (ddBoard.view(ienc))
                        {
                            yInfo("Initializing a getSpeeds thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetSpeeds.setInterface(ienc);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getEncodersSpeed::The time stamp initalization interfaces was successfull! \n");
                                myGetSpeeds.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetSpeeds);
                        }
                }
                else if (dataToDump[i] == "getEncoderAccelerations")
                {
                    if (ddBoard.view(ienc))
                        {
                            yInfo("Initializing a getAccs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetAccs.setInterface(ienc);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getEncoderAccelerations::The time stamp initalization interfaces was successfull! \n");
                                myGetAccs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetAccs);
                        }
                }
                else if (dataToDump[i] == "getPosPidReferences")
                {
                    if (ddBoard.view(ipid))
                        {
                            yInfo("Initializing a getErrs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetPidRefs.setInterface(ipid);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getPosPidReferences::The time stamp initalization interfaces was successfull! \n");
                                myGetPidRefs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetPidRefs);
                        }
                }
                 else if (dataToDump[i] == "getTrqPidReferences")
                 {
                     if (ddBoard.view(itrq))
                        {
                            yInfo("Initializing a getErrs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetTrqRefs.setInterface(itrq);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getTrqPidReferences::The time stamp initalization interfaces was successfull! \n");
                                myGetTrqRefs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetTrqRefs);
                        }
                }
                 else if (dataToDump[i] == "getControlModes")
                {
                    if (ddBoard.view(icmod))
                        {
                            yInfo("Initializing a getErrs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetControlModes.setInterface(icmod, nJoints);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getControlModes::The time stamp initalization interfaces was successfull! \n");
                                myGetControlModes.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetControlModes);
                        }
                 }
                 else if (dataToDump[i] == "getInteractionModes")
                 {
                     if (ddBoard.view(iimod))
                        {
                            yInfo("Initializing a getErrs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetInteractionModes.setInterface(iimod, nJoints);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getInteractionModes::The time stamp initalization interfaces was successfull! \n");
                                myGetInteractionModes.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetInteractionModes);
                        }
                 }
                else if (dataToDump[i] == "getPositionErrors")
                 {
                     if (ddBoard.view(ipid))
                        {
                            yInfo("Initializing a getErrs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetPosErrs.setInterface(ipid);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getPositionErrors::The time stamp initalization interfaces was successfull! \n");
                                myGetPosErrs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetPosErrs);
                        }
                 }
                else if (dataToDump[i] == "getOutputs")
                 {
                     if (ddBoard.view(ipid))
                        {
                            yInfo("Initializing a getOuts thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetOuts.setInterface(ipid);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getOutputs::The time stamp initalization interfaces was successfull! \n");
                                myGetOuts.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetOuts);
                        }
                }
                else if (dataToDump[i] == "getCurrents")
                {
                    if (ddBoard.view(iamp))
                        {
                            yInfo("Initializing a getCurrs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetCurrs.setInterface(iamp);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getCurrents::The time stamp initalization interfaces was successfull! \n");
                                myGetCurrs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetCurrs);
                        }
                }
                else if (dataToDump[i] == "getTorques")
                {
                    if (ddBoard.view(itrq))
                        {
                            yInfo("Initializing a getTorques thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetTrqs.setInterface(itrq);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getTorques::The time stamp initalization interfaces was successfull! \n");
                                myGetTrqs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetTrqs);
                        }
                }
                else if (dataToDump[i] == "getTorqueErrors")
                {
                    if (ddBoard.view(itrq))
                        {
                            yInfo("Initializing a getTorqueErrors thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetTrqErrs.setInterface(itrq);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getTorqueErrors::The time stamp initalization interfaces was successfull! \n");
                                myGetTrqErrs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetTrqErrs);
                        }
                }
                else if (dataToDump[i] == "getRotorPositions")
                {
                    {
                        if (idbg==0 && ddDebug.isValid()) ddDebug.view(idbg);
                        if (idbg!=0)
                        {
                            yInfo("Initializing a getRotorPosition thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetRotorPoss.setInterface(idbg);
                            if (ddDebug.view(istmp))
                            {
                                yInfo("getRotorPositions::The time stamp initalization interfaces was successfull! \n");
                                myGetRotorPoss.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetRotorPoss);
                        }
                        else
                        {
                            yError("Debug Interface not available, cannot dump %s.\n", dataToDump[i].c_str());
                            Network::fini();
                            return false;
                        }
                    }
                }
                else if (dataToDump[i] == "getRotorSpeeds")
                    {
                        if (idbg==0 && ddDebug.isValid()) ddDebug.view(idbg);
                        if (idbg!=0)
                        {
                            yInfo("Initializing a getRotorSpeed thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetRotorVels.setInterface(idbg);
                            if (ddDebug.view(istmp))
                            {
                                yInfo("getRotorSpeeds::The time stamp initalization interfaces was successfull! \n");
                                myGetRotorVels.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetRotorVels);
                        }
                        else
                        {
                            printf("Debug Interface not available, cannot dump %s.\n", dataToDump[i].c_str());
                            Network::fini();
                            return false;
                        }
                    }
                else if (dataToDump[i] == "getRotorAccelerations")
                    {
                        if (idbg==0 && ddDebug.isValid()) ddDebug.view(idbg);
                        if (idbg!=0)
                        {
                            yInfo("Initializing a getRotorAcceleration thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetRotorAccs.setInterface(idbg);
                            if (ddDebug.view(istmp))
                            {
                                yInfo("getRotorAccelerations::The time stamp initalization interfaces was successfull! \n");
                                myGetRotorAccs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetRotorAccs);
                        }
                        else
                        {
                            printf("Debug Interface not available, cannot dump %s.\n", dataToDump[i].c_str());
                            Network::fini();
                            return false;
                        }
                    }
                else if (dataToDump[i] == "getTemperatures")
                {
                    if (ddBoard.view(imotenc))
                        {
                            yInfo("Initializing a getTemps thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetTemps.setInterface(imot);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getEncoders::The time stamp initalization interfaces was successfull! \n");
                                myGetTemps.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetTemps);
                        }
                }
                else if (dataToDump[i] == "getMotorEncoders")
                {
                    if (ddBoard.view(imotenc))
                        {
                            yInfo("Initializing a getEncs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetMotorEncs.setInterface(imotenc);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getEncoders::The time stamp initalization interfaces was successfull! \n");
                                myGetMotorEncs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetMotorEncs);
                        }
                }
                else if (dataToDump[i] == "getMotorEncoderSpeeds")
                {
                    if (ddBoard.view(imotenc))
                        {
                            yInfo("Initializing a getSpeeds thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetMotorSpeeds.setInterface(imotenc);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getEncodersSpeed::The time stamp initalization interfaces was successfull! \n");
                                myGetMotorSpeeds.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetMotorSpeeds);
                        }
                }
                else if (dataToDump[i] == "getMotorEncoderAccelerations")
                {
                    if (ddBoard.view(imotenc))
                        {
                            yInfo("Initializing a getAccs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetMotorAccs.setInterface(imotenc);
                            if (ddBoard.view(istmp))
                            {
                                yInfo("getEncoderAccelerations::The time stamp initalization interfaces was successfull! \n");
                                myGetMotorAccs.setStamp(istmp);
                            }
                            else
                                yError("Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetMotorAccs);
                        }
                }
            }
        Time::delay(1);
        for (int i = 0; i < nData; i++)
            myDumper[i].start();

        return true;
    }

    virtual bool close()
    {

        yInfo("Stopping dumper class\n");
        for(int i = 0; i < nData; i++)
            myDumper[i].stop();

        yInfo("Deleting dumper class\n");
        delete[] myDumper;

        //finally close the dd of the remote control board
        yInfo("Closing the device driver\n");
        if (ddBoard.isValid()) ddBoard.close();
        yInfo("Device driver closed\n");

        //finally close the dd of the debug interface clien
        yInfo("Closing the debug interface\n");
        if (ddDebug.isValid()) ddDebug.close();
        yInfo("Debug interface closed\n");

        delete[] thetaMap;
        delete[] dataToDump;
        return true;
    }
};


int main(int argc, char *argv[]) 
{
    Network::init();
    ResourceFinder rf;

    rf.setVerbose(false);
    rf.setDefaultContext("controlBoardDumper");
    rf.setDefaultConfigFile("controlBoardDumper.ini");

    rf.setDefault("part", "head");
    rf.setDefault("robot", "icub");
    rf.configure(argc, argv);

    Property p;
    DumpModule mod;
    p.fromString(rf.toString());

    if (!p.check("rate"))
    {
        p.put("rate", 500);
    }
    if (!p.check("joints"))
    {
        Value v;
        v.fromString("(0)");
        p.put("joints", v);
    }
    if (!p.check("dataToDump"))
    {
        Value v;
        v.fromString("(getEncoders getEncoderSpeeds getEncoderAccelerations getPositionErrors getOutputs getCurrents getTorques getTorqueErrors)");
        p.put("dataToDump", v);
    }
    if (p.check("dataToDumpAll"))
    {
        Value v;
      v.fromString("(getEncoders getEncoderSpeeds getEncoderAccelerations getPositionErrors getOutputs getCurrents getTorques getTorqueErrors getPosPidReferences getTrqPidReferences getMotorEncoders getMotorEncoderSpeeds getMotorEncoderAccelerations getControlModes getInteractionModes getTemperatures)");
    //    v.fromString("(getControlModes getInteractionModes)");
        
        p.put("dataToDump", v);
    }
    if (p.check("help"))
    {
        printf ("\ncontrolBoardDumper usage:\n");
        printf ("1) controlBoardDumper --robot icub --part left_arm --rate 10  --joints \"(0 1 2)\" \n");
        printf ("   All data from the controlBoarWrapper will be dumped, except for advanced debugInterface (default).\n");
        printf ("\n2) controlBoardDumper --robot icub --part left_arm --rate 10  --joints \"(0 1 2)\" --dataToDump \"(xxx xxx)\"\n");
        printf (" where xxx can be uset to select one (or more) from the following:\n");
        printf (" getEncoders             (joint position)\n");
        printf (" getEncoderSpeeds        (joint velocity)\n");
        printf (" getEncoderAccelerations (joint acceleration)\n");
        printf (" getPositionErrors       (difference between desired and actual position)\n");
        printf (" getTorqueErrors         (difference between desired and measured torque, if available)\n");
        printf (" getOutputs              (voltage (PWM) given to the motor)\n");
        printf (" getCurrents             (current given to the motor)\n");
        printf (" getTorques              (joint torques, if available)\n");
        printf (" getPosPidReferences     (last position referenece)\n");
        printf (" getTrqPidReferences     (last torque reference)\n");
        printf (" getMotorEncoders              (motor encoder position)\n");
        printf (" getMotorEncoderSpeeds         (motor encoder velocity)\n");
        printf (" getMotorEncoderAccelerations  (motor encoder acceleration\n");
        printf (" getControlModes         (joint control mode)\n");
        printf (" getInteractionModes     (joint interaction mode)\n");
        printf (" getTemperatures         (motor temperatures)\n");
        printf ("\n3) controlBoardDumper --robot icub --part left_arm --rate 10  --joints \"(0 1 2)\" --dataToDumpAll\n");
        printf ("   All data from the controlBoarWrapper will be dumped, including data from the debugInterface (getRotorxxx).\n");
        printf ("\n --logToFile can be used to create log files storing the data\n\n");

        return 0;
    }

    if (mod.open(p))
        return mod.runModule();
    else 
        return 1;

    Network::fini();
}
