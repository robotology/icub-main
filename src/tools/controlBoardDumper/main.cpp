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
 * be dumped to a file using the \ref dataDumper. Time stamp
 * is forwarded if present in the collected data. If not present
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
 * dataToDump   (data1...dataN)  //type of data to be collected [default: all possible data]
 *
 * logToFile                     //if present, this options creates a log file for each data port
 *
 * \endcode
 * 
 * If no such file can be found, the application is started
 * with the default paramters. The first parameter ('robot' with 
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
 * <li> getRotorPositions       (hires rotor position, if available)
 * <li> getRotorSpeeds          (hires rotor velocity, if available)
 * <li> getRotorAccelerations   (hires rotor acceleration, if available) 
 * </ul>
 * Other data can be easily added by modyfing the classes in the 
 * file genericControlBoardDumper.cpp which contains a class named
 * GetData with a method getData which can be used to instantiate
 * a thread controlBoardDumper which does not depend on the specific
 * interface (e.g. IPositionControl) or function (e.g. getEncoders).
 *
 * The module can also be executed from a command line with the following
 * format:
 * \code
 *
 * controlBoardDumper --robot r --part p --rate r --joints "(i1 i2 i3...iN)" --dataToDump "(data1...dataN)"
 *
 * \endcode
 * and the example is:
 * \code
 *
 * controlBoardDumper --robot icub --part head --rate 20 --joints "(0 1 2)" --dataToDump "(getCurrents getOutputs)"
 *
 * \endcode

 * \section portsa_sec Ports Accessed
 * For each part initalized (e.g. right_arm):
 * <ul>
 * <li> /icub/right_arm/rpc:i 
 * <li> /icub/right_arm/command:i
 * <li> /icub/right_arm/state:i
 * </ul>
 *
 * \section portsc_sec Ports Created
 * For each part initalized (e.g. right_arm):
 * <ul>
 * <li> /robot/controlBoardDumper/part/data e.g. /icub/controlBoardDumper/right_arm/getEncoders
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

#include "dumperThread.h"

YARP_DECLARE_DEVICES(icubmod)

#define NUMBER_OF_AVAILABLE_DATA_TO_DUMP 12

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
        fprintf(stderr, "Missing option 'joints' in given config file\n");
        return 0;
    }
    Value& joints =  p.find("joints");
    Bottle *pJoints = joints.asList();
    if (pJoints == 0)
    {
         fprintf(stderr, "Error in option 'joints'\n");
         return 0;
    }
    n = pJoints->size();

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
    if (pJoints == 0)
    {
        fprintf(stderr, "Error in option 'joints'\n");
        return 0;
    }
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

int getNumberDataToDump(Property p, int &n)
{
    if (!p.check("dataToDump"))
    {
        fprintf(stderr, "Missing option 'dataToDump' in given config file\n");
        return 0;
    }
    Value& list = p.find("dataToDump");
    Bottle *pList = list.asList();
    if (pList == 0)
    {
            fprintf(stderr, "Error in option 'dataToDump'\n");
            return 0;
    }
    n = pList->size();
    return 1;
}

int getDataToDump(Property p, ConstString *listOfData, int n)
{

    ConstString availableDataToDump[NUMBER_OF_AVAILABLE_DATA_TO_DUMP];

    int j;

    availableDataToDump[0] = ConstString("getEncoders");
    availableDataToDump[1] = ConstString("getEncoderSpeeds");
    availableDataToDump[2] = ConstString("getEncoderAccelerations");
    availableDataToDump[3] = ConstString("getPositionErrors");
    availableDataToDump[4] = ConstString("getOutputs");
    availableDataToDump[5] = ConstString("getCurrents");
    availableDataToDump[6] = ConstString("getTorques");
    availableDataToDump[7] = ConstString("getTorqueErrors");
    availableDataToDump[8]  = ConstString("getRotorPositions");
    availableDataToDump[9]  = ConstString("getRotorSpeeds");
    availableDataToDump[10] = ConstString("getRotorAccelerations");
    availableDataToDump[11] = ConstString("getPidReferences");

    if (!p.check("dataToDump"))
        {
            fprintf(stderr, "Missing option 'dataToDump' in given config file\n");
            return 0;
        }
    Value& list = p.find("dataToDump");
    Bottle *pList = list.asList();

    for (int i = 0; i < n; i++)
    {
        listOfData[i] = pList->get(i).toString();
    for(j = 0; j< NUMBER_OF_AVAILABLE_DATA_TO_DUMP; j++)
    {
        if(listOfData[i] == (availableDataToDump[j]))
                break;
    }
    if(j == NUMBER_OF_AVAILABLE_DATA_TO_DUMP)
    {
        fprintf(stderr, "Illegal values for 'dataToDump': %s does not exist!\n",listOfData[i].c_str());
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

    //get the joints to be dumped
    int rate;
    int nJoints;
    int* thetaMap;
    ConstString *dataToDump;  
    int nData;

    boardDumperThread *myDumper;

    //time stamp
    IPreciselyTimed *istmp;
    //encoders
    IEncoders *ienc;
    GetSpeeds myGetSpeeds;
    GetAccs myGetAccs;
    GetEncs myGetEncs;
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
    GetTrqs myGetTrqs;
    GetTrqErrs myGetTrqErrs;
    //debug
    IDebugInterface *idbg;
    GetRotorPosition     myGetRotorPoss;
    GetRotorSpeed        myGetRotorVels;
    GetRotorAcceleration myGetRotorAccs;

public:
    DumpModule() 
    { 
        istmp=0;
        ienc=0;
        ipid=0;
        iamp=0;
        itrq=0;
        idbg=0;
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

        // get command file options

        Value& robot = options.find("robot");
        Value& part = options.find("part");
        configFileRobotPart = "config_";
        configFileRobotPart = configFileRobotPart + robot.asString();
        configFileRobotPart = configFileRobotPart + "_";
        configFileRobotPart = configFileRobotPart + part.asString();
        configFileRobotPart = configFileRobotPart + ".ini";

        //get the joints to be dumped
        getRate(options, rate);
        fprintf(stderr, "Selected rate is: %d\n", rate);

        if (getNumberUsedJoints(options, nJoints) == 0)
            return false;

        thetaMap = new int[nJoints];
        if (getUsedJointsMap(options, nJoints, thetaMap) == 0)
            return false;

        //get the type of data to be dumped
        if (getNumberDataToDump(options, nData) == 0)
            return false;

        dataToDump = new ConstString[nData];
        if (getDataToDump(options, dataToDump, nData) == 0)
            return false;        

        for (int i = 0; i < nData; i++)
            fprintf(stderr, "%s \n", dataToDump[i].c_str());
            
        //if (!fileOptions.check("d"))
        //    {
        //        fprintf(stderr, "Missing option 'd' in given config file\n");
        //        return 1;
        //    }
        //Value& d =  fileOptions.find("d");


        //open remote control board
        ddBoardOptions.put("device", "remote_controlboard");
        ddDebugOptions.put("device", "debugInterfaceClient");
    
        ConstString localPortName = name;
        ConstString localDebugPortName = name;
        localPortName = localPortName + "/controlBoardDumper/";
        localDebugPortName = localPortName + "debug/";
        //localPortName = localPortName + robot.asString();
        localPortName = localPortName + part.asString();
        localDebugPortName = localDebugPortName + part.asString();
        ddBoardOptions.put("local", localPortName.c_str());
        ddDebugOptions.put("local", localDebugPortName.c_str());

        ConstString remotePortName = "/";
        remotePortName = remotePortName + robot.asString();
        remotePortName = remotePortName + "/";
        remotePortName = remotePortName + part.asString();
        ddBoardOptions.put("remote", remotePortName.c_str());
        ddDebugOptions.put("remote", remotePortName.c_str());
    
        fprintf(stderr, "%s", ddBoardOptions.toString().c_str());    
        // create a device 
        ddBoard.open(ddBoardOptions);
        if (!ddBoard.isValid()) {
            printf("Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            Network::fini();
            return false;
        }

        ddDebug.open(ddDebugOptions);
        if (!ddDebug.isValid()) {
            printf("Debug Interface is mandatary to run this module.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            Network::fini();
            return false;
        }

        bool logToFile = false;
        if (options.check("logToFile")) logToFile = true;

        portPrefix = "/";
        portPrefix= portPrefix + "controlBoardDumper/" + part.asString() + "/";
        //boardDumperThread *myDumper = new boardDumperThread(&dd, rate, portPrefix, dataToDump[0]);
        //myDumper->setThetaMap(thetaMap, nJoints);

        myDumper = new boardDumperThread[nData];

        for (int i = 0; i < nData; i++)
            {
                if (dataToDump[i] == "getEncoders")
                    if (ddBoard.view(ienc))
                        {
                            fprintf(stderr, "Initializing a getEncs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetEncs.setInterface(ienc);
                            if (ddBoard.view(istmp))
                            {
                                fprintf(stderr, "getEncoders::The time stamp initalization interfaces was successfull! \n");
                                myGetEncs.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetEncs);
                        }
   
                if (dataToDump[i] == "getEncoderSpeeds")
                    if (ddBoard.view(ienc))
                        {
                            fprintf(stderr, "Initializing a getSpeeds thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetSpeeds.setInterface(ienc);
                            if (ddBoard.view(istmp))
                            {
                                fprintf(stderr, "getEncodersSpeed::The time stamp initalization interfaces was successfull! \n");
                                myGetSpeeds.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetSpeeds);
                        }
                if (dataToDump[i] == "getEncoderAccelerations")
                    if (ddBoard.view(ienc))
                        {
                            fprintf(stderr, "Initializing a getAccs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetAccs.setInterface(ienc);
                            if (ddBoard.view(istmp))
                            {
                                fprintf(stderr, "getEncoderAccelerations::The time stamp initalization interfaces was successfull! \n");
                                myGetAccs.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetAccs);
                        }
                if (dataToDump[i] == "getPidReferences")
                    if (ddBoard.view(ipid))
                        {
                            fprintf(stderr, "Initializing a getErrs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetPidRefs.setInterface(ipid);
                            if (ddBoard.view(istmp))
                            {
                                fprintf(stderr, "getPidReferences::The time stamp initalization interfaces was successfull! \n");
                                myGetPidRefs.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetPidRefs);
                        }
                if (dataToDump[i] == "getPositionErrors")
                    if (ddBoard.view(ipid))
                        {
                            fprintf(stderr, "Initializing a getErrs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetPosErrs.setInterface(ipid);
                            if (ddBoard.view(istmp))
                            {
                                fprintf(stderr, "getPositionErrors::The time stamp initalization interfaces was successfull! \n");
                                myGetPosErrs.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetPosErrs);
                        }
                if (dataToDump[i] == "getOutputs")
                    if (ddBoard.view(ipid))
                        {
                            fprintf(stderr, "Initializing a getOuts thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetOuts.setInterface(ipid);
                            if (ddBoard.view(istmp))
                            {
                                fprintf(stderr, "getOutputs::The time stamp initalization interfaces was successfull! \n");
                                myGetOuts.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetOuts);
                        }
                if (dataToDump[i] == "getCurrents")
                    if (ddBoard.view(iamp))
                        {
                            fprintf(stderr, "Initializing a getCurrs thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetCurrs.setInterface(iamp);
                            if (ddBoard.view(istmp))
                            {
                                fprintf(stderr, "getCurrents::The time stamp initalization interfaces was successfull! \n");
                                myGetCurrs.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetCurrs);
                        }
                if (dataToDump[i] == "getTorques")
                    if (ddBoard.view(itrq))
                        {
                            fprintf(stderr, "Initializing a getTorques thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetTrqs.setInterface(itrq);
                            if (ddBoard.view(istmp))
                            {
                                fprintf(stderr, "getTorques::The time stamp initalization interfaces was successfull! \n");
                                myGetTrqs.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetTrqs);
                        }
                if (dataToDump[i] == "getTorqueErrors")
                    if (ddBoard.view(itrq))
                        {
                            fprintf(stderr, "Initializing a getTorqueErrors thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetTrqErrs.setInterface(itrq);
                            if (ddBoard.view(istmp))
                            {
                                fprintf(stderr, "getTorqueErrors::The time stamp initalization interfaces was successfull! \n");
                                myGetTrqErrs.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetTrqErrs);
                        }
                if (dataToDump[i] == "getRotorPositions")
                    {
                        if (idbg==0 && ddDebug.isValid()) ddDebug.view(idbg);
                        if (idbg!=0)
                        {
                            fprintf(stderr, "Initializing a getRotorPosition thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetRotorPoss.setInterface(idbg);
                            if (ddDebug.view(istmp))
                            {
                                fprintf(stderr, "getRotorPositions::The time stamp initalization interfaces was successfull! \n");
                                myGetRotorPoss.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetRotorPoss);
                        }
                        else
                        {
                            printf("Debug Interface not available.  Here are the known devices:\n");
                            printf("%s", Drivers::factory().toString().c_str());
                            Network::fini();
                            return false;
                        }
                    }
                if (dataToDump[i] == "getRotorSpeeds")
                    {
                        if (idbg==0 && ddDebug.isValid()) ddDebug.view(idbg);
                        if (idbg!=0)
                        {
                            fprintf(stderr, "Initializing a getRotorSpeed thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetRotorVels.setInterface(idbg);
                            if (ddDebug.view(istmp))
                            {
                                fprintf(stderr, "getRotorSpeeds::The time stamp initalization interfaces was successfull! \n");
                                myGetRotorVels.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetRotorVels);
                        }
                        else
                        {
                            printf("Debug Interface not available.  Here are the known devices:\n");
                            printf("%s", Drivers::factory().toString().c_str());
                            Network::fini();
                            return false;
                        }
                    }
                if (dataToDump[i] == "getRotorAccelerations")
                    {
                        if (idbg==0 && ddDebug.isValid()) ddDebug.view(idbg);
                        if (idbg!=0)
                        {
                            fprintf(stderr, "Initializing a getRotorAcceleration thread\n");
                            myDumper[i].setDevice(&ddBoard, &ddDebug, rate, portPrefix, dataToDump[i], logToFile);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetRotorAccs.setInterface(idbg);
                            if (ddDebug.view(istmp))
                            {
                                fprintf(stderr, "getRotorAccelerations::The time stamp initalization interfaces was successfull! \n");
                                myGetRotorAccs.setStamp(istmp);
                            }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetRotorAccs);
                        }
                        else
                        {
                            printf("Debug Interface not available.  Here are the known devices:\n");
                            printf("%s", Drivers::factory().toString().c_str());
                            Network::fini();
                            return false;
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

        fprintf(stderr, "Stopping dumper class\n");
        for(int i = 0; i < nData; i++)
            myDumper[i].stop();

        fprintf(stderr, "Deleting dumper class\n");
        delete[] myDumper;

        //finally close the dd of the remote control board
        fprintf(stderr, "Closing the device driver\n");
        if (ddBoard.isValid()) ddBoard.close();
        fprintf(stderr, "Device driver closed\n");

        //finally close the dd of the debug interface clien
        fprintf(stderr, "Closing the debug interface\n");
        if (ddDebug.isValid()) ddDebug.close();
        fprintf(stderr, "Debug interface closed\n");

        delete[] thetaMap;
        delete[] dataToDump;
        return true;
    }
};


int main(int argc, char *argv[]) 
{
    YARP_REGISTER_DEVICES(icubmod)

    Network::init();
    ResourceFinder rf;

    rf.setVerbose(true);
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
        v.fromString("(0 1)");
        p.put("joints", v);
    }
    if (!p.check("dataToDump"))
    {
        Value v;
        v.fromString("(getEncoders getEncoderSpeeds getEncoderAccelerations getPositionErrors getOutputs getCurrents getTorques getTorqueErrors getPidReferences)");
        p.put("dataToDump", v);
    }
    if (p.check("dataToDumpAll"))
    {
        Value v;
        v.fromString("(getEncoders getEncoderSpeeds getEncoderAccelerations getPositionErrors getOutputs getCurrents getTorques getTorqueErrors getPidReferences getRotorPositions getRotorSpeeds getRotorAccelerations)");
        p.put("dataToDump", v);
    }
    if (p.check("help"))
    {
        printf ("controlBoardDumper usage:\n");
        printf ("1) controlBoardDumper --robot icub --part left_arm --rate 10  --joints \"(0 1 2)\" --dataToDump \"(xxx)\"\n");
        printf (" where xxx can be one of the following:\n");
        printf (" getEncoders             (joint position)\n");
        printf (" getEncoderSpeeds        (joint velocity)\n");
        printf (" getEncoderAccelerations (joint acceleration)\n");
        printf (" getPositionErrors       (difference between desired and actual position)\n");
        printf (" getTorqueErrors         (difference between desired and measured torque, if available)\n");
        printf (" getOutputs              (voltage (PWM) given to the motor)\n");
        printf (" getCurrents             (current given to the motor)\n");
        printf (" getTorques              (joint torques, if available)\n");
        printf (" getRotorPositions       (hi-res rotor position, if available)\n");
        printf (" getRotorSpeeds          (hi-res rotor velocity, if available)\n");
        printf (" getRotorAccelerations   (hi-res rotor acceleration, if available)\n");
        printf ("\n2) controlBoardDumper --robot icub --part left_arm --rate 10  --joints \"(0 1 2)\" --dataToDumpAll\n");

        return 0;
    }

    fprintf(stderr, "Current configuration is: %s\n", p.toString().c_str());
    if (mod.open(p))
        return mod.runModule();
    else 
        return 0;

    Network::fini();
}
