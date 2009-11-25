// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 *
 * \defgroup icub_controlBoardDumper controlBoardDumper
 *
 * A basic module for collecting data (position, current, 
 * applied voltage, pid errors, ...) from a controlBoard
 * and sending them trough a yarp port. These data can then
 * be dumped to a file using the \ref dataDumper. Time stamp
 * is forwarded if present in the collected data. If not present
 * a time stamp is added. <bf> Remark: </bf> all collected data
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
 * <li> broadcast_pid: if 1 enables getErrors  (difference between 
 *      desired and actual position) and getOutputs (voltage give to the motor)
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
 * robot        icub             //the robot name
 *
 * part         p                //the robot part to be dumped
 *
 * rate         r                //[ms] the sampling time for collected data
 *
 * nJoints      n                //joints to be read. Has to be the length of 'joints'
 *
 * joints       (i1 i2 i3...iN)  //indexes of the joints to be read
 *
 * dataToDump   (data1...dataN)  //type of data to be collected
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
 * <li> getEncoders (joint position)
 * <li> getEncoderSpeeds (joint velocity)
 * <li> getEncoderAccelerations (joint acceleration)
 * <li> getErrors  (difference between desired and actual position)
 * <li> getOutputs (voltage give to the motor)
 * <li> getCurrents (current given to the motor)
 * </ul>
 * Other data can be easily added by modyfing the classes in the 
 * file genericControlBoardDumper.cpp which contains a class named
 * GetData with a method getData which can be used to instantiate
 * a thread controlBoardDumper which does not depend on the specific
 * interface (e.g. IPositionControl) or function (e.g. getEncoders).

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


#include "controlBoardDumper.h"
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Module.h>

#define NUMBER_OF_AVAILABLE_DATA_TO_DUMP 6

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

int getNumberDataToDump(Property p, int &n)
{
    if (!p.check("dataToDump"))
        {
            fprintf(stderr, "Missing option 'dataToDump' in given config file\n");
            return 0;
        }
    Value& list = p.find("dataToDump");
    Bottle *pList = list.asList();
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
    availableDataToDump[3] = ConstString("getErrors");
    availableDataToDump[4] = ConstString("getOutputs");
    availableDataToDump[5] = ConstString("getCurrents");


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
    Property ddOptions;
    ConstString name;
    ConstString configFileRobotPart;
    ConstString portPrefix;

    PolyDriver dd;

    //get the joints to be dumped
    int rate;
    int nJoints;
    int* thetaMap;
    ConstString *dataToDump;  
    int nData;

    controlBoardDumper *myDumper;

    //time stamp
    IPreciselyTimed *stmp;
    //encoders
    IEncoders *enc;     
    GetSpeeds myGetSpeeds;
    GetAccs myGetAccs;
    GetEncs myGetEncs;
    //pid
    IPidControl *pid;
    GetErrs myGetErrs;
    GetOuts myGetOuts;
    //amp
    IAmplifierControl *amp;
    GetCurrs myGetCurrs;

public:
    DumpModule() 
    { 

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
        ddOptions.put("device", "remote_controlboard");
    
        ConstString localPortName = name;
        localPortName = localPortName + "/controlBoardDumper/";
        localPortName = localPortName + robot.asString();
        localPortName = localPortName + part.asString();
        ddOptions.put("local", localPortName.c_str());

        ConstString remotePortName = "/";
        remotePortName = remotePortName + robot.asString();
        remotePortName = remotePortName + "/";
        remotePortName = remotePortName + part.asString();
        ddOptions.put("remote", remotePortName.c_str());
    
        fprintf(stderr, "%s", ddOptions.toString().c_str());    
        // create a device 
        dd.open(ddOptions);
        if (!dd.isValid()) {
			printf("Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            Network::fini();
            return false;
        }
    
        portPrefix = "/";
        portPrefix= portPrefix + robot.asString() + "/controlBoardDumper/" + part.asString() + "/";
        //controlBoardDumper *myDumper = new controlBoardDumper(&dd, rate, portPrefix, dataToDump[0]);
        //myDumper->setThetaMap(thetaMap, nJoints);

        myDumper = new controlBoardDumper[nData];

        for (int i = 0; i < nData; i++)
            {
                if (dataToDump[i] == "getEncoders")
                    if (dd.view(enc))
                        {
                            fprintf(stderr, "Initializing a getEncs thread\n");
                            myDumper[i].setDevice(&dd, rate, portPrefix, dataToDump[i]);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetEncs.setInterface(enc);
                            if (dd.view(stmp))
                                {
                                    fprintf(stderr, "getEncoders::The time stamp initalization interfaces was successfull! \n");
                                    myGetEncs.setStamp(stmp);
                                }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetEncs);
                        }
   
                if (dataToDump[i] == "getEncoderSpeeds")
                    if (dd.view(enc))
                        {
                            fprintf(stderr, "Initializing a getSpeeds thread\n");
                            myDumper[i].setDevice(&dd, rate, portPrefix, dataToDump[i]);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetSpeeds.setInterface(enc);
                            if (dd.view(stmp))
                                {
                                    fprintf(stderr, "getEncodersSpeed::The time stamp initalization interfaces was successfull! \n");
                                    myGetSpeeds.setStamp(stmp);
                                }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetSpeeds);
                        }
                if (dataToDump[i] == "getEncoderAccelerations")
                    if (dd.view(enc))
                        {
                            fprintf(stderr, "Initializing a getAccs thread\n");
                            myDumper[i].setDevice(&dd, rate, portPrefix, dataToDump[i]);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetAccs.setInterface(enc);
                            if (dd.view(stmp))
                                {
                                    fprintf(stderr, "getEncoderAccelerations::The time stamp initalization interfaces was successfull! \n");
                                    myGetAccs.setStamp(stmp);
                                }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetAccs);
                        }
                if (dataToDump[i] == "getErrors")
                    if (dd.view(pid))
                        {
                            fprintf(stderr, "Initializing a getErrs thread\n");
                            myDumper[i].setDevice(&dd, rate, portPrefix, dataToDump[i]);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetErrs.setInterface(pid);
                            if (dd.view(stmp))
                                {
                                    fprintf(stderr, "getErrors::The time stamp initalization interfaces was successfull! \n");
                                    myGetErrs.setStamp(stmp);
                                }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetErrs);
                        }
                if (dataToDump[i] == "getOutputs")
                    if (dd.view(pid))
                        {
                            fprintf(stderr, "Initializing a getOuts thread\n");
                            myDumper[i].setDevice(&dd, rate, portPrefix, dataToDump[i]);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetOuts.setInterface(pid);
                            if (dd.view(stmp))
                                {
                                    fprintf(stderr, "getOutputs::The time stamp initalization interfaces was successfull! \n");
                                    myGetOuts.setStamp(stmp);
                                }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                            myDumper[i].setGetter(&myGetOuts);
                        }
                if (dataToDump[i] == "getCurrents")
                    if (dd.view(amp))
                        {
                            fprintf(stderr, "Initializing a getCurrs thread\n");
                            myDumper[i].setDevice(&dd, rate, portPrefix, dataToDump[i]);
                            myDumper[i].setThetaMap(thetaMap, nJoints);
                            myGetCurrs.setInterface(amp);
                            if (dd.view(stmp))
                                {
                                    fprintf(stderr, "getCurrents::The time stamp initalization interfaces was successfull! \n");
                                    myGetCurrs.setStamp(stmp);
                                }
                            else
                                fprintf(stderr, "Problems getting the time stamp interfaces \n");
                               
                            myDumper[i].setGetter(&myGetCurrs);
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

        //finally close the dd
        fprintf(stderr, "Closing the device driver\n");
        dd.close();
        fprintf(stderr, "Device driver closed\n");


        delete[] thetaMap;
        delete[] dataToDump;
        return true;
    }
};


int main(int argc, char *argv[]) 
{

    Network::init();
    ResourceFinder rf;

    rf.setVerbose(true);
    rf.setDefaultContext("dumpControlBoardData/conf");
    rf.setDefaultConfigFile("controlBoardDumper.ini");

    rf.setDefault("part", "head");
    rf.setDefault("robot", "icub");
    rf.configure("ICUB_ROOT", argc, argv);

    Property p;
    DumpModule mod;
    p.fromString(rf.toString());

    if (!p.check("rate"))
        p.put("rate", 500);
    if (!p.check("nJoints"))
        p.put("nJoints", 2);
    if (!p.check("joints"))
        {
            Value v;
            v.fromString("(0 1)");
            p.put("joints", v);
        }
    if (!p.check("dataToDump"))
        {
            Value v;
            v.fromString("(getEncoders getEncoderSpeeds getEncoderAccelerations getErrors getOutputs getCurrents)");
            p.put("dataToDump", v);
        }

    fprintf(stderr, "Current configuration is: %s\n", p.toString().c_str());
    if (mod.open(p))
        return mod.runModule();
    else 
        return 0;

    Network::fini();
}
