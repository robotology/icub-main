// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 *
 * \defgroup icub_graspDetectorConf graspDetectorConf
 *
 * A basic module for configuring the 
 * \ref icub_graspDetector module.
 *
 * \section intro_sec Description
 * 
 * This modules computes a description of the 
 * hand joints positions when moving without interacting with
 * an object. The following model is fitted to the available
 * data:
 *
 * <ul>
 * <li> lambda_0 q_0 + lambda_1 q_1 + ... + lambda_N q_N = 1 
 * </ul>
 *
 * where q_i represent the hand joints positions and where
 * the lambda_i are estimated via a least square optimization.
 *
 * \section libraries_sec Libraries
 * This module requires the GSL and yarp math libraries.
 * 
 * \section hardware_sec Hardware
 * The module requires the hand analog joints positions measurement (MAIS board).
 *
 * \section parameters_sec Parameters
 * The module requires some parameters. The first one (--robot)
 * specifies the robot name (e.g. --robot icub). The second
 * (--part) specifies the used part (e.g. --part left_arm).
 * The other parameters specify for each finger the:
 * <ul>
 * <li> "name": the analog sensors port name
 * <li> "rate": the rate of the thread reading the analog sensors
 * <li> "joint": index of the finger joint to be moved
 * <li> "posture": the position where the joint should be moved
 * <li> "analogs": the indeces of the analog sensors corrsponsing to the given joint
 * </ul>
 * All the parameters are specified 
 * according to the yarp resourceFinder (with default context graspDetector). 
 * They can be specified in a file
 * graspDetectorConf.ini with the following structure:
 *
 * \code
 *
 * name         analogPortName   //analog input port name
 *
 * robot        icub             //robot name
 *
 * part         right_arm        //part to be used
 *
 * rate         r                //rate at which the check will be performed
 *
 * [FINGER0]                     //first finger description
 *
 * joint       j0                //joint corresponding to finger0
 *
 * posture      p0                //posture to be reached by j0
 *
 * analogs     (a00 ... a0M)     //indeces of the analog sensors linked to j0
 * .
 * .
 * .
 * [FINGERN]                     //last finger description
 *
 * joint       jN                //joint corresponding to fingerN
 *
 * posure      pN                //posture to be reached by jN
 *
 * analogs     (aN0 ... aNM)     //indeces of the analog sensors linked to jN
 *
 * \endcode
 *
 * \section portsa_sec Ports Accessed
 * For each part initalized (e.g. left_arm):
 * <ul>
 * <li> /icub/left_arm/rpc:i 
 * <li> /icub/left_arm/command:i
 * <li> /icub/left_arm/state:o
 * </ul>
 * The port specified in the initialization file:
 * <ul>
 * <li> analogPortName (e.g. /icub/lefthand/analog:o)
 * </ul>
 *
 * \section portsc_sec Ports Created
 * For the initalized part (e.g. left_arm):
 * <ul>
 * <li> /icub/graspDetectorConf/left_arm/command:o
 * <li> /icub/graspDetectorConf/left_arm/rpc:o
 * <li> /icub/graspDetectorConf/left_arm/state:i 
 * </ul>
 * For each finger a port for reading the analog sensors:
 * <ul>
 * <li> analogPortName/graspDetectorConf/finger0/right_arm
 * <li> ...
 * <li> analogPortName/graspDetectorConf/fingerN/right_arm
 * </ul>
 * \section out_sec Output
 * For the initalized part (e.g. left_arm):
 * a configuration file left_armGraspDetector.ini to be used by the 
 * \ref icub_graspDetector. The file is stored in the directory
 * $ICUB_ROOT/app/graspDetector so that it will be automatically 
 * available to the \ref icub_graspDetector module by simplify 
 * specifying the --from left_armGraspDetector.ini configuration option.
 * \author Francesco Nori
 *
 * Copyright (C) 2008 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/graspDetector/main.cpp.
 */


#include "graspDetectorConf.h"
//ACE
#include <ace/OS.h> 
#include <ace/Log_Msg.h>
//YARP
#include <yarp/os/Bottle.h>
#include <yarp/os/impl/String.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Module.h>

/* itoa example */
#include <stdio.h>
#include <stdlib.h>

using namespace yarp::os::impl;

//
bool getNumberFingers(Property p, int &n)
{
    yarp::String s((size_t)1024);
    int i = 0;
    while(true)
        {
            ACE_OS::sprintf(&s[0], "FINGER%d", i);
            //fprintf(stderr, "%s\n", s.c_str());
            //fprintf(stderr, "%s\n", p.toString().c_str());
            if (p.findGroup(s.c_str()).isNull())
                {
                    fprintf(stderr, "Terminating ellipses acquisitions \n");
                    break;
                }
            else
                {
                    fprintf(stderr, ".");
                    i++;
                }
        }
    if (i == 0)
        {
            fprintf(stderr, "No fingers found \n");
            return false;
        }
    else
        {
            n = i;
            return true;
        }   
}

bool getParamFingers(Property p, int* a, double *b, Bottle *h, int n)
{
    yarp::String s((size_t)1024);
    for (int i=0; i<n; i++)
        {
            //fprintf(stderr, "Getting another position\n");
            ACE_OS::sprintf(&s[0], "FINGER%d", i);
            Bottle &xtmp = p.findGroup(&s[0]).findGroup("joint");
            //fprintf(stderr, "joint is: %s @ cycle %d\n", xtmp.toString().c_str(), i);
            if (!p.findGroup(s.c_str()).findGroup("joint").isNull()) 
                {
                    xtmp = p.findGroup(&s[0]).findGroup("joint");
                    a[i] = xtmp.get(1).asInt();
                }
            else
                {
                    fprintf(stderr, "Wrong number of 'joint' params");
                    return false;
                }

            xtmp = p.findGroup(&s[0]).findGroup("posture");
            if (!p.findGroup(&s[0]).findGroup("posture").isNull()) 
                {
                    xtmp = p.findGroup(&s[0]).findGroup("posture");
                    b[i] = xtmp.get(1).asDouble();
                }
            else
                {
                    fprintf(stderr, "Wrong number of 'posture' params");
                    return false;
                }
            
            xtmp = p.findGroup(&s[0]).findGroup("analogs");
            if (!p.findGroup(&s[0]).findGroup("analogs").isNull()) 
                {
                    xtmp = p.findGroup(&s[0]).findGroup("analogs");
                    Bottle *bot;
                    if (xtmp.get(1).isList())
                        {
                            bot = xtmp.get(1).asList();
                            //fprintf(stderr, "%s", b->toString().c_str());
                        }
                    h[i] = *bot;
                }
            else
                {
                    fprintf(stderr, "Wrong number of 'analogs' params");
                    return false;
                }

        }
    //for(int j=0; j < n; j++)
    //    fprintf(stderr, "Position j%d to %f\n", a[j], b[j]);
    return true;
}

class graspDetectModule: public RFModule
{
private:
    FILE * pFile;
    int nFingers;
    graspDetector **gd;

    int *joint; 
    double *posture; 
    Bottle *analogs; 

    PolyDriver ddArm;
    IPositionControl *ipos;

    BufferedPort<Bottle> *analogInputPort;
public:

    graspDetectModule() { }

    void initFile(Value r, Value p, Value a)
    {
        char string[1024];

        ACE_OS::sprintf(string, "robot\t%s\n", r.asString().c_str());
        fputs (string,pFile);

        ACE_OS::sprintf(string, "part\t%s\n", p.asString().c_str());
        fputs (string,pFile);

        ACE_OS::sprintf(string, "name\t%s\n", a.asString().c_str());
        fputs (string,pFile);
    }
    virtual bool configure(ResourceFinder &rf)
    {
        fprintf(stderr, "Entering configuration\n");
        // get command line options
        Property options;
        options.fromString(rf.toString());
        if (!options.check("robot") 
            || !options.check("part")) {
            ACE_OS::printf("Missing either --robot or --part options\n");
            return false;
        }

        fprintf(stderr, "Initializing network\n");
        Network::init();
        Time::turboBoost();
    
        yarp::String name((size_t)1024);
        Value& robot = options.find("robot");
        Value& part = options.find("part");
        Value& analogInput = options.find("name");
        int rate = options.find("rate").asInt();

        //opening the output file
        char outputFileName[1024];
        sprintf(outputFileName, "%s/app/graspDetector/%sGraspDetector.ini", ACE_OS::getenv("ICUB_ROOT"), part.asString().c_str());
        //fprintf(stderr, "Opening the output file: %s...", outputFileName);
        pFile = fopen (outputFileName,"w");
        //fprintf(stderr, "ok!\n");

        //write params to output file
        //fprintf(stderr, "Initializing the file...");
        initFile(robot, part, analogInput);
        //fprintf(stderr, "ok\n", outputFileName);

        // get command file options
        if (!getNumberFingers(options, nFingers))
            return false;

        joint = new int[nFingers];
        posture = new double[nFingers];
        analogs = new Bottle[nFingers];
        if (!getParamFingers(options, joint, posture, analogs, nFingers))
            return false;
        else
            fprintf(stderr, "Get param fingers was succefull!\n");
        //for(int i=0; i < nFingers; i++)
        //    fprintf(stderr, "Moving j%d to %d\n", (int) joint[i], (int) posture[i]);


        Property ddOptions;
        ddOptions.put("device", "remote_controlboard");
    
        ACE_OS::sprintf(&name[0], "/%s/graspDetectorConf/%s", robot.asString().c_str(), part.asString().c_str());
        ddOptions.put("local", name.c_str());

        ACE_OS::sprintf(&name[0], "/%s/%s", robot.asString().c_str(), part.asString().c_str());
        ddOptions.put("remote", name.c_str());
    
        //fprintf(stderr, "%s", ddOptions.toString().c_str());

    
        // create a device arm
        ddArm.open(ddOptions);
        if (!ddArm.isValid()) {
            ACE_OS::printf("Device arm not available.  Here are the known devices:\n");
            ACE_OS::printf("%s", Drivers::factory().toString().c_str());
            Network::fini();
            return false;
        }


        int nJnts;
        if (ddArm.view(ipos))
            {
                ipos->getAxes(&nJnts);
                fprintf(stderr, "Number of axes is: %d \n", nJnts);
            }
        else
            {
                fprintf(stderr, "Unable to access the number of axes \n");
                return false;
            }
    
        gd = new graspDetector*[nFingers];
        analogInputPort = new BufferedPort<Bottle>[nFingers];
        fprintf(stderr, "Creating the threads \n");
        for(int i=0; i < nFingers; i++)
            {
                //connect to analog input port
                ACE_OS::sprintf(&name[0], "%s/graspDetectorConf/finger%d/%s", analogInput.asString().c_str(), i, part.asString().c_str());
                analogInputPort[i].open(name.c_str());
                if(Network::connect(analogInput.asString().c_str(), name.c_str()))
                    fprintf(stderr, "Input connection to analog was successfull\n");
                else
                    {
                        fprintf(stderr, "Connection to %s  was NOT successfull\n", analogInput.asString().c_str());
                        return false;
                    }

                gd[i] = new graspDetector(&analogInputPort[i], rate);
            }
        return true;
    }

    void dumpPatternToFile(Vector l, double m, double M, int i)
    {
        char string[1024];
        ACE_OS::sprintf(string, "[FINGER%d]\n", i);
        fputs (string,pFile);

        ACE_OS::sprintf(string, "joint\t%d\n", joint[i]);
        fputs (string,pFile);

        ACE_OS::sprintf(string, "analogs\t(%s)\n", analogs[i].toString().c_str());
        fputs (string,pFile);

        ACE_OS::sprintf(string, "lambda\t(%s)\n", l.toString().c_str());
        fputs (string,pFile);

        ACE_OS::sprintf(string, "min\t%.2f\nmax\t%.2f\n", m, M);
        fputs (string,pFile);

        ACE_OS::sprintf(string, "\n", i);
        fputs (string,pFile);
    }

    virtual bool updateModule()
    {
        //Start the checker
        fprintf(stderr, "Starting the module with nFingers: %d\n", nFingers);
        Time::delay(1);
        for(int i=0; i < nFingers; i++)
            gd[i]->start();
        
        fprintf(stderr, "Collecting analog data...\n");
        //fprintf(stderr, "Moving j%d to %f\n", joint[i], posture[i]);
        for(int i=0; i < nFingers; i++)
            {
                //bool ok = ipos->setRefSpeed(joint[i], 30);
                //ok &= ipos->positionMove(joint[i], posture[i]);

                while(!gd[i]->startCollect(analogs[i]))
                    fprintf(stderr, "Thread %d waiting for analog data...\n", i);
            }
            
        for(int i=0; i < nFingers; i++)
            while(!gd[i]->endedMovement())
                {
                    //fprintf(stderr, "Waiting for data to be acquired...\n");
                    Time::delay(1);
                }
        
        fprintf(stderr, "Stopping collecting analog data...\n");
        for(int i=0; i < nFingers; i++)
            gd[i]->stopCollect();

        fprintf(stderr, "Analyzing analog data...\n");
        for(int i=0; i < nFingers; i++)
            {        
                //retrieving patter description
                Vector lambda;
                double min, max;
                gd[i]->getPattern(lambda, min, max);

                //printing the results
                ACE_OS::printf("Spanned vector space was: %s, min=%f, max=%f\n", lambda.toString().c_str(), min, max);
                dumpPatternToFile(lambda, min, max, i);
            }   
        //fprintf(stderr, "Time diff is: %f\n", Time::now()-tStart);
        return true;
    }

    virtual bool close()
    {

        fprintf(stderr, "Closing device driver \n");
        ddArm.close();
    
        fprintf(stderr, "Closing file\n");
        fclose (pFile);

        fprintf(stderr, "Stopping the grasp detector\n");
        for (int i = 0; i < nFingers; i++)
            gd[i]->stop();
        fprintf(stderr, "Deleting graspDectorConf class\n");
        delete[] gd;

        delete[] joint;
        delete[] posture;
        delete[] analogs;

        Network::fini();
        return 0;
    }
};

int main(int argc, char *argv[]) 
{
    ResourceFinder rf;

    rf.setVerbose(true);
    rf.setDefaultContext("graspDetector");
    rf.setDefaultConfigFile("graspDetectorConf.ini");
    if(!rf.configure("ICUB_ROOT", argc, argv))
        {
            fprintf(stderr, "Problems in instantiating the module. Closing \n");
            return -1;
        }
    else
        fprintf(stderr, "Resource finder configured correctly\n");

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    graspDetectModule mod;
    fprintf(stderr, "Configuring the module\n");
    if (mod.configure(rf))
        if(mod.updateModule())
            {
                fprintf(stderr, "Module terminate successfully. Closing \n");
                mod.close();
                return 0;
            }
    fprintf(stderr, "Unable to configure the module. Quitting! \n");
    return -1;
}
