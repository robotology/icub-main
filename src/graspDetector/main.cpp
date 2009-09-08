// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 *
 * \defgroup icub_graspDetector graspDetector
 *
 * A basic module for detecting a successfull grasp by using 
 * the Hall effect sensors integrated in the hand. This module
 * has to be configured by a \ref icub_graspDetectorConf module.
 *
 * \section intro_use How to use the module
 * 
 * Simply launch:
 *
 * \code
 * graspDetector --from left_armGraspDetector.ini
 * \endcode
 * 
 * NOTE: the configuration file is automatically searched in the
 * $ICUB_ROOT/app/graspDetector/conf directory. This file should
 * have been already created in the correct location by the 
 * \ref icub_graspDetectorConf module.
 * 
 * \section intro_sec Description
 * 
 * This modules assumes that the hand Hall effect 
 * sensors joints satisfy the following linear model when freely 
 * moving (not grasping any object):
 *
 * \f[ q = q_0 + q_1 \cdot t, \quad \quad t \in [t_{min}, t_{max}], \f]
 *
 * where \f$ q \f$ represents the finger joints positions.
 * If the model is not satisfied a grasp action is assumed to
 * be performed. The model (\f$ q_0, q_1, t_{max}, t_{min} \f$)
 *  is described in the configuration 
 * file (partGraspDetector.ini where part can be either
 * right_arm or left_arm) which can be created by using the
 * module \ref icub_graspDetectorConf .
 * 
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
 * <li> "analogs": the indeces of the analog sensors corrsponsing to the given joint
 * <li> "q0", "q1", "minT", "maxT": model description (\f$ q_0, q_1, t_{min}, t_{max} \f$)
 * <li> "min", "max": the maximum and minimum distance from the model
 * </ul>
 * All the parameters can be specified 
 * according to the yarp resourceFinder (with default context graspDetector).
 * Typically this module is launched as:
 * \code
 * graspDetector --from left_armGraspDetector.ini
 * \endcode
 * or:
 * \code
 * graspDetector --from right_armGraspDetector.ini
 * \endcode
 * where the configuration files (e.g.: left_armGraspDetector.ini) are 
 * created and stored in the suitable directory ($ICUB_ROOT/app/graspDetector/conf) 
 * by the \ref icub_graspDetectorConf module.
 * Configuration files have the following structure:
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
 * analogs     (a00 ... a0M)     //indeces of the analog sensors linked to j0
 * 
 * q0     (q0_00 ... q0_0M)     //coefficients of the model linked to j0
 *
 * q1     (q1_00 ... q1_0M)     //coefficients of the model linked to j0
 *
 * min        m0                //min deviation from the model
 *
 * max        M0                //max deviation from the model
 *
 * minT        t0                //minT
 *
 * maxT        T0                //maxT
 * .
 * .
 * .
 * [FINGERN]                     //last finger description
 *
 * joint       jN                //joint corresponding to fingerN
 *
 * analogs     (aN0 ... aNM)     //indeces of the analog sensors linked to jN
 * 
 * q0     (q0_N0 ... q0_lNM)     //coefficients of the model linked to jN
 * 
 * q1     (q1_N0 ... q1_lNM)     //coefficients of the model linked to jN
 *
 * min        mN                //min deviation from the model
 *
 * max        MN                //max deviation from the model
 *
 * minT        tN                //min deviation from the model
 *
 * maxT        TN                //max deviation from the model
 *
 * \endcode
 *
 * \section portsa_sec Ports Accessed
 * The port specified in the initialization file:
 * <ul>
 * <li> analogPortName (e.g. /icub/lefthand/analog:o)
 * </ul>
 *
 * \section portsc_sec Ports Created
 * A port named:
 * <ul>
 * <li> analogPortName/graspDetector/status:o
 * </ul>
 * which reports for each finger the current grasp status.
 * The port will contain a bottle which contains a double 
 * for each finger. Finger status will be 0.0 if the finger
 * is not contacting anything (model is satisfied). Finger
 * status will be differnt from zero if the model is not 
 * satisfied (either \f$ t \notin [t_{min}, t_{max}], \f$ or 
 * deviation from the model bigger than the deviation specified
 * in the configuration file 
 * \f$ \left| q - q^{*} \right|  \notin [q_{min}, q_{max}] \f$).
 * Moreover, for each finger, a port for reading the analog sensors:
 * <ul>
 * <li> analogPortName/graspDetector/finger0/right_arm
 * <li> ...
 * <li> analogPortName/graspDetector/fingerN/right_arm
 * </ul>
 * \author Francesco Nori
 *
 * Copyright (C) 2008 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/graspDetector/main.cpp.
 */


#include "graspDetector.h"
//ACE
#include <ace/OS.h>
#include <ace/Log_Msg.h>
//YARP
#include <yarp/String.h>
#include <yarp/os/impl/String.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Module.h>


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

bool getParamFingers(Property p, Bottle *h, double* a, double* b, Bottle *q0, Bottle *q1, double *minT, double *maxT, int n)
{
    yarp::String s((size_t)1024);
    for (int i=0; i<n; i++)
        {
            //fprintf(stderr, "Getting another position\n");
            ACE_OS::sprintf(&s[0], "FINGER%d", i);
            Bottle &xtmp = p.findGroup(&s[0]).findGroup("min");
            //fprintf(stderr, "min is: %s @ cycle %d\n", xtmp.toString().c_str(), i);
            if (!p.findGroup(s.c_str()).findGroup("min").isNull()) 
                {
                    xtmp = p.findGroup(&s[0]).findGroup("min");
                    a[i] = xtmp.get(1).asDouble();
                }
            else
                {
                    fprintf(stderr, "Wrong number of 'min' params");
                    return false;
                }

            xtmp = p.findGroup(&s[0]).findGroup("max");
            //fprintf(stderr, "min is: %s @ cycle %d\n", xtmp.toString().c_str(), i);
            if (!p.findGroup(s.c_str()).findGroup("max").isNull()) 
                {
                    xtmp = p.findGroup(&s[0]).findGroup("max");
                    b[i] = xtmp.get(1).asDouble();
                }
            else
                {
                    fprintf(stderr, "Wrong number of 'max' params");
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

            xtmp = p.findGroup(&s[0]).findGroup("q0");
            if (!p.findGroup(&s[0]).findGroup("q0").isNull()) 
                {
                    xtmp = p.findGroup(&s[0]).findGroup("q0");
                    Bottle *bot;
                    if (xtmp.get(1).isList())
                        {
                            bot = xtmp.get(1).asList();
                            //fprintf(stderr, "%s", b->toString().c_str());
                        }
                    q0[i] = *bot;
                }
            else
                {
                    fprintf(stderr, "Wrong number of 'q0' params");
                    return false;
                }

            xtmp = p.findGroup(&s[0]).findGroup("q1");
            if (!p.findGroup(&s[0]).findGroup("q1").isNull()) 
                {
                    xtmp = p.findGroup(&s[0]).findGroup("q1");
                    Bottle *bot;
                    if (xtmp.get(1).isList())
                        {
                            bot = xtmp.get(1).asList();
                            //fprintf(stderr, "%s", b->toString().c_str());
                        }
                    q1[i] = *bot;
                }
            else
                {
                    fprintf(stderr, "Wrong number of 'q1' params");
                    return false;
                }

            xtmp = p.findGroup(&s[0]).findGroup("maxT");
            //fprintf(stderr, "min is: %s @ cycle %d\n", xtmp.toString().c_str(), i);
            if (!p.findGroup(s.c_str()).findGroup("maxT").isNull()) 
                {
                    xtmp = p.findGroup(&s[0]).findGroup("maxT");
                    maxT[i] = xtmp.get(1).asDouble();
                }
            else
                {
                    fprintf(stderr, "Wrong number of 'maxT' params");
                    return false;
                }

            xtmp = p.findGroup(&s[0]).findGroup("minT");
            //fprintf(stderr, "min is: %s @ cycle %d\n", xtmp.toString().c_str(), i);
            if (!p.findGroup(s.c_str()).findGroup("minT").isNull()) 
                {
                    xtmp = p.findGroup(&s[0]).findGroup("minT");
                    minT[i] = xtmp.get(1).asDouble();
                }
            else
                {
                    fprintf(stderr, "Wrong number of 'minT' params");
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
    int nFingers;
    fingerDetector **fd;
    graspDetector *gd;

    double *max; 
    double *min; 
    double *maxT; 
    double *minT; 
    Bottle *analogs; 
    Bottle *q0; 
    Bottle *q1; 

    BufferedPort<Bottle> *analogInputPort;
    Port statusPort;
public:

    graspDetectModule() { }

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

        fprintf(stderr, "Initializing network...");
        Network::init();
        fprintf(stderr, "ok\n");
    
        yarp::String name((size_t)1024);
        Value& robot = options.find("robot");
        Value& part = options.find("part");
        Value& analogInput = options.find("name");
        int rate = options.find("rate").asInt();

        // get command file options
        if (!getNumberFingers(options, nFingers))
            return false;

        //connect to analog input ports
        analogInputPort = new BufferedPort<Bottle>[nFingers];
        for (int i = 0; i < nFingers; i++)
            {
                ACE_OS::sprintf(&name[0], "%s/fingerDetector/finger%d/%s", analogInput.asString().c_str(), i, part.asString().c_str());
                //fprintf(stderr, "Trying to open port %s\n", name.c_str());
                analogInputPort[i].open(name.c_str());
                //fprintf(stderr, "Port %s opened correctly\n", name.c_str());
                if(Network::connect(analogInput.asString().c_str(), name.c_str()))
                    fprintf(stderr, "Input connection to analog was successfull\n");
                else
                    {
                        fprintf(stderr, "Connection to %s  was NOT successfull\n", analogInput.asString().c_str());
                        return false;
                    }
            }

        min  = new double[nFingers];
        max  = new double[nFingers];
        minT = new double[nFingers];
        maxT = new double[nFingers];
        analogs = new Bottle[nFingers];
        q0 = new Bottle[nFingers];
        q1 = new Bottle[nFingers];
        if (!getParamFingers(options, analogs, min, max, q0, q1, minT, maxT, nFingers))
            return false;
        else
            fprintf(stderr, "Get param fingers was succefull!\n");
        //for(int i=0; i < nFingers; i++)
        //    fprintf(stderr, "Moving j%d to %d\n", (int) min[i], (int) posture[i]);
        
        //starting the threads for detecting the finger status
        fd = new fingerDetector*[nFingers];
        for(int i=0; i < nFingers; i++)
            {
                //fprintf(stderr, "Creating the threads %d\n", i);
                fd[i] = new fingerDetector(&analogInputPort[i], rate);
                fd[i]->setIndex(analogs[i]);
                fd[i]->setModel(q0[i], q1[i], min[i], max[i], minT[i], maxT[i]);
            }

        //starting the thread for processing the hand status (i.e. all fingers status)
        ACE_OS::sprintf(&name[0], "%s/fingerDetector/status:o", analogInput.asString().c_str());
        statusPort.open(name.c_str());
        gd = new graspDetector(nFingers, fd, &statusPort, 100);

        return true;
    }

    virtual bool updateModule()
    {
        static bool first = true;
        if (first)
            {
                for(int i=0; i < nFingers; i++)
                    fd[i]->start();
                gd->start();
            }
        first = false;
        return true;
    }

    virtual bool close()
    {
        fprintf(stderr, "Stopping the grasp detectors\n");
        for(int i=0; i < nFingers; i++)
            fd[i]->stop();
        gd->stop();
            
        fprintf(stderr, "Deleting grapsDetect class\n");
        delete[] fd;
        delete gd;

        delete[] min;
        delete[] max;
        delete[] minT;
        delete[] maxT;
        delete[] analogs;
        delete[] q0;
        delete[] q1;

        fprintf(stderr, "Closing the staus port\n");
        statusPort.close();

        Network::fini();
        return 0;
    }
};

int main(int argc, char *argv[]) 
{
    ResourceFinder rf;

    rf.setVerbose(true);
    rf.setDefaultContext("graspDetector/conf");
    rf.setDefaultConfigFile("left_armGraspDetector.ini");
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
        {
            fprintf(stderr, "Module configured successfully. Running \n");
            return mod.runModule();
        }
    else
        {
            fprintf(stderr, "Unable to configure the module. Quitting! \n");
            return -1;
        }
}
