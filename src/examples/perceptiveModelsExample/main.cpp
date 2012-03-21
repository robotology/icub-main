/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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
\defgroup perceptiveModelsExample perceptiveModelsExample
 
@ingroup icub_module  
 
Example module for the use of \ref PerceptiveModels library.

Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 
\section intro_sec Description 
This simple module gives a very brief introduction to the use 
of \ref PerceptiveModels framework for the case of detection of
contacts of the fingers with external objects.

Two types of models are envisaged: \n
-# the springy approach that learns the relations between the motor
joints and the distal fingers joints to detect discrepancies caused
by external contacts;
-# a direct approach that relies on the output of tactile 
 sensors.

The output of this module is printed out on the screen reporting the
data as gathered from the sensors as well as a synthetic number which
accounts for the contact detection: the larger it becomes, the stronger
should be the force excerted by the external object. 

\section lib_sec Libraries 
- YARP libraries. 
- \ref PerceptiveModels library. 
 
\section parameters_sec Parameters
--name \e name
- specify the module name, which is \e percex by default. 
 
--hand \e hand 
- specify which hand has to be used: \e hand can be \e left or 
  \e right (by default).
 
--model \e type 
- specify the type of perceptive model to be employed for 
  sensing external contacts; possible choices are: \e springy
  and \e tactile. In case the \e springy model is used, a
  preliminary calibration phase is carried out.
 
--finger \e finger 
- specify the finger to be used; possible choices are: \e thumb, 
  \e index, \e middle, \e ring and \e little.

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <string>
#include <stdio.h>

#include <yarp/os/Network.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iCub/ctrl/math.h>
#include <iCub/perception/springyFingers.h>
#include <iCub/perception/tactileFingers.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace iCub::perception;


/************************************************************************/
class ExampleModule: public RFModule
{
    Model *model;
    PolyDriver driver;
    bool calibrate;
    string fingerName;

    IPositionControl *ipos;
    IEncoders        *ienc;

    double min,max,*val;
    int joint;

public:
    /************************************************************************/
    ExampleModule() : calibrate(true) { }

    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();
        string hand=rf.find("hand").asString().c_str();
        string modelType=rf.find("modelType").asString().c_str();
        fingerName=rf.find("finger").asString().c_str();

        if (fingerName=="thumb")
            joint=10;
        else if (fingerName=="index")
            joint=12;
        else if (fingerName=="middle")
            joint=14;
        else if (fingerName=="ring")
            joint=15;
        else if (fingerName=="little")
            joint=15;
        else
        {
            fprintf(stdout,"unknown finger!\n");
            return false;
        }

        Property driverOpt("(device remote_controlboard)");
        driverOpt.put("remote",("/icub/"+hand+"_arm").c_str());
        driverOpt.put("local",("/"+name).c_str());
        if (!driver.open(driverOpt))
            return false;

        driver.view(ipos);
        driver.view(ienc);

        IControlLimits *ilim;
        driver.view(ilim);

        ilim->getLimits(joint,&min,&max);
        double margin=0.1*(max-min);
        min=min+margin;
        max=max-margin;
        val=&min;

        Property genOpt;
        genOpt.put("name",(name+"/"+modelType).c_str());
        genOpt.put("type",hand.c_str());
        genOpt.put("verbose",1);
        string general(genOpt.toString().c_str());
        string thumb( "(thumb  (name thumb))");
        string index( "(index  (name index))");
        string middle("(middle (name middle))");
        string ring(  "(ring   (name ring))");
        string little("(little (name little))");

        Property options((general+" "+thumb+" "+index+" "+middle+" "+ring+" "+little).c_str());
        fprintf(stdout,"configuring options: %s\n",options.toString().c_str());

        if (modelType=="springy")
            model=new SpringyFingersModel;
        else if (modelType=="tactile")
            model=new TactileFingersModel;
        else
        {
            fprintf(stdout,"unknown model type!\n");
            return false;
        }

        if (model->fromProperty(options))
            return true;
        else
        {
            delete model;
            return false;
        }
    }

    /************************************************************************/
    bool close()
    {
        driver.close();

        Property options;
        model->toProperty(options);
        fprintf(stdout,"saving options: %s\n",options.toString().c_str());
        
        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /************************************************************************/
    bool updateModule()
    {
        if (calibrate)
        {
            Property options;
            options.put("finger",fingerName.c_str());
            model->calibrate(options);
            calibrate=false;

            ipos->setRefAcceleration(joint,1e9);

            if ((fingerName=="ring")||(fingerName=="little"))
                ipos->setRefSpeed(joint,60.0);
            else
                ipos->setRefSpeed(joint,30.0);

            ipos->positionMove(joint,*val);            
        }
        else
        {
            if (Node *finger=model->getNode(fingerName))
            {
                Value data; finger->getSensorsData(data);
                Value out;  finger->getOutput(out);

                fprintf(stdout,"%s sensors data = %s; output = %s\n",
                        finger->getName().c_str(),data.toString().c_str(),out.toString().c_str());
            }
            
            double fb; ienc->getEncoder(joint,&fb);
            if (fabs(*val-fb)<5.0)
            {
                if (val==&min)
                    val=&max;
                else
                    val=&min;

                ipos->positionMove(joint,*val);
            }
        }

        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","percex");
    rf.setDefault("hand","right");
    rf.setDefault("modelType","springy");
    rf.setDefault("finger","index");
    rf.configure("ICUB_ROOT",argc,argv);

    ExampleModule mod;
    return mod.runModule(rf);
}


