/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Marco Randazzo
  * email: marco.randazzo@iit.it
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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <string>

#include "robot_interfaces.h"

using namespace iCub::skinDynLib;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

#define POS  0
#define TRQ  1

YARP_DECLARE_DEVICES(icubmod)

//                robot->icmd[rd->id]->setPositionMode(0);


#define jjj 0
class CtrlThread: public yarp::os::RateThread
{
    public:
    robot_interfaces *robot;
    bool   left_arm_master;
    double encoders_master [16];
    double encoders_slave  [16];
    bool   autoconnect;
    Stamp  info;
    
    BufferedPort<iCub::skinDynLib::skinContactList> *port_skin_contacts;
    BufferedPort<Vector> *port_left_arm;
    BufferedPort<Vector> *port_right_arm;


    CtrlThread(unsigned int _period, ResourceFinder &_rf) :
               RateThread(_period)
    {
        autoconnect = false;
        robot=0;
        left_arm_master=false;
        port_skin_contacts=0;
    };

    virtual bool threadInit()
    {
        robot=new robot_interfaces(LEFT_ARM, RIGHT_ARM);
        robot->init();

        port_skin_contacts = new BufferedPort<skinContactList>;
        port_left_arm = new BufferedPort<Vector>;
        port_right_arm = new BufferedPort<Vector>;
        port_skin_contacts->open(string("/demoForceControl2/skin_contacts:i").c_str());
        port_left_arm->open("/demoForceControl2/left_arm:o");
        port_right_arm->open("/demoForceControl2/right_arm:o");

        if (autoconnect)
        {
            Network::connect(string("/armSkinDriftComp/skin_events:o").c_str(),string("/demoForceControl2/skin_contacs:i").c_str(),"tcp",false);
        }
        
        robot->iimp[LEFT_ARM]->setImpedance(0,0.2,0.02);
        robot->iimp[LEFT_ARM]->setImpedance(1,0.2,0.02);
        robot->iimp[LEFT_ARM]->setImpedance(2,0.2,0.02);
        robot->iimp[LEFT_ARM]->setImpedance(3,0.2,0.02);
        robot->iimp[LEFT_ARM]->setImpedance(4,0.1,0.00);

        robot->iimp[RIGHT_ARM]->setImpedance(0,0.2,0.02);
        robot->iimp[RIGHT_ARM]->setImpedance(1,0.2,0.02);
        robot->iimp[RIGHT_ARM]->setImpedance(2,0.2,0.02);
        robot->iimp[RIGHT_ARM]->setImpedance(3,0.2,0.02);
        robot->iimp[RIGHT_ARM]->setImpedance(4,0.1,0.00);

        /*
        for (int i=0; i<5; i++)
        {
            robot->ipos[RIGHT_ARM]->setRefSpeed(i,40);
            robot->ipos[LEFT_ARM]->setRefSpeed(i,40);
            robot->ivel[RIGHT_ARM]->setRefAcceleration(i,40);
            robot->ivel[LEFT_ARM]->setRefAcceleration(i,40);
        }
        */

        change_master();
        return true;
    }
    virtual void run()
    {    
        int  i_touching_left=0;
        int  i_touching_right=0;
        int  i_touching_diff=0;
        info.update();
        
        skinContactList *skinContacts  = port_skin_contacts->read(false);
        if(skinContacts)
        {
            for(skinContactList::iterator it=skinContacts->begin(); it!=skinContacts->end(); it++){
                if(it->getBodyPart() == LEFT_ARM)
                    i_touching_left += it->getActiveTaxels();
                else if(it->getBodyPart() == RIGHT_ARM)
                    i_touching_right += it->getActiveTaxels();
            }
        }
        i_touching_diff=i_touching_left-i_touching_right;

        if (abs(i_touching_diff)<5)
        {
            fprintf(stdout,"nothing!\n");
        }
        else
        if (i_touching_left>i_touching_right)
        {
            fprintf(stdout,"Touching left arm! \n");
            if (!left_arm_master) change_master();
        }
        else
        if (i_touching_right>i_touching_left)
        {
            fprintf(stdout,"Touching right arm! \n");
            if (left_arm_master) change_master();
        }

        if (left_arm_master)
        {
            robot->ienc[LEFT_ARM] ->getEncoders(encoders_master);
            robot->ienc[RIGHT_ARM]->getEncoders(encoders_slave);
            if (port_left_arm->getOutputCount()>0)
            {
                port_left_arm->prepare()= Vector(16,encoders_master);
                port_left_arm->setEnvelope(info);
                port_left_arm->write();
            }
            if (port_right_arm->getOutputCount()>0)
            {
                port_right_arm->prepare()= Vector(16,encoders_slave);
                port_right_arm->setEnvelope(info);
                port_right_arm->write();
            }            
            
        //    robot->ipos[RIGHT_ARM] ->positionMove(3,encoders_master[3]);
            for (int i=jjj; i<5; i++)
            {
                robot->ipid[RIGHT_ARM]->setReference(i,encoders_master[i]);
            }
        }
        else
        {
            robot->ienc[RIGHT_ARM]->getEncoders(encoders_master);
            robot->ienc[LEFT_ARM] ->getEncoders(encoders_slave);
            for (int i=jjj; i<5; i++)
            {
                robot->ipid[LEFT_ARM]->setReference(i,encoders_master[i]);
            }

        }
    }

    void change_master()
    {
        left_arm_master=(!left_arm_master);
        if (left_arm_master)
        {
            for (int i=jjj; i<5; i++)
            {
                robot->icmd[LEFT_ARM]->setTorqueMode(i);

                robot->icmd[RIGHT_ARM]->setPositionMode(i);
                robot->iint[RIGHT_ARM]->setInteractionMode(i,VOCAB_IM_COMPLIANT);
            }
        }
        else
        {
            for (int i=jjj; i<5; i++)
            {
                robot->icmd[LEFT_ARM]->setPositionMode(i);
                robot->iint[LEFT_ARM]->setInteractionMode(i,VOCAB_IM_COMPLIANT);

                robot->icmd[RIGHT_ARM]->setTorqueMode(i);
            }
        }
    }

    void closePort(Contactable *_port)
    {
        if (_port)
        {
            _port->interrupt();
            _port->close();

            delete _port;
            _port = 0;
        }
    }

    virtual void threadRelease()
    {  
        for (int i=0; i<5; i++)
        {
            robot->icmd[LEFT_ARM] ->setPositionMode(i);
            robot->icmd[RIGHT_ARM]->setPositionMode(i);
        }
        closePort(port_skin_contacts);
        closePort(port_left_arm);
        closePort(port_right_arm);
    }
};

    

class CtrlModule: public RFModule
{
    public:
    CtrlThread       *control_thr;
    CtrlModule();

    virtual bool configure(ResourceFinder &rf)
    {
        int rate = rf.check("rate",Value(20)).asInt();
        control_thr=new CtrlThread(rate,rf);
        if (!control_thr->start())
        {
            delete control_thr;
            return false;
        }
        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
    virtual bool   close()
    {
        if (control_thr)
        {
            control_thr->stop();
            delete control_thr;
        }
        return true;
    }
    bool respond(const Bottle& command, Bottle& reply) 
    {
        fprintf(stdout,"rpc respond\n");
        Bottle cmd;
        reply.clear(); 
        
        return true;
    }
};

CtrlModule::CtrlModule()
{

}

int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);
    //rf.setDefaultContext("empty");
    //rf.setDefaultConfigFile("empty");

    if (rf.check("help"))
    {
        //help here
    }

    //initialize yarp network
    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    CtrlModule mod;

    return mod.runModule(rf);
}


