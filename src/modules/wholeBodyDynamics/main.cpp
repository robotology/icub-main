/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo, Matteo Fumagalli
 * email:  marco.randazzo@iit.it
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
@ingroup icub_module

\defgroup wholeBodyDynamics wholeBodyDynamics
 
Estimates the external forces and torques acting at the end effector
through a model based estimation of the robot dynamics
 
\author Matteo Fumagalli, Marco Randazzo
 
\section intro_sec Description

This module estimates the external wrench acting at the end
effector of the iCub limbs, through a model based compensation 
of the 6-axis force/torque (FT) sensor's measurements, which are 
acquired through an input YARP port and provides them to an 
output YARP ports.
The estimation is perfomed relying on rigid body dynamics using CAD 
parameters. 
The intrinsic offsets of the sensors, which are due to the stresses
generated during mounting, are defined by the first FT data. In the 
future it will also be given the possibility to set the offsets of 
the sensors.
The model of the sensor measurements considers a fixed base, with z-axis 
pointing upwards. The estimation of the external wrench applied at the 
end-effector of the limb has the same orientation of the fixed base frame.
For further information about the use of this module and of the iCub force control interface, please refer to the force control page:
http://wiki.icub.org/wiki/Force_Control
 
\section lib_sec Libraries 
- YARP libraries. 
- ctrlLib library. 
- iKin library.
- iDyn library.  

\section parameters_sec Parameters

--robot \e name 
- The parameter \e name identifies the robot name. If not specified
  \e icub is assumed. 

--rate \e r 
- The parameter \e r identifies the rate the thread will work. If not
  specified \e 10ms is assumed. 

--no_legs   
- this option disables the dynamics computation for the legs joints

\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created
 
- \e <name>/<part>/FT:i (e.g. /wholeBodyDynamics/right_arm/FT:i) 
  receives the input data vector.
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None
 
\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example
By launching the following command: 
 
\code 
wholeBodyDynamics --rate 10  
\endcode 
 
the module will create the listening port 
/wholeBodyDynamics/right_arm/FT:i for the acquisition of data 
vector coming for istance from the right arm analog port. 
 
Try now the following: 
 
\code 
yarp connect /icub/right_arm/analog:o /wholeBodyDynamics/right_arm/FT:i
\endcode 
 
\author Matteo Fumagalli

This file can be edited at src/wholeBodyDynamics/main.cpp.
*/ 

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iostream>
#include <iomanip>
#include <string.h>

#include "observerThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;

#define MAX_JN 12
#define MAX_FILTER_ORDER 6

class dataFilter : public BufferedPort<Bottle>
{
private:
    BufferedPort<Vector> &port_filtered_output;
    Vector g;
    
    virtual void onRead(Bottle &b)
    {
        Stamp info;
        BufferedPort<Bottle>::getEnvelope(info);

        size_t sz = b.size();
        Vector x(sz);
        Vector inertial(sz);
        for(unsigned int i=0;i<sz;i++)
        {
            x[i]=b.get(i).asDouble();
            inertial(i)=lpf_ord1_3hz(x(i), i);
        }
        g[0] = inertial[3];
        g[1] = inertial[4];
        g[2] = inertial[5]; 
        g[3] = inertial[6];
        g[4] = inertial[7];
        g[5] = inertial[8];
        //g = (9.81/norm(g))*g;
        
        port_filtered_output.prepare() = g;
        port_filtered_output.setEnvelope(info);
        port_filtered_output.write();
    }
public:
    dataFilter(BufferedPort<Vector> &_port_filtered_output, ResourceFinder &rf):    
    port_filtered_output(_port_filtered_output)
    {
        g.resize(6);
    }
};

// The main module
class wholeBodyDynamics: public RFModule
{
private:
    Property OptionsLeftArm;
    Property OptionsRightArm;
    Property OptionsHead;
    Property OptionsLeftLeg;
    Property OptionsRightLeg;
    Property OptionsTorso;
    bool     legs_enabled;
    bool     com_enabled;
    bool     w0_dw0_enabled;
    bool     com_vel_enabled;
    bool     left_arm_enabled;
    bool     right_arm_enabled;
    bool     head_enabled;
    bool     dummy_ft;
    bool     dump_vel_enabled;
    bool     auto_drift_comp;
    bool     default_ee_cont;       // true: when skin detects no contact, the ext contact is supposed at the end effector
                                    // false: ext contact is supposed at the last location where skin detected a contact
    
    dataFilter *port_inertial_input;
    BufferedPort<Vector> port_filtered_output;    
    Port rpcPort;

    inverseDynamics *inv_dyn;

    PolyDriver *dd_left_arm;
    PolyDriver *dd_right_arm;
    PolyDriver *dd_head;
    PolyDriver *dd_left_leg;
    PolyDriver *dd_right_leg;
    PolyDriver *dd_torso;

public:
    wholeBodyDynamics()
    {
        inv_dyn=0;
        dd_left_arm=0;
        dd_right_arm=0;
        dd_head=0;
        dd_left_leg=0;
        dd_right_leg=0;
        dd_torso=0;
        com_vel_enabled=false;
        com_enabled=true;
        legs_enabled = true;
        left_arm_enabled = true;
        right_arm_enabled = true;
        head_enabled = true;
        w0_dw0_enabled = false;
        dummy_ft = false;
        dump_vel_enabled = false;
        auto_drift_comp = false;
        default_ee_cont = false;
    }

    virtual bool createDriver(PolyDriver *&_dd, Property options)
    {
        int trials=0;
        double start_time = yarp::os::Time::now();

        do
        {
            double current_time = yarp::os::Time::now();

            //remove previously existing drivers
            if (_dd)
            {
                delete _dd;
                _dd=0;
            }

            //creates the new device driver
            _dd = new PolyDriver(options);
            bool connected =_dd->isValid();

            //check if the driver is connected
            if (connected) break;
        
            //check if the timeout (60s) is expired
            if (current_time-start_time > 60.0)
            {
                fprintf(stderr,"It is not possible to instantiate the device driver. I tried %d times!\n", trials);
                return false;
            }

            yarp::os::Time::delay(5);
            trials++;
            fprintf(stderr,"\nUnable to connect the device driver, trying again...\n");
        }
        while (true);

        IEncoders *encs;

        bool ok = true;
        ok = ok & _dd->view(encs);
        if (!ok)
        {
            fprintf(stderr,"ERROR: one or more devices has not been viewed\nreturning...");
            return false;
        }

        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
        reply.clear(); 
        
        if (command.get(0).isInt())
        {
            if (command.get(0).asInt()==0)
            {
                fprintf(stderr,"Asking recalibration...\n");
                if (inv_dyn)
                {
                    inv_dyn->suspend();
                    inv_dyn->calibrateOffset(); 
                    inv_dyn->resume();
                }
                fprintf(stderr,"Recalibration complete.\n");
                reply.addString("Recalibrated");
                return true;
            }
        }

        if (command.get(0).isString())
        {
            if (command.get(0).asString()=="help")
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString("Available commands:");
                reply.addString("calib all");
                reply.addString("calib arms");
                reply.addString("calib legs");
                reply.addString("calib feet");
                return true;
            }
            else if (command.get(0).asString()=="calib")
            {
                fprintf(stderr,"Asking recalibration...\n");
                if (inv_dyn)
                {
                    calib_enum calib_code=CALIB_ALL;
                          if (command.get(1).asString()=="all")  calib_code=CALIB_ALL;
                    else  if (command.get(1).asString()=="arms") calib_code=CALIB_ARMS;
                    else  if (command.get(1).asString()=="legs") calib_code=CALIB_LEGS;
                    else  if (command.get(1).asString()=="feet") calib_code=CALIB_FEET;

                    inv_dyn->suspend();
                    inv_dyn->calibrateOffset(calib_code);
                    inv_dyn->resume();
                }
                fprintf(stderr,"Recalibration complete.\n");
                reply.addString("Recalibrated");
                return true;
            }
        }

        reply.addString("Unknown command");
        return true;
    }

    bool configure(ResourceFinder &rf)
    {
        //---------------------LOCAL NAME-----------------------//
        string local_name = "wholeBodyDynamics";
        if (rf.check("local"))
        {
            local_name=rf.find("local").asString();
        }


        //-----------------GET THE ROBOT NAME-------------------//
        string robot_name;
        if (rf.check("robot"))
             robot_name = rf.find("robot").asString();
        else robot_name = "icub";

        //------------SPECIAL PARAM TP DEFINE THE HEAD TYPE-----//
        version_tag icub_type;

        icub_type.head_version = 1;
        icub_type.legs_version = 1;

        if (rf.check("headV2"))
        {
            fprintf(stderr,"'headV2' option found. Using icubV2 head kinematics.\n");
            icub_type.head_version = 2;
        }

        //----------SPECIAL PARAM TO DEFINE LEGS VERSION--------//
        if(rf.check("legsV2"))
        {
            fprintf(stderr, "'legsV2' option found. Using legsV2 kinematics. \n");
            icub_type.legs_version = 2;
        }

        //-----------------CHECK IF AUTOCONNECT IS ON-----------//
        bool autoconnect;
        if (rf.check("autoconnect"))
        {
             fprintf(stderr,"'autoconnect' option enabled.\n");
             autoconnect = true;
        }
        else
        { 
              autoconnect = false;
        }

        //------------CHECK IF COM COMPUTATION IS ENABLED-----------//
        if (rf.check("no_com"))
        {
            com_enabled= false;
            fprintf(stderr,"'no_com' option found. COM computation will be disabled.\n");
        }

        //------------DEBUG ONLY-----------//
        if (rf.check("disable_w0_dw0"))
        {
            w0_dw0_enabled= false;
            fprintf(stderr,"'disable_w0_dw0' option found. w0 and dw0 will be set to zero.\n");
        }
        //------------DEBUG ONLY-----------//
        if (rf.check("enable_w0_dw0"))
        {
            w0_dw0_enabled= true;
            fprintf(stderr,"'enable_w0_dw0' option found. w0 and dw0 will be used.\n");
        }

        //------------CHECK IF COM VELOCITY COMPUTATION IS ENABLED-----------//
        if (rf.check("experimental_com_vel"))
        {
            com_vel_enabled= true;
            fprintf(stderr,"'enable_com_vel' option found. Extra COM velocity computation will be enabled.\n");
        }

        //------------------CHECK IF LEGS ARE ENABLED-----------//
        if (rf.check("no_legs"))
        {
            legs_enabled= false;
            fprintf(stderr,"'no_legs' option found. Legs will be disabled.\n");
        }

        //------------------CHECK IF HEAD IS ENABLED-----------//
        if (rf.check("no_head"))
        {
            head_enabled= false;
            fprintf(stderr,"'no_head' option found. Head will be disabled.\n");
        }

        //------------------CHECK IF ARMS ARE ENABLED-----------//
        if (rf.check("no_left_arm"))
        {
            left_arm_enabled= false;
            fprintf(stderr,"'no_left_arm' option found. Left arm will be disabled.\n");
        }
        if (rf.check("no_right_arm"))
        {
            right_arm_enabled= false;
            fprintf(stderr,"'no_right_arm' option found. Right arm will be disabled.\n");
        }

        //---------------------RATE-----------------------------//
        int rate = 10;
        if (rf.check("rate"))
        {
            rate = rf.find("rate").asInt();
            fprintf(stderr,"rateThread working at %d ms\n", rate);
        }
        else
        {
            fprintf(stderr,"Could not find rate in the config file\nusing 10ms as default");
            rate = 10;
        }

        //---------------------DUMMY_FT-------------------------//
        if (rf.check("dummy_ft"))
        {
            dummy_ft = true;
            fprintf(stderr,"Using dummy FT sensors (debug mode)\n");
        }

        //---------------------DUMP-VEL-------------------------//
        if (rf.check("dumpvel"))
        {
            dump_vel_enabled = true;
            fprintf(stderr,"Dumping joint velocities and accelerations (debug mode)\n");
        }

        if (rf.check("auto_drift_comp"))
        {
            auto_drift_comp = true;
            fprintf(stderr,"Enabling automatic drift compensation (experimental)\n");
        }

        if (rf.check("default_ee_cont"))
        {
            default_ee_cont = true;
            fprintf(stderr,"Default contact at the end effector\n");
        }

        //---------------------DEVICES--------------------------//
        if(head_enabled)
        {
            OptionsHead.put("device","remote_controlboard");
            OptionsHead.put("local",string("/"+local_name+"/head/client").c_str());
            OptionsHead.put("remote",string("/"+robot_name+"/head").c_str());

            if (!createDriver(dd_head, OptionsHead))
            {
                fprintf(stderr,"ERROR: unable to create head device driver...quitting\n");
                return false;
            }
            else
                fprintf(stderr,"device driver created\n");
        }

        if (left_arm_enabled)
        {
            OptionsLeftArm.put("device","remote_controlboard");
            OptionsLeftArm.put("local",string("/"+local_name+"/left_arm/client").c_str());
            OptionsLeftArm.put("remote",string("/"+robot_name+"/left_arm").c_str());

            if (!createDriver(dd_left_arm, OptionsLeftArm))
            {
                fprintf(stderr,"ERROR: unable to create left arm device driver...quitting\n");
                return false;
            }
        }
        
        if (right_arm_enabled)
        {
            OptionsRightArm.put("device","remote_controlboard");
            OptionsRightArm.put("local",string("/"+local_name+"/right_arm/client").c_str());
            OptionsRightArm.put("remote",string("/"+robot_name+"/right_arm").c_str());

            if (!createDriver(dd_right_arm, OptionsRightArm))
            {
                fprintf(stderr,"ERROR: unable to create right arm device driver...quitting\n");
                return false;
            }
        }

        if (legs_enabled)
        {
            OptionsLeftLeg.put("device","remote_controlboard");
            OptionsLeftLeg.put("local",string("/"+local_name+"/left_leg/client").c_str());
            OptionsLeftLeg.put("remote",string("/"+robot_name+"/left_leg").c_str());

            if (!createDriver(dd_left_leg, OptionsLeftLeg))
            {
                fprintf(stderr,"ERROR: unable to create left leg device driver...quitting\n");
                return false;
            }

            OptionsRightLeg.put("device","remote_controlboard");
            OptionsRightLeg.put("local",string("/"+local_name+"/right_leg/client").c_str());
            OptionsRightLeg.put("remote",string("/"+robot_name+"/right_leg").c_str());

            if (!createDriver(dd_right_leg, OptionsRightLeg))
            {
                fprintf(stderr,"ERROR: unable to create right leg device driver...quitting\n");
                return false;
            }
        }

        OptionsTorso.put("device","remote_controlboard");
        OptionsTorso.put("local",string("/"+local_name+"/torso/client").c_str());
        OptionsTorso.put("remote",string("/"+robot_name+"/torso").c_str());

        if (!createDriver(dd_torso, OptionsTorso))
        {
            fprintf(stderr,"ERROR: unable to create head device driver...quitting\n");
            return false;
        }
        else
            fprintf(stderr,"device driver created\n");

        //--------------------CHECK FT SENSOR------------------------
        if (!dummy_ft)
        {
            if ((dd_left_arm  && Network::exists(string("/"+robot_name+"/left_arm/analog:o").c_str())  == false ) || 
                (dd_right_arm && Network::exists(string("/"+robot_name+"/right_arm/analog:o").c_str()) == false ) ||
                (dd_left_leg  && Network::exists(string("/"+robot_name+"/left_leg/analog:o").c_str())  == false ) ||
                (dd_right_leg && Network::exists(string("/"+robot_name+"/right_leg/analog:o").c_str()) == false ) )
                {     
                    fprintf(stderr,"Unable to detect the presence of F/T sensors in your iCub...quitting\n");
                    return false;
                }
        }     

        //---------------OPEN RPC PORT--------------------//
        string rpcPortName = "/"+local_name+"/rpc:i";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);                  

        //---------------OPEN INERTIAL PORTS--------------------//
        port_filtered_output.open(string("/"+local_name+"/filtered/inertial:o").c_str());
        port_inertial_input = new dataFilter(port_filtered_output, rf);
        port_inertial_input->useCallback();
        port_inertial_input->open(string("/"+local_name+"/unfiltered/inertial:i").c_str());

        //--------------------------THREAD--------------------------
        inv_dyn = new inverseDynamics(rate, dd_left_arm, dd_right_arm, dd_head, dd_left_leg, dd_right_leg, dd_torso, robot_name, local_name, icub_type, autoconnect);
        inv_dyn->com_enabled=com_enabled;
        inv_dyn->auto_drift_comp=auto_drift_comp;
        inv_dyn->com_vel_enabled=com_vel_enabled;
        inv_dyn->dummy_ft=dummy_ft;
        inv_dyn->w0_dw0_enabled=w0_dw0_enabled;
        inv_dyn->dumpvel_enabled=dump_vel_enabled;
        inv_dyn->default_ee_cont=default_ee_cont;

        fprintf(stderr,"ft thread istantiated...\n");
        Time::delay(5.0);

        inv_dyn->start();
        fprintf(stderr,"thread started\n");
        return true;
    }

    bool close()
    {
        //The order of execution of the following closures is important, do not change it.
        fprintf(stderr,"Closing wholeBodyDynamics module... \n");     

        if (inv_dyn)
          {
            thread_status_enum thread_status = inv_dyn->getThreadStatus();
            if (thread_status!=STATUS_DISCONNECTED)
            {
                fprintf(stderr, "Setting the icub in stiff mode\n");
                inv_dyn->setStiffMode();
            }
            fprintf(stderr,"Stopping the inv_dyn thread...");     
            inv_dyn->stop();
            fprintf(stderr,"inv_dyn thread stopped\n");     
            delete inv_dyn;
            inv_dyn=0;
          }

        if(port_inertial_input)
        {
            fprintf(stderr,"Closing the inertial input port \n");     
            port_inertial_input->interrupt();
            port_inertial_input->close();
            delete port_inertial_input;
            port_inertial_input=0;
        }

        fprintf(stderr,"Closing the filtered inertial output port \n");     
        port_filtered_output.interrupt();
        port_filtered_output.close();

        fprintf(stderr,"Closing the rpc port \n");     
        rpcPort.close();

        if (dd_left_arm)
        {
            fprintf(stderr,"Closing dd_left_arm \n");     
            dd_left_arm->close();
            delete dd_left_arm;
            dd_left_arm=0;
        }
        if (dd_right_arm)
        {
            fprintf(stderr,"Closing dd_right_arm \n");     
            dd_right_arm->close();
            delete dd_right_arm;
            dd_right_arm=0;
        }
        if (dd_head)
        {
            fprintf(stderr,"Closing dd_head \n");     
            dd_head->close();
            delete dd_head;
            dd_head=0;
        }

        if (dd_left_leg)
        {
            fprintf(stderr,"Closing dd_left_leg \n");     
            dd_left_leg->close();
            delete dd_left_leg;
            dd_left_leg=0;
        }
        if (dd_right_leg)
        {
            fprintf(stderr,"Closing dd_right_leg \n");     
            dd_right_leg->close();
            delete dd_right_leg;
            dd_right_leg=0;
        }
        if (dd_torso)
        {
            fprintf(stderr,"Closing dd_torso \n");     
            dd_torso->close();
            delete dd_torso;
            dd_torso=0;
        }

        fprintf(stderr,"wholeBodyDynamics module was closed successfully! \n");     
        return true;
    }

    double getPeriod()
    {
        return 1.0;
    }
    bool updateModule() 
    {
        double avgTime, stdDev, period;
        period = inv_dyn->getRate();
        inv_dyn->getEstPeriod(avgTime, stdDev);
        if(avgTime > 1.3 * period){
        //   printf("(real period: %3.3f +/- %3.3f; expected period %3.3f)\n", avgTime, stdDev, period);
        }

        static unsigned long int alive_counter = 0;
        static double curr_time = Time::now();
        if (Time::now() - curr_time > 60)
        {
            printf ("wholeBodyDynamics is alive! running for %ld mins.\n",++alive_counter);
            curr_time = Time::now();
        }

        if (inv_dyn==0) 
            return false;
        thread_status_enum thread_status = inv_dyn->getThreadStatus();
        if (thread_status==STATUS_OK)
            return true;
        else if (thread_status==STATUS_DISCONNECTED)
        {
            printf ("wholeBodyDynamics module lost connection with iCubInterface, now closing...\n");
            return false;
        }
        else
        {
            fprintf(stderr,"wholeBodyDynamics module was closed successfully! \n");    
            return true;
        }
            
    }
};


int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("wholeBodyDynamics");
    rf.setDefaultConfigFile("wholeBodyDynamics.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--context context: where to find the called resource (referred to $ICUB_ROOT/app: default wholeBodyDynamics)" << endl;
        cout << "\t--from       from: the name of the file.ini to be used for calibration"                                       << endl;
        cout << "\t--rate       rate: the period used by the module. default: 10ms"                                              << endl;
        cout << "\t--robot      robot: the robot name. default: iCub"                                                            << endl;
        cout << "\t--local      name: the prefix of the ports opened by the module. defualt: wholeBodyDynamics"                  << endl;
        cout << "\t--autoconnect     automatically connects the module ports to iCubInterface"                                   << endl;        
        cout << "\t--no_legs         this option disables the dynamics computation for the legs joints"                          << endl;  
        cout << "\t--headV2          use the model of the headV2"                                                                << endl;
        cout << "\t--legsV2          use the model of legsV2"                                                                    << endl;
        cout << "\t--no_left_arm     disables the left arm"                                                                      << endl;
        cout << "\t--no_right_arm    disables the right arm"                                                                     << endl;
        cout << "\t--no_com          disables the com computation"                                                               << endl;
        cout << "\t--dummy_ft        uses fake FT sensors (debug use only)"                                                      << endl;
        cout << "\t--dumpvel         dumps joint velocities and accelerations (debug use only)"                                  << endl;
        cout << "\t--experimental_com_vel  enables com velocity computation (experimental)"                                      << endl;
        cout << "\t--auto_drift_comp  enables automatic drift compensation  (experimental, under debug)"                         << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    wholeBodyDynamics obs;
    return obs.runModule(rf);
}

