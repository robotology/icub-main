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

--imuPortName
- The parameter identifies the prefix of the multipleanalogsensorsserver
  to be attached to for retreiving inertial data.
  The default value is /<robot>/head/inertials.

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
#include <cstring>

#include "observerThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;


class dataFilter : public PeriodicThread
{
private:
    BufferedPort<Vector> &port_filtered_output;
    Vector g;
    IThreeAxisGyroscopes* m_iGyro{nullptr};
    IThreeAxisLinearAccelerometers* m_iAcc{nullptr};

public:
    dataFilter(BufferedPort<Vector> &_port_filtered_output,
               IThreeAxisGyroscopes* iGyro,
               IThreeAxisLinearAccelerometers* iAcc): PeriodicThread(0.001),
                                                      port_filtered_output(_port_filtered_output),
                                                      m_iGyro(iGyro),
                                                      m_iAcc(iAcc)
    {
        g.resize(6);
    }

    void run() override
    {
        if (!m_iGyro || !m_iAcc || port_filtered_output.isClosed()) {
            yError()<<"dataFilter: something went wrong during configuration closing";
            this->askToStop();
            return;
        }

        double acc_ts{0.0}, gyro_ts{0.0};
        Vector acc, gyro;
        bool ok = true;
        ok &= m_iAcc->getThreeAxisLinearAccelerometerMeasure(0, acc, acc_ts);
        ok &= m_iGyro->getThreeAxisGyroscopeMeasure(0, gyro, gyro_ts);
        if (!ok) {
            yError()<<"dataFilter: error while reading from inertial sensor";
            return;
        }
        static Stamp info;
        info.update(gyro_ts);
        Vector temp(6,0.0);
        temp.setSubvector(0,acc);
        temp.setSubvector(3,gyro);
        for(size_t i=0;i<temp.size();i++)
        {
            temp(i)  = lpf_ord1_3hz(temp(i), i);
        }
        g[0] = temp[0]; // acc
        g[1] = temp[1]; // acc
        g[2] = temp[2]; // acc
        g[3] = temp[3]; // gyro
        g[4] = temp[4]; // gyro
        g[5] = temp[5]; // gyro
        //g = (9.81/norm(g))*g;

        port_filtered_output.prepare() = g;
        port_filtered_output.setEnvelope(info);
        port_filtered_output.write();
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
    bool     torso_enabled;
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

    dataFilter *inertialFilter{};
    BufferedPort<Vector> port_filtered_output;
    Port rpcPort;

    inverseDynamics *inv_dyn;
    IThreeAxisLinearAccelerometers* m_iAcc{nullptr};
    IThreeAxisGyroscopes* m_iGyro{nullptr};

    PolyDriver *dd_left_arm;
    PolyDriver *dd_right_arm;
    PolyDriver *dd_head;
    PolyDriver *dd_left_leg;
    PolyDriver *dd_right_leg;
    PolyDriver *dd_torso;
    PolyDriver  dd_MASClient;

public:
    wholeBodyDynamics()
    {
        inv_dyn=nullptr;
        dd_left_arm=nullptr;
        dd_right_arm=nullptr;
        dd_head=nullptr;
        dd_left_leg=nullptr;
        dd_right_leg=nullptr;
        dd_torso=nullptr;
        com_vel_enabled=false;
        com_enabled=true;
        legs_enabled = true;
        torso_enabled = true;
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
                yError("It is not possible to instantiate the device driver. I tried %d times!\n", trials);
                return false;
            }

            yarp::os::Time::delay(5);
            trials++;
            yWarning("\nUnable to connect the device driver, trying again...\n");
        }
        while (true);

        IEncoders *encs;

        bool ok = true;
        ok = ok & _dd->view(encs);
        if (!ok)
        {
            yError("one or more devices has not been viewed\nreturning...");
            return false;
        }

        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) override
    {
        reply.clear(); 
        
        if (command.get(0).isInt())
        {
            if (command.get(0).asInt()==0)
            {
                yInfo("Asking recalibration...\n");
                if (inv_dyn)
                {
                    inv_dyn->suspend();
                    inv_dyn->calibrateOffset(); 
                    inv_dyn->resume();
                }
                yInfo("Recalibration complete.\n");
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
                yInfo("Asking recalibration...\n");
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
                yInfo("Recalibration complete.\n");
                reply.addString("Recalibrated");
                return true;
            }
        }

        reply.addString("Unknown command");
        return true;
    }

    bool configure(ResourceFinder &rf) override
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
            yInfo("'headV2' option found. Using icubV2 head kinematics.\n");
            icub_type.head_version = 2;
        }

        if (rf.check("headV2.6"))
        {
            yInfo("'headV2.6' option found. Using icubV2.6 head kinematics.\n");
            icub_type.head_version = 2;
            icub_type.head_subversion = 6;
        }

        //----------SPECIAL PARAM TO DEFINE LEGS VERSION--------//
        if(rf.check("legsV2"))
        {
            yInfo("'legsV2' option found. Using legsV2 kinematics. \n");
            icub_type.legs_version = 2;
        }

        //-----------------CHECK IF AUTOCONNECT IS ON-----------//
        bool autoconnect;
        if (rf.check("autoconnect"))
        {
             yInfo("'autoconnect' option enabled.\n");
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
            yInfo("'no_com' option found. COM computation will be disabled.\n");
        }

        //------------DEBUG ONLY-----------//
        if (rf.check("disable_w0_dw0"))
        {
            w0_dw0_enabled= false;
            yInfo("'disable_w0_dw0' option found. w0 and dw0 will be set to zero.\n");
        }
        //------------DEBUG ONLY-----------//
        if (rf.check("enable_w0_dw0"))
        {
            w0_dw0_enabled= true;
            yInfo("'enable_w0_dw0' option found. w0 and dw0 will be used.\n");
        }

        //------------CHECK IF COM VELOCITY COMPUTATION IS ENABLED-----------//
        if (rf.check("experimental_com_vel"))
        {
            com_vel_enabled= true;
            yInfo("'enable_com_vel' option found. Extra COM velocity computation will be enabled.\n");
        }

        //------------------CHECK IF LEGS ARE ENABLED-----------//
        if (rf.check("no_legs"))
        {
            legs_enabled= false;
            yInfo("'no_legs' option found. Legs will be disabled.\n");
        }

        //------------------CHECK IF TORSO IS ENABLED-----------//
        if (rf.check("no_torso_legs"))
        {
            torso_enabled= false;
            legs_enabled= false;
            yInfo("no_torso_legs' option found. Torso and legs will be disabled.\n");
        }
        if (rf.check("no_torso"))
        {
            torso_enabled= false;
            yInfo("'no_torso' option found. Torso will be disabled.\n");
        }


        //------------------CHECK IF HEAD IS ENABLED-----------//
        if (rf.check("no_head"))
        {
            head_enabled= false;
            yInfo("'no_head' option found. Head will be disabled.\n");
        }

        //------------------CHECK IF ARMS ARE ENABLED-----------//
        if (rf.check("no_left_arm"))
        {
            left_arm_enabled= false;
            yInfo("'no_left_arm' option found. Left arm will be disabled.\n");
        }
        if (rf.check("no_right_arm"))
        {
            right_arm_enabled= false;
            yInfo("'no_right_arm' option found. Right arm will be disabled.\n");
        }

        //---------------------RATE/PERIOD-----------------------------//
        int rate = 10;
        if (rf.check("period"))
        {
            rate = rf.find("period").asInt();
            yInfo("rateThread working at %d ms\n", rate);
        }
        else
        {
            yInfo("Could not find period in the config file\nusing 10ms as default");
            rate = 10;
        }

        if (rf.check("rate"))
        {
            yError ("'rate' parameter is deprecated. Use 'period' instead");
            return false;
        }
        std::string remoteInertialName{"/"+robot_name+"/head/inertials"};
        if (rf.check("imuPortName"))
        {
            remoteInertialName = rf.find("imuPortName").asString();
        }

        //---------------------DUMMY_FT-------------------------//
        if (rf.check("dummy_ft"))
        {
            dummy_ft = true;
            yInfo("Using dummy FT sensors (debug mode)\n");
        }

        //---------------------DUMP-VEL-------------------------//
        if (rf.check("dumpvel"))
        {
            dump_vel_enabled = true;
            yInfo("Dumping joint velocities and accelerations (debug mode)\n");
        }

        if (rf.check("auto_drift_comp"))
        {
            auto_drift_comp = true;
            yInfo("Enabling automatic drift compensation (experimental)\n");
        }

        if (rf.check("default_ee_cont"))
        {
            default_ee_cont = true;
            yInfo("Default contact at the end effector\n");
        }

        //---------------------DEVICES--------------------------//
        if(head_enabled)
        {
            OptionsHead.put("device","remote_controlboard");
            OptionsHead.put("local","/"+local_name+"/head/client");
            OptionsHead.put("remote","/"+robot_name+"/head");

            if (!createDriver(dd_head, OptionsHead))
            {
                yError("unable to create head device driver...quitting\n");
                return false;
            }
            else
                yInfo("device driver created\n");
        }

        if (left_arm_enabled)
        {
            OptionsLeftArm.put("device","remote_controlboard");
            OptionsLeftArm.put("local","/"+local_name+"/left_arm/client");
            OptionsLeftArm.put("remote","/"+robot_name+"/left_arm");

            if (!createDriver(dd_left_arm, OptionsLeftArm))
            {
                yError("unable to create left arm device driver...quitting\n");
                return false;
            }
        }
        
        if (right_arm_enabled)
        {
            OptionsRightArm.put("device","remote_controlboard");
            OptionsRightArm.put("local","/"+local_name+"/right_arm/client");
            OptionsRightArm.put("remote","/"+robot_name+"/right_arm");

            if (!createDriver(dd_right_arm, OptionsRightArm))
            {
                yError("unable to create right arm device driver...quitting\n");
                return false;
            }
        }

        if (legs_enabled)
        {
            OptionsLeftLeg.put("device","remote_controlboard");
            OptionsLeftLeg.put("local","/"+local_name+"/left_leg/client");
            OptionsLeftLeg.put("remote","/"+robot_name+"/left_leg");

            if (!createDriver(dd_left_leg, OptionsLeftLeg))
            {
                yError("unable to create left leg device driver...quitting\n");
                return false;
            }

            OptionsRightLeg.put("device","remote_controlboard");
            OptionsRightLeg.put("local","/"+local_name+"/right_leg/client");
            OptionsRightLeg.put("remote","/"+robot_name+"/right_leg");

            if (!createDriver(dd_right_leg, OptionsRightLeg))
            {
                yError("unable to create right leg device driver...quitting\n");
                return false;
            }
        }

        if (torso_enabled)
        {
            OptionsTorso.put("device","remote_controlboard");
            OptionsTorso.put("local","/"+local_name+"/torso/client");
            OptionsTorso.put("remote","/"+robot_name+"/torso");

            if (!createDriver(dd_torso, OptionsTorso))
            {
                yError("unable to create head device driver...quitting\n");
                return false;
            }
            else
                yInfo("device driver created\n");
        }

        Property masConf {{"device",Value("multipleanalogsensorsclient")},
                          {"local", Value("/"+local_name+"/inertials")},
                          {"remote",Value(remoteInertialName)},
                          {"timeout",Value(0.04)}};

        if (!dd_MASClient.open(masConf))
        {
            yError("unable to open the MAS client...quitting\n");
            return false;
        }

        if(!dd_MASClient.view(m_iAcc) || !dd_MASClient.view(m_iGyro))
        {
            yError("view of one of the MAS interfaces required failed...quitting\n");
            return false;
        }

        //--------------------CHECK FT SENSOR------------------------
        if (!dummy_ft)
        {
            if ((dd_left_arm  && !Network::exists("/" + robot_name + "/left_arm/analog:o"))  ||
                (dd_right_arm && !Network::exists("/" + robot_name + "/right_arm/analog:o")) ||
                (dd_left_leg  && !Network::exists("/" + robot_name + "/left_leg/analog:o"))  ||
                (dd_right_leg && !Network::exists("/" + robot_name + "/right_leg/analog:o")) )
                {     
                    yError("Unable to detect the presence of F/T sensors in your iCub...quitting\n");
                    return false;
                }
        }     

        //---------------OPEN RPC PORT--------------------//
        string rpcPortName = "/"+local_name+"/rpc:i";
        rpcPort.open(rpcPortName);
        attach(rpcPort);                  

        //---------------OPEN INERTIAL PORTS--------------------//
        port_filtered_output.open("/"+local_name+"/filtered/inertial:o");
        inertialFilter = new dataFilter(port_filtered_output, m_iGyro, m_iAcc);

        //--------------------------THREAD--------------------------
        inv_dyn = new inverseDynamics(rate, dd_left_arm, dd_right_arm, dd_head, dd_left_leg, dd_right_leg, dd_torso, robot_name, local_name, icub_type, autoconnect);
        inv_dyn->com_enabled=com_enabled;
        inv_dyn->auto_drift_comp=auto_drift_comp;
        inv_dyn->com_vel_enabled=com_vel_enabled;
        inv_dyn->dummy_ft=dummy_ft;
        inv_dyn->w0_dw0_enabled=w0_dw0_enabled;
        inv_dyn->dumpvel_enabled=dump_vel_enabled;
        inv_dyn->default_ee_cont=default_ee_cont;

        yInfo("ft thread istantiated...\n");
        Time::delay(5.0);

        inertialFilter->start();
        inv_dyn->start();
        yInfo("thread started\n");
        return true;
    }

    bool close() override
    {
        //The order of execution of the following closures is important, do not change it.
        yInfo("Closing wholeBodyDynamics module... \n");     

        if (inv_dyn)
          {
            thread_status_enum thread_status = inv_dyn->getThreadStatus();
            if (thread_status!=STATUS_DISCONNECTED)
            {
                yInfo("Setting the icub in stiff mode\n");
                inv_dyn->setStiffMode();
            }
            yInfo("Stopping the inv_dyn thread...");     
            inv_dyn->stop();
            yInfo("inv_dyn thread stopped\n");     
            delete inv_dyn;
            inv_dyn=nullptr;
          }

        if(inertialFilter)
        {
            yInfo("Stopping the inertial filter thread \n");
            inertialFilter->stop();
            delete inertialFilter;
            inertialFilter=nullptr;
            dd_MASClient.close();
        }

        yInfo("Closing the filtered inertial output port \n");     
        port_filtered_output.interrupt();
        port_filtered_output.close();

        yInfo("Closing the rpc port \n");     
        rpcPort.close();

        if (dd_left_arm)
        {
            yInfo("Closing dd_left_arm \n");     
            dd_left_arm->close();
            delete dd_left_arm;
            dd_left_arm=nullptr;
        }
        if (dd_right_arm)
        {
            yInfo("Closing dd_right_arm \n");     
            dd_right_arm->close();
            delete dd_right_arm;
            dd_right_arm=nullptr;
        }
        if (dd_head)
        {
            yInfo("Closing dd_head \n");     
            dd_head->close();
            delete dd_head;
            dd_head=nullptr;
        }

        if (dd_left_leg)
        {
            yInfo("Closing dd_left_leg \n");     
            dd_left_leg->close();
            delete dd_left_leg;
            dd_left_leg=nullptr;
        }
        if (dd_right_leg)
        {
            yInfo("Closing dd_right_leg \n");     
            dd_right_leg->close();
            delete dd_right_leg;
            dd_right_leg=nullptr;
        }
        if (dd_torso)
        {
            yInfo("Closing dd_torso \n");     
            dd_torso->close();
            delete dd_torso;
            dd_torso=nullptr;
        }

        yInfo("wholeBodyDynamics module was closed successfully! \n");     
        return true;
    }

    double getPeriod() override
    {
        return 1.0;
    }
    bool updateModule() override
    {
        double avgTime, stdDev, period;
        period = inv_dyn->getPeriod();
        inv_dyn->getEstimatedPeriod(avgTime, stdDev);
        if(avgTime > 1.3 * period){
        //   yDebug("(real period: %3.3f +/- %3.3f; expected period %3.3f)\n", avgTime, stdDev, period);
        }

        static unsigned long int alive_counter = 0;
        static double curr_time = Time::now();
        if (Time::now() - curr_time > 60)
        {
            yInfo ("wholeBodyDynamics is alive! running for %ld mins.\n",++alive_counter);
            curr_time = Time::now();
        }

        if (inv_dyn==nullptr)
            return false;
        thread_status_enum thread_status = inv_dyn->getThreadStatus();
        if (thread_status==STATUS_OK)
            return true;
        else if (thread_status==STATUS_DISCONNECTED)
        {
            yError ("wholeBodyDynamics module lost connection with iCubInterface, now closing...\n");
            return false;
        }
        else
        {
            yInfo("wholeBodyDynamics module was closed successfully! \n");    
            return true;
        }
            
    }
};


int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.setDefaultContext("wholeBodyDynamics");
    rf.setDefaultConfigFile("wholeBodyDynamics.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--context context: where to find the called resource (referred to $ICUB_ROOT/app: default wholeBodyDynamics)" << endl;
        cout << "\t--from       from: the name of the file.ini to be used for calibration"                                       << endl;
        cout << "\t--period     period: the period used by the module. default: 10ms"                                           << endl;
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
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return 1;
    }

    wholeBodyDynamics obs;
    return obs.runModule(rf);
}

