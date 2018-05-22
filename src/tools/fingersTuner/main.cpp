/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
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
\defgroup icub_fingersTuner Fingers PID Tuner
@ingroup icub_tools

A module that provides tuning capabilities of the low-level PID
controllers for the robot's fingers.

\section intro_sec Description
At start-up, the module performs a synchronization among the PID
values stored aboard the robot and the ones available locally in
the configuration file. At run-time, user can ask the module to
carry out an online tuning of the PID parameters in order to
improve the performance.

\section lib_sec Libraries
- YARP.
- ctrlLib.

\section parameters_sec Parameters
The configuration parameter looks like the following:

\code
[general]
name              fingersTuner
robot             icub
ping_robot_tmo    40.0
relevantParts     (left_hand)
relevantJoints    (8 9 10 11 12 13 14 15)

numAlias          2
alias_0           (tag index)  (joints (11 12))
alias_1           (tag all)    (joints (9 10 11 12 13 14 15))

[left_hand]
device            left_arm
joint_8           (encs_ratio 17.50) (status download) (idling (9 10))
joint_9           (encs_ratio -2.10) (status download) (idling (10))
joint_10          (encs_ratio -2.36) (status download)
joint_11          (encs_ratio -2.58) (status download) (idling (12))
joint_12          (encs_ratio -2.32) (status download)
joint_13          (encs_ratio -2.10) (status download) (idling (14))
joint_14          (encs_ratio -2.57) (status download)
joint_15          (encs_ratio -1.70) (status download)
\endcode

Most of the options are self-explaining, whereas the <b>status</b> and
<b>idling</b> deserve an explanation. \n
Values of <b>status</b> option can be either <i>download</i> or <i>upload</i>:
in the first case it is asked the module at startup to store locally the PIDs
values available onboard the robot, while in the second case the local
configuration values will be uploaded to the robot. \n
The option <b>idling</b> allows putting in idle specific coupled joints that
should be not controlled in position during the tuning of the joint under subject.

\section portsif_sec Ports Interface
The interface to this module is implemented through
\ref fingersTuner_IDL . \n
Be careful that from the console a yarp::os::Value must be typed
by the user enclosed between parentheses, so that typical
commands are <i>tune left_hand (12)</i> or <i>tune left_hand
(index)</i>.

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/

#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/Vector.h>
#include <iCub/ctrl/tuning.h>

#include "fingersTuner_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;


/************************************************************************/
class Tuner
{
protected:
    typedef enum { download, upload, synced } pidStatus;
    struct PidData
    {
        double Kp;
        double Ki;
        double Kd;
        double scale;
        double st_up;
        double st_down;
        double encs_ratio;
        pidStatus status;
        vector<int> idling_joints;
        PidData() : Kp(0.0), Ki(0.0), Kd(0.0), scale(0.0),
                    st_up(0.0), st_down(0.0), encs_ratio(1.0),
                    status(download) { }
        void toRobot(Pid &pid)
        {
            pid.kp=Kp;
            pid.ki=Ki;
            pid.kd=Kd;
            pid.scale=scale;
            pid.stiction_up_val=st_up;
            pid.stiction_down_val=st_down;
        }
        void fromRobot(const Pid &pid)
        {
            Kp=pid.kp;
            Ki=pid.ki;
            Kd=pid.kd;
            scale=pid.scale;
            st_up=pid.stiction_up_val;
            st_down=pid.stiction_down_val;
        }
    };

    static unsigned int instances;
    bool interrupting;
    string name,robot,part,device;
    PolyDriver *driver;

    Bottle rJoints;
    map<int,PidData> pids;
    map<string,Bottle> alias;

    /************************************************************************/
    PolyDriver *waitPart(const Property &partOpt, const double ping_robot_tmo)
    {
        PolyDriver *pDrv=NULL;

        double t0=Time::now();
        while (Time::now()-t0<ping_robot_tmo)
        {
            if (pDrv!=NULL)
                delete pDrv;

            pDrv=new PolyDriver(const_cast<Property&>(partOpt));
            bool ok=pDrv->isValid();

            if (ok)
            {
                yInfo("Checking if %s is active ... yes",device.c_str());
                return pDrv;
            }
            else
            {
                double dt=ping_robot_tmo-(Time::now()-t0);
                yInfo("Checking if %s is active ... not yet: still %.1f [s] to timeout expiry",
                      device.c_str(),dt>0.0?dt:0.0);

                double t1=Time::now();
                while (Time::now()-t1<1.0)
                    Time::delay(0.1);
            }

            if (interrupting)
                break;
        }

        return pDrv;
    }

    /************************************************************************/
    PidData getPidData(Bottle &bGroup, const int i)
    {
        PidData pid;

        ostringstream joint;
        joint<<"joint_"<<i;
        Bottle &bJoint=bGroup.findGroup(joint.str());

        pid.Kp=bJoint.check("Kp",Value(0.0)).asDouble();
        pid.Ki=bJoint.check("Ki",Value(0.0)).asDouble();
        pid.Kd=bJoint.check("Kd",Value(0.0)).asDouble();
        pid.scale=bJoint.check("scale",Value(0.0)).asDouble();
        pid.st_up=bJoint.check("st_up",Value(0.0)).asDouble();
        pid.st_down=bJoint.check("st_down",Value(0.0)).asDouble();
        pid.encs_ratio=bJoint.check("encs_ratio",Value(1.0)).asDouble();
        pid.status=(bJoint.check("status",Value("download")).asString()=="download"?download:upload);
        if (bJoint.check("idling"))
        {
            if (Bottle *bIdlingJoints=bJoint.find("idling").asList())
            {
                pid.idling_joints.clear();
                for (int j=0; j<bIdlingJoints->size(); j++)
                {
                    int k=bIdlingJoints->get(j).asInt();

                    int l;
                    for (l=0; l<rJoints.size(); l++)
                    {
                        if (rJoints.get(l).asInt()==k)
                        {
                            pid.idling_joints.push_back(k);
                            break;
                        }
                    }

                    if (l>=rJoints.size())
                        yError("unrecognized joint %d to put in idle",k);
                }
            }
        }

        return pid;
    }

    /************************************************************************/
    void idlingCoupledJoints(const int i, const bool sw)
    {
        IControlMode *imod;
        driver->view(imod);

        PidData &pid=pids[i];
        for (size_t j=0; j<pid.idling_joints.size(); j++)
            imod->setControlMode(pid.idling_joints[j],
                                 sw?VOCAB_CM_IDLE:VOCAB_CM_POSITION);
    }

    /************************************************************************/
    bool tune(const int i)
    {
        PidData &pid=pids[i];

        Property pGeneral;
        pGeneral.put("joint",i);
        string sGeneral="(general ";
        sGeneral+=pGeneral.toString();
        sGeneral+=')';

        Bottle bGeneral,bPlantEstimation,bStictionEstimation;
        bGeneral.fromString(sGeneral);
        bPlantEstimation.fromString("(plant_estimation (Ts 0.01) (Q 1.0) (R 1.0) (P0 100000.0) (tau 1.0) (K 1.0) (max_pwm 800.0))");
        bStictionEstimation.fromString("(stiction_estimation (Ts 0.01) (T 2.0) (vel_thres 5.0) (e_thres 1.0) (gamma (10.0 10.0)) (stiction (0.0 0.0)))");

        Bottle bConf=bGeneral;
        bConf.append(bPlantEstimation);
        bConf.append(bStictionEstimation);

        Property pOptions(bConf.toString().c_str());
        OnlineCompensatorDesign designer;
        if (!designer.configure(*driver,pOptions))
        {
            yError("designer configuration failed!");
            return false;
        }

        idlingCoupledJoints(i,true);

        Property pPlantEstimation;
        pPlantEstimation.put("max_time",20.0);
        pPlantEstimation.put("switch_timeout",2.0);
        designer.startPlantEstimation(pPlantEstimation);

        yInfo("Estimating plant for joint %d: max duration = %g seconds",
              i,pPlantEstimation.find("max_time").asDouble());

        double t0=Time::now();
        while (!designer.isDone())
        {
            yInfo("elapsed %d [s]",(int)(Time::now()-t0));
            Time::delay(1.0);
            if (interrupting)
            {
                idlingCoupledJoints(i,false);
                return false;
            }
        }

        Property pResults;
        designer.getResults(pResults);
        double tau=pResults.find("tau_mean").asDouble();
        double K=pResults.find("K_mean").asDouble();
        yInfo("plant = %g/s * 1/(1+s*%g)",K,tau);

        Property pControllerRequirements,pController;
        pControllerRequirements.put("tau",tau);
        pControllerRequirements.put("K",K);
        pControllerRequirements.put("f_c",0.75);

        if (i!=15)
        {
            pControllerRequirements.put("T_dr",1.0);
            pControllerRequirements.put("type","PI");
        }
        else
            pControllerRequirements.put("type","P");

        designer.tuneController(pControllerRequirements,pController);
        yInfo("tuning results: %s",pController.toString().c_str());
        double Kp=pController.find("Kp").asDouble();
        double Ki=pController.find("Ki").asDouble();
        pid.scale=4.0;
        int scale=(int)pid.scale; int shift=1<<scale;
        double fwKp=floor(Kp*pid.encs_ratio*shift);
        double fwKi=floor(Ki*pid.encs_ratio*shift/1000.0);
        pid.Kp=yarp::math::sign(pid.Kp*fwKp)>0.0?fwKp:-fwKp;
        pid.Ki=yarp::math::sign(pid.Ki*fwKi)>0.0?fwKi:-fwKi;
        pid.Kd=0.0;
        yInfo("Kp (FW) = %g; Ki (FW) = %g; Kd (FW) = %g; shift factor = %d",pid.Kp,pid.Ki,pid.Kd,scale);

        Property pStictionEstimation;
        pStictionEstimation.put("max_time",60.0);
        pStictionEstimation.put("Kp",Kp);
        pStictionEstimation.put("Ki",0.0);
        pStictionEstimation.put("Kd",0.0);
        designer.startStictionEstimation(pStictionEstimation);

        yInfo("Estimating stiction for joint %d: max duration = %g seconds",
              i,pStictionEstimation.find("max_time").asDouble());

        t0=Time::now();
        while (!designer.isDone())
        {
            yInfo("elapsed %d [s]",(int)(Time::now()-t0));
            Time::delay(1.0);
            if (interrupting)
            {
                idlingCoupledJoints(i,false);
                return false;
            }
        }

        designer.getResults(pResults);
        pid.st_up=floor(pResults.find("stiction").asList()->get(0).asDouble());
        pid.st_down=floor(pResults.find("stiction").asList()->get(1).asDouble());
        yInfo("Stiction values: up = %g; down = %g",pid.st_up,pid.st_down);

        IControlMode *imod;
        IPositionControl *ipos;
        IEncoders *ienc;
        driver->view(imod);
        driver->view(ipos);
        driver->view(ienc);
        imod->setControlMode(i,VOCAB_CM_POSITION);
        ipos->setRefSpeed(i,50.0);
        ipos->positionMove(i,0.0);
        yInfo("Driving the joint back to rest... ");
        t0=Time::now();
        while (Time::now()-t0<5.0)
        {
            double enc;
            ienc->getEncoder(i,&enc);
            if (fabs(enc)<1.0)
                break;

            if (interrupting)
            {
                idlingCoupledJoints(i,false);
                return false;
            }

            Time::delay(0.2);
        }
        yInfo("done!");

        idlingCoupledJoints(i,false);
        return true;
    }

public:
    /************************************************************************/
    Tuner() : interrupting(false), driver(NULL)
    {
        instances++;
    }

    /************************************************************************/
    bool configure(ResourceFinder &rf, const string &part)
    {
        this->part=part;
        Bottle &bGeneral=rf.findGroup("general");
        if (bGeneral.isNull())
        {
            yError("group [general] is missing!");
            return false;
        }

        Bottle &bPart=rf.findGroup(part);
        if (bPart.isNull())
        {
            yError("group [%s] is missing!",part.c_str());
            return false;
        }

        if (!bPart.check("device"))
        {
            yError("\"device\" option is missing!");
            return false;
        }

        name=bGeneral.check("name",Value("fingersTuner")).asString();
        robot=bGeneral.check("robot",Value("icub")).asString();
        double ping_robot_tmo=bGeneral.check("ping_robot_tmo",Value(0.0)).asDouble();
        device=bPart.find("device").asString();

        if (Bottle *rj=bGeneral.find("relevantJoints").asList())
            rJoints=*rj;
        else
        {
            yError("\"relevantJoints\" option is missing!");
            return false;
        }

        int numAlias=bGeneral.check("numAlias",Value(0)).asInt();
        for (int i=0; i<numAlias; i++)
        {
            ostringstream item;
            item<<"alias_"<<i;
            Bottle &bAlias=bGeneral.findGroup(item.str());
            if (Bottle *joints=bAlias.find("joints").asList())
                alias[bAlias.find("tag").asString()]=*joints;
        }

        // special wildcard to point to all the joints
        alias["*"]=rJoints;

        for (int i=0; i<rJoints.size(); i++)
        {
            int j=rJoints.get(i).asInt();
            pids[j]=getPidData(bPart,j);
        }

        ostringstream portsSuffix;
        portsSuffix<<instances<<"/"<<device;

        Property option("(device remote_controlboard)");
        option.put("remote","/"+robot+"/"+device);
        option.put("local","/"+name+"/"+portsSuffix.str());

        if (ping_robot_tmo>0.0)
            driver=waitPart(option,ping_robot_tmo);
        else
            driver=new PolyDriver(option);

        if (!driver->isValid())
        {
            yError("%s device driver not available!",device.c_str());
            return false;
        }

        return true;
    }

    /************************************************************************/
    bool sync(const Value &sel)
    {
        Bottle joints;
        if (sel.isInt())
            joints.addInt(sel.asInt());
        else if (sel.isString())
        {
            map<string,Bottle>::iterator it=alias.find(sel.asString());
            if (it!=alias.end())
                joints=it->second;
            else
                return false;
        }
        else
            return false;

        for (int i=0; i<joints.size(); i++)
        {
            int j=rJoints.get(i).asInt();
            map<int,PidData>::iterator it=pids.find(j);
            if (it==pids.end())
                continue;

            IPidControl *ipid;
            driver->view(ipid);
            Pid _pid;
            ipid->getPid(VOCAB_PIDTYPE_POSITION,j,&_pid);

            PidData &pid=it->second;
            if (pid.status==download)
                pid.fromRobot(_pid);
            else
            {
                pid.toRobot(_pid);
                ipid->setPid(VOCAB_PIDTYPE_POSITION,j,_pid);
            }

            pid.status=synced;
        }

        return true;
    }

    /************************************************************************/
    bool tune(const Value &sel)
    {
        Bottle joints;
        if (sel.isInt())
            joints.addInt(sel.asInt());
        else if (sel.isString())
        {
            map<string,Bottle>::iterator it=alias.find(sel.asString());
            if (it!=alias.end())
                joints=it->second;
            else
                return false;
        }
        else
            return false;

        for (int i=0; i<joints.size(); i++)
        {
            int j=joints.get(i).asInt();
            map<int,PidData>::iterator it=pids.find(j);
            if (it==pids.end())
                continue;

            if (tune(j))
            {
                IPidControl *ipid;
                driver->view(ipid);
                Pid _pid;
                ipid->getPid(VOCAB_PIDTYPE_POSITION,j,&_pid);

                PidData &pid=it->second;
                pid.toRobot(_pid);
                ipid->setPid(VOCAB_PIDTYPE_POSITION,j,_pid);
                pid.status=synced;
            }

            if (interrupting)
                return false;
        }

        return true;
    }

    /************************************************************************/
    string toString()
    {
        ostringstream stream;
        stream<<"["<<part<<"]"<<endl;
        stream<<"device   "<<device<<endl;

        for (int i=0; i<rJoints.size(); i++)
        {
            int j=rJoints.get(i).asInt();
            PidData &pid=pids[j];

            Property prop;
            prop.put("Kp",pid.Kp);
            prop.put("Ki",pid.Ki);
            prop.put("Kd",pid.Kd);
            prop.put("scale",pid.scale);
            prop.put("st_up",pid.st_up);
            prop.put("st_down",pid.st_down);
            prop.put("encs_ratio",pid.encs_ratio);
            prop.put("status",pid.status==download?"download":"upload");

            stream<<"joint_"<<j<<"   ";
            stream<<prop.toString()<<endl;
        }

        return stream.str();
    }

    /************************************************************************/
    void interrupt()
    {
        interrupting=true;
    }

    /************************************************************************/
    ~Tuner()
    {
        delete driver;
    }
};


unsigned int Tuner::instances=0;
/************************************************************************/
class TunerModule: public RFModule, public fingersTuner_IDL
{
protected:
    ResourceFinder *rf;
    map<string,Tuner*> tuners;
    bool interrupting;
    bool closing;
    RpcServer rpcPort;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        interrupting=false;
        closing=false;
        this->rf=&rf;

        Bottle &bGeneral=rf.findGroup("general");
        if (bGeneral.isNull())
        {
            yError("group [general] is missing!");
            return false;
        }

        string name=bGeneral.check("name",Value("fingersTuner")).asString();
        setName(name.c_str());

        if (Bottle *bParts=bGeneral.find("relevantParts").asList())
        {
            for (int i=0; (i<bParts->size()) && !interrupting; i++)
            {
                string part=bParts->get(i).asString();
                tuners[part]=new Tuner;
                Tuner *tuner=tuners[part];

                if (!tuner->configure(rf,part))
                {
                    dispose();
                    return false;
                }

                tuner->sync(Value("*"));
            }
        }
        else
        {
            yError("\"relevantParts\" option is missing!");
            return false;
        }

        rpcPort.open("/"+name+"/rpc");
        attach(rpcPort);

        // request high resolution scheduling
        Time::turboBoost();

        return true;
    }

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /************************************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /************************************************************************/
    bool updateModule()
    {
        return !closing;
    }

    /************************************************************************/
    bool interruptModule()
    {
        interrupting=true;
        for (map<string,Tuner*>::iterator it=tuners.begin(); it!=tuners.end(); ++it)
            it->second->interrupt();

        return true;
    }

    /************************************************************************/
    void dispose()
    {
        for (map<string,Tuner*>::iterator it=tuners.begin(); it!=tuners.end(); ++it)
            delete it->second;
    }

    /************************************************************************/
    bool sync(const string &part, const Value &val)
    {
        map<string,Tuner*>::iterator it=tuners.find(part);
        if (it!=tuners.end())
            if (it->second->sync(val))
                return true;

        return false;
    }

    /************************************************************************/
    bool tune(const string &part, const Value &val)
    {
        map<string,Tuner*>::iterator it=tuners.find(part);
        if (it!=tuners.end())
            if (it->second->tune(val))
                return true;

        return false;
    }

    /************************************************************************/
    bool save()
    {
        string fileName=rf->getHomeContextPath();
        fileName+="/";
        fileName+=rf->find("from").asString();

        ofstream fout;
        fout.open(fileName.c_str());

        Bottle &bGeneral=rf->findGroup("general");
        fout<<"["<<bGeneral.get(0).asString()<<"]"<<endl;
        for (int i=1; i<bGeneral.size(); i++)
            fout<<bGeneral.get(i).toString()<<endl;

        fout<<endl;
        for (map<string,Tuner*>::iterator it=tuners.begin(); it!=tuners.end(); ++it)
            fout<<it->second->toString()<<endl;

        fout.close();
        return true;
    }

    /************************************************************************/
    bool quit()
    {
        return closing=true;
    }

    /************************************************************************/
    bool close()
    {
        save();
        dispose();
        rpcPort.close();
        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    ResourceFinder rf;
    rf.setDefaultContext("fingersTuner");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    TunerModule mod;
    return mod.runModule(rf);
}
