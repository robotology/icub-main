/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include <algorithm>
#include <string>
#include <sstream>

#include <gsl/gsl_math.h>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/ctrl/tuning.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/**********************************************************************/
OnlineDCMotorEstimator::OnlineDCMotorEstimator()
{
    Vector x0(4,0.0);
    x0[2]=x0[3]=1.0;
    init(0.01,1.0,1.0,1e5,x0);
}


/**********************************************************************/
bool OnlineDCMotorEstimator::init(const double Ts, const double Q,
                                  const double R, const double P0,
                                  const Vector &x0)
{
    if (x0.length()<4)
        return false;

    A=F=eye(4,4);
    B.resize(4,0.0);
    C.resize(1,4); C(0,0)=1.0;
    Ct=C.transposed();

    P=P0*eye(4,4);
    this->Q=Q*eye(4,4);
    this->R=R;

    this->Ts=Ts;
    x=_x=x0.subVector(0,3);
    x[2]=1.0/_x[2];
    x[3]=_x[3]/_x[2];

    uOld=0.0;

    return true;
}


/**********************************************************************/
bool OnlineDCMotorEstimator::init(const double P0, const Vector &x0)
{
    if (x0.length()<4)
        return false;

    P=P0*eye(4,4);
    x=_x=x0.subVector(0,3);
    x[2]=1.0/_x[2];
    x[3]=_x[3]/_x[2];

    uOld=0.0;

    return true;
}


/**********************************************************************/
Vector OnlineDCMotorEstimator::estimate(const double u, const double y)
{
    double &x1=x[0];
    double &x2=x[1];
    double &x3=x[2];
    double &x4=x[3];

    double _exp=exp(-Ts*x3);
    double _exp_1=1.0-_exp;
    double _x3_2=x3*x3;
    double _tmp_1=(Ts*x3-_exp_1)/_x3_2;

    A(0,1)=_exp_1/x3;
    A(1,1)=_exp;

    B[0]=x4*_tmp_1;
    B[1]=x4*A(0,1);

    F(0,1)=A(0,1);
    F(1,1)=A(1,1);

    F(0,2)=-(x2*_exp_1)/_x3_2      + (uOld*x4*Ts*_exp_1)/_x3_2 - (2.0*uOld*B[0])/x3 + (Ts*x2*_exp)/x3;
    F(1,2)=-(uOld*x4*_exp_1)/_x3_2 - Ts*x2*_exp                + (uOld*x4*Ts*_exp)/x3;

    F(0,3)=uOld*_tmp_1;
    F(1,3)=uOld*A(0,1);

    // prediction
    x=A*x+B*uOld;
    P=F*P*F.transposed()+Q;

    // Kalman gain
    Matrix tmpMat=C*P*Ct;
    Matrix K=P*Ct/(tmpMat(0,0)+R);

    // correction
    x+=K*(y-C*x);
    P=(eye(4,4)-K*C)*P;

    _x[0]=x[0];
    _x[1]=x[1];
    _x[2]=1.0/x[2];
    _x[3]=x[3]/x[2];

    uOld=u;

    return _x;
}


/**********************************************************************/
OnlineStictionEstimator::OnlineStictionEstimator() :
                         RateThread(1000),   velEst(32,4.0), accEst(32,4.0),
                         trajGen(1,1.0,1.0), intErr(1.0,Vector(2,0.0)), done(2)
{
    imod=NULL;
    ilim=NULL;
    ienc=NULL;
    ipid=NULL;
    configured=false;
}


/**********************************************************************/
bool OnlineStictionEstimator::configure(PolyDriver &driver, const Property &options)
{
    Property &opt=const_cast<Property&>(options);
    if (driver.isValid() && opt.check("joint"))
    {
        bool ok=true;
        ok&=driver.view(imod);
        ok&=driver.view(ilim);
        ok&=driver.view(ienc);
        ok&=driver.view(ipid);

        if (!ok)
            return false;

        joint=opt.find("joint").asInt();
        setRate((int)(1000.0*opt.check("Ts",Value(0.01)).asDouble()));

        T=opt.check("T",Value(2.0)).asDouble();
        Kp=opt.check("Kp",Value(10.0)).asDouble();
        Ki=opt.check("Ki",Value(250.0)).asDouble();
        Kd=opt.check("Kd",Value(15.0)).asDouble();
        vel_thres=fabs(opt.check("vel_thres",Value(5.0)).asDouble());
        e_thres=fabs(opt.check("e_thres",Value(1.0)).asDouble());

        gamma.resize(2,1.0);
        if (Bottle *pB=opt.find("gamma").asList()) 
        {
            size_t len=std::min(gamma.length(),(size_t)pB->size());
            for (size_t i=0; i<len; i++)
                gamma[i]=pB->get(i).asDouble();
        }

        stiction.resize(2,0.0);
        if (Bottle *pB=opt.find("stiction").asList()) 
        {
            size_t len=std::min(stiction.length(),(size_t)pB->size());
            for (size_t i=0; i<len; i++)
                stiction[i]=pB->get(i).asDouble();
        }

        return configured=true;
    }
    else
        return false;
}


/**********************************************************************/
bool OnlineStictionEstimator::reconfigure(const Property &options)
{
    Property &opt=const_cast<Property&>(options);
    if (configured)
    {
        if (opt.check("joint"))
            joint=opt.find("joint").asInt();

        if (opt.check("Ts"))
            setRate((int)(1000.0*opt.find("Ts").asDouble()));

        if (opt.check("T"))
            T=opt.find("T").asDouble();

        if (opt.check("Kp"))
            Kp=opt.find("Kp").asDouble();

        if (opt.check("Ki"))
            Ki=opt.find("Ki").asDouble();

        if (opt.check("Kd"))
            Kd=opt.find("Kd").asDouble();

        if (opt.check("vel_thres"))
            vel_thres=opt.find("vel_thres").asDouble();

        if (opt.check("e_thres"))
            e_thres=opt.find("e_thres").asDouble();

        if (opt.check("gamma"))
        {
            if (Bottle *pB=opt.find("gamma").asList()) 
            {
                size_t len=std::min(gamma.length(),(size_t)pB->size());
                for (size_t i=0; i<len; i++)
                    gamma[i]=pB->get(i).asDouble();
            }
        }

        if (opt.check("stiction"))
        {
            if (Bottle *pB=opt.find("stiction").asList()) 
            {
                size_t len=std::min(stiction.length(),(size_t)pB->size());
                for (size_t i=0; i<len; i++)
                    stiction[i]=pB->get(i).asDouble();
            }
        }

        return true;
    }
    else
        return false;
}


/**********************************************************************/
void OnlineStictionEstimator::applyStictionLimit()
{
    stiction[0]=std::min(std::max(stiction[0],-stiction_limit),stiction_limit);
    stiction[1]=std::min(std::max(stiction[1],-stiction_limit),stiction_limit);
}


/**********************************************************************/
bool OnlineStictionEstimator::threadInit()
{
    if (!configured)
        return false;

    ilim->getLimits(joint,&x_min,&x_max);
    double x_range=x_max-x_min;
    x_min+=0.1*x_range;
    x_max-=0.1*x_range;
    imod->setControlMode(joint,VOCAB_CM_OPENLOOP);

    ienc->getEncoder(joint,&x_pos);
    x_vel=0.0;
    x_acc=0.0;

    tg=x_min;
    xd_pos=x_pos;
    state=(tg-x_pos>0.0)?rising:falling;
    adapt=adaptOld=false;

    trajGen.setTs(0.001*getRate());
    trajGen.setT(T);
    trajGen.init(Vector(1,x_pos));

    Vector _Kp(1,Kp),  _Ki(1,Ki),  _Kd(1,Kd);
    Vector _Wp(1,1.0), _Wi(1,1.0), _Wd(1,1.0);
    Vector _N(1,10.0), _Tt(1,1.0);

    Pid pidInfo;
    ipid->getPid(joint,&pidInfo);
    dpos_dV=(pidInfo.kp>=0.0?-1.0:1.0);
    stiction*=dpos_dV;
    Matrix _satLim(1,2);
    _satLim(0,0)=-pidInfo.max_int;
    _satLim(0,1)=pidInfo.max_int;
    stiction_limit=pidInfo.max_output;
    applyStictionLimit();

    pid=new parallelPID(0.001*getRate(),_Kp,_Ki,_Kd,_Wp,_Wi,_Wd,_N,_Tt,_satLim);
    pid->reset(Vector(1,0.0));

    intErr.setTs(0.001*getRate());
    intErr.reset(stiction);

    done=0.0;
    doneEvent.reset();
    t0=Time::now();

    return true;
}


/**********************************************************************/
void OnlineStictionEstimator::run()
{
    mutex.lock();

    ienc->getEncoder(joint,&x_pos);

    AWPolyElement el(Vector(1,x_pos),Time::now());
    Vector vel=velEst.estimate(el); x_vel=vel[0];
    Vector acc=accEst.estimate(el); x_acc=acc[0];

    double t=Time::now()-t0;
    if (t>2.0*trajGen.getT())
    {
        tg=(tg==x_min)?x_max:x_min;
        state=(tg-x_pos>0.0)?rising:falling;
        adapt=(fabs(x_vel)<vel_thres);
        t0=Time::now();
    }

    trajGen.computeNextValues(Vector(1,tg));
    xd_pos=trajGen.getPos()[0];        

    Vector pid_out=pid->compute(Vector(1,xd_pos),Vector(1,x_pos));
    double e_pos=xd_pos-x_pos;
    double fw=(state==rising)?stiction[0]:stiction[1];
    double u=fw+pid_out[0];

    Vector adaptGate(stiction.length(),0.0);
    if ((fabs(x_vel)<vel_thres) && adapt)
        adaptGate[(state==rising)?0:1]=1.0;
    else
        adapt=false;

    Vector cumErr=intErr.integrate(e_pos*adaptGate);

    // trigger on falling edge
    if (!adapt && adaptOld)
    {
        Vector e_mean=cumErr/t;
        if (yarp::math::norm(e_mean)>e_thres)
        {
            stiction+=gamma*e_mean;
            applyStictionLimit();
            done[state]=0.0;
        }
        else
            done[state]=1.0;

        intErr.reset(Vector(stiction.length(),0.0));
    }

    ipid->setOffset(joint,dpos_dV*u);
    adaptOld=adapt;

    // fill in info
    info.unput("voltage");   info.put("voltage",u);
    info.unput("position");  info.put("position",x_pos);
    info.unput("reference"); info.put("reference",xd_pos);

    mutex.unlock();

    if (done[0]*done[1]!=0.0)
        doneEvent.signal();
}


/**********************************************************************/
void OnlineStictionEstimator::threadRelease()
{
    ipid->setOffset(joint,0.0);
    imod->setControlMode(joint,VOCAB_CM_POSITION);
    delete pid;

    doneEvent.signal();
}


/**********************************************************************/
bool OnlineStictionEstimator::isDone()
{
    if (!configured)
        return false;

    mutex.lock();
    bool ret=(done[0]*done[1]!=0.0);
    mutex.unlock();

    return ret;
}


/**********************************************************************/
bool OnlineStictionEstimator::waitUntilDone()
{
    if (!configured)
        return false;

    doneEvent.wait();
    return isDone();
}


/**********************************************************************/
bool OnlineStictionEstimator::getResults(Vector &results)
{
    if (!configured)
        return false;

    mutex.lock();
    results=dpos_dV*stiction;
    mutex.unlock();

    return true;
}


/**********************************************************************/
bool OnlineStictionEstimator::getInfo(Property &info)
{
    if (!configured)
        return false;

    mutex.lock();
    info=this->info;
    mutex.unlock();

    return true;
}


/**********************************************************************/
OnlineCompensatorDesign::OnlineCompensatorDesign() : RateThread(1000),
                         predictor(Matrix(2,2),Matrix(2,1),Matrix(1,2),
                                   Matrix(2,2),Matrix(1,1))
{
    imod=NULL;
    ilim=NULL;
    ienc=NULL;
    ipos=NULL;
    ipid=NULL;    
    configured=false;
}


/**********************************************************************/
bool OnlineCompensatorDesign::configure(PolyDriver &driver, const Property &options)
{
    Property &opt=const_cast<Property&>(options);
    if (!driver.isValid())
        return false;

    bool ok=true;
    ok&=driver.view(imod);
    ok&=driver.view(ilim);
    ok&=driver.view(ienc);
    ok&=driver.view(ipos);
    ok&=driver.view(ipid);

    if (!ok)
        return false;

    // general options
    Bottle &optGeneral=opt.findGroup("general");
    if (optGeneral.isNull())
        return false;

    if (!optGeneral.check("joint"))
        return false;

    joint=optGeneral.find("joint").asInt();
    
    if (optGeneral.check("port"))
    {
        string name=optGeneral.find("port").asString().c_str();
        if (name[0]!='/')
            name="/"+name;

        if (!port.open(name.c_str()))
            return false;
    }

    // configure plant estimator
    Bottle &optPlant=opt.findGroup("plant_estimation");
    if (optPlant.isNull())
        return false;

    Pid pidInfo;
    ipid->getPid(joint,&pidInfo);
    dpos_dV=(pidInfo.kp>=0.0?-1.0:1.0);
    
    ilim->getLimits(joint,&x_min,&x_max);
    double x_range=x_max-x_min;
    x_min+=0.1*x_range;
    x_max-=0.1*x_range;

    x0.resize(4,0.0);
    x0[2]=optPlant.check("tau",Value(1.0)).asDouble();
    x0[3]=optPlant.check("K",Value(1.0)).asDouble();

    double Ts=optPlant.check("Ts",Value(0.01)).asDouble();
    double Q=optPlant.check("Q",Value(1.0)).asDouble();
    double R=optPlant.check("R",Value(1.0)).asDouble();
    P0=optPlant.check("P0",Value(1e5)).asDouble();
    max_pwm=optPlant.check("max_pwm",Value(800)).asDouble();

    setRate((int)(1000.0*Ts));

    if (!plant.init(Ts,Q,R,P0,x0))
        return false;

    // configure stiction estimator
    Bottle &optStiction=opt.findGroup("stiction_estimation");
    if (!optStiction.isNull())
    {
        Property propStiction(optStiction.toString().c_str());

        // enforce the equality between the common properties
        propStiction.unput("joint");
        propStiction.put("joint",joint);

        if (!stiction.configure(driver,propStiction))
            return false;
    }

    meanParams.resize(2,0.0);

    return configured=true;
}


/**********************************************************************/
bool OnlineCompensatorDesign::threadInit()
{
    bool ret=true;
    switch (mode)
    {
        // -----
        case plant_estimation:
        {
            imod->setControlMode(joint,VOCAB_CM_OPENLOOP);
            ienc->getEncoder(joint,&x0[0]);
            plant.init(P0,x0);
            meanParams=0.0;
            meanCnt=0;
            x_tg=x_max;
            pwm_pos=true;
            t1=Time::now();
            break;
        }

        // -----
        case plant_validation:
        {
            Vector _x0(2,0.0);
            imod->setControlMode(joint,VOCAB_CM_OPENLOOP);
            ienc->getEncoder(joint,&_x0[0]);
            predictor.init(_x0,predictor.get_P());
            measure_update_cnt=0;
            x_tg=x_max;
            pwm_pos=true;
            t1=Time::now();
            break;
        }

        // -----
        case stiction_estimation:
        {
            ret=stiction.startEstimation();
            break;
        }

        // -----
        case controller_validation:
        {
            pidCur=&pidOld;
            imod->setControlMode(joint,VOCAB_CM_POSITION);
            x_tg=x_max;
            if (controller_validation_ref_square)
                ipid->setReference(joint,x_tg);
            else
            {
                ipos->setRefAcceleration(joint,1e9);
                ipos->setRefSpeed(joint,(x_max-x_min)/controller_validation_ref_period);
                ipos->positionMove(joint,x_tg);
            }
            controller_validation_num_cycles=0;
            t1=Time::now();
            break;
        }
    }

    doneEvent.reset();
    t0=Time::now();

    return ret;
}


/**********************************************************************/
void OnlineCompensatorDesign::commandJoint(double &enc, double &u)
{
    double t=Time::now();
    bool timeoutExpired=(switch_timeout>0.0?t-t1>switch_timeout:false);
    ienc->getEncoder(joint,&enc);    

    // switch logic
    if (x_tg==x_max)
    {
        if ((enc>x_max) || timeoutExpired)
        {
            x_tg=x_min;
            pwm_pos=false;
            t1=t;
        }
    }
    else if ((enc<x_min) || timeoutExpired)
    {
        x_tg=x_max;
        pwm_pos=true;
        t1=t;
    }

    u=(pwm_pos?max_pwm:-max_pwm);
    ipid->setOffset(joint,dpos_dV*u);
}


/**********************************************************************/
void OnlineCompensatorDesign::run()
{
    double t=Time::now();
    if (max_time>0.0)
        if (t-t0>max_time)
            askToStop();

    mutex.lock();
    switch (mode)
    {
        // -----
        case plant_estimation:
        {
            double enc,u;
            commandJoint(enc,u);
            plant.estimate(u,enc);

            // average parameters tau and K
            meanParams*=meanCnt;
            meanParams+=plant.get_parameters();
            meanParams/=++meanCnt;

            if (port.getOutputCount()>0)
            {
                Vector &info=port.prepare();
                info.resize(3);

                info[0]=0.0;
                info[1]=u;
                info[2]=enc;
                info=cat(info,plant.get_x());
                info=cat(info,meanParams);

                port.write();
            }

            break;
        }

        // -----
        case plant_validation:
        {
            double enc,u;
            commandJoint(enc,u);
            predictor.predict(Vector(1,u));

            // correction only when requested
            if (measure_update_ticks>0)
            {
                if (++measure_update_cnt>=measure_update_ticks)
                {
                    predictor.correct(Vector(1,enc));
                    measure_update_cnt=0;
                }
            }

            if (port.getOutputCount()>0)
            {
                Vector &info=port.prepare();
                info.resize(3);

                info[0]=1.0;
                info[1]=u;
                info[2]=enc;
                info=cat(info,predictor.get_x());
                info=cat(info,Vector(4,0.0));   // zero-padding

                port.write();
            }

            break;
        }

        // -----
        case stiction_estimation:
        {
            if (stiction.isDone())
                askToStop();

            if (port.getOutputCount()>0)
            {
                Property stiction_info;
                stiction.getInfo(stiction_info);

                Vector &info=port.prepare();
                info.resize(4);

                info[0]=2.0;
                info[1]=stiction_info.find("voltage").asDouble();
                info[2]=stiction_info.find("position").asDouble();
                info[3]=stiction_info.find("reference").asDouble();

                Vector results;
                stiction.getResults(results);
                info=cat(info,results);
                info=cat(info,Vector(3,0.0));   // zero-padding

                port.write();
            }

            break;
        }

        // -----
        case controller_validation:
        {
            if (t-t1>controller_validation_ref_period+
                controller_validation_ref_sustain_time)
            {
                x_tg=(x_tg==x_max?x_min:x_max);
                t1=t;

                if (x_tg==x_max)
                {
                    if (++controller_validation_num_cycles>=controller_validation_cycles_to_switch)
                    {
                        pidCur=(pidCur==&pidOld?&pidNew:&pidOld);                    
                        if ((pidCur==&pidOld) && controller_validation_stiction_yarp)
                            ipid->setOffset(joint,0.0);

                        ipid->setPid(joint,*pidCur);
                        controller_validation_num_cycles=0;
                    }
                }

                if ((pidCur==&pidNew) && controller_validation_stiction_yarp)
                    ipid->setOffset(joint,(x_tg==x_max)?controller_validation_stiction_up:
                                                        controller_validation_stiction_down);

                if (controller_validation_ref_square)
                    ipid->setReference(joint,x_tg);
                else
                    ipos->positionMove(joint,x_tg);
            }

            if (port.getOutputCount()>0)
            {
                Vector &info=port.prepare();
                info.resize(5);

                info[0]=3.0;
                ipid->getOutput(joint,&info[1]);
                ienc->getEncoder(joint,&info[2]);
                ipid->getReference(joint,&info[3]);
                info[4]=(pidCur==&pidOld?0.0:1.0);
                info=cat(info,Vector(4,0.0));   // zero-padding

                port.write();
            }

            break;
        }
    }
    mutex.unlock();
}


/**********************************************************************/
void OnlineCompensatorDesign::threadRelease()
{
    switch (mode)
    {
        // -----
        case plant_estimation:
        // -----
        case plant_validation:
        {
            ipid->setOffset(joint,0.0);
            imod->setControlMode(joint,VOCAB_CM_POSITION);
            break;
        }

        // -----
        case stiction_estimation:
        {
            stiction.stopEstimation();
            break;
        }

        // -----
        case controller_validation:
        {
            ipos->stop(joint);
            ipid->setPid(joint,pidOld);
            break;
        }
    }

    doneEvent.signal();
}


/**********************************************************************/
bool OnlineCompensatorDesign::tuneController(const Property &options,
                                             Property &results)
{
    Property &opt=const_cast<Property&>(options);
    if (!opt.check("tau")  || !opt.check("K") ||
        !opt.check("type") || !opt.check("f_c"))
        return false;

    double tau=opt.find("tau").asDouble();
    double K=opt.find("K").asDouble();
    string type=opt.check("type",Value("PI")).asString().c_str();
    double omega_c=2.0*M_PI*opt.find("f_c").asDouble();
    double Kp,Ki;

    // P design
    if (type=="P")
    {
        Kp=(omega_c/K)*sqrt(1.0+omega_c*omega_c*tau*tau);
        Ki=0.0;
    }
    // PI design
    else if (type=="PI")
    {
        double T_dr=1.0/omega_c;
        if (opt.check("T_dr"))
            T_dr=opt.find("T_dr").asDouble();

        double tau_dr=T_dr/3.0;
        double omega_dr=1.0/tau_dr;

        // reinforce omega_dr as the slowest pole among the three
        omega_dr=std::min(omega_dr,1.0/(3.0*tau));

        Kp=(omega_c/K)*sqrt(1.0+omega_c*omega_c*tau*tau);
        Ki=omega_dr*(Kp-(omega_dr*(1.0-omega_dr*tau))/K);

        // reinforce no mutual dependency between Kp and Ki design
        Ki=std::min(Ki,sqrt(10.0)*omega_c*Kp);

        // reinforce stable closed loop system
        Ki=std::min(Ki,Kp/tau);
    }
    else
        return false;

    results.clear();
    results.put("Kp",Kp);
    results.put("Ki",Ki);

    return true;
}


/**********************************************************************/
bool OnlineCompensatorDesign::startPlantEstimation(const Property &options)
{
    Property &opt=const_cast<Property&>(options);
    if (!configured)
        return false;

    max_time=opt.check("max_time",Value(0.0)).asDouble();
    switch_timeout=opt.check("switch_timeout",Value(0.0)).asDouble();

    mode=plant_estimation;
    return RateThread::start();
}


/**********************************************************************/
bool OnlineCompensatorDesign::startPlantValidation(const Property &options)
{
    Property &opt=const_cast<Property&>(options);
    if (!configured || !opt.check("tau") || !opt.check("K"))
        return false;
    
    max_time=opt.check("max_time",Value(0.0)).asDouble();
    switch_timeout=opt.check("switch_timeout",Value(0.0)).asDouble();
    measure_update_ticks=opt.check("measure_update_ticks",Value(100)).asInt();

    double tau=opt.find("tau").asDouble();
    double K=opt.find("K").asDouble();
    double Ts=0.001*getRate();
    double a=1.0/tau;
    double b=K/tau;

    double Q=opt.check("Q",Value(1.0)).asDouble();
    double R=opt.check("R",Value(1.0)).asDouble();
    double P0=opt.check("P0",Value(this->P0)).asDouble();

    // set up the Kalman filter
    double _exp=exp(-Ts*a);
    double _exp_1=1.0-_exp;

    Matrix A=eye(2,2);
    A(0,1)=_exp_1/a;
    A(1,1)=_exp;

    Matrix B(2,1);
    B(0,0)=b*(a*Ts-_exp_1)/(a*a);
    B(1,0)=b*A(0,1);

    Matrix H(1,2);
    H(0,0)=1.0;
    H(0,1)=0.0;

    predictor.set_A(A);
    predictor.set_B(B);
    predictor.set_H(H);
    predictor.set_Q(Q*eye(2,2));
    predictor.set_R(R*eye(1,1));
    predictor.init(Vector(2,0.0),P0*eye(2,2));

    mode=plant_validation;
    return RateThread::start();
}


/**********************************************************************/
bool OnlineCompensatorDesign::startStictionEstimation(const Property &options)
{    
    if (!configured)
        return false;

    Property opt=options;
    max_time=opt.check("max_time",Value(0.0)).asDouble();

    opt.unput("joint");
    if (!stiction.reconfigure(opt))
        return false;

    mode=stiction_estimation;
    return RateThread::start();
}


/**********************************************************************/
bool OnlineCompensatorDesign::startControllerValidation(const Property &options)
{
    Property &opt=const_cast<Property&>(options);
    if (!configured || !opt.check("Kp"))
        return false;

    max_time=opt.check("max_time",Value(0.0)).asDouble();

    ipid->getPid(joint,&pidOld);
    pidNew=pidOld;

    // enforce the correct sign of gains
    double Kp=opt.find("Kp").asDouble();
    double Ki=opt.check("Ki",Value(0.0)).asDouble();
    double Kd=opt.check("Kd",Value(0.0)).asDouble();
    Kp=(Kp*pidOld.kp>0.0)?Kp:-Kp;
    Ki=(Ki*pidOld.ki>0.0)?Ki:-Ki;
    Kd=(Kd*pidOld.kd>0.0)?Kd:-Kd;
    pidNew.setKp(Kp);
    pidNew.setKi(Ki);
    pidNew.setKd(Kd);
    pidNew.setStictionValues(0.0,0.0);

    if (opt.check("scale"))
        pidNew.setScale(opt.find("scale").asInt());

    controller_validation_ref_square=(opt.check("ref_type",Value("square")).asString()=="square");
    controller_validation_ref_period=opt.check("ref_period",Value(2.0)).asDouble();
    controller_validation_ref_sustain_time=opt.check("ref_sustain_time",Value(0.0)).asDouble();
    controller_validation_cycles_to_switch=opt.check("cycles_to_switch",Value(1)).asInt();
    controller_validation_stiction_yarp=(opt.check("stiction_compensation",Value("firmware")).asString()!="firmware");
    controller_validation_stiction_up=controller_validation_stiction_down=0.0;

    if (opt.check("stiction"))
    {
        if (Bottle *pB=opt.find("stiction").asList())
        {
            if (pB->size()>=2)
            {
                controller_validation_stiction_up=pB->get(0).asDouble();
                controller_validation_stiction_down=pB->get(1).asDouble();

                if (!controller_validation_stiction_yarp)
                    pidNew.setStictionValues(controller_validation_stiction_up,
                                             controller_validation_stiction_down);
            }
        }
    }

    mode=controller_validation;
    return RateThread::start();
}


/**********************************************************************/
bool OnlineCompensatorDesign::isDone()
{
    if (!configured)
        return false;

    return !isRunning();
}


/**********************************************************************/
bool OnlineCompensatorDesign::waitUntilDone()
{
    if (!configured)
        return false;

    doneEvent.wait();
    return isDone();
}


/**********************************************************************/
bool OnlineCompensatorDesign::getResults(Property &results)
{
    if (!configured)
        return false;

    results.clear();

    mutex.lock();
    switch (mode)
    {
        // -----
        case plant_estimation:
        {
            Vector params=plant.get_parameters();
            results.put("tau",params[0]);
            results.put("K",params[1]);
            results.put("tau_mean",meanParams[0]);
            results.put("K_mean",meanParams[1]);
            break;
        }

        // -----
        case plant_validation:
        {
            Vector response=predictor.get_x();
            results.put("position",response[0]);
            results.put("velocity",response[1]);
            break;
        }

        // -----
        case stiction_estimation:
        {
            Vector values;
            stiction.getResults(values);
            ostringstream str;
            str<<"( ";
            str<<values[0];
            str<<" ";
            str<<values[1];
            str<<" )";
            Value val; val.fromString(str.str().c_str());
            results.put("stiction",val);
            break;
        }

        // -----
        case controller_validation:
        {
            Vector info(3);
            ipid->getOutput(joint,&info[0]);
            ienc->getEncoder(joint,&info[1]);
            ipid->getReference(joint,&info[2]);

            results.put("voltage",info[0]);
            results.put("position",info[1]);
            results.put("reference",info[2]);
            results.put("pid",pidCur==&pidOld?"old":"new");
            break;
        }
    }
    mutex.unlock();

    return true;
}


/**********************************************************************/
OnlineCompensatorDesign::~OnlineCompensatorDesign()
{
    port.close();
}




