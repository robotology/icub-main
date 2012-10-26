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

    F(0,2)=-(x2*_exp_1)/_x3_2   + (u*x4*Ts*_exp_1)/_x3_2 - (2.0*u*B[0])/x3 + (Ts*x2*_exp)/x3;
    F(1,2)=-(u*x4*_exp_1)/_x3_2 - Ts*x2*_exp             + (u*x4*Ts*_exp)/x3;

    F(0,3)=u*_tmp_1;
    F(1,3)=u*A(0,1);

    // prediction
    x=A*x+B*u;
    P=F*P*F.transposed()+Q;

    // Kalman gain
    Matrix tmpMat=C*P*Ct;
    Matrix K=P*Ct/(tmpMat(0,0)+R);

    // correction
    x+=K*(y-C*x);
    P=(eye(4,4)-K*C)*P;

    _x=x;
    _x[2]=1.0/x[2];
    _x[3]=x[3]/x[2];

    return _x;
}


/**********************************************************************/
OnlineStictionEstimator::OnlineStictionEstimator() :
                         RateThread(1000),   velEst(32,4.0), accEst(32,4.0),
                         trajGen(1,1.0,1.0), intErr(1.0,Vector(2,0.0))
{
    imod=NULL;
    ilim=NULL;
    ienc=NULL;
    ipid=NULL;
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
        kp=opt.check("kp",Value(10.0)).asDouble();
        ki=opt.check("ki",Value(250.0)).asDouble();
        kd=opt.check("kd",Value(15.0)).asDouble();
        vel_thres=fabs(opt.check("vel_thres",Value(5.0)).asDouble());
        e_thres=fabs(opt.check("e_thres",Value(1.0)).asDouble());

        gamma.resize(2,0.001);
        if (Bottle *pB=opt.find("gamma").asList()) 
        {
            size_t len=std::min(gamma.length(),(size_t)pB->size());
            for (size_t i=0; i<len; i++)
                gamma[i]=pB->get(i).asDouble();
        }

        theta.resize(2,0.0);
        if (Bottle *pB=opt.find("theta").asList()) 
        {
            size_t len=std::min(theta.length(),(size_t)pB->size());
            for (size_t i=0; i<len; i++)
                theta[i]=pB->get(i).asDouble();
        }

        return true;
    }
    else
        return false;
}


/**********************************************************************/
bool OnlineStictionEstimator::threadInit()
{
    ilim->getLimits(joint,&x_min,&x_max);
    double x_range=x_max-x_min;
    x_min+=0.1*x_range;
    x_max-=0.1*x_range;
    imod->setOpenLoopMode(joint);

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

    Vector Kp(1,kp),  Ki(1,ki),  Kd(1,kd);
    Vector Wp(1,1.0), Wi(1,1.0), Wd(1,1.0);
    Vector N(1,10.0), Tt(1,1.0);

    Pid pidInfo;
    ipid->getPid(joint,&pidInfo);
    Matrix satLim(1,2);
    satLim(0,0)=-pidInfo.max_int; satLim(0,1)=pidInfo.max_int;

    pid=new parallelPID(0.001*getRate(),Kp,Ki,Kd,Wp,Wi,Wd,N,Tt,satLim);
    pid->reset(Vector(1,0.0));

    intErr.setTs(0.001*getRate());
    intErr.reset(theta);

    done=0.0;
    t0=Time::now();

    return true;
}


/**********************************************************************/
void OnlineStictionEstimator::run()
{
    mutex.wait();

    ienc->getEncoder(joint,&x_pos);

    AWPolyElement el;
    el.data.resize(1,x_pos);
    el.time=Time::now();
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
    double fw=(state==rising)?theta[0]:theta[1];
    double u=fw+pid_out[0];

    Vector adaptGate(theta.length(),0.0);
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
            theta+=gamma*e_mean;
            done[state]=0.0;
        }
        else
            done[state]=1.0;

        intErr.reset(Vector(theta.length(),0.0));
    }

    ipid->setOffset(joint,u);
    adaptOld=adapt;

    mutex.post();
}


/**********************************************************************/
void OnlineStictionEstimator::threadRelease()
{
    ipid->setOffset(joint,0.0);
    imod->setPositionMode(joint);
    delete pid;
}


/**********************************************************************/
Vector OnlineStictionEstimator::getEstimation()
{
    mutex.wait();
    Vector values=theta;
    mutex.post();

    return values;
}


/**********************************************************************/
bool OnlineStictionEstimator::isDone()
{
    mutex.wait();
    bool ret=(done[0]*done[1]!=0.0);
    mutex.post();

    return ret;
}


