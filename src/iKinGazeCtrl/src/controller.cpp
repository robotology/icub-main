
#include <iCub/controller.hpp>


/************************************************************************/
Controller::Controller(PolyDriver *_drvTorso, PolyDriver *_drvHead, exchangeData *_commData,
                       const string &_robotName, const string &_localName, double _neckTime,
                       double _eyesTime, unsigned int _period) :
                       RateThread(_period),   drvTorso(_drvTorso),   drvHead(_drvHead),
                       commData(_commData),   robotName(_robotName), localName(_localName),
                       neckTime(_neckTime),   eyesTime(_eyesTime),   period(_period),
                       Ts(_period/1000.0),    printAccTime(0.0)
{
    Robotable=drvTorso&&drvHead;

    // Instantiate eye object for getting limits
    eyeLim=new iCubEye();

    // Get the chain object attached to the arm
    chainEyeLim=eyeLim->asChain();

    Matrix lim;

    if (Robotable)
    {
        // create interfaces
        bool ok;
        ok =drvTorso->view(limTorso);
        ok&=drvTorso->view(encTorso);
        ok&=drvHead->view(limHead);
        ok&=drvHead->view(encHead);
        ok&=drvHead->view(velHead);

        if (!ok)
            cout << "Problems acquiring interfaces!" << endl;

        // read number of joints
        encTorso->getAxes(&nJointsTorso);
        encHead->getAxes(&nJointsHead);

        // joints bounds alignment
        lim=alignJointsBounds(chainEyeLim,limTorso,limHead);

        // reinforce vergence min bound
        lim(nJointsHead-1,0)=0.0;

        // read starting position
        fbTorso.resize(nJointsTorso);
        fbHead.resize(nJointsHead);
        getFeedback(fbTorso,fbHead,encTorso,encHead);

        // exclude acceleration constraints by fixing
        // thresholds at high values
        Vector a_robHead(nJointsHead); a_robHead=1e9;
        velHead->setRefAccelerations(a_robHead.data());
    }
    else
    {
        nJointsTorso=3;
        nJointsHead =6;
        
        fbTorso.resize(nJointsTorso,0.0);
        fbHead.resize(nJointsHead,0.0);

        // create bounds matrix for integrators
        lim.resize(nJointsHead,2);
        for (int i=0; i<nJointsHead-1; i++)
        {
            lim(i,0)=(*chainEyeLim)[nJointsTorso+i].getMin();
            lim(i,1)=(*chainEyeLim)[nJointsTorso+i].getMax();
        }

        // vergence
        lim(nJointsHead-1,0)=0.0;
        lim(nJointsHead-1,1)=lim(nJointsHead-2,1);
    }

    fbNeck.resize(3); fbEyes.resize(3);
    qdNeck.resize(3); qdEyes.resize(3);
    vNeck.resize(3);  vEyes.resize(3);

    // Set the task execution time
    setTeyes(eyesTime);
    setTneck(neckTime);

    mjCtrlNeck=new minJerkVelCtrl(Ts,fbNeck.length());
    mjCtrlEyes=new minJerkVelCtrl(Ts,fbEyes.length());
    Int=new Integrator(Ts,fbHead,lim);

    v.resize(nJointsHead,0.0);
    vdegOld=v;
}


/************************************************************************/
void Controller::stopLimbsVel()
{
    if (Robotable)
    {
        // this timeout prevents the stop() from
        // being overwritten by the last velocityMove()
        // which travels on a different connection.
        Time::delay(2*Ts);

        velHead->stop();
    }
}


/************************************************************************/
void Controller::printIter(Vector &xd, Vector &fp, Vector &qd, Vector &q,
                           Vector &v, double printTime)
{
    if ((printAccTime+=Ts)>=printTime)
    {
        printAccTime=0.0;

        cout << endl;
        cout << "norm(e)           = " << norm(xd-fp)   << endl;
        cout << "Target fix. point = " << xd.toString() << endl;
        cout << "Actual fix. point = " << fp.toString() << endl;
        cout << "Target Joints     = " << qd.toString() << endl;
        cout << "Actual Joints     = " << q.toString()  << endl;
        cout << "Velocity          = " << v.toString()  << endl;
        cout << endl;
    }
}


/************************************************************************/
bool Controller::threadInit()
{
    port_x=new BufferedPort<Vector>;
    string n1=localName+"/x:o";
    port_x->open(n1.c_str());

    port_qd=new BufferedPort<Vector>;
    string n2=localName+"/qd:o";
    port_qd->open(n2.c_str());

    port_q=new BufferedPort<Vector>;
    string n3=localName+"/q:o";
    port_q->open(n3.c_str());

    port_v=new BufferedPort<Vector>;
    string n4=localName+"/v:o";
    port_v->open(n4.c_str());

    cout << "Starting Controller at " << period << " ms" << endl;

    return true;
}


/************************************************************************/
void Controller::afterStart(bool s)
{
    if (s)
        cout << "Controller started successfully" << endl;
    else
        cout << "Controller did not start" << endl;
}


/************************************************************************/
void Controller::run()
{
    // get data
    xd=commData->get_xd();
    qd=commData->get_qd();
    fp=commData->get_x();

    // Introduce the feedback within the control computation
    if (Robotable)
    {
        if (!getFeedback(fbTorso,fbHead,encTorso,encHead))
        {
            cout << endl;
            cout << "Communication timeout detected!" << endl;
            cout << endl;

            suspend();

            return;
        }

        Int->reset(fbHead);
    }
    
    for (unsigned int i=0; i<3; i++)
    {
        qdNeck[i]=qd[i];
        qdEyes[i]=qd[3+i];

        fbNeck[i]=fbHead[i];
        fbEyes[i]=fbHead[3+i];
    }

    // control loop
    vNeck=mjCtrlNeck->computeCmd(neckTime,qdNeck-fbNeck);
    vEyes=mjCtrlEyes->computeCmd(eyesTime,qdEyes-fbEyes)-commData->get_compv();

    for (unsigned int i=0; i<3; i++)
    {
        v[i]  =vNeck[i];
        v[3+i]=vEyes[i];
    }

    // convert to degrees
    qddeg=CTRL_RAD2DEG*qd;
    qdeg =CTRL_RAD2DEG*fbHead;
    vdeg =CTRL_RAD2DEG*v;

    // send velocities to the robot
    if (Robotable && !(vdeg==vdegOld))
    {    
        velHead->velocityMove(vdeg.data());
        vdegOld=vdeg;
    }

    // print info
    printIter(xd,fp,qddeg,qdeg,vdeg,1.0);

    // send qd,x,q,v through YARP ports
    Vector &x  =port_x->prepare();
    Vector &qd1=port_qd->prepare();
    Vector &q1 =port_q->prepare();
    Vector &v1 =port_v->prepare();

    qd1.resize(nJointsTorso+nJointsHead);
    q1.resize(nJointsTorso+nJointsHead);

    int j;
    for (j=0; j<nJointsTorso; j++)
        qd1[j]=q1[j]=CTRL_RAD2DEG*fbTorso[j];
    for (; j<nJointsTorso+nJointsHead; j++)
    {
        qd1[j]=qddeg[j-nJointsTorso];
        q1[j] =qdeg[j-nJointsTorso];
    }

    x=fp;
    v1=vdeg;

    port_x->write();
    port_qd->write();
    port_q->write();
    port_v->write();

    // update joints angles
    fbHead=Int->integrate(v);
    commData->get_q()=fbHead;
    commData->get_torso()=fbTorso;
    commData->get_v()=v;
}


/************************************************************************/
void Controller::threadRelease()
{
    stopLimbsVel();

    port_x->interrupt();
    port_qd->interrupt();
    port_q->interrupt();
    port_v->interrupt();

    port_x->close();
    port_qd->close();
    port_q->close();
    port_v->close();

    delete port_x;
    delete port_qd;
    delete port_q;
    delete port_v;
    delete eyeLim;
    delete mjCtrlNeck;
    delete mjCtrlEyes;
    delete Int;
}


/************************************************************************/
void Controller::suspend()
{
    stopLimbsVel();

    cout << endl;
    cout << "Controller has been suspended!" << endl;
    cout << endl;

    RateThread::suspend();
}


/************************************************************************/
void Controller::resume()
{
    if (Robotable)
    {
        getFeedback(fbTorso,fbHead,encTorso,encHead);
    
        for (unsigned int i=0; i<3; i++)
        {
            fbNeck[i]=fbHead[i];
            fbEyes[i]=fbHead[3+i];
        }
    }

    cout << endl;
    cout << "Controller has been resumed!" << endl;
    cout << endl;

    RateThread::resume();
}


/************************************************************************/
double Controller::getTneck()
{
    return neckTime;
}


/************************************************************************/
double Controller::getTeyes()
{
    return eyesTime;
}


/************************************************************************/
void Controller::setTneck(const double execTime)
{
    double lowerThresNeck=eyesTime+0.2;
    if (execTime<lowerThresNeck)
    {        
        cout << "Warning: neck execution time is under the lower bound!"           << endl;
        cout << "A new neck execution time of " << lowerThresNeck << "s is chosen" << endl;        

        neckTime=lowerThresNeck;
    }
    else
        neckTime=execTime;
}


/************************************************************************/
void Controller::setTeyes(const double execTime)
{
    double lowerThresEyes=10.0*Ts;
    if (execTime<lowerThresEyes)
    {        
        cout << "Warning: eyes execution time is under the lower bound!"           << endl;
        cout << "A new eyes execution time of " << lowerThresEyes << "s is chosen" << endl;        

        eyesTime=lowerThresEyes;
    }
    else
        eyesTime=execTime;
}


