
#include "controller.h"


/************************************************************************/
Controller::Controller(PolyDriver *_drvTorso, PolyDriver *_drvArm, exchangeData *_commData,
                       const string &_localName, const string &partName, double _execTime,
                       unsigned int _ctrlTorso, unsigned int _ctrlPose, unsigned int _period,
                       unsigned int _dwnFactor) :
                       RateThread(_period),   drvTorso(_drvTorso),   drvArm(_drvArm),
                       commData(_commData),   localName(_localName), execTime(_execTime),
                       ctrlTorso(_ctrlTorso), ctrlPose(_ctrlPose),   period(_period),
                       dwnFactor(_dwnFactor), Ts(_period/1000.0)
{
    Robotable=drvTorso&&drvArm;

    // Instantiate iCub arm
    arm=createArm(partName,ctrlTorso>0);

    // Get the chain object attached to the arm
    chain=arm->asChain();

    if (Robotable)
    {
        // create interfaces
        bool ok;
        ok =drvTorso->view(limTorso);
        ok&=drvTorso->view(encTorso);
        ok&=drvTorso->view(velTorso);
        ok&=drvArm->view(limArm);
        ok&=drvArm->view(encArm);
        ok&=drvArm->view(velArm);

        if (!ok)
            cout << "Problems acquiring interfaces!" << endl;

        // read number of joints
        encTorso->getAxes(&nJointsTorso);
        encArm->getAxes(&nJointsArm);
        nJoints=ctrlTorso ? nJointsTorso+nJointsArm : nJointsArm;
            
        // joints bounds alignment
        alignJointsBounds(chain,limTorso,limArm);

        // exclude acceleration constraints by fixing
        // thresholds at high values
        if (ctrlTorso)
        {
            Vector a_robTorso(nJointsTorso); a_robTorso=1e9;
            velTorso->setRefAccelerations(a_robTorso.data());
        }

        Vector a_robArm(nJointsArm); a_robArm=1e9;
        velArm->setRefAccelerations(a_robArm.data());
    }
    else
    {
        nJointsTorso=3;
        nJointsArm  =7;
        nJoints=chain->getDOF();
    }

    // re-adjust torso bounds
    switch (ctrlTorso)
    {
    case 1: // only yaw enabled
        (*chain)[0].setMin(0.0);
        (*chain)[0].setMax(0.0);

    case 2: // yaw and pitch enabled
        (*chain)[1].setMin(0.0);
        (*chain)[1].setMax(0.0);
    }

    fb.resize(nJoints);   fb  =0.0;
    vOld.resize(nJoints); vOld=0.0;

    sendCnt=0;
}


/************************************************************************/
void Controller::stopLimbsVel()
{
    if (Robotable)
    {
        velArm->stop();

        if (ctrlTorso)
            velTorso->stop();
    }
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
    cout << endl;
    cout << "Controller has been resumed!" << endl;
    cout << endl;

    RateThread::resume();
}


/************************************************************************/
bool Controller::threadInit()
{
    // Instantiate controller
    ctrl=new MultiRefMinJerkCtrl(*chain,ctrlPose,chain->getAng(),Ts,Robotable);

    // Set the task execution time
    execTime=ctrl->set_execTime(execTime,true);

    port_x=new BufferedPort<Vector>();
    string n1=localName+"/x:o";
    port_x->open(n1.c_str());

    port_q=new BufferedPort<Vector>();
    string n2=localName+"/q:o";
    port_q->open(n2.c_str());

    port_v=new BufferedPort<Vector>();
    string n3=localName+"/v:o";
    port_v->open(n3.c_str());

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
    Vector xd,qd;
    
    // get the current target pose (both xd and qd are required)
    commData->getDesired(xd,qd);

    // Introduce the feedback within the control computation.
    if (Robotable)
    {
        if (!getFeedback(fb,chain,encTorso,encArm,nJointsTorso,nJointsArm,ctrlTorso>0))
        {
            cout << endl;
            cout << "Communication timeout detected!" << endl;
            cout << endl;

            suspend();

            return;
        }

        ctrl->set_q(fb);
    }

    // control the arm and dump all available information at rate of 1/100th
    Vector q=(180.0/M_PI)*ctrl->iterate(xd,qd,0x0064ffff);
    Vector v=(180.0/M_PI)*ctrl->get_qdot();

    // send velocities to the robot
    if (Robotable && !(v==vOld))
    {
        if (ctrlTorso)
        {
            // filled in reversed order
            Vector v_robTorso(nJointsTorso);
            for (int i=0; i<nJointsTorso; i++)
                v_robTorso[nJointsTorso-i-1]=v[i];

            velTorso->velocityMove(v_robTorso.data());
        }

        Vector v_robArm(nJointsArm); v_robArm=0.0;
        unsigned int i1=ctrlTorso ? nJointsTorso : 0;
        unsigned int i2=chain->getDOF();

        for (unsigned int i=i1; i<i2; i++)
            v_robArm[i-i1]=v[i];

        velArm->velocityMove(v_robArm.data());

        vOld=v;
    }

    // send x,q,qdot through YARP ports
    // at a downsampled rate to reduce
    // bandwidth usage
    if (++sendCnt>=(int)dwnFactor)
    {
        Vector &q1=port_q->prepare();
        Vector &v1=port_v->prepare();
        Vector &x =port_x->prepare();

        if (ctrlTorso)
        {
            q1=q;
            v1=v;
        }
        else
        {
            q1.resize(nJointsTorso+chain->getDOF());
            v1.resize(nJointsTorso+chain->getDOF()); v1=0.0;

            for (int i=0; i<nJointsTorso; i++)
                q1[i]=(180.0/M_PI)*chain->getAng(i);

            for (unsigned int i=0; i<chain->getDOF(); i++)
            {    
                q1[nJointsTorso+i]=q[i];
                v1[nJointsTorso+i]=v[i];
            }
        }

        x=ctrl->get_x();

        port_x->write();
        port_q->write();
        port_v->write();

        sendCnt=0;
    }

    commData->set_q(chain->getAng());    
}


/************************************************************************/
void Controller::threadRelease()
{
    stopLimbsVel();

    port_x->interrupt();
    port_q->interrupt();
    port_v->interrupt();

    port_x->close();
    port_q->close();
    port_v->close();

    delete port_x;
    delete port_q;
    delete port_v;
    delete ctrl;
    delete arm;
}



