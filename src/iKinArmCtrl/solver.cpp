
#include "solver.h"


/************************************************************************/
iCubShoulderConstr::iCubShoulderConstr(unsigned int dofs, double lower_bound_inf,
                                       double upper_bound_inf) :
                                      iKinLinIneqConstr(lower_bound_inf,upper_bound_inf)
{
    unsigned int offs=dofs<=7 ? 0 : 3;

    double joint1_0= 10.0*(M_PI/180.0);
    double joint1_1= 15.0*(M_PI/180.0);
    double joint2_0=-33.0*(M_PI/180.0);
    double joint2_1= 60.0*(M_PI/180.0);
    double m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
    double n=joint1_0-m*joint2_0;    

    // Linear inequalities matrix
    C.resize(5,dofs); C.zero();
    // constraints on the cables length
    C(0,offs)=1.71; C(0,offs+1)=-1.71;
    C(1,offs)=1.71; C(1,offs+1)=-1.71; C(1,offs+2)=-1.71;
                    C(2,offs+1)=1.0;   C(2,offs+2)=1.0;
    // constraints to prevent arm from touching torso
                    C(3,offs+1)=1.0;   C(3,offs+2)=-m;
    // constraints to limit shoulder abduction
                    C(4,offs+1)=1.0;

    // lower and upper bounds
    lB.resize(5); uB.resize(5);
    lB[0]= -347.0*(M_PI/180.0); uB[0]=upperBoundInf;
    lB[1]=-366.57*(M_PI/180.0); uB[1]=112.42*(M_PI/180.0);
    lB[2]=  -66.6*(M_PI/180.0); uB[2]= 213.3*(M_PI/180.0);
    lB[3]=n;                    uB[3]=upperBoundInf;
    lB[4]=lowerBoundInf;        uB[4]=SHOULDER_MAXABDUCTION;
}


/************************************************************************/
Solver::Solver(PolyDriver *_drvTorso, PolyDriver *_drvArm, exchangeData *_commData,
               const string &_localName, const string &partName, unsigned int _ctrlTorso,
               unsigned int _ctrlPose, unsigned int _period) :
               RateThread(_period), drvTorso(_drvTorso),   drvArm(_drvArm),
               commData(_commData), localName(_localName), ctrlTorso(_ctrlTorso),
               ctrlPose(_ctrlPose), period(_period)
{
    Robotable=drvTorso&&drvArm;

    // Instantiate iCub arm
    arm=createArm(partName,ctrlTorso>0);

    // Get the chain object attached to the arm
    chain=arm->asChain();

    // Instantiate solver callback object
    slvCallbackObj=new slvCallback(commData);

    if (Robotable)
    {
        // create interfaces
        bool ok;
        ok =drvTorso->view(limTorso);
        ok&=drvTorso->view(encTorso);
        ok&=drvTorso->view(posTorso);
        ok&=drvArm->view(limArm);
        ok&=drvArm->view(encArm);

        if (!ok)
            cout << "Problems acquiring interfaces!" << endl;

        encTorso->getAxes(&nJointsTorso);
        encArm->getAxes(&nJointsArm);
        nJoints=ctrlTorso ? nJointsTorso+nJointsArm : nJointsArm;

        // joints bounds alignment
        alignJointsBounds(chain,limTorso,limArm);

        // arm's starting position
        fb.resize(nJoints,0.0);
        getFeedback(fb,chain,encTorso,encArm,nJointsTorso,nJointsArm,ctrlTorso>0);
        chain->setAng(fb);
    }
    else
    {
        nJointsTorso=3;
        nJointsArm  =7;
        nJoints=chain->getDOF();
        q0.resize(nJoints);
        int offs=0;
        if (ctrlTorso)
        {
            q0[0]=q0[1]=q0[2]=0.0;
            offs=3;
        }
        q0[offs]  =-26.0*(M_PI/180.0);
        q0[offs+1]=+22.0*(M_PI/180.0);
        q0[offs+2]=+43.0*(M_PI/180.0);
        q0[offs+3]=+61.0*(M_PI/180.0);
        q0[offs+4]=+00.0*(M_PI/180.0);
        q0[offs+5]=-24.0*(M_PI/180.0);
        q0[offs+6]=-00.0*(M_PI/180.0);
        chain->setAng(q0);

        fb.resize(nJoints,0.0);
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

    // Pose initialization with the current joints position.
    // Remind that the representation used is the axis/angle,
    // the default one.
    xd_old=chain->EndEffPose();

    qTorso_old.resize(nJointsTorso);
    for (int i=0; i<nJointsTorso; i++)
        qTorso_old[i]=chain->getAng(i);

    commData->setDesired(xd_old,chain->getAng());
}


/************************************************************************/
void Solver::setStart()
{
    if (Robotable)
    {
        switch (ctrlTorso)
        {
        case 1: // command pitch to zero
            posTorso->positionMove(2,0.0);

        case 2: // command roll to zero
            posTorso->positionMove(1,0.0);
        }

        if (ctrlTorso)
        {
            bool rollDone=false;

            while (!rollDone)
                posTorso->checkMotionDone(1,&rollDone);
        }

        if (ctrlTorso==1)
        {
            bool pitchDone=false;

            while (!pitchDone)
                posTorso->checkMotionDone(2,&pitchDone);
        }

        getFeedback(fb,chain,encTorso,encArm,nJointsTorso,nJointsArm,ctrlTorso>0);
        chain->setAng(fb);
    }
    else
        chain->setAng(commData->get_q());

    xd_old=chain->EndEffPose();
        
    port_xd->set_xd(xd_old);
    commData->setDesired(xd_old,chain->getAng());
}


/************************************************************************/
bool Solver::threadInit()
{
    // Instantiate the optimizer with the passed chain, the ctrlPose control
    // mode, the tolerance and a maximum number of iteration set equal to 200.
    slv=new iKinIpOptMin(*chain,ctrlPose,1e-3,200);

    // Identify the elbow xyz position to be used as 2nd task
    slv->specify2ndTaskEndEff(6);

    // Enforce shoulders joints constraints to protect cables
    double lower_bound_inf, upper_bound_inf;
    slv->getBoundsInf(lower_bound_inf,upper_bound_inf);
    shouConstr=new iCubShoulderConstr(chain->getDOF(),2.0*lower_bound_inf,2.0*upper_bound_inf);
    slv->attachLIC(*shouConstr);
    slv->getLIC().setActive(true);

    // In order to speed up the process, a scaling for the problem 
    // is usually required (a good scaling holds each element of the jacobian
    // of constraints and the hessian of lagrangian in norm between 0.1 and 10.0).
    slv->setUserScaling(true,100.0,100.0,100.0);

    port_xd=new xdPort(xd_old);
    port_xd->useCallback();
    string n1=localName+"/xd:i";
    port_xd->open(n1.c_str());

    port_qd=new BufferedPort<Vector>();
    string n2=localName+"/qd:o";
    port_qd->open(n2.c_str());

    cout << "Starting Solver at " << period << " ms" << endl;

    return true;
}


/************************************************************************/
void Solver::afterStart(bool s)
{
    if (s)
    {    
        cout << "Solver started successfully" << endl;
        setStart();
    }
    else
        cout << "Solver did not start" << endl;    
}


/************************************************************************/
void Solver::run()
{
    // get the current target pose
    Vector xd=port_xd->get_xd();
    bool movedTorso=false;

    // if torso is not controlled but it's been moved, update end-effector pose 
    if (Robotable && !ctrlTorso)
    {
        getFeedback(fb,chain,encTorso,encArm,nJointsTorso,nJointsArm,ctrlTorso>0);

        Vector qTorso(nJointsTorso);
        for (int i=0; i<nJointsTorso; i++)
            qTorso[i]=chain->getAng(i);

        movedTorso=norm(qTorso-qTorso_old)>1.0*(M_PI/180.0);
    }

    if (norm(xd-xd_old)>1e-6 || movedTorso)
    {
        Vector dummyVector(1);

        // try to keep elbow height as low as possible
        double weight2ndTask=0.01;
        Vector xdElb(3); xdElb=0.0; xdElb[2]=-1.0;
        Vector w_2nd(3); w_2nd=0.0; w_2nd[2]=1.0;

        double weight3rdTask=0.0;
        Vector qd_3rd=dummyVector;
        Vector w_3rd=dummyVector;

        // minimize the torso displacement wrt rest position (0,0,0)
        if (ctrlTorso)
        {
            weight3rdTask=0.01;

            qd_3rd.resize(chain->getDOF(),0.0);

            w_3rd=qd_3rd;
            w_3rd[0]=w_3rd[1]=w_3rd[2]=1.0;
        }

        // Call the solver and start the convergence from the current point.
        Vector _qd=slv->solve(chain->getAng(),xd,weight2ndTask,xdElb,w_2nd,
                              weight3rdTask,qd_3rd,w_3rd,
                              NULL,NULL,NULL);
                              //NULL,NULL,slvCallbackObj);        

        // _qd is an estimation of the real qd, so that x is the actual achieved pose
        Vector x=chain->EndEffPose(_qd);

        // update the exchange structure straightaway
        commData->setDesired(x,_qd);

        // send qd (always 10 DOFs) through YARP port
        Vector &qd=port_qd->prepare();

        if (ctrlTorso)
            qd=(180.0/M_PI)*_qd;
        else
        {
            qd.resize(nJointsTorso+chain->getDOF());

            for (int i=0; i<nJointsTorso; i++)
                qd[i]=(180.0/M_PI)*chain->getAng(i);

            for (unsigned int i=0; i<chain->getDOF(); i++)
                qd[nJointsTorso+i]=(180.0/M_PI)*_qd[i];
        }

        port_qd->write();

        xd_old=xd;
    }

    // latch torso joints
    for (int i=0; i<nJointsTorso; i++)
        qTorso_old[i]=chain->getAng(i);
}


/************************************************************************/
void Solver::threadRelease()
{
    port_xd->interrupt();
    port_qd->interrupt();
    port_xd->close();
    port_qd->close();

    delete port_xd;
    delete port_qd;
    delete slv;
    delete slvCallbackObj;
    delete shouConstr;
    delete arm;    
}

