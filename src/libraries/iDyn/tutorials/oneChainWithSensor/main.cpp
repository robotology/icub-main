/** 
* Copyright: 2010-2011 RobotCub Consortium
* Author: Serena Ivaldi
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
**/ 

//
// The most basic example on the use of iDyn when the chain is provided with
// a force/torque sensor, and the sensor is placed inside the chain. 
//
// Author: Serena Ivaldi - <serena.ivaldi@iit.it>

#include <iostream>
#include <iomanip>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynInv.h>

using namespace std;
using namespace yarp::sig;
using namespace iCub::iDyn;



// useful print functions
// print a matrix nicely
void printMatrix(string s, const Matrix &m)
{
	cout<<s<<endl;
	for(int i=0;i<m.rows();i++)
	{
		for(int j=0;j<m.cols();j++)
			cout<<setw(15)<<m(i,j);
		cout<<endl;
	}
}
// print a vector nicely
void printVector(string s, const Vector &v)
{
	cout<<s<<endl;
	for(int j=0;j<v.length();j++)
		cout<<setw(15)<<v(j);
	cout<<endl;
}


////////////////
//   MAIN
///////////////

int main()
{
	// we create a iDyn::iCubArmNoTorsoDyn = arm only
    // if you are familiar with iKin's iCubArm, beware this one is different!
    // iDyn::iCubArmDyn is exactly like iKin::iCubArm, with the same kinematic properties and so on.
    // iDyn::iCubArmNoTorsoDyn, as the name says, does not have the 3 torso links: so here N==DOF.
	iCubArmNoTorsoDyn *arm = new iCubArmNoTorsoDyn("right");
    
    // we create a iDyn::iDynSensorArmNoTorso for the right arm, with force/moment computation in the static case
    // (STATIC) and verbosity flag on (VERBOSE).
    // if you want dynamic computations you can substitute STATIC with DYNAMIC.
    // if you want to disable verbosity, just put NO_VERBOSE.
    // note that there is no need to specify the sensor properties for the icub arm, since the sensor
    // type is automatically set after getting the arm information
    iDynSensorArmNoTorso *armWSensorSolver = new iDynSensorArmNoTorso(arm,STATIC,VERBOSE);

    // the arm must be "prepared" for Newton-Euler computation, i.e. for computing the dynamics...
    // since we already chose STATIC computations, we tell the arm to getting ready..
    // in order to perform the kinematic and wrench computation in a chain, we must first tell the limb
    // that we want to use the iDyn::RecursiveNewtonEuler library.
    // the reason is twofold:
    // 1) one can use a iDynLimb without dynamics, since it includes the kinematics of iKin
    //    for example one may only want to set joint angles and compute the jacobian
    // 2) one may prefer to use its own dynamic library to compute internal forces/torques, and not
    //    necessarily the Newton-Euler one we provide. to use our method, a new chain, parallel to the 
    //    iDynChain, is built: it is a OneChainNewtonEuler, and it is made of OneLinkNewtonEuler virtual
    //    links, each having its "real" corresponding iDynLink: in addition, a BaseLink and a FinalLink
    //    elements are added to the chain, defining the entry points for the kinematic and wrench phases
    //    of Newton-Euler computations. 
    // the method to call is prepareNewtonEuler(), and we must specify the type of computations we want
    // to perform, which are basically:
    // - STATIC: we only need q (joints positions)
    // - DYNAMIC: we need q,dq,ddq (joints position, velocity, acceleration) --> default
    arm->prepareNewtonEuler(STATIC);

    // now the chain must be updated with all the necessary informations!!!!

    // 1) the joint angles (pos, vel, acc) of the limbs: we can take this values from the encoders..
    Vector q(arm->getN()); q.zero();
    Vector dq(q); Vector ddq(q);
    // now we set the joints values
    // note: if the joint angles exceed the joint limits, their value is saturated to the min/max automatically
    // note: STATIC computations only use q, while DYNAMIC computations use q,dq,ddq.. but we set everything
    arm->setAng(q);
    arm->setDAng(dq);
    arm->setD2Ang(ddq);

    // 2) the inertial information to initialize the kinematic phase
    // this information must be set at the base of the chain, where the base is intended to be the first link (first.. in the Denavit
    // Hartenberg convention)
    // note: if nothing is moving, it can be simply zero, but ddp0 must provide the gravity information :)
	Vector w0(3); Vector dw0(3); Vector ddp0(3);
	w0=dw0=ddp0=0.0; ddp0[2]=9.81;
    // here we set the kinematic information on the arm chain
    arm->initKinematicNewtonEuler(w0,dw0,ddp0);

    // 3) the wrench (force/moment) measured by the FT sensor placed inside the chain
    Vector Fsens(3); Fsens.zero(); 
    Fsens=2.0; // or whatever number you like... here its is 2 (Newton) per force component..
    Vector Musens(3); Musens.zero();
    Musens=0.1; //or whatever number you like... here its is 0.1 (Newton per meter) per moment component..

    // finally we set the measurements on the sensor, and we start the computation...
    // this method autonomously perform a kinematic phase (to this we set the kinematics information before..)
    // then the wrench phase is "split": from the sensor back to the base, the computations are performed in the
    // backward sense, while from sensor to the end-effector in the forward sense.
    armWSensorSolver->computeFromSensorNewtonEuler(Fsens,Musens);
   
    // then we can retrieve our results...
    // forces moments and torques
    Matrix F = arm->getForces();
    Matrix Mu = arm->getMoments();
    Vector Tau = arm->getTorques();

    // now print the information

    cout<<"iCubArmNoTorsoDyn has "<<arm->getN()<<" links."<<endl;

    cout<<"\n\n";

    printVector("Joint Angles - command ", q);
    printVector("Joint Angles - real ", arm->getAng());

    cout<<"\n\n";

    printVector("Measured Force in the FT sensor ",Fsens);
    printVector("Measured Moment in the FT sensor ",Musens);

    cout<<"\n\n";

    printMatrix("Forces ",F);
    printMatrix("Moments ",Mu);
    printVector("Torques ",Tau);


    //exit
	cin.get();

    // remember to delete everything before exiting :)
    cout<<"\nDestroying sensor..."<<endl;
    delete armWSensorSolver; armWSensorSolver=NULL;
    cout<<"Destroying arm..."<<endl;
    delete arm; arm=NULL;

    return 0;
}
      
      
