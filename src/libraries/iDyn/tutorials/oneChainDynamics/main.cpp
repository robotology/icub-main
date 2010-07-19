//
// The most basic example on the use of iDyn: the computation of forces and torques in a single chain.
//
// Author: Serena Ivaldi - <serena.ivaldi@iit.it>

#include <iostream>
#include <iomanip>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/iDyn/iDyn.h>

using namespace std;
using namespace yarp::sig;
using namespace iCub::iDyn;



// useful print functions
// print a matrix nicely
void printMatrix(string s, Matrix &m)
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
void printVector(string s, Vector &v)
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
	// we create a iDyn::iCubArmDyn = arm+torso
    // if you are familiar with iKin, it is exactly the same limb type, with the same kinematic properties..
    // .. but it also has dynamics ^_^
	iCubArmDyn armTorsoDyn("right");

	// by default in iCubArmDyn the torso is blocked, as it is in its corresponding iKin limb, iCubArm;
    // we unblock it, so we can also use the torso links
    // note: releasing the links we have N==DOF
	armTorsoDyn.releaseLink(0);
	armTorsoDyn.releaseLink(1);
	armTorsoDyn.releaseLink(2);

	// we prepare the necessary variables for our computations
    // 1) the joint angles (pos, vel, acc) of the limbs: we can take this values from the encoders..
    Vector q(armTorsoDyn.getN()); q.zero();
    Vector dq(q); Vector ddq(q);
	
    // 2) the inertial information to initialize the kinematic phase
    // this information must be set at the base of the chain, where the base is intended to be the first link (first.. in the Denavit
    // Hartenberg convention)
    // note: if nothing is moving, it can be simply zero, but ddp0 must provide the gravity information :)
	Vector w0(3); Vector dw0(3); Vector ddp0(3);
	w0=dw0=ddp0=0.0; ddp0[2]=9.81;
    
    // 3) the end-effector external wrench (force/moment), to initialize the wrench phase
    //    the end-effector wrench is zero by default, if we are not touching anything..
    Vector Fend(3); Fend.zero();
    Vector Muend(Fend);

    // now we set the joints values
    // note: if the joint angles exceed the joint limits, their value is saturated to the min/max automatically
    armTorsoDyn.setAng(q);
    armTorsoDyn.setDAng(dq);
    armTorsoDyn.setD2Ang(ddq);

    // we can start using iDyn :)
    // in order to perform the kinematic and wrench computation in a chain, we must first tell the limb
    // that we want to use the iDyn::RecursiveNewtonEuler library
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
    armTorsoDyn.prepareNewtonEuler(DYNAMIC);

    // then we perform the computations
    // 1) Kinematic Phase
    // 2) Wrench Phase
    // the computeNewtonEuler() methoddo everything autonomously!
    armTorsoDyn.computeNewtonEuler(w0,dw0,ddp0,Fend,Muend);
	
    // then we can retrieve our results...
    // forces moments and torques
    Matrix F = armTorsoDyn.getForces();
    Matrix Mu = armTorsoDyn.getMoments();
    Vector Tau = armTorsoDyn.getTorques();

    // now print the information

    cout<<"iCubArmDyn has "<<armTorsoDyn.getN()<<" links."<<endl;

    cout<<"\n\n";

    printVector("Joint Angles - command ", q);
    printVector("Joint Angles - real ", armTorsoDyn.getAng());

    cout<<"\n\n";

    printMatrix("Forces ",F);
    printMatrix("Moments ",Mu);
    printVector("Torques ",Tau);


    //exit
	cin.get();
    return 0;
}
      
      
