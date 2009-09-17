
#include <gsl/gsl_math.h>
#include <iostream>
#include <iomanip>

#include <iCub/iKinFwd.h>
#include <iCub/iKinIpOpt.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;


// this inherited class (re-)implements the iCub right arm
// but shows at the same time how to handle any generic serial
// link chain
class genericRightArm : public iKinLimb
{
public:
    genericRightArm() 
    {
         _allocate_limb("don't care");
    }

protected:
    void _allocate_limb(const string &_type)
    {
        // the type is used to discriminate between left and right limb

        // remind to call the father's method
        iKinLimb::_allocate_limb(_type);

        // you have to specify the rototranslational matrix H0 from the origin
        // to the root reference so as from iCub specs.
        H0.zero();
        H0(0,1)=-1;
        H0(1,2)=-1;
        H0(2,0)=1;
        H0(3,3)=1;

        linkList.resize(10);

        // define the links in standard D-H convention
        //                                A,        D,     alpha,         offset(*),        min theta,        max theta
        linkList[0]=new iKinLink(     0.032,      0.0,  M_PI/2.0,               0.0, -22.0*M_PI/180.0,  84.0*M_PI/180.0);
        linkList[1]=new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0, -39.0*M_PI/180.0,  39.0*M_PI/180.0);
        linkList[2]=new iKinLink(-0.0233647,  -0.1433,  M_PI/2.0, -105.0*M_PI/180.0, -59.0*M_PI/180.0,  59.0*M_PI/180.0);
        linkList[3]=new iKinLink(       0.0, -0.10774,  M_PI/2.0,         -M_PI/2.0, -95.5*M_PI/180.0,   5.0*M_PI/180.0);
        linkList[4]=new iKinLink(       0.0,      0.0, -M_PI/2.0,         -M_PI/2.0,              0.0, 160.8*M_PI/180.0);
        linkList[5]=new iKinLink(       0.0, -0.15228, -M_PI/2.0, -105.0*M_PI/180.0, -37.0*M_PI/180.0,  90.0*M_PI/180.0);
        linkList[6]=new iKinLink(     0.015,      0.0,  M_PI/2.0,               0.0,   5.5*M_PI/180.0, 106.0*M_PI/180.0);
        linkList[7]=new iKinLink(       0.0,  -0.1373,  M_PI/2.0,         -M_PI/2.0, -90.0*M_PI/180.0,  90.0*M_PI/180.0);
        linkList[8]=new iKinLink(       0.0,      0.0,  M_PI/2.0,          M_PI/2.0, -90.0*M_PI/180.0,   0.0*M_PI/180.0);
        linkList[9]=new iKinLink(    0.0625,    0.016,       0.0,              M_PI, -20.0*M_PI/180.0,  40.0*M_PI/180.0);
        // (*) remind that offset is added to theta before computing the rototranslational matrix    

        // populate the serial kinematic chain
        for (unsigned int i=0; i<10; i++)
            *this << *linkList[i];

        // usually the first three links which describes the torso kinematic come
        // as blocked, i.e. they do not belong to the set of arm's dof.
        blockLink(0,0.0);
        blockLink(1,0.0);
        blockLink(2,0.0);
    }
};


int main()
{
    // some useful variables
    Vector q0,qf,qhat,xf,xhat;
    Vector dummy(1);    

    // declare the limb
    genericRightArm genArm;    

    // you can get the same result by creating an iCubArm object;
    // iKin already provides internally coded limbs for iCub, such as
    // iCubArm, iCubLeg, iCubEye, ..., along with the proper H0 matrix
    iCubArm libArm("right");

    // get a chain on the arm; you can use arm object directly but then some
    // methods will not be available, such as the access to links through
    // [] or () operators. This prevent the user from adding/removing links
    // to iCub limbs as well as changing their properties too easily.
    // Anyway, arm object is affected by modifications on the chain.
    iKinChain *chain;
    if (true)   // selector
        chain=genArm.asChain();
    else
        chain=libArm.asChain();

    // get initial joints configuration
    q0=chain->getAng();

    // dump DOF bounds using () operators and set
    // a second joints configuration in the middle of the compact set.
    // Remind that angles are expressed in radians
    qf.resize(chain->getDOF());
    for (unsigned int i=0; i<chain->getDOF(); i++)
    {    
        double min=(*chain)(i).getMin();
        double max=(*chain)(i).getMax();
        qf[i]=(min+max)/2.0;

        // last joint set to 1° higher than the bound
        if (i==chain->getDOF()-1)
            qf[i]=max+1.0*(M_PI/180.0);

        cout << "joint " << i << " in [" << (180.0/M_PI)*min << "," << (180.0/M_PI)*max
             << "] set to " << (180.0/M_PI)*qf[i] << endl;
    }

    // it is not allowed to overcome the bounds...
    // ...see the result
    qf=chain->setAng(qf);
    cout << "Actual joints set to " << ((180.0/M_PI)*qf).toString() << endl;

    // there are three links for the torso which do not belong to the
    // DOF set since they are blocked. User can access them through [] operators
    cout << "Torso blocked links at:" << endl;
    for (unsigned int i=0; i<chain->getN()-chain->getDOF(); i++)
        cout << (180.0/M_PI)*(*chain)[i].getAng() << " ";
    cout << endl;

    // user can unblock blocked links augumenting the number of DOF
    cout << "Unblocking the first torso joint... ";	
    chain->releaseLink(0);
    cout << chain->getDOF() << " DOFs available" << endl;
    cout << "Blocking the first torso joint again... ";    
    chain->blockLink(0);
    cout << chain->getDOF() << " DOFs available" << endl;

    // retrieve end-effector pose.
    // Translational part is in meters.
    // Rotational part is in axis-angle representation
    xf=chain->EndEffPose();
    cout << "Current arm end-effector pose: " << xf.toString() << endl;

    // go back to the starting joints configuration
    chain->setAng(q0);    

    // instantiate a IPOPT solver for inverse kinematic
    // for both translational and rotational part
    iKinIpOptMin slv(*chain,IKINCTRL_POSE_FULL,1e-3,100);

    // when the complete pose is to be achieved, we have
    // a dedicated tolerance for the translational part
    // which is by default equal to 1e-6
    // note that the tolerance is applied to the squared norm
    slv.setTranslationalTol(1e-8);

    // solve for xf starting from current configuration q0
    // 2nd and 3rd tasks are not treated
    qhat=slv.solve(chain->getAng(),xf,0.0,dummy,dummy,0.0,dummy,dummy);

    // in general the solved qf is different from the initial qf
    // due to the redundancy
    cout << "qhat: " << ((180.0/M_PI)*qhat).toString() << endl;

    // check how much we achieve our goal
    // note that the chain has been manipulated by the solver,
    // so it's already in the final configuration
    xhat=chain->EndEffPose();
    cout << "Desired arm end-effector pose       xf= " << xf.toString()   << endl;
    cout << "Achieved arm end-effector pose K(qhat)= " << xhat.toString() << endl;
    cout << "||xf-K(qhat)||=" << norm(xf-xhat) << endl;

    return 0;
}
      
      
