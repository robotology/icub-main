
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


int main()
{
    Vector q0,qf,qhat,xf,xhat;
    Vector dummy(1);

    // declare a right arm
    iCubArm arm("right");

    // get a chain on it; you can use arm object directly but then some
    // methods will not be available, such as the access to links through
    // [] or () operators. This prevent the user from adding/removing links
    // to iCub limbs as well as changing their properties too easily.
    iKinChain &chain=*arm.asChain();

    // get initial joints configuration
    q0=chain.getAng();

    // dump DOF bounds using () operators and set
    // a second joints configuration in the middle of the compact set.
    // Remind that angles are expressed in radians
    qf.resize(chain.getDOF());
    for (unsigned int i=0; i<chain.getDOF(); i++)
    {    
        double min=chain(i).getMin();
        double max=chain(i).getMax();
        qf[i]=(min+max)/2.0;

        // last joint set to 1° higher than the bound
        if (i==chain.getDOF()-1)
            qf[i]=max+1.0*(M_PI/180.0);

        cout << "joint " << i << " in [" << (180.0/M_PI)*min << "°," << (180.0/M_PI)*max
             << "°] set to " << (180.0/M_PI)*qf[i] << "°" << endl;
    }

    // it is not allowed to overcome the bounds...
    // ...see the result
    qf=chain.setAng(qf);
    cout << "Actual joints set to " << ((180.0/M_PI)*qf).toString() << endl;

    // there are three links for the torso which do not belong to the
    // DOF set since they are blocked. User can access them through [] operators
    cout << "Torso blocked links at:" << endl;
    for (unsigned int i=0; i<chain.getN()-chain.getDOF(); i++)
        cout << (180.0/M_PI)*chain[i].getAng() << " ";
    cout << endl;

    // user can unblock blocked links augumenting the number of DOF
    cout << "Unblocking the first torso joint... ";
    chain.releaseLink(0);
    cout << chain.getDOF() << " DOFs available" << endl;
    cout << "Blocking the first torso joint again... ";
    chain.blockLink(0);
    cout << chain.getDOF() << " DOFs available" << endl;

    // notice that also the arm object is affected
    // by modifications on chain object
    cout << "Current arm joints configuration: " << ((180.0/M_PI)*arm.getAng()).toString() << endl;

    // retrieve end-effector pose.
    // Translational part is in meters.
    // Rotational part is in axis-angle representation
    xf=chain.EndEffPose();
    cout << "Current arm end-effector pose: " << xf.toString() << endl;

    // go back to the starting joints configuration
    chain.setAng(q0);

    // instantiate a IPOPT solver for inverse kinematic
    // for both translational and rotational part
    iKinIpOptMin slv(chain,IKINCTRL_POSE_FULL,1e-3,100);

    // when the complete pose is to be achieved, we have
    // a dedicated tolerance for the translational part
    // which is by default equal to 1e-6
    // note that the tolerance is applied to the squared norm
    slv.setTranslationalTol(1e-8);

    // solve for xf starting from current configuration q0
    // 2nd and 3rd tasks are not treated
    qhat=slv.solve(chain.getAng(),xf,0.0,dummy,dummy,0.0,dummy,dummy);

    // in general the solved qf is different from the initial qf
    // due to the redundancy
    cout << "qhat: " << ((180.0/M_PI)*qhat).toString() << endl;

    // check how much we achieve our goal
    // note that the chain has been manipulated by the solver,
    // so it's already in the final configuration
    xhat=chain.EndEffPose();
    cout << "Desired arm end-effector pose       xf= " << xf.toString()   << endl;
    cout << "Achieved arm end-effector pose K(qhat)= " << xhat.toString() << endl;
    cout << "||xf-K(qhat)||=" << norm(xf-xhat) << endl;

    return 0;
}
      
      
