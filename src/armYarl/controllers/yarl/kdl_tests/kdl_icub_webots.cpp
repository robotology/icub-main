#include <kdl/kinfam/serialchain.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam/serialchaincartpos2jnt.hpp>

using namespace std;
using namespace KDL;

int main(int argc,char* argv[]){

    int i, res;

    // The kinematic chain for the webots icub model
    int joints_numof = 4;
    SerialChain* iw = new SerialChain("icub_webots",joints_numof,0);
    double s0x = -0.0912;
    double s0y = 0.655;
    double s0z = 0.01;
    Frame f0(Rotation::Identity(),Vector(s0x,s0y,s0z));
    iw->addJoint(new JointRotX(f0));
    double s1x = -0.01;
    double s1y = 0.0;
    double s1z = 0.0;
    Frame f1(Rotation::Identity(),Vector(s1x,s1y,s1z));
    iw->addJoint(new JointRotZ(f1));
    double s2x = -0.005;
    double s2y = 0.01;
    double s2z = 0.0;
    Frame f2(Rotation::Identity(),Vector(s2x,s2y,s2z));
    iw->addJoint(new JointRotY(f2));
    double s3x = 0.0;
    double s3y = -0.14;
    double s3z = 0.0;
    Frame f3(Rotation::Identity(),Vector(s3x,s3y,s3z));
    iw->addJoint(new JointRotX(f3));
    double s4x = 0.0;
    double s4y = -0.19;
    double s4z = 0.0;
    Frame f4(Rotation::Identity(),Vector(s4x,s4y,s4z));
    iw->setLastJointToEE(f4);

    // Initial joint angles for the webots iCub model
    JointVector iw_q(iw->nrOfJoints());
    iw_q[0] = 0*deg2rad;
    iw_q[1] = -6.183*deg2rad;
    iw_q[2] = 0*deg2rad;
    iw_q[3] = 0*deg2rad;

    // The forward kinematics for the webots iCub model
	Jnt2CartPos* jnt2cartpos = iw->createJnt2CartPos();
	assert(jnt2cartpos!=0);
    cout << endl << "Forward kinematics of the Webots iCub model:" << endl;
    res = jnt2cartpos->evaluate(iw_q);
    assert(res==0);
    Frame F_base_ee;
    jnt2cartpos->getFrame(F_base_ee);
    cout << "End effector frame origin joint " << iw->nrOfJoints();
    cout << ":" << endl;
    cout << F_base_ee.p << endl;
    cout << "End effector frame orientation:" << endl;
    cout << F_base_ee.M << endl;

    // The inverse kinematics
	CartPos2Jnt* cartpos2jnt = iw->createCartPos2Jnt();
    assert(cartpos2jnt!=0);
    cartpos2jnt->setFrame(F_base_ee);
    JointVector q_initial(iw->nrOfJoints()); // start for iterative algorithm
    q_initial[0] = 0*deg2rad;
    q_initial[1] = 0*deg2rad;
    q_initial[2] = 0*deg2rad;
    q_initial[3] = 0*deg2rad;
	cartpos2jnt->setConfiguration(q_initial);
    JointVector q_solved(iw->nrOfJoints());
    res = cartpos2jnt->evaluate(q_solved);
    assert(res==0);
    cout << endl << "Inverse kinematics of the Webots iCub model:" << endl;
    cout << endl << " - calculated from an initial vector of 0s" << endl;
    cout << "The calculated joint vector:" << endl;
    for(i=0;i<q_solved.size();i++)
        cout << q_solved[i]*rad2deg << endl;

    // Tidy up
	delete jnt2cartpos;
    return 1;
};

