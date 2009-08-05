#include <kdl/kinfam/serialchain.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam/serialchaincartpos2jnt.hpp>

using namespace std;
using namespace KDL;

int main(int argc,char* argv[]){

    int i, res;

    // The kinematic chain for icub
    double L0 = 0.0;
    double L1 = 10.0;
    double L2 = 10.0;
    SerialChain* icub = new SerialChain("icub",2,0);
    icub->addJoint(new JointRotY(Frame(Rotation::Identity(),Vector(0,L0,0))));
    icub->addJoint(new JointRotY(Frame(Rotation::Identity(),Vector(0,L1,0))));
    icub->setLastJointToEE(Frame(Rotation::Identity(),Vector(0,L2,0)));

    // Some joint angles for iCub
    JointVector icub_q(icub->nrOfJoints());
    icub_q[0] = 20*deg2rad;
    icub_q[1] = 0*deg2rad;

    // The forward kinematics for iCub
	Jnt2CartPos* jnt2cartpos = icub->createJnt2CartPos();
	assert(jnt2cartpos!=0);
    res = jnt2cartpos->evaluate(icub_q);
    assert(res==0);
    Frame F_base_ee;
    jnt2cartpos->getFrame(F_base_ee);
    cout << endl << "Forward kinematics for iCub:" << endl;
    cout << "End effector frame origin:" << endl;
    cout << F_base_ee.p << endl;
    cout << "End effector frame orientation:" << endl;
    cout << F_base_ee.M << endl;

    // The inverse kinematics
	CartPos2Jnt* cartpos2jnt = icub->createCartPos2Jnt();
    assert(cartpos2jnt!=0);
    cartpos2jnt->setFrame(F_base_ee);
    JointVector q_initial(icub->nrOfJoints());
    q_initial[0] = 0*deg2rad;
    q_initial[1] = 0*deg2rad;
	cartpos2jnt->setConfiguration(q_initial); // start for iterative algorithm
    JointVector q_solved(icub->nrOfJoints());
    res = cartpos2jnt->evaluate(q_solved);
    assert(res==0);
    cout << endl << "Inverse kinematics:" << endl;
    cout << "The calculated joint vector:" << endl;
    for(i=0;i<q_solved.size();i++)
        cout << q_solved[i]*rad2deg << endl;

    // Tidy up
	delete jnt2cartpos;
	delete cartpos2jnt;
    return 1;
};

