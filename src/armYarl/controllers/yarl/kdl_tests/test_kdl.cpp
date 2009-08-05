#include <kdl/kinfam/serialchain.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam/serialchaincartpos2jnt.hpp>

using namespace std;
using namespace KDL;

int main(int argc,char* argv[]){

    int i, res;

    // The kinematic chain
    double L0 = 0.0;
    double L1 = 10.0;
    double L2 = 10.0;
    SerialChain* sc = new SerialChain("simple",2,0);
    sc->addJoint(new JointRotY(Frame(Rotation::Identity(),Vector(0,L0,0))));
    sc->addJoint(new JointRotY(Frame(Rotation::Identity(),Vector(0,L1,0))));
    sc->setLastJointToEE(Frame(Rotation::Identity(),Vector(0,L2,0)));

    // Joint angles
    JointVector q(sc->nrOfJoints());
    q[0] = 20*deg2rad;
    q[1] = 0*deg2rad;

    // The forward kinematics
	Jnt2CartPos* jnt2cartpos = sc->createJnt2CartPos();
	assert(jnt2cartpos!=0);
    res = jnt2cartpos->evaluate(q);
    assert(res==0);
    Frame F_base_ee;
    jnt2cartpos->getFrame(F_base_ee);
    cout << endl << "Forward kinematics:" << endl;
    cout << "End effector frame origin:" << endl;
    cout << F_base_ee.p << endl;
    cout << "End effector frame orientation:" << endl;
    cout << F_base_ee.M << endl;

    // The inverse kinematics
	CartPos2Jnt* cartpos2jnt = sc->createCartPos2Jnt();
    assert(cartpos2jnt!=0);
    cartpos2jnt->setFrame(F_base_ee);
    JointVector q_initial(sc->nrOfJoints()); // start for iterative algorithm
    q_initial[0] = 0*deg2rad;
    q_initial[1] = 0*deg2rad;
	cartpos2jnt->setConfiguration(q_initial);
    res = cartpos2jnt->evaluate(q);
    assert(res==0);
    JointVector q_solved(sc->nrOfJoints());
    cartpos2jnt->getConfiguration(q_solved);
    cout << endl << "Inverse kinematics:" << endl;
    cout << "The calculated joint vector:" << endl;
    for(i=0;i<q_solved.size();i++)
        cout << q_solved[i]*rad2deg << endl;

    // Tidy up
	delete jnt2cartpos;
	delete cartpos2jnt;
    return 1;
};

