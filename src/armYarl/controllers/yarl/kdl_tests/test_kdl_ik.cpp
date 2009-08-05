#include <kdl/kinfam/serialchain.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam/serialchaincartpos2jnt.hpp>

using namespace std;
using namespace KDL;

class Simple: public SerialChain {

public:
    explicit Simple(int jointoffset=0):SerialChain("simple",2,0){
        double L1 = 10.0;
        double L2 = 10.0;
        addJoint(new JointRotY(Frame::Identity())); // Joint is at anchor
        addJoint(new JointRotY(Frame(Rotation::Identity(),Vector(0,L1,0))));
        setLastJointToEE(Frame(Rotation::Identity(),Vector(0,L2,0)));
    };
};

//
// Test whether Jnt2CartPos and CartPos2Jnt give a consistent result.
//
class TestForwardAndInverse {
	KinematicFamily*      family;
	Jnt2CartPos*          jnt2cartpos;
	Frame			      F_base_ee;
	Frame			      F_base_ee2;
	JointVector           q_solved; 
	JointVector           q_initial;
public:
	CartPos2Jnt*          cartpos2jnt;
public:
	static void TestFamily(KinematicFamily* _family){
		JointVector q_initial(2); // To be overwritten
		q_initial[0] = 0*deg2rad;
		q_initial[1] = 0*deg2rad;
		TestForwardAndInverse testobj(_family,q_initial);
		JointVector q(2);
   		q[0] = 0*deg2rad;
   		q[1] = 0*deg2rad;
		testobj.test(q);
	}

	TestForwardAndInverse(KinematicFamily* _family,const JointVector& q_i):
		family(_family),
		jnt2cartpos(_family->createJnt2CartPos()),
		cartpos2jnt(_family->createCartPos2Jnt()),
		q_solved(_family->nrOfJoints()),
		q_initial(_family->nrOfJoints())
    {
		assert(jnt2cartpos!=0);
		assert(cartpos2jnt!=0);
		copy(q_i.begin(),q_i.end(),q_initial.begin());
	}
	
	void test(JointVector& q){
		assert((int)q.size() == family->nrOfJoints());
		int result = jnt2cartpos->evaluate(q);
		assert(result==0);
		jnt2cartpos->getFrame(F_base_ee);
		cartpos2jnt->setFrame(F_base_ee);
		cartpos2jnt->setConfiguration(q_initial);
		result = cartpos2jnt->evaluate(q_solved);
        if(result!=0){
            cout << "nr of itn in cartpos2jnt " << result << endl;
            assert(result==0);
        }
		result=jnt2cartpos->evaluate(q_solved);
		assert(result==0);
		jnt2cartpos->getFrame(F_base_ee2);
        cout << "original F_base_ee:" << endl;
        cout << F_base_ee << endl;
        cout << "calculated F_base_ee2:" << endl;
        cout << F_base_ee2 << endl;
		cout << "Difference between frames: " << endl;
        cout << diff(F_base_ee,F_base_ee2) << endl;
        if(!Equal(F_base_ee,F_base_ee2,1E-6)){
            cout << "The calculated inverse solution ";
            cout << "does not result in the same end frame ";
            cout << F_base_ee << endl;
            cout << F_base_ee2 << endl;
            exit(-1);
        }else
            cout << "The matrices are equal.\n" << endl;
        cout << "Frame origin:" << F_base_ee.p << endl;
        cout << "Frame orientation:" << endl << F_base_ee.M << endl;
	}	

	~TestForwardAndInverse(){
		delete jnt2cartpos;
		delete cartpos2jnt;
	}
};

int main(int argc,char* argv[]){
    KinematicFamily* kf = new Simple();
    TestForwardAndInverse::TestFamily(kf);
};
