//
// An example on finding a multi-limb Jacobian exploiting iDynNode.
// A node with torso and right arm is created. Then the Jacobian
// "torso-arm" is computed, and confronted with the one computed by
// iKin and iDyn in the arm chain (the one with torso and arm, with the
// torso links unblocked). The end-effector pose is also computed.
//
// Author: Serena Ivaldi - <serena.ivaldi@iit.it>

#include <iostream>
#include <iomanip>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace iKin;
using namespace iDyn;


// the class we'll use for this example is inherited from iDynNode.
// even if the arm has a FT sensor, in this example we're just using kinematics computations
// and we're not using the sensor at all, so there's no need to use a iDynSensorNode.
// we start building an empty node, then adding a right arm (index 0) and a torso limb (index 1)
// one can create its own limb (iDynLimb), here we use an icub arm and torso
class ArmWithTorso : public iDynNode
{
public:	
	
	iDynLimb *arm;
	iDynLimb *torso;

	// construct a node with right arm and torso
	ArmWithTorso()
	:iDynNode("node with arm & torso")
	{
		//first create the two limbs
		arm		= new iCubArmNoTorsoDyn("right",KINFWD_WREBWD);
		torso	= new iCubTorsoDyn("lower",KINBWD_WREBWD);

		// then create their rigid-body transformations matrices
		Matrix Harm(4,4); Harm.eye();
		Matrix Htorso(4,4); Htorso.eye();

		double theta = CTRL_DEG2RAD * (180.0-15.0);
		Harm(0,0) = -cos(theta);	Harm(0,1) = 0.0;	Harm(0,2) = -sin(theta);	Harm(0,3) = 0.00294;
		Harm(1,0) = 0.0;			Harm(1,1) = -1.0;	Harm(1,2) = 0.0;			Harm(1,3) = -0.050;
		Harm(2,0) = -sin(theta);	Harm(2,1) = 0.0;	Harm(2,2) = cos(theta);		Harm(2,3) = 0.10997;
		Harm(3,3) = 1.0;

		// we can add the limbs just setting the roto-translational matrix
		// since the other parameters (kinematic and wrench flow, sensor flag) are not useful
		// for our example: the default values are ok
		addLimb(arm,Harm);
		addLimb(torso,Htorso);
	}

	//ridefinition of the jacobian function
	Matrix computeJacobianTorsoArm()
	{
		return computeJacobian(1,JAC_KIN,0,JAC_KIN);
	}

	//ridefinition of the pose function
	Vector computeArmPose(bool axisRep)
	{
		return computePose(1,JAC_KIN,0,JAC_KIN,axisRep);
	}

};

// useful print functions
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
	int i;

	// we create the node with arm and torso
	ArmWithTorso node;
	
	// we also create a iKin::iCubArm = arm+torso
	// we'll use that to compare the two jacobians
	iCubArm armTorso("right");
	// by default in iKin the torso is blocked: so we unblock it
	armTorso.releaseLink(0);
	armTorso.releaseLink(1);
	armTorso.releaseLink(2);

	// we also create a iDyn::iCubArmDyn = arm+torso
	// we'll use that to compare the jacobians
	iCubArmDyn armTorsoDyn("right");
	// by default in iDyn the torso is blocked: so we unblock it
	// note: this choice has been made to be coherent with iKin!
	armTorsoDyn.releaseLink(0);
	armTorsoDyn.releaseLink(1);
	armTorsoDyn.releaseLink(2);

	// now we set the joint angles for the two limbs
	// if connected to the real robot, we can take this values from the encoders
	Vector q_rarm(node.arm->getN());		q_rarm.zero();
	Vector q_torso(node.torso->getN());		q_torso.zero();
	Vector q_all(q_torso.size()+q_rarm.size());
	
	for(i=0;i<q_torso.size();i++)	q_all.push_back(q_torso[i]);
	for(i=0;i<q_rarm.size();i++)	q_all.push_back(q_rarm[i]);
	
	// here we set the joints positions: the only needed for the jacobian
	node.arm->setAng(q_rarm);
	node.torso->setAng(q_torso);
	armTorso.setAng(q_all);
	armTorsoDyn.setAng(q_all);

	// now we can compute the jacobian and pose vector

	// JAC torso -> arm

	Matrix Jnode = node.computeJacobianTorsoArm();
	Matrix Jikin = armTorso.GeoJacobian();
	Matrix Jidyn = armTorsoDyn.GeoJacobian();

	cout<<endl<<"\n\n************************\n\n";
	printMatrix("Jacobian torso - hand made with node \n",	Jnode);
	printMatrix("\nJacobian torso - hand made with iKin \n",Jikin);
	printMatrix("\nJacobian torso - hand made with iDyn \n",Jidyn);

	// pose torso -> arm	

	Vector pose_node_arm = node.computeArmPose(false);
	Vector pose_ikin_arm = armTorso.EndEffPose(false);
	Vector pose_idyn_arm = armTorsoDyn.EndEffPose(false);

	cout<<endl<<"\n\n************************\n\n";
	printVector("Pose arm in node \n", pose_node_arm);
	printVector("\nPose arm in ikin \n", pose_ikin_arm);
	printVector("\nPose arm in idyn \n", pose_idyn_arm);


	cin.get();
    return 0;
}
      
      
