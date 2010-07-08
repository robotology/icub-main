//
// An example on finding a multi-limb Jacobian exploiting iDynNode.
// A node with torso, head, a left and right arm is created. Then the Jacobian
// and the end-effector pose are computed, for different couples of limbs.
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
// even if the arms have a FT sensor, in this example we're just using kinematics computations
// and we're not using the sensor at all, so there's no need to use a iDynSensorNode.
// we start building an empty node, then adding a right arm (index 0), a torso limb (index 1),
// a head (index 2) and a left arm (index 3).
// it's important to remember the indeces because they identify the limb during computations.
// as usual, one can create its own limbs (iDynLimb), but here we use icub arms, torso and head.

class UpTorso : public iDynNode
{
public:	
	
	iDynLimb *arm_right;
	iDynLimb *head;
	iDynLimb *torso;
	iDynLimb *arm_left;

	// construct the node
	UpTorso()
	:iDynNode("node with arm, torso and head")
	{
		//first create the limbs
		arm_right	= new iCubArmNoTorsoDyn("right",KINFWD_WREBWD);
		torso		= new iCubTorsoDyn("lower",KINBWD_WREBWD);
		head		= new iCubNeckInertialDyn(KINBWD_WREBWD);
		arm_left	= new iCubArmNoTorsoDyn("left",KINFWD_WREBWD);

		// then create their rigid-body transformations matrices
		Matrix Harm_right(4,4); Harm_right.eye();
		Matrix Htorso(4,4); Htorso.eye();
		Matrix Hhead(4,4); Hhead.eye();
		Matrix Harm_left(4,4); Harm_left.eye();

		// an example of RBT matrix
		double theta = CTRL_DEG2RAD * (180.0-15.0);
		Harm_right(0,0) = -cos(theta);	Harm_right(0,1) = 0.0;	Harm_right(0,2) = -sin(theta);	Harm_right(0,3) = 0.00294;
		Harm_right(1,0) = 0.0;			Harm_right(1,1) = -1.0;	Harm_right(1,2) = 0.0;			Harm_right(1,3) = -0.050;
		Harm_right(2,0) = -sin(theta);	Harm_right(2,1) = 0.0;	Harm_right(2,2) = cos(theta);	Harm_right(2,3) = 0.10997;
		Harm_right(3,3) = 1.0;

		// we can add the limbs just setting the roto-translational matrix
		// since the other parameters (kinematic and wrench flow, sensor flag) are not useful
		// for our example: the default values are ok
		addLimb(arm_right,Harm_right);
		addLimb(torso,Htorso);
		addLimb(head,Hhead);
		addLimb(arm_left,Harm_left);
	}

	// ridefinitions of the jacobian functions 
	// note that the jacobians starting at the head are the only ones
	// with JAC_IKIN direction in the first limb (because we start the jacobian
	// at the end-effector of the head, go throught its base to the node, and then
	// to the arm)

	Matrix Jacobian_TorsoArmRight()		{	return computeJacobian(1,JAC_KIN, 0,JAC_KIN);	}
	Matrix Jacobian_TorsoArmLeft()		{	return computeJacobian(1,JAC_KIN, 3,JAC_KIN);	}
	Matrix Jacobian_TorsoHead()			{	return computeJacobian(1,JAC_KIN, 2,JAC_KIN);	}
	Matrix Jacobian_HeadArmRight()		{	return computeJacobian(2,JAC_IKIN,0,JAC_KIN);	}
	Matrix Jacobian_HeadArmLeft()		{	return computeJacobian(2,JAC_IKIN,3,JAC_KIN);	}
	Matrix Jacobian_HeadTorso()			{	return computeJacobian(2,JAC_IKIN,1,JAC_IKIN);	}
	Matrix Jacobian_ArmLeftArmRight()	{	return computeJacobian(3,JAC_IKIN,0,JAC_KIN);	}
	Matrix Jacobian_ArmRightArmLeft()	{	return computeJacobian(0,JAC_IKIN,3,JAC_KIN);	}

	// ridefinitions of the pose function
	// again the pose is closely related to the sequence of chains, e.g. the pose starting from the 
	// head has JAC_IKIN direction in the first limb 

	Vector Pose_TorsoArmRight(bool axisRep = false) 	{	return computePose(1,JAC_KIN, 0,JAC_KIN, axisRep);	}		
	Vector Pose_TorsoArmLeft(bool axisRep = false)		{	return computePose(1,JAC_KIN, 3,JAC_KIN, axisRep);	}
	Vector Pose_HeadArmRight(bool axisRep = false)		{	return computePose(2,JAC_IKIN,0,JAC_KIN, axisRep);	}		
	Vector Pose_HeadArmLeft(bool axisRep = false)		{	return computePose(2,JAC_IKIN,3,JAC_KIN, axisRep);	}
	Vector Pose_TorsoHead(bool axisRep = false)			{	return computePose(1,JAC_KIN, 2,JAC_KIN, axisRep);	}
	Vector Pose_HeadTorso(bool axisRep = false)			{	return computePose(2,JAC_IKIN,1,JAC_IKIN,axisRep);	}
	Vector Pose_ArmLeftArmRight(bool axisRep = false)	{	return computePose(3,JAC_IKIN,0,JAC_KIN, axisRep);	}
	Vector Pose_ArmRightArmLeft(bool axisRep = false)	{	return computePose(0,JAC_IKIN,3,JAC_KIN, axisRep);	}

};

// useful print methods
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


/////////////////
//    MAIN     //
/////////////////
int main()
{
	
	// we create the node with arm and torso
	UpTorso node;
	
	// now we set the joint angles for the two limbs
	// if connected to the real robot, we can take this values from the encoders
	Vector q_rarm(node.arm_right->getN());	q_rarm.zero();
	Vector q_larm(node.arm_left->getN());	q_larm.zero();
	Vector q_torso(node.torso->getN());		q_torso.zero();
	Vector q_head(node.head->getN());		q_head.zero();
	
	// here we set the joints positions: the only needed for the jacobian
	node.arm_right->setAng(q_rarm);
	node.arm_left->setAng(q_rarm);
	node.torso->setAng(q_torso);
	node.head->setAng(q_head);

	// now we can make all the computations we want..

	Matrix J(6,1);  J.zero();
	Vector pose(6); pose.zero();
	string sc = "a";
	char c = 'a';
	bool ok=true;

	while( c!= 'q' )
	{
		cin.clear();
		cout<<endl
			<<"************************\n"
			<<"* 1 - torso to right arm \n"
			<<"* 2 - torso to left arm \n"
			<<"* 3 - torso to head \n"
			<<"* 4 - head to right arm \n"
			<<"* 5 - head to left arm \n"
			<<"* 6 - head to torso \n"
			<<"* 7 - right arm to left arm \n"
			<<"* 8 - left arm to right arm \n"
			<<"* \n"
			<<"* q - quit \n"
			<<"* \n"
			<<"* Enter your choice: ";
		cin>>sc; c = sc[0];

		switch(c)
		{
			case '1': J = node.Jacobian_TorsoArmRight();	pose = node.Pose_TorsoArmRight(); ok=true; break;
			case '2': J = node.Jacobian_TorsoArmLeft();		pose = node.Pose_TorsoArmLeft(); ok=true; break;
			case '3': J = node.Jacobian_TorsoHead();		pose = node.Pose_TorsoHead(); ok=true; break;
			case '4': J = node.Jacobian_HeadArmRight();		pose = node.Pose_HeadArmRight(); ok=true; break;
			case '5': J = node.Jacobian_HeadArmLeft();		pose = node.Pose_HeadArmLeft(); ok=true; break;
			case '6': J = node.Jacobian_HeadTorso();		pose = node.Pose_HeadTorso(); ok=true; break;
			case '7': J = node.Jacobian_ArmLeftArmRight();	pose = node.Pose_ArmLeftArmRight(); ok=true; break;
			case '8': J = node.Jacobian_ArmRightArmLeft();	pose = node.Pose_ArmRightArmLeft(); ok=true; break;	
			case 'q': cout<<"  .. quitting, bye.\n"<<endl; ok=false; break;
			default:  cout<<"  .. this is not a correct choice!!\n"<<endl; ok=false;
		}

		if(ok)
		{
			cout<<endl<<endl;
			printMatrix("\nJacobian\n",J);
			printVector("\nPose\n",pose);
			cout<<endl;
		}
	
	}

	cout<<endl<<"\n\n************************\n\n";
	cin.clear();
	cin.get();
    return 0;
}
      
      
