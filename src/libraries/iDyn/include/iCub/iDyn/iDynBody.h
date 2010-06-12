/**
 * \defgroup iDynBody iDynBody
 *    
 * @ingroup iDyn
 *  
 * Classes for connecting multiple limbs and solve whole body dynamics.
 *
 * \note <b>SI units adopted</b>: meters for lengths and radians
 *       for angles.
 *
 * \section dep_sec Dependencies 
 * - ctrlLib 
 * - iKin
 * - IPOPT: see the <a
 *   href="http://eris.liralab.it/wiki/Installing_IPOPT">wiki</a>.
 * 
 * \section intro_sec Description
 * 
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows
 *
 * \section example_sec Example
 *
 * TO DO
 *
 * \note <b>Release status</b>:  this library is currently under development!
 * Date: first draft 06/2010
 *
 * \author Serena Ivaldi, Matteo Fumagalli
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * 
 **/ 

#ifndef __IDYNBODY_H__
#define __IDYNBODY_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/ctrlMath.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iDyn/iDynInv.h>
#include <deque>
#include <string>


namespace iDyn
{

	class OneLinkNewtonEuler;
	class BaseLinkNewtonEuler;
	class FinalLinkNewtonEuler;
	class SensorLinkNewtonEuler;
	class OneChainNewtonEuler;
	class OneChainSensorNewtonEuler;
	class iDynSensor;
	class iFTransform;
    class iSFrame;
	class iFB;
	class iDynSensorLeg;
	class iDynSensorArm;



// Interaction type
// used to define what side of the chain (limb) is attached to the node
// the base or the end-effector
// note: head, arm and legs are attached with the base
// torso is attached with the end-effector to the upper arm, and with the 
enum InteractionType { RBT_BASE, RBT_ENDEFF };

// Flow Type
// define the flow of kinematic/wrench information for each RBT in a node
// RBT_NODE_IN = the variable is set from outside, "propagates" into the limb and then goes into the node
// RBT_NODE_OUT = the variable is read from the node and goes into the limb
enum FlowType{ RBT_NODE_IN, RBT_NODE_OUT };


/**
* \ingroup iDynBody
*
* A class for setting a rigid body transformation between iDynLimb and iDynNode. This
* class is used by iDynNode to connect two or mutiple limbs and exchanging kinematic and 
* wrench information between limbs.
*/
class RigidBodyTransformation
{
	friend class iDynNode;
	friend class iDynLimb;

protected:

	/// the limb attached to the RigidBodyTransformation
	iDyn::iDynLimb *limb;

	/// kinematic flow: in-to/out-from node (RBT_NODE_IN/RBT_NODE_OUT)
	FlowType kinFlow;

	/// wrench flow: in-to/out-from node (RBT_NODE_IN/RBT_NODE_OUT)
	FlowType wreFlow;

	/// the roto-translation between the limb and the node
	yarp::sig::Matrix H;

	/// STATIC/DYNAMIC/DYNAMIC_W_ROTOR/DYNAMIC_CORIOLIS_GRAVITY
	NewEulMode mode;	
	
	///info or useful notes
	std::string	info;
	
	///verbosity flag
	unsigned int verbose;

	// these variables are not redundant: because multiple RBT can be attached 
	// to a node, and different policies of information sharing can exist

	/// angular velocity
	yarp::sig::Vector w;	
	/// angular acceleration
	yarp::sig::Vector dw;
	/// linear acceleration
	yarp::sig::Vector ddp;	
	/// force
	yarp::sig::Vector F;	
	/// moment
	yarp::sig::Vector Mu;	
		
	
	// useful methods

	/**
	* @return the rotational matrix of the RBT
	*/
	yarp::sig::Matrix	getR();	
	
	/**
	* @param proj=true/false
	* @return the distance vector of the RBT
	*/
	yarp::sig::Vector	getr(bool proj=false);


	void computeKinematic();

	void computeWrench();

public:

	/**
	* Constructor, defining the limb attached to the node.
	* @param _limb pointer to a iDynLimb
	* @param _H a (4x4) roto-translational matrix defining the rigid body transformation
	* @param kin the kinematic flow
	* @param wre the wrench flow
	* @param verb verbosity flag
	*/
	RigidBodyTransformation(iDyn::iDynLimb *_limb, const yarp::sig::Matrix &_H, const std::string &_info, const FlowType kin=RBT_NODE_OUT, const FlowType wre=RBT_NODE_IN, const NewEulMode _mode=STATIC, unsigned int verb=VERBOSE);

	/**
	* Destructor
	*/
	~RigidBodyTransformation();

	/**
	* Set the roto-translational matrix between the limb and the node,
	* defining the rigid body transformation
	* @param _H a (4x4) roto-translational matrix
	* @return true if succeeds, false otherwise
	*/
	bool setRBT(const yarp::sig::Matrix &_H);

	/**
	* Set the kinematic variables (w,dw,ddp) of the limb. This method calls initKinematicNewtonEuler()
	* in the limb, and the limb itself knows whether to set this information in the base or in the end-
	* effector, depending on its iteration mode for kinematics. This method is used by iDynNode to connect
	* two or multiple limbs.
	* @param w0	the angular velocity
	* @param dw0 the angular acceleration
	* @param ddp0 the linear acceleration
	* @return true if succeeds, false otherwise
	*/
	bool setKinematic(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0); 

	/**
	* Set the kinematic variables (w,dw,ddp) of the limb, coming from external measurements (ie a sensor). 
	* This method calls initKinematicNewtonEuler()
	* in the limb, and the limb itself knows whether to set this information in the base or in the end-
	* effector, depending on its iteration mode for kinematics. This method is used by iDynNode to connect
	* two or multiple limbs.
	* @param w0	the angular velocity
	* @param dw0 the angular acceleration
	* @param ddp0 the linear acceleration
	* @return true if succeeds, false otherwise
	*/
	bool setKinematicMeasure(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0); 

	/**
	* Set the wrench variables (F,Mu) of the limb. This method calls initWrenchNewtonEuler() in the limb,
	* and the limb itself knows whether to set this information in the base ot in the end-effector, 
	* depending on its iteration mode for wrenches. This method is used by iDynNode to connect two or 
	* multiple limbs. 
	* @param F0	the force
	* @param Mu0 the moment
	* @return true if succeeds, false otherwise
	*/
	bool setWrench(const yarp::sig::Vector &F0, const yarp::sig::Vector &Mu0); 

	/**
	* Set the wrench variables (F,Mu) of the limb. This method calls initWrenchNewtonEuler() in the limb,
	* and the limb itself knows whether to set this information in the base ot in the end-effector, 
	* depending on its iteration mode for wrenches. This method is used by iDynNode to connect two or 
	* multiple limbs. 
	* @param F0	the force
	* @param Mu0 the moment
	* @return true if succeeds, false otherwise
	*/
	bool setWrenchMeasure(const yarp::sig::Vector &F0, const yarp::sig::Vector &Mu0); 
	
	/**
	* @return H, the (4x4) roto-translational matrix defining the rigid body transformation
	*/
	yarp::sig::Matrix getRBT() const;
	
	/**
	* Get the kinematic variables (w,dw,ddp) of the limb, applies the RBT transformation and compute
	* the kinematic variables of the node. The variables are stored into the param vectors,
	* which are accessible. This method calls getKinematicNewtonEuler()
	* in the limb, and the limb itself knows whether to get this information in the base or in the end-
	* effector, depending on its iteration mode for kinematics. This method is used by iDynNode to connect
	* two or multiple limbs.
	* @param wNode	the angular velocity of the Node
	* @param dwNode the angular acceleration of the Node
	* @param ddpNode the linear acceleration of the Node
	*/
	void getKinematic( yarp::sig::Vector &wNode, yarp::sig::Vector &dwNode, yarp::sig::Vector &ddpNode); 
	
	/**
	* Get the wrench variables (F,Mu) of the limb, transform it according to the RBT and
	* add it to the node wrench. The variables are stored into the param vectors,
	* which are accessible. This method calls getWrenchNewtonEuler() in the limb,
	* and the limb itself knows whether to get this information in the base ot in the end-effector, 
	* depending on its iteration mode for wrenches. This method is used by iDynNode to connect two or 
	* multiple limbs. 
	* @param FNode	the iDynNode force
	* @param MuNode the iDynNode moment
	*/
	void getWrench( yarp::sig::Vector &FNode,  yarp::sig::Vector &MuNode); 

	/**
	* Set the flow of kinematic/wrench information: input to node or output from node.
	* @param kin the kinematic flow
	* @param wre the wrench flow
	*/
	void setInfoFlow(const FlowType kin, const FlowType wre);

	/**
	* @return the kinematic flow
	*/
	FlowType getKinematicFlow() const;
	
	/**
	* @return the wrench flow
	*/
	FlowType getWrenchFlow() const;


	
	//other
	std::string toString() const;



};


/**
* \ingroup iDynBody
*
* A class for connecting two or mutiple limbs and exchanging kinematic and 
* wrench information between limbs.
* A virtual node, connecting multiple limbs, is set.
* The limbs can exchange kinematics and wrench information with the node 
* through a RigidBodyTransfromation. The node only has kinematic (w,dw,ddp) and wrench (F,Mu) information: no
* mass, lenght, inertia, COM, or else. 
* Each limb is connected to the node by a roto-translation matrix, which must be set when a limb is 
* attached to the node: a RigidBodyTransfromation object is then created, which allows the proper 
* computation of wrench and kinematic variables.
* When multiple limbs are attached to a node, the kinematic variables are set by a single limb, having
* kinematic flow = RBT_NODE_IN, while the wrench variables are found as the sum of the wrench 
* contribution of all the links (inbound and outbound wrenches must balance in the node).
*/

class iDynNode
{
protected:

	/// the list of RBT
	std::deque<RigidBodyTransformation> rbtList;

	/// STATIC/DYNAMIC/DYNAMIC_W_ROTOR/DYNAMIC_CORIOLIS_GRAVITY
	NewEulMode mode;	
	
	///info or useful notes
	std::string	info;
	
	///verbosity flag
	unsigned int verbose;

	/// angular velocity
	yarp::sig::Vector w;	
	/// angular acceleration
	yarp::sig::Vector dw;
	/// linear acceleration
	yarp::sig::Vector ddp;	
	/// force
	yarp::sig::Vector F;	
	/// moment
	yarp::sig::Vector Mu;	

	/**
	* Reset all data to zero. The list of limbs is not modified or deleted.
	*/
	void zero();

public:

	/**
	*	Default constructor
	*/
	iDynNode(const NewEulMode _mode=STATIC);

	/**
	*	Constructor
	*/
	iDynNode(const std::string &_info, const NewEulMode _mode=STATIC, unsigned int verb=VERBOSE);

	/**
	* Add one limb to the node, defining its RigidBodyTransformation. A new RigidBodyTransformation
	* is added to the RBT list.
	* @param limb pointer to generic limb
	* @param H a (4x4) roto-translational matrix defining the transformation between node and limb base/end
	* @param kinFlow the type of information flow of kinematics variables
	* @param wreFlow the type of information flow of wrench variables
	*/
	void addLimb(iDyn::iDynLimb *limb, const yarp::sig::Matrix &H, const FlowType kinFlow=RBT_NODE_OUT, const FlowType wreFlow=RBT_NODE_IN);

	/**
	* Main function to manage the exchange of kinematic information among the limbs attached to the node.
	* One single limb with kinematic flow of input type must exist: this limb is initilized with the kinematic variables
	* w0,dw0,ddp0 (eg the head receives this information from the inertia sensor). The limb itself knows where to init the 
	* chain (base/end) depending on how it is attached to the node. Then the first limb kinematics is solved.
	* The kinematic variables are retrieved from the RBT, which applies its roto-translation. Then the kinematic
	* variables are sent to the other limbs, having kinematic flow of output type: the RBT transformation is applied from node
	* to limb.
	* @param w0 the initial angular velocity
	* @param dw0 the initial angular acceleration
	* @param ddp0 the initial linear acceleration
	*/
	virtual bool solveKinematics( const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0);

	/**
	* Main function to manage the exchange of wrench information among the limbs attached to the node.
	* Multiple limbs with wrench flow of input type can exist, but at least one limb with output type must exist, to 
	* compute the wrench balance on the node.
	* The measured/input wrenches to the limbs are here assumed to be set 
	* elsewhere: eg another class or the main is setting the measured wrenches.
	* Note: RBT calls computeWrenchNewtonEuler in the limb, meaning that
	* performs a "basic" wrench computation without any sensor, just
	* setting wrenches at the end-effector or at the base, and calling
	* recursive wrench computation.
	*/
	virtual bool solveWrench();

	/**
	*
	* Input (eg measured) wrenches are stored in a 6xN matrix: each column is a 6x1 vector
	* with force/moment; N is the number of columns, ie the number of measured/input wrenches to the limb
	* the order is assumed coherent with the one built when adding limbs
	* eg: 
	* adding limbs: addLimb(limb1), addLimb(limb2), addLimb(limb3)
	* where limb1, limb3 have wrench flow input
	* passing wrenches: Matrix FM(6,2), FM.setcol(0,fm1), FM.setcol(1,fm3)
	*
	* Note: RBT calls computeWrenchNewtonEuler in the limb, meaning that
	* perform a "basic" wrench computation without any sensor, just
	* setting wrenches at the end-effector or at the base, and calling
	* recursive wrench computation.
	*/
	virtual bool solveWrench(const yarp::sig::Matrix &FM);

	/**
	*
	* input (eg measured) wrenches are stored in two 3xN matrix: each column is a 3x1 vector
	* with force/moment; N is the number of columns, ie the number of measured/input wrenches to the limb
	* the order is assumed coherent with the one built when adding limbs
	* eg: 
	* adding limbs: addLimb(limb1), addLimb(limb2), addLimb(limb3)
	* where limb1, limb3 have wrench flow input
	* passing wrenches: Matrix F(3,2), F.setcol(0,f1), F.setcol(1,f3) and similar for moment
	*
	* Note: RBT calls computeWrenchNewtonEuler in the limb, meaning that
	* perform a "basic" wrench computation without any sensor, just
	* setting wrenches at the end-effector or at the base, and calling
	* recursive wrench computation.
	*/
	virtual bool solveWrench(const yarp::sig::Matrix &F, const yarp::sig::Matrix &M);

	yarp::sig::Vector getForce() const;
	yarp::sig::Vector getMoment() const;
	yarp::sig::Vector getAngVel() const;
	yarp::sig::Vector getAngAcc() const;
	yarp::sig::Vector getLinAcc() const;


};



} //end of namespace iDyn
#endif