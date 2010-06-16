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
 * 
 * \section intro_sec Description
 * 
 * iDynNode represents a virtual node where multiple limbs are connected, and exchange
 * kinematic and wrench information. The mutual exchange bewteen node and limb (full duplex)
 * is managed used a RigidBodyTransformation class, containing the roto-translational matrix
 * which describes the connection. Multiple limbs can be attached to the Node. A connection
 * is defined also by the flows of kinematic and wrench variables: from limb to node 
 * (RBT_NODE_IN) or from limb to node (RBT_NODE_OUT). 
 *
 * \section tested_os_sec Tested OS
 * 
 * Windows
 *
 * \section example_sec Example
 *
 * head->setAng(q); 
 * head->setDAng(dq);
 * head->setD2Ang(ddq);
 * node->solveKinematics(w0,dw0,ddp0);
 * Matrix FM(6,1); FM.zero();
 * node->solveWrench(FM);
 * 
 * Now that the node is solved, one can get kinematic/dynamic information from 
 * the up (or any attached limb) 
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
#include <iCub/iDyn/iDynFwd.h>
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
// note: up, arm and legs are attached with the base
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

	///flag for sensor or not - only used for setWrenchMeasures()
	bool hasSensor;

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

	/**
	* Basic computations for applying RBT on kinematic variables.
	* The computations are similar to the Forward/Backward ones in 
	* OneLinkNewtonEuler: compute AngVel/AngAcc/LinAcc.
	* Here they are fastened and adapted to the RBT.	
	*/
	void computeKinematic();

	/**
	* Basic computations for applying RBT on wrench variables.
	* The computations are similar to the Forward/Backward ones in 
	* OneLinkNewtonEuler: compute Force/Moment.
	* Here they are fastened and adapted to the RBT.
	*/

	void computeWrench();

public:

	/**
	* Constructor, defining the limb attached to the node.
	* @param _limb pointer to a iDynLimb
	* @param _H a (4x4) roto-translational matrix defining the rigid body transformation
	* @param _info a string with information
	* @param kin the kinematic flow
	* @param wre the wrench flow
	* @param _mode the NewEulMode for computations
	* @param verb verbosity flag
	*/
	RigidBodyTransformation(iDyn::iDynLimb *_limb, const yarp::sig::Matrix &_H, const std::string &_info, bool _hasSensor = false, const FlowType kin=RBT_NODE_OUT, const FlowType wre=RBT_NODE_IN, const NewEulMode _mode=DYNAMIC, unsigned int verb=VERBOSE);

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
	* Set the wrench variables in the sensor. 
	* @param sensor the sensor attached to the limb
	* @param Fsens	the force
	* @param Musens the moment
	* @return true if succeeds, false otherwise
	*/
	bool setWrenchMeasure(iDyn::iDynSensor *sensor, const yarp::sig::Vector &Fsens, const yarp::sig::Vector &Musens); 

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
	* @param wNode	the angular velocity of the Node - it is modified!
	* @param dwNode the angular acceleration of the Node - it is modified!
	* @param ddpNode the linear acceleration of the Node - it is modified!
	*/
	void getKinematic( yarp::sig::Vector &wNode, yarp::sig::Vector &dwNode, yarp::sig::Vector &ddpNode); 
	
	/**
	* Get the wrench variables (F,Mu) of the limb, transform it according to the RBT and
	* add it to the node wrench. The variables are stored into the param vectors,
	* which are accessible. This method calls getWrenchNewtonEuler() in the limb,
	* and the limb itself knows whether to get this information in the base ot in the end-effector, 
	* depending on its iteration mode for wrenches. This method is used by iDynNode to connect two or 
	* multiple limbs. 
	* @param FNode	the iDynNode force - it is modified!
	* @param MuNode the iDynNode moment - it is modified!
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

	/**
	* @return a string with information
	*/
	std::string toString() const;

	/**
	* @return false
	*/
	bool isSensorized() const;

	/**
	* Calls the compute kinematic of the limb
	*/
	void computeLimbKinematic();

	/**
	* Calls the compute wrench of the limb
	*/
	void computeLimbWrench();


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
	iDynNode(const NewEulMode _mode=DYNAMIC);

	/**
	*	Constructor
	*/
	iDynNode(const std::string &_info, const NewEulMode _mode=DYNAMIC, unsigned int verb=VERBOSE);

	/**
	* Add one limb to the node, defining its RigidBodyTransformation. A new RigidBodyTransformation
	* is added to the RBT list.
	* @param limb pointer to generic limb
	* @param H a (4x4) roto-translational matrix defining the transformation between node and limb base/end
	* @param kinFlow the type of information flow of kinematics variables
	* @param wreFlow the type of information flow of wrench variables
	* @param hasSensor flag for having or not a FT sensor
	*/
	virtual void addLimb(iDyn::iDynLimb *limb, const yarp::sig::Matrix &H, const FlowType kinFlow=RBT_NODE_OUT, const FlowType wreFlow=RBT_NODE_IN, bool hasSensor=false);

	/**
	* Main function to manage the exchange of kinematic information among the limbs attached to the node.
	* One single limb with kinematic flow of input type must exist: this limb is initilized with the kinematic variables
	* w0,dw0,ddp0 (eg the up receives this information from the inertia sensor). The limb itself knows where to init the 
	* chain (base/end) depending on how it is attached to the node. Then the first limb kinematics is solved.
	* The kinematic variables are retrieved from the RBT, which applies its roto-translation. Then the kinematic
	* variables are sent to the other limbs, having kinematic flow of output type: the RBT transformation is applied from node
	* to limb.
	* @param w0 the initial/measured angular velocity
	* @param dw0 the initial/measured angular acceleration
	* @param ddp0 the initial/measured linear acceleration
	* @return true if succeeds, false otherwise
	*/
	bool solveKinematics( const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0);

	/**
	* Main function to manage the exchange of kinematic information among the limbs attached to the node.
	* @return true if succeeds, false otherwise
	*/
	bool solveKinematics();

	/**
	* Set the kinematic measurement (w,dw,ddp) on the limb where the kinematic flow is of type RBT_NODE_IN.
	*/
	bool setKinematicMeasure(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0);

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
	* @return true if succeeds, false otherwise
	*/
	virtual bool solveWrench();

	/**
	* This is to manage the exchange of wrench information among the limbs attached to the node.
	* Multiple limbs with wrench flow of input type can exist, but at least one limb with output type must exist, to 
	* compute the wrench balance on the node.
	* The measured/input wrenches to the limbs are here passed as a big matrix. In this function the input wrench
	* is set in the limb calling initWrenchNewtonEuler(), which simply set the measured forces in the base/final
	* link of the limb.
	* elsewhere: eg another class or the main is setting the measured wrenches.
	* Input (eg measured) wrenches are stored in a 6xN matrix: each column is a 6x1 vector
	* with force/moment; N is the number of columns, ie the number of measured/input wrenches to the limb
	* the order is assumed coherent with the one built when adding limbs
	* eg: 
	* adding limbs: addLimb(limb1), addLimb(limb2), addLimb(limb3)
	* where limb1, limb3 have wrench flow input
	* setting wrenches: Matrix FM(6,2), FM.setcol(0,fm1), FM.setcol(1,fm3)
	*
	* Note: RBT calls computeWrenchNewtonEuler in the limb, meaning that
	* perform a "basic" wrench computation without any sensor, just
	* setting wrenches at the end-effector or at the base, and calling
	* recursive wrench computation.
	*
	* @param FM a (6xN) matrix with forces and moments 
	* @return true if succeeds, false otherwise
	*/
	bool solveWrench(const yarp::sig::Matrix &FM);

	/**
	* This is to manage the exchange of wrench information among the limbs attached to the node.
	* Multiple limbs with wrench flow of input type can exist, but at least one limb with output type must exist, to 
	* compute the wrench balance on the node.
	* The measured/input wrenches to the limbs are here passed as a big matrix. In this function the input wrench
	* is set in the limb calling initWrenchNewtonEuler(), which simply set the measured forces in the base/final
	* link of the limb.
	* Input (eg measured) wrenches are stored in two 3xN matrix: each column is a 3x1 vector
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
	*
	* @param F a (3xN) matrix with forces
	* @param M a (3xN) matrix with moments 
	* @return true if succeeds, false otherwise
	*/
	bool solveWrench(const yarp::sig::Matrix &F, const yarp::sig::Matrix &M);

	/**
	* @param F a (3xN) matrix with forces
	* @param M a (3xN) matrix with moments 
	* @return true if succeeds, false otherwise
	*/
	virtual bool setWrenchMeasure(const yarp::sig::Matrix &F, const yarp::sig::Matrix &M);

	/**
	* @param FM a (6xN) matrix with forces and moments 
	* @return true if succeeds, false otherwise
	*/
	virtual bool setWrenchMeasure(const yarp::sig::Matrix &FM);

	/**
	* @return the node force
	*/
	yarp::sig::Vector getForce() const;
	
	/**
	* @return the node moment
	*/
	yarp::sig::Vector getMoment() const;
	
	/**
	* @return the node angular velocity
	*/
	yarp::sig::Vector getAngVel() const;

	/**
	* @return the node angular acceleration
	*/
	yarp::sig::Vector getAngAcc() const;

	/**
	* @return the node linear acceleration
	*/
	yarp::sig::Vector getLinAcc() const;


};


/**
* \ingroup iDynBody
*
* A class for connecting two or mutiple limbs and exchanging kinematic and 
* wrench information between limbs, when one or multiple limbs have FT sensors.
*/
class iDynSensorNode : public iDynNode
{

protected:

	/// the list of iDynSensors used to solve each limb after FT sensor measurements
	std::deque<iDyn::iDynSensor *> sensorList;

public:

	/**
	* Default constructor
	* @param _mode the computation mode for kinematic/wrench using Newton-Euler's formula
	*/
	iDynSensorNode(const NewEulMode _mode=DYNAMIC);

	/**
	* Constructor
	* @param _info some information, ie the node name
	* @param _mode the computation mode for kinematic/wrench using Newton-Euler's formula
	* @param verb verbosity flag
	*/
	iDynSensorNode(const std::string &_info, const NewEulMode _mode=DYNAMIC, unsigned int verb=VERBOSE);

	/**
	* Add one limb to the node, defining its RigidBodyTransformation. A new RigidBodyTransformation
	* is added to the RBT list. Since there's not an iDynSensor for this limb, it is set to NULL in the
	* sensorList.
	* @param limb pointer to generic limb
	* @param H a (4x4) roto-translational matrix defining the transformation between node and limb base/end
	* @param kinFlow the type of information flow of kinematics variables
	* @param wreFlow the type of information flow of wrench variables
	*/
	virtual void addLimb(iDyn::iDynLimb *limb, const yarp::sig::Matrix &H, const FlowType kinFlow=RBT_NODE_OUT, const FlowType wreFlow=RBT_NODE_IN);

	/**
	* Add one limb to the node, defining its RigidBodyTransformation and the iDynSensor used to 
	* solve it after FT sensor measurements. A new RigidBodyTransformation
	* is added to the RBT list. The iDynSensor is added to the sensorList.
	* @param limb pointer to generic limb
	* @param H a (4x4) roto-translational matrix defining the transformation between node and limb base/end
	* @param sensor pointer to iDynSensor of the limb
	* @param kinFlow the type of information flow of kinematics variables
	* @param wreFlow the type of information flow of wrench variables
	*/
	void addLimb(iDyn::iDynLimb *limb, const yarp::sig::Matrix &H, iDyn::iDynSensor *sensor, const FlowType kinFlow=RBT_NODE_OUT, const FlowType wreFlow=RBT_NODE_IN);

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
	* @return true if succeeds, false otherwise
	*/
	virtual bool solveWrench();

	/**
	* @param F a (3xN) matrix with forces
	* @param M a (3xN) matrix with moments 
	* @return true if succeeds, false otherwise
	*/
	virtual bool setWrenchMeasure(const yarp::sig::Matrix &F, const yarp::sig::Matrix &M);

	/**
	* @param FM a (6xN) matrix with forces and moments 
	* @return true if succeeds, false otherwise
	*/
	virtual bool setWrenchMeasure(const yarp::sig::Matrix &FM);

	/**
	* Exploit iDynInvSensor methods to retrieve FT sensor measurements after
	* solving wrenches in the limbs.
	*/
	yarp::sig::Matrix estimateSensorsWrench(const yarp::sig::Matrix &FM);



};


/**
* \ingroup iDynBody
*
* A class for connecting a central-up limb, a left and right limb of the iCub, and exchanging kinematic and 
* wrench information between limbs, when both left/right limb have FT sensors and the central-up one use the 
* kinematic and wrench information coming from a inertial measurements or another iDynSensorNode. 
* This is the base class of UpperTorso and LowerTorso. The connection between UpperTorso
* and LowerTorso is not handled here: it is simply supposed that a Torso Node receives correct 
* kinematic and wrench input from outside.
*/
class iDynSensorTorsoNode : protected iDynSensorNode
{
protected:

	/// left limb
	iDyn::iDynLimb * left;
	/// right limb
	iDyn::iDynLimb * right;
	/// central-up limb
	iDyn::iDynLimb * up;

	/// left leg - FT sensor and solver
	iDyn::iDynSensor * leftSensor;
	/// right leg - FT sensor and solver
	iDyn::iDynSensor * rightSensor;

	/// roto-translational matrix defining the central-up base frame with respect to the torso node
	yarp::sig::Matrix HUp;
	/// roto-translational matrix defining the left limb base frame with respect to the torso node
	yarp::sig::Matrix HLeft;
	/// roto-translational matrix defining the right limb base frame with respect to the torso node
	yarp::sig::Matrix HRight;

	/// name of left limb
	std::string left_name;
	/// name of right limb
	std::string right_name;
	/// name of central-up limb
	std::string up_name;
	/// the torso node name
	std::string name;

	/**
	* Build the node.
	*/
	virtual void build();

public:

	/**
	* Constructor
	* @param _mode the computation mode for kinematic/wrench using Newton-Euler's formula
	* @param verb verbosity flag
	*/
	iDynSensorTorsoNode(const NewEulMode _mode=DYNAMIC, unsigned int verb=VERBOSE);

	/**
	* Constructor
	* @param _info some information, ie the node name
	* @param _mode the computation mode for kinematic/wrench using Newton-Euler's formula
	* @param verb verbosity flag
	*/
	iDynSensorTorsoNode(const std::string &_info, const NewEulMode _mode=DYNAMIC, unsigned int verb=VERBOSE);

	/**
	* Destructor
	*/
	~iDynSensorTorsoNode();

	/**
	* Main method for solving kinematics and wrench among limbs, where information are shared.
	*/
	bool update();
	
	/**
	* Main method for solving kinematics and wrench among limbs, where information are shared.
	* @param w0 a 3x1 vector with the initial/measured angular velocity
	* @param dw0 a 3x1 vector with the initial/measured angular acceleration
	* @param ddp0 a 3x1 vector with the initial/measured linear acceleration
	* @param FM_right a 6x1 vector with forces and moments measured by the FT sensor in the right limb
	* @param FM_left a 6x1 vector with forces and moments measured by the FT sensor in the left limb
	* @param FM_up a 6x1 vector with forces and moments initializing the central limb
	* @return true if succeeds, false otherwise
	*/
	bool update(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0, const yarp::sig::Vector &FM_right, const yarp::sig::Vector &FM_left, const yarp::sig::Vector &FM_up);


	//----------------
	//      GET
	//----------------

	/**
	* @param limbType a string with the limb name
	* @return the chosen limb forces
	*/
	yarp::sig::Matrix getForces(const std::string &limbType);	

	/**
	* @param limbType a string with the limb name
	* @param iLink the link index in the limb
	* @return the chosen limb-link force
	*/
	yarp::sig::Vector getForce(const std::string &limbType, const unsigned int iLink) const	;

	/**
	* @param limbType a string with the limb name
	* @return the chosen limb moments
	*/
	yarp::sig::Matrix getMoments(const std::string &limbType);

	/**
	* @param limbType a string with the limb name
	* @param iLink the link index in the limb
	* @return the chosen limb-link moment
	*/
	yarp::sig::Vector getMoment(const std::string &limbType, const unsigned int iLink) const;

	/**
	* @param limbType a string with the limb name
	* @return the chosen limb torques
	*/
	yarp::sig::Vector getTorques(const std::string &limbType);

	/**
	* @param limbType a string with the limb name
	* @param iLink the link index in the limb
	* @return the chosen limb-link torque
	*/
	double getTorque(const std::string &limbType, const unsigned int iLink) const;

	/**
	* @return the torso force
	*/
	yarp::sig::Vector getTorsoForce() const;
	/**
	* @return the torso moment
	*/
	yarp::sig::Vector getTorsoMoment() const;	
	/**
	* @return the torso angular velocity
	*/
	yarp::sig::Vector getTorsoAngVel() const;
	/**
	* @return the torso angular acceleration
	*/
	yarp::sig::Vector getTorsoAngAcc() const;
	/**
	* @return the torso linear acceleration
	*/
	yarp::sig::Vector getTorsoLinAcc() const;


	//----------------
	//      SET
	//----------------

	/**
	* Set the inertial sensor measurements on the central-up limb
	* @param w0 a 3x1 vector with the initial/measured angular velocity
	* @param dw0 a 3x1 vector with the initial/measured angular acceleration
	* @param ddp0 a 3x1 vector with the initial/measured linear acceleration
	* @return true if succeeds (correct vectors size), false otherwise
	*/
	bool setInertialMeasure(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0);
	
	/**
	* Set the FT sensor measurements on the sensor in right and left limb. This operation is necessary to 
	* initialize the wrench phase correctly. The central-up limb wrench is initialized with a null vector (=0).
	* @param FM_right a 6x1 vector with forces and moments measured by the FT sensor in the right limb
	* @param FM_left a 6x1 vector with forces and moments measured by the FT sensor in the left limb
	* @return true if succeeds, false otherwise
	*/
	bool setSensorMeasurement(const yarp::sig::Vector &FM_right, const yarp::sig::Vector &FM_left);

	/**
	* Set the FT sensor measurements on the sensor in right and left limb. This operation is necessary to 
	* initialize the wrench phase correctly. The central-up limb wrench initializing wrench is also specified.
	* @param FM_right a 6x1 vector with forces and moments measured by the FT sensor in the right limb
	* @param FM_left a 6x1 vector with forces and moments measured by the FT sensor in the left limb
	* @param FM_up a 6x1 vector with forces and moments initializing the central-up limb
	* @return true if succeeds, false otherwise
	*/
	bool setSensorMeasurement(const yarp::sig::Vector &FM_right, const yarp::sig::Vector &FM_left, const yarp::sig::Vector &FM_up);


	//------------------
	//    LIMB CALLS
	//------------------

	yarp::sig::Vector setAng(const std::string &limbType, const yarp::sig::Vector &_q);
    yarp::sig::Vector getAng(const std::string &limbType);
    double            setAng(const std::string &limbType, const unsigned int i, double _q);
    double            getAng(const std::string &limbType, const unsigned int i);

	yarp::sig::Vector setDAng(const std::string &limbType, const yarp::sig::Vector &_dq);
    yarp::sig::Vector getDAng(const std::string &limbType);
    double            setDAng(const std::string &limbType, const unsigned int i, double _dq);
    double            getDAng(const std::string &limbType, const unsigned int i);                                  

	yarp::sig::Vector setD2Ang(const std::string &limbType, const yarp::sig::Vector &_ddq);
    yarp::sig::Vector getD2Ang(const std::string &limbType);
    double            setD2Ang(const std::string &limbType, const unsigned int i, double _ddq);
    double            getD2Ang(const std::string &limbType, const unsigned int i);

	/**
	* @param limbType a string with the limb name
	* @return the number of links of the chosen limb
	*/
	unsigned int	  getNLinks(const std::string &limbType) const;

	/**
	* Redefinition from iDynSensorNode.
	* Exploit iDynInvSensor methods to retrieve FT sensor measurements after
	* solving wrenches in the limbs.
	* @param FM a (6xN) matrix of forces/moments where N is the number 
	*/
	yarp::sig::Matrix estimateSensorsWrench(const yarp::sig::Matrix &FM) { return iDynSensorNode::estimateSensorsWrench(FM); }

	
};



/**
* \ingroup iDynBody
*
* A class for connecting head, left and right arm of the iCub, and exchanging kinematic and 
* wrench information between limbs, when both arms have FT sensors and the head use the 
* inertial sensor.
*/
class iCubUpperTorso : public iDynSensorTorsoNode
{
	friend class iDyn::iCubWholeBody;

protected:
	/**
	* Build the node.
	*/
	void build();

public:

	/**
	* Constructor
	* @param _info some information, ie the node name
	* @param _mode the computation mode for kinematic/wrench using Newton-Euler's formula
	* @param verb verbosity flag
	*/
	iCubUpperTorso(const NewEulMode _mode=DYNAMIC, unsigned int verb=VERBOSE);

};


/**
* \ingroup iDynBody
*
* A class for connecting torso, left and right leg of the iCub, and exchanging kinematic and 
* wrench information between limbs, when both legs have FT sensors and the torso use the 
* kinematic and wrench information coming from UpperTorso. The correct connection bewteen UpperTorso
* and LowerTorso is not handled here; it is supposed that LowerTorso receives correct kinematic
* and wrench variables for the initialization of the kinematic and wrench phases.
*/
class iCubLowerTorso : public iDynSensorTorsoNode
{
	friend class iDyn::iCubWholeBody;

protected:
	/**
	* Build the node.
	*/
	void build();

public:

	/**
	* Constructor
	* @param _info some information, ie the node name
	* @param _mode the computation mode for kinematic/wrench using Newton-Euler's formula
	* @param verb verbosity flag
	*/
	iCubLowerTorso(const NewEulMode _mode=DYNAMIC, unsigned int verb=VERBOSE);

};




/**
* \ingroup iDynBody
*
* A class for connecting UpperTorso and LowerTorso of the iCub, then getting the 
* WholeBody in the dynamic framework. It is merely a container: pointers to upper and
* lower torso objects are accessible, so all public methods of the two objects can be used. 
*/
class iCubWholeBody
{
protected:
	/// the rigid body transformation linking the UpperTorso node with the final link of the iCubTorsoDyn chain,
	/// defining the connection between Upper and Lower Torso
	RigidBodyTransformation * rbt;

public:

	/// pointer to UpperTorso = head + right arm + left arm
	iCubUpperTorso * upperTorso;
	/// pointer to LowerTorso = torso + right leg + left leg
	iCubLowerTorso * lowerTorso;

	/**
	* Constructor: build the nodes and creates the whole body
	* @param mode the computation mode: DYNAMIC/STATIC/DYNAMIC_W_ROTOR/DYNAMIC_CORIOLIS_GRAVITY
	* @param verbose the verbosity level: NO_VERBOSE/VERBOSE/MORE_VERBOSE
	*/
	iCubWholeBody(const NewEulMode mode=DYNAMIC, unsigned int verbose=VERBOSE);

	/**
	* Destructor
	*/
	~iCubWholeBody();

	/**
	* Connect upper and lower torso: this procedure handle the exchange of kinematic and
	* wrench variables between the two parts.
	*/
	void attachTorso();

};


} //end of namespace iDyn
#endif