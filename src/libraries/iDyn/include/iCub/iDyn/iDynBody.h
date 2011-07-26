/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Serena Ivaldi, Matteo Fumagalli
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

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
#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iDyn/iDynInv.h>
#include <deque>
#include <string>


namespace iCub
{

namespace iDyn
{

	class OneLinkNewtonEuler;
	class BaseLinkNewtonEuler;
	class FinalLinkNewtonEuler;
	class SensorLinkNewtonEuler;
    class ContactNewtonEuler;
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

// Jacobian Flow
// used to specify if the direction of computation of the Jacobian is the same as 
// the one of the kinematics of the chain
// JAC_KIN = same flow of kinematics
// JAC_IKIN = inverse flow wrt kinematics
enum JacobType{ JAC_KIN, JAC_IKIN }; 

#define RBT_HAS_SENSOR	    true
#define RBT_NO_SENSOR	    false
#define NODE_AFTER_ATTACH	true
#define NODE_NO_ATTACH      false

enum partEnum{ LEFT_ARM=0, RIGHT_ARM, LEFT_LEG, RIGHT_LEG, TORSO, HEAD, ALL }; 

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
    * Return the rotational 3x3 matrix of the RBT
	* @return the rotational matrix of the RBT
	*/
	yarp::sig::Matrix	getR();	
	
	/**
    * Return the translational part of the RBT matrix
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
    * Return the the (4x4) roto-translational matrix defining the rigid body transformation
	* @return H, the 4x4 matrix of the RBT
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
    * Return the kinematic flow type
	* @return the kinematic flow
	*/
	FlowType getKinematicFlow() const;
	
	/**
    * return the wrench flow type
	* @return the wrench flow
	*/
	FlowType getWrenchFlow() const;

	/**
    * Return some information
	* @return a string with information
	*/
	std::string toString() const;

	/**
    * Return a boolean, depending if the limb attached to the RBT has a FT sensor
    * or not.
	* @return true if the limb has a FT sensor, false otherwise
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

	/**
    * Return the number of links of the limb (N)
	* @return the number of links in the limb
	*/
	unsigned int getNLinks() const;

	/**
    * Return the number of DOF of the limb (DOF <= N)
	* @return the number of DOF in the limb
	*/
	unsigned int getDOF() const;

	/**
    * Return the i-th roto-translational matrix of the chain. This method
    * basically calls iKinChain::getH(i,allLink). the boolean allLink specifies if
    * all the links are considered, or only the unblocked ones (DOF)
	* @param i the link index in the chain (0<=i<N)
    * @param allLink if all the links are considered
    * @return H of the i-th link
	*/
    yarp::sig::Matrix getH(const unsigned int i, const bool allLink=false);        
    
	/**
	* Return the end-effector roto-translational matrix of the end-effector
    * H of the end-effector
	*/
	yarp::sig::Matrix getH(); 

	/**
	* Return the end-effector pose: x-y-z Cartesian position and 3/4 angles of orientation.
	* @param axisRep a flag for the axis representation
	* @return the end effector pose
	*/
	yarp::sig::Vector getEndEffPose(const bool axisRep = true);

	/**
	* This method is used to compute the Jacobian between two links in two different chains (eg from link 4 in chain_A
	* to link 3 in chain_B.
	* @param iLink the index of the link, in the chain, being the base frame for the Jacobian computation
	* @param Pn the matrix describing the roto-translational matrix between base and end-effector (in two different limbs)
	* @param rbtRoto if false, simply return Jacobian; if true return T * Jacobian, where T is a 6x6 diagonal matrix with the rotational part of the RBT
	* @return the Jacobian matrix from the base of the chain until the iLink  (e.g. from link 0 to 4)
	*/
	yarp::sig::Matrix computeGeoJacobian(const unsigned int iLink, const yarp::sig::Matrix &Pn, bool rbtRoto = false);

    /**
	* This method is used to compute the Jacobian between two links in two different chains.
	* @param iLink the index of the link, in the chain, being the base frame for the Jacobian computation
	* @param Pn the matrix describing the roto-translational matrix between base and end-effector (in two different limbs)
	* @param H0 the H0 matrix of the base to be used for the Jacobian computation
	* @param rbtRoto if false, simply return Jacobian; if true return T * Jacobian, where T is a 6x6 diagonal matrix with the rotational part of the RBT
	* @return the Jacobian matrix from the iLink of the chain until the base of the chain (ie from link 4 to 0)
	*/
	yarp::sig::Matrix computeGeoJacobian(const unsigned int iLink, const yarp::sig::Matrix &Pn, const yarp::sig::Matrix &H0, bool rbtRoto = false);

	/**
	* This method is used to compute the Jacobian between two links in two different chains: in this case
    * it returns the jacobian matrix of the whole chain (from base to end-effector) when Pn is an external vector.
	* @param Pn the matrix describing the roto-translational matrix between base and end-effector (in two different limbs)
	* @param rbtRoto if false, simply return Jacobian; if true return T * Jacobian, where T is a 6x6 diagonal matrix with the rotational part of the RBT
	* @return the Jacobian matrix of the chain
	*/
	yarp::sig::Matrix computeGeoJacobian(const yarp::sig::Matrix &Pn, bool rbtRoto = false);

    /**
	* This method is used to compute the Jacobian between two links in two different chains: in this case
    * it returns the jacobian matrix of the whole chain (from base to end-effector) when Pn is an external vector,
    * and H0 is an external matrix.
	* @param Pn the matrix describing the roto-translational matrix between base and end-effector (in two different limbs)
	* @param H0 the H0 matrix of the base to be used for the Jacobian computation
	* @param rbtRoto if false, simply return Jacobian; if true return T * Jacobian, where T is a 6x6 diagonal matrix with the rotational part of the RBT
	* @return the Jacobian matrix of the chain
	*/
	yarp::sig::Matrix computeGeoJacobian(const yarp::sig::Matrix &Pn, const yarp::sig::Matrix &H0, bool rbtRoto = false);

    /**
	* @param rbtRoto if false, simply return Jacobian; if true return T * Jacobian, where T is a 6x6 diagonal matrix with the rotational part of the RBT
	* @return the Jacobian matrix of the chain
	*/
	yarp::sig::Matrix computeGeoJacobian(bool rbtRoto = false);

    /**
    * Returns the Jacobian matrix of the limb until the link whose index is iLink in the chain.
    * This method basically calls GeoJacobian(iLink) in the iDynChain of the limb.
	* @param iLink the index of the link, in the chain
	* @param rbtRoto if false, simply return Jacobian; if true return T * Jacobian, where T is a 6x6 diagonal matrix with the rotational part of the RBT
	* @return the Jacobian matrix of the chain - from the base to the i-th link
	*/
    yarp::sig::Matrix computeGeoJacobian(const unsigned int iLink, bool rbtRoto = false);

	/**
    * Return a 6x6 diagonal matrix with the rotational matrix of the RBT 
	* @return a 6x6 diagonal matrix with the rotational matrix of the RBT 
	*/
	yarp::sig::Matrix	getR6() const;

    /**
    * Return the H0 matrix of the limb attached to the RBT
    * @return the H0 matrix of the limb attached to the RBT
    */
	yarp::sig::Matrix getH0() const;

    /**
    * Set a new H0 matrix in the limb attached to the RBT
    * @param _H0 the new H0 matrix of the limb attached to the RBT
    * @return true if succeed, false otherwise
    */
	bool setH0(const yarp::sig::Matrix &_H0);

    //---------------
    //   JAC COM
    //---------------

    /**
    * Returns the Jacobian matrix of the COM of the selected link (index = iLink) in the chain.
	* @param iLink the index of the link, in the chain
	* @param rbtRoto if false, simply return Jacobian; if true return T * Jacobian, where T is a 6x6 diagonal matrix with the rotational part of the RBT
	* @return the Jacobian matrix of the selected link COM 
	*/
    yarp::sig::Matrix computeCOMJacobian(const unsigned int iLink, bool rbtRoto = false);

    /**
    * Returns the Jacobian matrix of the COM of the selected link (index = iLink) in the chain: in this case
    * Pn is an external vector, that happens when multiple limbs are connected.
	* @param Pn the matrix describing the roto-translational matrix between base and end-effector (in two different limbs)
	* @param iLink the index of the link, in the chain
	* @param rbtRoto if false, simply return Jacobian; if true return T * Jacobian, where T is a 6x6 diagonal matrix with the rotational part of the RBT
	* @return the Jacobian matrix of the selected link COM 
	*/
	yarp::sig::Matrix computeCOMJacobian(const unsigned int iLink, const yarp::sig::Matrix &Pn, bool rbtRoto = false);

    /**
    * Returns the Jacobian matrix of the COM of the selected link (index = iLink) in the chain: in this case
    * Pn is an external vector, that happens when multiple limbs are connected. The H0 matrix used to initialize
    * the Jacobian computation is external too.
	* @param Pn the matrix describing the roto-translational matrix between base and end-effector (in two different limbs)
	* @param H0 the H0 matrix of the base to be used for the Jacobian computation
	* @param iLink the index of the link, in the chain
	* @param rbtRoto if false, simply return Jacobian; if true return T * Jacobian, where T is a 6x6 diagonal matrix with the rotational part of the RBT
	* @return the Jacobian matrix of the selected link COM 
	*/
    yarp::sig::Matrix computeCOMJacobian(const unsigned int iLink, const yarp::sig::Matrix &Pn, const yarp::sig::Matrix &_H0, bool rbtRoto = false );

    /**
    * Returns the COM matrix of the selected link (index = iLink) in the chain.
	* @param iLink the index of the link, in the chain
	* @return the roto-translational matrix of the selected link COM 
    */
    yarp::sig::Matrix getHCOM(unsigned int iLink);

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
	/// COM position of the node
	yarp::sig::Vector COM;	
	/// total mass of the node
	double mass;

	/**
	* Reset all data to zero. The list of limbs is not modified or deleted.
	*/
	void zero();

	/**
	* Compute Pn and H_A_Node matrices given two chains. This function is private, and
	* is used by computeJacobian() and computePose() to merely avoid code duplication.
	*/
	void compute_Pn_HAN(unsigned int iChainA, JacobType dirA, unsigned int iChainB, JacobType dirB, yarp::sig::Matrix &Pn, yarp::sig::Matrix &H_A_Node);

    /**
	* Compute Pn and H_A_Node matrices given two chains. This function is private, and
	* is used by computeJacobian() and computePose() to merely avoid code duplication.
	*/
	void compute_Pn_HAN(unsigned int iChainA, JacobType dirA, unsigned int iChainB, unsigned int iLinkB, JacobType dirB, yarp::sig::Matrix &Pn, yarp::sig::Matrix &H_A_Node);

    /**
	* Compute Pn and H_A_Node matrices given two chains. This function is private, and
	* is used by computeCOMJacobian() and computeCOMPose() to merely avoid code duplication.
	*/
	void compute_Pn_HAN_COM(unsigned int iChainA, JacobType dirA, unsigned int iChainB, unsigned int iLinkB, JacobType dirB, yarp::sig::Matrix &Pn, yarp::sig::Matrix &H_A_Node);

    /**
    * Return the number of limbs with wrench input, i.e. receiving wrench information from
    * external measurements.
	* @param afterAttach =true only if the limb received wrench parameters during an
	* attachTorso() procedure
	* @return the number of limbs with wrench input, if afterAttach=false; if
	* afterAttach=true, the number is of limbs - 1
	*/
	unsigned int howManyWrenchInputs(bool afterAttach=false) const;

    /**
    * Return the number of limbs with kinematic input, i.e. receiving kinematic
    * information from external measurements.
	* @param afterAttach =true only if the limb received kinematic parameters during an
	* attachTorso() procedure
	* @return the number of limbs with kinematic input, if afterAttach=false; if
	* afterAttach=true, the number is of limbs - 1
	*/
	unsigned int howManyKinematicInputs(bool afterAttach=false) const;

public:

	/**
	* Default constructor
    * @param _mode the modality for dynamic computation
	*/
	iDynNode(const NewEulMode _mode=DYNAMIC);

	/**
	* Constructor with parameters
    * @param _info some information on the node, i.e. its description
    * @param _mode the modality for dynamic computation
    * @param verb verbosity level
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
    * Return the RBT matrix of a certain limb attached to the node.
    * @param iLimb the index of the limb - the index is the number of insertion of the limb in the node
    * @return the RBT matrix of that limb, attached to the node
    */
    yarp::sig::Matrix getRBT(unsigned int iLimb) const;

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
    * Set the wrench measure on the limbs with input wrench
	* @param F a (3xN) matrix with forces
	* @param M a (3xN) matrix with moments 
	* @return true if succeeds, false otherwise
	*/
	virtual bool setWrenchMeasure(const yarp::sig::Matrix &F, const yarp::sig::Matrix &M);

	/**
    * Set the wrench measure on the limbs with input wrench
	* @param FM a (6xN) matrix with forces and moments 
	* @return true if succeeds, false otherwise
	*/
	virtual bool setWrenchMeasure(const yarp::sig::Matrix &FM);

	/**
    * Return the node force
	* @return the node force
	*/
	yarp::sig::Vector getForce() const;
	
	/**
	* Return the node moment
	* @return the node moment
	*/
	yarp::sig::Vector getMoment() const;
	
	/**
    * Return the node angular velocity
	* @return the node angular velocity
	*/
	yarp::sig::Vector getAngVel() const;

	/**
    * Return the node angular acceleration
	* @return the node angular acceleration
	*/
	yarp::sig::Vector getAngAcc() const;

	/**
    * Return the node linear acceleration
	* @return the node linear acceleration
	*/
	yarp::sig::Vector getLinAcc() const;

		//-----------------
		//    jacobians
		//-----------------

	/**
	* Compute the Jacobian of the limb with index iChain in the node, in its default direction (as
	* it would be done by iKin).
	* @param iChain the index of the chain (limb) in the node 
	* @return the Jacobian matrix
	*/
	yarp::sig::Matrix computeJacobian(unsigned int iChain);

	/**
	* Compute the Jacobian of the i-th link of the limb with index iChain in the node, in its default 
    * direction (as it would be done by iKin).
    * If the link index is not correct, a null Jacobian is returned.
    *
    * Important note: since we are specifying the link index in the chain, the Jacobian computation
    * will deal with all the links, even blocked links. 
    * The Jacobian size is not 6x(the DOF until iLinkB) but 6xiLinkB, where 0<iLinkB<N
    *
	* @param iChain the index of the chain (limb) in the node 
    * @param iLink  the index of the limnk in the limb
	* @return the Jacobian matrix
	*/
	yarp::sig::Matrix computeJacobian(unsigned int iChain, unsigned int iLink);

	/**
	* Compute the Jacobian between two links in two different chains. The chains are
	* specified by their index in the list (the progressive number of insertion).
	* The first limb (limb A - index=iChainA) has the base link of the jacobian  while
	* the second limb (limb B - index=iChainB) has the final link of the jacobian.
    * Whether the base/final of the Jacobian coincides with the base/end of the chains, depends
    * on the flags dirA,dirB: if dirA=JAC_DIR, then the beginning is at the base of chain, otherwise it is
    * at the end-effector; if dirB=JAC_DIR, the final link of the jacobian is on the end-effector
    * of the chain, otherwise on its base.
	* @param iChainA the index of the chain (the limb) in the node having the base frame
	* @param dirA the 'direction' of the chain wrt the jacobian computation
	* @param iChainB the index of the chain (the limb) in the node having the final frame
	* @param dirB the 'direction' of the chain wrt the jacobian computation
	* @return the Jacobian matrix
	*/
	yarp::sig::Matrix computeJacobian(unsigned int iChainA, JacobType dirA, unsigned int iChainB, JacobType dirB);

    /**
	* Compute the Jacobian between two links in two different chains. The chains are
	* specified by their index in the list (the progressive number of insertion).
	* The first limb has the base of the jacobian (base or end-effector of the limb, depending
    * on the Jacobian direction JacA) while
	* the second limb (limb B - index=iChainB) has the final link of the jacobian (index=iLinkB).
    *
    * Important note: since we are specifying the link index in chain B, the Jacobian computation
    * on chain B will deal with all the links, even blocked links. 
    * The Jacobian size is not 6x(DOF_A + the DOF until iLinkB) but 6x(DOF_A+iLinkB), where 0<iLinkB<N
    *
	* @param iChainA the index of the chain (the limb) in the node having the base (<0>) frame
	* @param dirA the 'direction' of the chain wrt the jacobian computation
	* @param iChainB the index of the chain (the limb) in the node having the final (<N>) frame
	* @param iLinkB the index of the link, in the indexChainN chain, being the final (<N>) frame for the Jacobian computation
	* @param dirB the 'direction' of the chain wrt the jacobian computation
	* @return the Jacobian matrix
	*/
	yarp::sig::Matrix computeJacobian(unsigned int iChainA, JacobType dirA, unsigned int iChainB, unsigned int iLinkB, JacobType dirB);

	/**
	* Compute the Pose of the end-effector, given a "virtual" chain connecting two limbs.
    * The chains are specified by their index in the list (the progressive number of insertion).
	* The first limb (limb A - index=iChainA) has the base link of the augmented chain while
	* the second limb (limb B - index=iChainB) has the final link of the augmented chain .
    * Whether the base/end of the augmented chain coincides with the base/end of the single chains, depends
    * on the flags dirA,dirB: if dirA=JAC_DIR, then the beginning is at the base of chain, otherwise it is
    * at the end-effector; if dirB=JAC_DIR, the final link of the augmented chain is on the end-effector
    * of the chain, otherwise on its base.
    * This method is useful to compute the end-effector pose (i.e. computing the arm pose, when the chain
    * torso + arm is considered) in a multi-limb chain.
	* @param iChainA the index of the chain (the limb) in the node having the base frame
	* @param dirA the 'direction' of visit of the chain
	* @param iChainB the index of the chain (the limb) in the node having the final frame
	* @param dirB the 'direction' of visit of the chain
	* @return the Jacobian matrix
	*/
	yarp::sig::Vector computePose(unsigned int iChainA, JacobType dirA, unsigned int iChainB, JacobType dirB, const bool axisRep);

    /**
	* Compute the Pose of the end-effector, given a "virtual" chain connecting two limbs.
    * The chains are specified by their index in the list (the progressive number of insertion).
	* The first limb (limb A - index=iChainA) has the base link of the augmented chain while
	* the second limb (limb B - index=iChainB) has the final link of the augmented chain, ending in the link iLinkB.
    * Whether the base/end of the augmented chain coincides with the base/end of the single chains, depends
    * on the flags dirA,dirB: if dirA=JAC_DIR, then the beginning is at the base of chain, otherwise it is
    * at the end-effector; if dirB=JAC_DIR, the final link of the augmented chain is on the end-effector
    * of the chain, otherwise on its base.
    * This method is useful to compute the end-effector pose (i.e. computing the elbow pose, when the chain
    * torso + arm is considered) in a multi-limb chain.
	* @param iChainA the index of the chain (the limb) in the node having the base frame
	* @param dirA the 'direction' of visit of the chain
	* @param iChainB the index of the chain (the limb) in the node having the final frame
	* @param iLinkB the index of the link, in the indexChainN chain, being the final (<N>) frame for the Jacobian computation
	* @param dirB the 'direction' of visit of the chain
	* @return the Jacobian matrix
	*/
	yarp::sig::Vector computePose(unsigned int iChainA, JacobType dirA, unsigned int iChainB, unsigned int iLinkB, JacobType dirB, const bool axisRep);

    
    //---------------
    //   JAC COM
    //---------------

	/**
	* Compute the Jacobian of the COM of the i-th link of the limb with index iChain in the node.
    * If the link index is not correct, a null Jacobian is returned.
	* @param iChain the index of the chain (limb) in the node 
    * @param iLink  the index of the limnk in the limb
	* @return the Jacobian matrix of the COM
	*/
    yarp::sig::Matrix computeCOMJacobian(unsigned int iChain, unsigned int iLink);

    /**
	* Compute the Jacobian of the COM of link iLinkB, in chainB, when two different chains (A and B) are connected. The chains are
	* specified by their index in the list (the progressive number of insertion).
	* The first limb has the base of the jacobian (base or end-effector of the limb, depending
    * on the Jacobian direction JacA) while
	* the second limb (limb B - index=iChainB) has the final link of the jacobian (index=iLinkB).
  	* @param iChainA the index of the chain (the limb) in the node having the base (<0>) frame
	* @param dirA the 'direction' of the chain wrt the jacobian computation
	* @param iChainB the index of the chain (the limb) in the node having the final (<N>) frame
	* @param iLinkB the index of the link, in the indexChainN chain, being the final (<N>) frame for the Jacobian computation
	* @param dirB the 'direction' of the chain wrt the jacobian computation
	* @return the Jacobian matrix of the COM
	*/
    yarp::sig::Matrix computeCOMJacobian(unsigned int iChainA, JacobType dirA, unsigned int iChainB, unsigned int iLinkB, JacobType dirB);


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

	/**
	* @return the number of limbs with sensor
	*/
	unsigned int howManySensors() const;

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
	* Set the Wrench measures on the limbs attached to the node.
	* The parameters F and M are (3xN) matrices, where each column
	* is an external wrench to be used for initializing the wrench phase; N is the number of 
	* limbs Nlimbs attached to the node. The boolean flag is used in case the external wrench on
	* the first limb has already been set; this is useful whenever two different nodes are
	* connected and share information: the first node sends kinematic and wrench information
	* to the 'first' limb of the second node (eg the torso). In that case the F and M matrices
	* should only contain external wrench for the other limbs, so it should be a (6x(Nlimbs-1)).
	* @param F a (3xN) matrix with forces, where N is the number of limbs 
	* if afterAttach=false, number of limbs -1 if afterAttach=true
	* @param M a (3xN) matrix with moments , where N is the number of limbs 
	* if afterAttach=false, number of limbs -1 if afterAttach=true
	* @param afterAttach a flag for specifying if the external wrench of the first limb has been already set or not
	* @return true if succeeds, false otherwise
	*/
	virtual bool setWrenchMeasure(const yarp::sig::Matrix &F, const yarp::sig::Matrix &M, bool afterAttach=false);

	/**
	* Set the Wrench measures on the limbs attached to the node. 
	* The parameter FM is a (6xN), where each column
	* is an external wrench to be used for initializing the wrench phase; N is the number of 
	* limbs Nlimbs attached to the node. The boolean flag is used in case the external wrench on
	* the first limb has already been set; this is useful whenever two different nodes are
	* connected and share information: the first node sends kinematic and wrench information
	* to the 'first' limb of the second node (eg the torso). In that case the FM matrix
	* should only contain external wrench for the other limbs, so it should be a (6x(Nlimbs-1)).
	* @param FM a (6xN) matrix with the external wrenches, where N is the number of limbs 
	* if afterAttach=false, number of limbs -1 if afterAttach=true
	* @param afterAttach a flag for specifying if the external wrench of the first limb has been already set or not
	* @return true if succeeds, false otherwise
	*/
	virtual bool setWrenchMeasure(const yarp::sig::Matrix &FM, bool afterAttach=false);

	/**
	* Exploit iDynInvSensor methods to retrieve FT sensor measurements after
	* solving wrenches in the limbs. The parameter FM is a (6xN), where each column
	* is an external wrench to be used for initializing the wrench phase; N is the number of 
	* limbs Nlimbs attached to the node. The boolean flag is used in case the external wrench on
	* the first limb has already been set; this is useful whenever two different nodes are
	* connected and share information: the first node sends kinematic and wrench information
	* to the 'first' limb of the second node (eg the torso). In that case the FM matrix
	* should only contain external wrench for the other limbs, so it should be a (6x(Nlimbs-1)).
	* @param FM a (6xN) matrix with the external wrenches,where N is the number of limbs 
	* if afterAttach=false, number of limbs -1 if afterAttach=true
	* @param afterAttach a flag for specifying if the external wrench of the first limb has been already set or not
	*/
	yarp::sig::Matrix estimateSensorsWrench(const yarp::sig::Matrix &FM, bool afterAttach=false);



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
class iDynSensorTorsoNode : public iDynSensorNode
{
protected:

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

	/// left limb
	iDyn::iDynLimb * left;
	/// right limb
	iDyn::iDynLimb * right;
	/// central-up limb
	iDyn::iDynLimb * up;

	/// COMs and masses of the limbs
	yarp::sig::Vector total_COM_UP;
	yarp::sig::Vector total_COM_LF;
	yarp::sig::Vector total_COM_RT;
	double            total_mass_UP;
	double            total_mass_LF;
	double            total_mass_RT;

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
    * @return true if kinematics and wrench phases succeed, false otherwise
	*/
	bool update();

	/**
	* Main method for solving kinematics and wrench among limbs, where information are shared.
	* This method assumes that the initial kinematics informations have already been set, as the 
	* external wrench on the central limb: i.e. this method is the one called after attachTorso().
	* @param FM_right a 6x1 vector with forces and moments measured by the FT sensor in the right limb
	* @param FM_left a 6x1 vector with forces and moments measured by the FT sensor in the left limb
	* @return true if succeeds, false otherwise
	*/
	bool update(const yarp::sig::Vector &FM_right, const yarp::sig::Vector &FM_left, bool afterAttach=true);
	
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
    * Return the chosen limb forces, as a 6xN matrix.
	* @param limbType a string with the limb name
	* @return the chosen limb forces
	*/
	yarp::sig::Matrix getForces(const std::string &limbType);	

	/**
    * Return the chosen limb-link force, as a 3x1 vector
	* @param limbType a string with the limb name
	* @param iLink the link index in the limb
	* @return the chosen limb-link force
	*/
	yarp::sig::Vector getForce(const std::string &limbType, const unsigned int iLink) const	;

	/**
    * Return the chosen limb moments, as a 6xN matrix
	* @param limbType a string with the limb name
	* @return the chosen limb moments
	*/
	yarp::sig::Matrix getMoments(const std::string &limbType);

	/**
    * Return the chosen limb-link moment, as a 3x1 vector
	* @param limbType a string with the limb name
	* @param iLink the link index in the limb
	* @return the chosen limb-link moment
	*/
	yarp::sig::Vector getMoment(const std::string &limbType, const unsigned int iLink) const;

	/**
    * Return the chosen limb torques, as a Nx1 vector
	* @param limbType a string with the limb name
	* @return the chosen limb torques
	*/
	yarp::sig::Vector getTorques(const std::string &limbType);

	/**
    * Return the chosen limb-link torque, as a real value
	* @param limbType a string with the limb name
	* @param iLink the link index in the limb
	* @return the chosen limb-link torque
	*/
	double getTorque(const std::string &limbType, const unsigned int iLink) const;

	/**
	* Performs the computation of the center of mass (COM) of the node
	* @return true if succeeds, false otherwise
	*/
	bool computeCOM();

	/**
    * Retrieves the result of the last COM computation
	* @param COM the computed COM of the node
	* @param mass the computed mass of the node 
	* @return true if succeeds, false otherwise
	*/
	bool getCOM(yarp::sig::Vector &COM, double & mass);

	/**
    * Return the torso force
	* @return the torso force
	*/
	yarp::sig::Vector getTorsoForce() const;
	/**
    * Return the torso moment
	* @return the torso moment
	*/
	yarp::sig::Vector getTorsoMoment() const;	
	/**
    * Return the torso angular velocity
	* @return the torso angular velocity
	*/
	yarp::sig::Vector getTorsoAngVel() const;
	/**
    * Return the torso angular acceleration
	* @return the torso angular acceleration
	*/
	yarp::sig::Vector getTorsoAngAcc() const;
	/**
    * Return the torso linear acceleration
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
	* initialize the wrench phase correctly. 
	* The central-up limb wrench is assumed to be already initialized: for example after an attachTorso().
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

    /**
    * Set joints angle position in the chosen limb
    * @param _q the joints position
    * @param limbType a string with limb type
    * @return the effective joint angles, considering min/max values
    */
	yarp::sig::Vector setAng(const std::string &limbType, const yarp::sig::Vector &_q);
    /**
    * Get joints angle position in the chosen limb
    * @param limbType a string with limb type
    * @return the joint angles
    */
    yarp::sig::Vector getAng(const std::string &limbType);
    /**
    * Set the i-th joint angle position in the chosen limb
    * @param _q the joint position
    * @param i the link index in the limb
    * @param limbType a string with limb type
    * @return the effective joint angle, considering min/max values
    */
    double            setAng(const std::string &limbType, const unsigned int i, double _q);
    /**
    * Get a joint angle position in the chosen limb
    * @param limbType a string with limb type
    * @param i the link index in the limb
    * @return the joint angle
    */
    double            getAng(const std::string &limbType, const unsigned int i);

    /**
    * Set joints angle velocity in the chosen limb
    * @param _dq the joints velocity
    * @param limbType a string with limb type
    * @return the effective joint velocity
    */
	yarp::sig::Vector setDAng(const std::string &limbType, const yarp::sig::Vector &_dq);
    /**
    * Get joints angle velocity in the chosen limb
    * @param limbType a string with limb type
    * @return the joint velocity
    */
    yarp::sig::Vector getDAng(const std::string &limbType);
    /**
    * Set the i-th joint angle velocity in the chosen limb
    * @param _dq the joint velocity
    * @param i the link index in the limb
    * @param limbType a string with limb type
    * @return the effective joint velocity
    */
    double            setDAng(const std::string &limbType, const unsigned int i, double _dq);
    /**
    * Get a joint angle velocity in the chosen limb
    * @param limbType a string with limb type
    * @param i the link index in the limb
    * @return the joint velocity
    */
    double            getDAng(const std::string &limbType, const unsigned int i);                                  

    /**
    * Set joints angle acceleration in the chosen limb
    * @param _dq the joints acceleration
    * @param limbType a string with limb type
    * @return the effective joint acceleration
    */
	yarp::sig::Vector setD2Ang(const std::string &limbType, const yarp::sig::Vector &_ddq);
    /**
    * Get joints angle acceleration in the chosen limb
    * @param limbType a string with limb type
    * @return the joint acceleration
    */
    yarp::sig::Vector getD2Ang(const std::string &limbType);
    /**
    * Set the i-th joint angle acceleration in the chosen limb
    * @param _ddq the joint acceleration
    * @param i the link index in the limb
    * @param limbType a string with limb type
    * @return the effective joint acceleration
    */
    double            setD2Ang(const std::string &limbType, const unsigned int i, double _ddq);
    /**
    * Get a joint angle acceleration in the chosen limb
    * @param limbType a string with limb type
    * @param i the link index in the limb
    * @return the joint acceleration
    */
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
	* @param FM a (6xN) matrix of forces/moments where N is the number of external wrenches for the N limbs of the node
	* @return a (6xM), matrix with estimated wrench for the M sensors
	*/
	yarp::sig::Matrix estimateSensorsWrench(const yarp::sig::Matrix &FM, bool afterAttach=false) 
    { return iDynSensorNode::estimateSensorsWrench(FM,afterAttach); }

	
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
	std::string tag;
	void build();

public:

	/**
	* Constructor
	* @param _info some information, ie the node name
	* @param _mode the computation mode for kinematic/wrench using Newton-Euler's formula
	* @param verb verbosity flag
	*/
	iCubUpperTorso(const NewEulMode _mode=DYNAMIC, unsigned int verb=VERBOSE, std::string _tag="");

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
	std::string tag;
	void build();

public:

	/**
	* Constructor
	* @param _info some information, ie the node name
	* @param _mode the computation mode for kinematic/wrench using Newton-Euler's formula
	* @param verb verbosity flag
	*/
	iCubLowerTorso(const NewEulMode _mode=DYNAMIC, unsigned int verb=VERBOSE, std::string _tag = "" );

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
	std::string tag;

public:

	/// pointer to UpperTorso = head + right arm + left arm
	iCubUpperTorso * upperTorso;
	/// pointer to LowerTorso = torso + right leg + left leg
	iCubLowerTorso * lowerTorso;
	
	/// masses and position of the center of mass of the iCub sub-parts
	double whole_mass;
	double lower_mass;
	double upper_mass;
	yarp::sig::Vector whole_COM;
	yarp::sig::Vector upper_COM;
	yarp::sig::Vector lower_COM;

	/**
	* Constructor: build the nodes and creates the whole body
	* @param mode the computation mode: DYNAMIC/STATIC/DYNAMIC_W_ROTOR/DYNAMIC_CORIOLIS_GRAVITY
	* @param verbose the verbosity level: NO_VERBOSE/VERBOSE/MORE_VERBOSE
	*/
	iCubWholeBody(const NewEulMode mode=DYNAMIC, unsigned int verbose=VERBOSE, std::string _tag="");

	/**
	* Standard destructor
	*/
	~iCubWholeBody();

	/**
	* Connect upper and lower torso: this procedure handles the exchange of kinematic and
	* wrench variables between the two parts.
	*/
	void attachLowerTorso(const yarp::sig::Vector &FM_right_leg, const yarp::sig::Vector &FM_left_leg);

    /**
    * Not available yet! We'll do it as soon as possible..
    */
	yarp::sig::Matrix computeJacobian(bool whichTorsoA, unsigned int iChainA, JacobType dirA, bool whichTorsoB, unsigned int iChainB, JacobType dirB)
	{
        iCub::iDyn::notImplemented(VERBOSE,"iCubWholeBody does not have this computeJacobian() yet. Sorry!");
	}

    /**
    * Not available yet! We'll do it as soon as possible..
    */
	yarp::sig::Vector computePose(bool whichTorsoA, unsigned int iChainA, JacobType dirA, bool whichTorsoB, unsigned int iChainB, JacobType dirB, const bool axisRep)
	{
        iCub::iDyn::notImplemented(VERBOSE,"iCubWholeBody does not have this computePose() yet. Sorry! ");	
	}


	/**
	* Performs the computation of the center of mass (COM) of the whole iCub
	* @return true if succeeds, false otherwise
	*/
	bool computeCOM();

	/**
    * Retrieves the result of the last COM computation
	* @param which_part selects the result (e.g: LEFT_LEG, RIGHT_ARM, ALL, etc..)
	* @param COM the computed COM of the selected part
	* @param mass the computed mass of the selected part
	* @return true if succeeds, false otherwise
	*/
	bool getCOM(partEnum which_part, yarp::sig::Vector &COM, double & mass);
};


}// end of namespace iDyn
}//end of namespace iCub

#endif



