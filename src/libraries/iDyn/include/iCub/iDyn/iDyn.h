/**
 * \defgroup iDyn iDyn 
 *    
 * @ingroup icub_libraries
 *  
 * Classes for forward-inverse kinematics and dynamics of serial-links chains
 * of revolute joints and iCub limbs. It is basically an extension of the iKin
 * library, including dynamics. The library also provides a Newton-Euler 
 * based method to compute forces, moments, and joint torques, for each link 
 * and for FT sensor placed anywhere in the chain.
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
 * iDyn is basically an extension of the iKin library, including serial-links
 * chain dynamics. Thus, starting from Links (from iKinLink to iDynLink) the 
 * library provides classes for links, chain of links, and generic limbs.
 * Of course, iCub limbs are included. 
 * In order to compute forces, moments, and joint torques, a Newton-Euler 
 * recursive formulation is provided: in the iDynChain class, the dynamics of
 * the chain (ie computing forces and torques for each link given the dynamic
 information of each link and the joints configuration - pos, vel, acc) can 
 * be computed by means of the classical Newton-Euler algorithm. 
 * It is also possible to retrieve joint torques of iCub limbs with single FT
 * sensor measurements: to this scope, iDynSensor must be used.
 * Furthermore, projection of forces along the chain is supported: to this 
 * scope, iDynTransform must be used.
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows
 *
 * \section example_sec Example
 *
 * Exe
 *
 * \note <b>Release status</b>:  this library is currently under development!
 * Date: first draft 05/2010
 *
 * \author Serena Ivaldi, Matteo Fumagalli
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * 
 **/ 

#ifndef __IDYN_H__
#define __IDYN_H__

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
	class iGenericFrame;
    class iFrameOnLink;
	class iFTransformation;
	class iDynSensorLeg;
	class iDynSensorArm;
	class iCubWholeBody;



/**
* \ingroup iDyn
*
* A base class for defining a Link with standard Denavit-Hartenberg convention, providing kinematic and dynamic information. 
*  
* \note This class implements revolute joints only, as they are 
*       the unique type of joints used in humanoid robotics
*
*/
class iDynLink : public iKin::iKinLink
{
	friend class iDynChain;
	friend class OneLinkNewtonEuler;

protected:

	/// m_i, mass
	double m;	
	///4x4, H^i_{C_i} = R^i_{C_i}, r^i_{i,C_i}, roto-translation matrix from i to Ci, constant
	yarp::sig::Matrix HC;	
	///3x3, I^i_i, inertia matrix, constant
	yarp::sig::Matrix I;	
	/// dq, joint vel
	double dq;
	/// ddq=d2q, joint acc
	double ddq;					
	///1x3, w^i_i				angular velocity
	yarp::sig::Vector w;
	///1x3, dw^i_i				angular acceleration
	yarp::sig::Vector dw;
	///1x3, dw^{i-1}_{m_i}		angular acceleration of rotor
	yarp::sig::Vector dwM;		

	///1x3, dp^i_i				linear velocity of frame i
	yarp::sig::Vector dp;	
	///1x3, dp^i_{C_i}			linear velocity of center of mass C_i
	yarp::sig::Vector dpC;	
	///1x3, d2p=ddp^i_i			linear acceleration of frame i
	yarp::sig::Vector ddp;	
	///1x3, d2p=ddp^i_{C_i}		linear acceleration of center of mass C_i
	yarp::sig::Vector ddpC;		

	///1x3, f^i_i			force
	yarp::sig::Vector F;
	///1x3, mu^i_i			moment
	yarp::sig::Vector Mu;	
	/// tau_i				joint torque
	double Tau;					

	//only including motor dynamic

	///I_{m_i}	rotor inertia
	double Im;	
	///k_{r,i}	inertia constant
	double kr;	
	///F_{v,i}	viscous friction
	double Fv;	
	///F_{s,i}	static friction
	double Fs;					

	/**
	* Default constructor : not implemented
	*/
	iDynLink();

	/**
	* Clone function
	*/
	virtual void clone(const iDynLink &l);  

public:

	//~~~~~~~~~~~~~~~~~~~~~~
	//   set methods
	//~~~~~~~~~~~~~~~~~~~~~~

	/**
	* Set the dynamic parameters of both Link and motor.
    * @param _m is the Link mass
    * @param _HC is the rototranslation matrix from the link frame to the center of mass
    * @param _I is the Inertia matrix
	* @param _kr is the rotor constant
	* @param _Fv is the viscous friction constant
	* @param _Fs is the static friction constant
	* @param _Im is the rotor inertia
	* @return true if operation is successful (ie matrices size is correct), false otherwise
	*/
	 bool setDynamicParameters(const double _m, const yarp::sig::Matrix &_HC, const yarp::sig::Matrix &_I, const double _kr, const double _Fv, const double _Fs, const double _Im);

	/**
	* Set the dynamic parameters of the Link.
    * @param _m is the Link mass
    * @param _HC is the rototranslation matrix from the link frame to the center of mass
    * @param _I is the Inertia matrix
	* @return true if operation is successful (ie matrices size is correct), false otherwise
	*/
	 bool setDynamicParameters(const double _m, const yarp::sig::Matrix &_HC, const yarp::sig::Matrix &_I);

	/**
	* Set the dynamic parameters of the Link if the chain is in a static situation (inertia is null).
    * @param _m is the Link mass
    * @param _HC is the rototranslation matrix from the link frame to the center of mass
	* @return true if operation is successful (ie matrices size is correct), false otherwise
	*/
	 bool setStaticParameters(const double _m, const yarp::sig::Matrix &_HC);
	 
	/**
    * Sets the link inertia
    * @param _I is the inertia matrix 
	* @return true if operation is successful (ie matrices size is correct), false otherwise
    */
	 bool setInertia(const yarp::sig::Matrix &_I);

 	/**
    * Sets the link inertia matrix by setting the 6 inertial parameters
    * @param Ixx is the xx component of the inertia matrix of the link
    * @param Ixy is the xy component of the inertia matrix of the link
    * @param Ixz is the xz component of the inertia matrix of the link
    * @param Iyy is the yy component of the inertia matrix of the link
    * @param Iyz is the yz component of the inertia matrix of the link
    * @param Izz is the zz component of the inertia matrix of the link
    */
	 void setInertia(const double Ixx, const double Ixy, const double Ixz, const double Iyy, const double Iyz, const double Izz);

	/**
    * Sets the link mass
    * @param _m is the mass
    */
	 void setMass(const double _m);

	/**
    * Sets the joint velocity 
    * @param _dteta is the new velocity value 
    * @return current joint velocity (velocity constraints can be evaluated - not done at the moment)
    */
	 double setDAng(const double _dteta);
    
	 /**
    * Sets the joint acceleration 
    * @param _ddteta is the new acceleration value 
    * @return current joint acceleration (acceleration constraints can be evaluated - not done at the moment)
    */
	 double setD2Ang(const double _ddteta);

	 /**
    * Gets the linear velocity of the link 
    * @return current link velocity (the computation is not implemented at the moment)
    */
 	 yarp::sig::Vector getLinVel()  const;

 	 /**
    * Gets the linear velocity of the COM 
    * @return current COM velocity (the computation is not implemented at the moment)
    */
	 yarp::sig::Vector getLinVelC() const;

	 /**
    * Sets the joint angle position, velocity, acceleration. constraints are automatically checked if present.
    * @param _teta is the new position value 
	* @param _dteta is the new velocity value 
    * @param _ddteta is the new acceleration value 
    */
	 void setAngPosVelAcc(const double _teta,const double _dteta,const double _ddteta);

	/**
	* Set the roto-translation matrix from i to COM
    * @param _HC is the roto-translational matrix describing the rotation of the link reference frame w.r.t. the COM and the distance vector bewteen them
	* @return true if operation is successful (ie matrix size is correct), false otherwise
	*/
	 bool setCOM(const yarp::sig::Matrix &_HC);
	/**
	* Set the distance vector from i to COM; the rotation is not modified (set to identity as default)
    * @param _rC is distance vector bewteen the link reference frame and the COM
	* @return true if operation is successful (ie vector size is correct), false otherwise
	*/
	 bool setCOM(const yarp::sig::Vector &_rC);

	/**
	* Set the roto-translation matrix from i to COM, where the rotational part is and identity matrix, and the traslation is specified by the three parameters
    * @param _rCx is the x component of the distance vector of COM wrt the link frame 
    * @param _rCy is the y component of the distance vector of COM wrt the link frame 
    * @param _rCz is the z component of the distance vector of COM wrt the link frame
	*/
	 void setCOM(const double _rCx, const double _rCy, const double _rCz);

	/**
    * Sets the joint force, in the link frame: F^i_i
    * @param _F is the measured/computed force 
	* @return true if operation is successful (ie vector size is correct), false otherwise
    */
	 bool setForce(const yarp::sig::Vector &_F);

 	/**
    * Sets the joint moment, in the link frame: Mu^i_i
    * @param _Mu is the measured/computed moment 
	* @return true if operation is successful (ie vector size is correct), false otherwise
    */
	 bool setMoment(const yarp::sig::Vector &_Mu);

 	/**
    * Sets the joint force and moment, in the link frame: F^i_i , Mu^i_i
    * @param _F is the measured/computed force 
	* @param _Mu is the measured/computed moment 
	* @return true if operation is successful (ie vectors size is correct), false otherwise
    */	 
	 bool setForceMoment(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);

 	/**
    * Sets the joint moment, in the link frame: Tau_i
    * @param _Tau is the measured/computed moment 
    */
	 void setTorque(const double _Tau);


	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   get methods
	 //~~~~~~~~~~~~~~~~~~~~~~
	
     /**
	* Get the inertia matrix
	* @return I the inertia matrix (4x4)
	*/
	 yarp::sig::Matrix		getInertia()	const;
     /**
	* Get the link mass
	* @return m the link mass
	*/
	 double					getMass()		const;
	 double					getIm()			const;
	 double					getKr()			const;
	 double					getFs()			const;
	 double					getFv()			const;
     /**
	* Get the roto-translational matrix describing the COM
	* @return HC the roto-translational matrix of the COM (4x4)
	*/
	 yarp::sig::Matrix		getCOM()		const;
    /**
	* Get the joint velocity
	* @return dq the joint velocity
	*/
	 double					getDAng()		const;
    /**
	* Get the joint acceleration
	* @return ddq =d2q the joint acceleration
	*/
	 double					getD2Ang()		const;
     /**
	* Get the angular velocity of the link
	* @return w the angular velocity of the link (3x1)
	*/
	 yarp::sig::Vector	getW()		const;
    /**
	* Get the angular acceleration of the link
	* @return dW the angular acceleration of the link (3x1)
	*/	
	 yarp::sig::Vector	getdW()		const;
   	/**
	* Get the angular acceleration of the motor
	* @return dWM the angular acceleration of the motor (3x1)
	*/	
	 yarp::sig::Vector	getdWM()	const;
   	/**
	* Get the linear acceleration of the link
	* @return ddpC the linear acceleration of the link (3x1)
	*/	
	 yarp::sig::Vector	getLinAcc()		const;
  	/**
	* Get the linear acceleration of the COM
	* @return ddpC the linear acceleration of the COM (3x1)
	*/	
	 yarp::sig::Vector	getLinAccC()	const;
  	/**
	* Get the link force
	* @return Mu the link force (3x1)
	*/	
	 yarp::sig::Vector	getForce()		const;
 	/**
	* Get the link moment
	* @return Mu the link moment (3x1)
	*/	
	 yarp::sig::Vector	getMoment()		const;
	/**
	* Get the joint torque
	* @return Tau the joint torque
	*/	
	 double				getTorque()		const;
	/**
	* Get the link rotational matrix, from the Denavit-Hartenberg matrix
	* @return R the link rotational matrix (3x3)
	*/	
	yarp::sig::Matrix		getR();
	/**
	* Get the link COM's rotational matrix, from the COM matrix
	* @return R the COM rotational matrix (3x3)
	*/	
	yarp::sig::Matrix		getRC();

	/**
	* Get the link distance vector r, or r*R if projection is specified
	* @param proj true/false enables or disable the projection along the rotation of the link
	* @return r if false, r*R if true
	*/	
	yarp::sig::Vector		getr(bool proj=false);
	/**
	* Get the COM distance vector rC, or rC*R if projection is specified
	* @param proj true/false enables or disable the projection along the rotation of the link
	* @return rC if false, rC*R if true
	*/	
	yarp::sig::Vector		getrC(bool proj=false);

	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   basic functions
	 //~~~~~~~~~~~~~~~~~~~~~~
  
	/**
     * Constructor, with initialization of kinematic data; dynamic data are set to zero, while the roto-translational matrix for the COM 
    * @param _A is the link length
    * @param _D is the link offset 
    * @param _Alpha is the link twist 
    * @param _Offset is the joint angle offset in [-pi,pi]
    * @param _Min is the joint angle lower bound in [-pi,pi] (-pi by default)
    * @param _Max is the joint angle higher bound in [-pi,pi] (pi by default)
    */
	iDynLink(double _A, double _D, double _Alpha, double _Offset, double _Min=-M_PI, double _Max=M_PI);
	 
	/**
     * Constructor, with initialization of kinematic and dynamic data
    * @param _m is the Link mass
    * @param _HC is the rototranslation matrix from the link frame to the center of mass
    * @param _I is the Inertia matrix
    * @param _A is the Link length
    * @param _D is the Link offset 
    * @param _Alpha is the Link twist 
    * @param _Offset is the joint angle offset in [-pi,pi]
    * @param _Min is the joint angle lower bound in [-pi,pi] (-pi by default)
    * @param _Max is the joint angle higher bound in [-pi,pi] (pi by default)
     */
	iDynLink(const double _m, const yarp::sig::Matrix &_HC, const yarp::sig::Matrix &_I, double _A, double _D, double _Alpha, double _Offset, double _Min=-M_PI, double _Max=M_PI);

	/**
    * Constructor, with initialization of kinematic and dynamic data
    * @param _m is the Link mass
    * @param _C is the distance vector of COM wrt the link frame, the orientation of COM is the same of the link
    * @param _I is the Inertia matrix
    * @param _A is the Link length
    * @param _D is the Link offset 
    * @param _Alpha is the Link twist 
    * @param _Offset is the joint angle offset in [-pi,pi]
    * @param _Min is the joint angle lower bound in [-pi,pi] (-pi by default)
    * @param _Max is the joint angle higher bound in [-pi,pi] (pi by default)
     */
	iDynLink(const double _m, const yarp::sig::Vector &_C, const yarp::sig::Matrix &_I, double _A, double _D, double _Alpha, double _Offset, double _Min=-M_PI, double _Max=M_PI);

	/**
    * Constructor, with initialization of kinematic and dynamic data
    * @param _m is the Link mass
    * @param _rCx is the x component of the distance vector of COM wrt the link frame 
    * @param _rCy is the y component of the distance vector of COM wrt the link frame 
    * @param _rCz is the z component of the distance vector of COM wrt the link frame
    * @param Ixx is the xx component of the inertia matrix of the link
    * @param Ixy is the xy component of the inertia matrix of the link
    * @param Ixz is the xz component of the inertia matrix of the link
    * @param Iyy is the yy component of the inertia matrix of the link
    * @param Iyz is the yz component of the inertia matrix of the link
    * @param Izz is the zz component of the inertia matrix of the link
    * @param _A is the Link length
    * @param _D is the Link offset 
    * @param _Alpha is the Link twist 
    * @param _Offset is the joint angle offset in [-pi,pi]
    * @param _Min is the joint angle lower bound in [-pi,pi] (-pi by default)
    * @param _Max is the joint angle higher bound in [-pi,pi] (pi by default)
     */
	iDynLink(const double _m, const double _rCx, const double _rCy, const double _rCz, const double Ixx, const double Ixy, const double Ixz, const double Iyy, const double Iyz, const double Izz, double _A, double _D, double _Alpha, double _Offset, double _Min=-M_PI, double _Max=M_PI);

	/**
     * Copy constructor
     */
	iDynLink(const iDynLink &c);

	/**
     * Set all dynamic parameters to zero
     */
	void zero();
	 
	/**
     * Overload of operator =
     */
	iDynLink &operator=(const iDynLink &c);

};



/**
* \ingroup iDyn
*
* A Base class for defining a Serial Link Chain, using dynamics and kinematics. 
*/
class iDynChain : public iKin::iKinChain
{
	friend class OneChainNewtonEuler;
	friend class iDynInvSensor;
	friend class iDynSensor;
	friend class RigidBodyTransformation;

protected:
	
	/// specifies the 'direction' of recursive computation of kinematics variables (w,dw,d2p): FORWARD, BACKWARD
	ChainIterationMode iterateMode_kinematics;	
	/// specifies the 'direction' of recursive computation of wrenches (F,Mu): FORWARD, BACKWARD
	ChainIterationMode iterateMode_wrench;

	//curr_q = q pos is already in iKinChain
	///q vel
	yarp::sig::Vector curr_dq;	
	///q acc
	yarp::sig::Vector curr_ddq;	

	///pointer to OneChainNewtonEuler class, to be used for computing forces and torques
	OneChainNewtonEuler *NE;

	/**
	* Clone function
	*/
	virtual void clone(const iDynChain &c);

	/**
	* Build chains and lists
	*/	
	virtual void build();

	/**
	* Dispose
	*/	
	virtual void dispose();

	/**
	* Returns a pointer to the ith iDynLink
	* @param i is the Link position
	* @return pointer to ith link
	*/
	iDynLink * refLink(const unsigned int i);


public:

	/**
    * Default constructor 
    */
    iDynChain();

    /**
    * Constructor
    * @param _H0 describes the rigid roto-translation from the root reference frame to points in the 0th frame in the
    *            standard D-H convention
    */
    iDynChain(const yarp::sig::Matrix &_H0);

    /**
    * Creates a new Chain from an already existing Chain object
    * @param c is the Chain to be copied
    */
    iDynChain(const iDynChain &c);

    /**
    * Copies a Chain object into the current one
    * @param c is a reference to an object of type iKinChain
    * @return a reference to the current object
    */
    iDynChain &operator=(const iDynChain &c);

	/**
    * Sets the free joint angles velocity to values of dq[i]
    * @param dq is a vector containing values for each DOF's velocity
    * @return the actual DOF velocity vector (velocity constraints can be evaluated - not done at the moment)
    */
    yarp::sig::Vector setDAng(const yarp::sig::Vector &dq);

	/**
    * Sets the free joint angles acceleration to values of ddq[i]
    * @param ddq is a vector containing values for each DOF's acceleration
    * @return the actual DOF acceleration values (acceleration constraints can be evaluated - not done at the moment)
    */
    yarp::sig::Vector setD2Ang(const yarp::sig::Vector &ddq);

    /**
    * Returns the current free joint angles velocity
    * @return the actual DOF values
    */
    yarp::sig::Vector getDAng();

    /**
    * Returns the current free joint angles acceleration
    * @return the actual DOF values
    */
    yarp::sig::Vector getD2Ang();

    /**
    * Sets the ith joint angle velocity 
    * @param i is the Link position 
    * @param Ang the new angle's velocity value 
    * @return current ith joint angle (velocity constraints can be evaluated - not done at the moment)
    */
    double setDAng(const unsigned int i, double dq);

    /**
    * Sets the ith joint angle acceleration 
    * @param i is the Link position 
    * @param Ang the new angle's acceleration value 
    * @return current ith joint angle (acceleration constraints can be evaluated - not done at the moment)
    */
    double setD2Ang(const unsigned int i, double ddq);

    /**
    * Returns the current angle velocity of ith joint 
    * @param i is the Link position  
    * @return current ith joint angle velocity
    */
    double getDAng(const unsigned int i);

    /**
    * Returns the current angle acceleration of ith joint 
    * @param i is the Link position  
    * @return current ith joint angle acceleration
    */
    double getD2Ang(const unsigned int i);

    /**
    * Returns the link masses as a vector
    * @return a vector with all the link masses
    */
	yarp::sig::Vector getMasses() const;

	/**
    * Set the link masses at once
    * @return true if operation is successful, false otherwise
    */
	 bool setMasses(yarp::sig::Vector _m);

     /**
    * Returns the i-th link mass
    * @return the i-th link mass
    */
 	double getMass(const unsigned int i) const;

	/**
    * Set the i-th link mass
    * @return true if operation is successful, false otherwise
    */
	bool setMass(const unsigned int i, const double _m);

	/**
    * Returns the links forces as a matrix, where the i-th col is the i-th force
    * @return a 3xN matrix with forces, in the form: i-th col = F_i
    */
	yarp::sig::Matrix getForces() const;

	/**
    * Returns the links moments as a matrix, where the i-th col is the i-th moment
    * @return a 3xN matrix with moments, in the form: i-th col = Mu_i
    */
	yarp::sig::Matrix getMoments() const;

	/**
    * Returns the links torque as a vector
    * @return a vector with the torques
    */
	yarp::sig::Vector getTorques() const;

	/**
    * Returns the i-th link force
    * @return the i-th link force
    */
	yarp::sig::Vector getForce(const unsigned int iLink) const;

	/**
    * Returns the i-th link moment
    * @return the i-th link moment
    */
	yarp::sig::Vector getMoment(const unsigned int iLink) const;

	/**
    * Returns the i-th link torque
    * @return the i-th link torque
    */
	double getTorque(const unsigned int iLink) const;

	/**
    * Returns the i-th link linear acceleration
    * @return the i-th link linear acceleration
    */
	yarp::sig::Vector getLinAcc(const unsigned int i) const;

	/**
    * Returns the i-th link linear acceleration of the COM
    * @return the i-th link linear acceleration of the COM
    */
	yarp::sig::Vector getLinAccCOM(const unsigned int i) const;

	/**
	* Set the dynamic parameters of the i-th Link with motor.
    * @param i the i-th Link
    * @param _m is the Link mass
    * @param _HC is the rototranslation matrix from the link frame to the center of mass
    * @param _I is the Inertia matrix
	* @param _kr is the rotor constant
	* @param _Fv is the viscous friction constant
	* @param _Fs is the static friction constant
	* @param _Im is the rotor inertia
	*/
	bool setDynamicParameters(const unsigned int i, const double _m, const yarp::sig::Matrix &_HC, const yarp::sig::Matrix &_I, const double _kr, const double _Fv, const double _Fs, const double _Im);

	/**
	* Set the dynamic parameters of the i-th Link.
    * @param i the i-th Link
    * @param _m is the Link mass
    * @param _HC is the rototranslation matrix from the link frame to the center of mass
    * @param _I is the Inertia matrix
	*/
	 bool setDynamicParameters(const unsigned int i, const double _m, const yarp::sig::Matrix &_HC, const yarp::sig::Matrix &_I);

	/**
	* Set the dynamic parameters of the i-th Link if the chain is in a static situation (inertia is null).
    * @param i the i-th Link
	* @param _m is the Link mass
    * @param _HC is the rototranslation matrix from the link frame to the center of mass
	*/
	 bool setStaticParameters(const unsigned int i, const double _m, const yarp::sig::Matrix &_HC);

	 /**
	 * Prepare for the Newton-Euler recursive computation of forces and torques
	 */
	 void prepareNewtonEuler(const NewEulMode NewEulMode_s=DYNAMIC);

	 /**
	 * Compute forces and torques with the Newton-Euler recursive algorithm: forward
	 * and backward phase are performed, and results are stored in the links; to get
	 * resulting forces and torques, one can call getForces() and getMoments() methods
	 */
	 bool computeNewtonEuler(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0, const yarp::sig::Vector &Fend, const yarp::sig::Vector &Muend);

	 /**
	 * Compute forces and torques with the Newton-Euler recursive algorithm: forward
	 * and backward phase are performed, and results are stored in the links; to get
	 * resulting forces and torques, one can call getForces() and getMoments() methods; 
	 * before calling this method, initNewtonEuler() must be called
	 */
	 bool computeNewtonEuler();

	/**
	* Initialize the Newton-Euler method by setting the base (virtual link) velocity and accelerations ( w0, dw0 and ddp0 )
	* and the final (virtual link) forces and moments Fend and Muend
	*/
	 bool initNewtonEuler(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0, const yarp::sig::Vector &Fend, const yarp::sig::Vector &Muend);
	
	/**
	* Initialize the Newton-Euler method by setting the base (virtual link) velocity and accelerations ( w0, dw0 and ddp0 )
	* and the final (virtual link) forces and moments Fend and Muend all to zero
	*/
	 bool initNewtonEuler();

	 /**
	* Set the computation mode for Newton-Euler (static/dynamic/etc)
	*/
	 void setModeNewtonEuler(const NewEulMode NewEulMode_s=DYNAMIC);

	/**
    * Returns the links forces as a matrix, where the i-th col is the i-th force
    * @return a 3x(N+2) matrix with forces, in the form: i-th col = F_i
    */
	yarp::sig::Matrix getForcesNewtonEuler() const;

	/**
    * Returns the links moments as a matrix, where the i-th col is the i-th moment
    * @return a 3x(N+2) matrix with moments, in the form: i-th col = Mu_i
    */
	yarp::sig::Matrix getMomentsNewtonEuler() const;

	/**
    * Returns the links torque as a vector
    * @return a Nx1 vector with the torques
    */
	yarp::sig::Vector getTorquesNewtonEuler() const;

	/**
	* Returns the end effector force-moment as a single (6x1) vector
	* @return a (6x1) vector, in the form 0:2=F 3:5=Mu 
	*/
	yarp::sig::Vector getForceMomentEndEff() const;

	/**
    * Set the iteration direction during recursive computation of kinematics variables
	* (w,dw,d2p,d2pC). Default is FORWARD, which is also set in the constructor.
    * @param _iterateMode_kinematics FORWARD/BACKWARD
    */
	void setIterModeKinematic(const ChainIterationMode _iterateMode_kinematics = FORWARD );

	/**
    * Set the iteration direction during recursive computation of wrench variables
	* (F,Mu,Tau). Default is BACKWARD, which is also set in the constructor.
    * @param _iterateMode_wrench FORWARD/BACKWARD
    */
	void setIterModeWrench(const ChainIterationMode _iterateMode_wrench = BACKWARD );

	/**
    * Set the computation mode during recursive computation of kinematics (w,dw,d2p,d2pC)
	* and wrench variables(F,Mu,Tau). The mode is NE_KIN_WRE_kw, where the suffix 'kw' identifies 
	* the modes for the kinematics ('k') and wrench ('w') computations: 
	* {FF,FB,BF,BB} where the F stands for FORWARD and B for BACKWARD. 
	* Default mode is KINFWD_WREBWD, which sets Kinematics=FORWARD and Wrench=BACKWARD. 
	* @param mode NE_KIN_WRE_{FF,FB,BF,BB}
    */
	void setIterMode(const ChainComputationMode mode = KINFWD_WREBWD);

	/**
    * Get the iteration direction during recursive computation of kinematics variables
	* (w,dw,d2p,d2pC). 
    * @return iterateMode_kinematics
    */
	ChainIterationMode getIterModeKinematic() const;

	/**
    * Get the iteration direction during recursive computation of wrench variables
	* (F,Mu,Tau).
    * @return iterateMode_wrench
    */
	ChainIterationMode getIterModeWrench() const;

	/**
	* Calls the proper method to compute kinematics variables: either
	* ForwardKinematicFromBase() or BackwardKinematicFromEnd(). This method 
	* is protected and it is used by iDynSensor and iDynNode for the Kinematics computations.
	*/
	void computeKinematicNewtonEuler();

	/**
	* Calls the proper method to compute wrench variables: either
	* BackwardWrenchFromEnd() or ForwardWrenchFromBase(). This method 
	* is protected and it is used by iDynSensor and iDynNode for the Wrench computations.
	*/
	void computeWrenchNewtonEuler();

	/**
	* Calls the proper method to set kinematics variables in OneChainNewtonEuler: either
	* initKinematicBase() or initKinematicEnd(). This method 
	* is protected and it is used by RigidBodyTransformation for setting the Kinematics 
	* variables.
	*/
	bool initKinematicNewtonEuler(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0);

	/**
	* Calls the proper method to set wrench variables in OneChainNewtonEuler: either
	* initKinematicBase() or initKinematicEnd(). This method 
	* is protected and it is used by RigidBodyTransformation for setting the Kinematics 
	* variables.
	*/
	bool initWrenchNewtonEuler(const yarp::sig::Vector &Fend, const yarp::sig::Vector &Muend);

	/**
	* Calls the proper method to get kinematics variables in OneChainNewtonEuler either
	* in the base or in the final link. This method is used by RigidBodyTransformation 
	* for setting the kinematics variables.
	*/
	void getKinematicNewtonEuler( yarp::sig::Vector &w, yarp::sig::Vector &dw, yarp::sig::Vector &ddp);
	void getFrameKinematic(unsigned int i, yarp::sig::Vector &w, yarp::sig::Vector &dw, yarp::sig::Vector &ddp);
	void getFrameWrench(unsigned int i, yarp::sig::Vector &F, yarp::sig::Vector &Mu);
	/**
	* Calls the proper method to get wrench variables in OneChainNewtonEuler either
	* in the base or in the final link. This method is used by RigidBodyTransformation 
	* for setting the wrench variables.
	*/
	void getWrenchNewtonEuler( yarp::sig::Vector &F,  yarp::sig::Vector &Mu);
	
	unsigned int      getNTOT()             { return NE->nEndEff; }

	/**
    * Destructor. 
    */
    virtual ~iDynChain();

};






/**
* \ingroup iDyn
*
* A class for defining a generic Limb within the iDyn framework.
*
* Note:
* Since iKinLimb is derived with protected modifier from iKinChain in order to make some methods hidden
* to the user such as addLink, rmLink and so on, all the remaining public methods have been
* redeclared and simply inherited.
* iDynLimb follows the same rule of iKinLimb, therefore the methods must be redeclared.
*/
class iDynLimb : protected iDynChain
{
protected:
    std::deque<iDynLink*> linkList;
    std::string           type;
    bool                  configured;

    virtual void allocate(const std::string &_type);
    virtual void clone(const iDynLimb &limb);
    virtual void dispose();

public:
    /**
    * Default constructor: right arm is default
    */
    iDynLimb();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and "right" limb
    */
    iDynLimb(const std::string &_type);

    /**
    * Creates a new Limb from an already existing Limb object.
    * @param limb is the Limb to be copied.
    */
    iDynLimb(const iDynLimb &limb);

    /**
    * Creates a new Limb from a list of properties wherein links parameters are specified. 
    * @param option is the list of links properties. 
    * @see fromProperty
    */
    iDynLimb(const yarp::os::Property &option);

    /**
    * Initializes the Limb from a list of properties wherein links 
    * parameters are specified. 
    * @param option is the list of links properties. 
    *  
    * @note Available options are: 
    *  
    * \b type <string>: specifies the limb handedness [left/right] 
    *    (default=right).
    *  
    * \b H0 <list of 4x4 doubles (per rows)>: specifies the rigid 
    *    roto-translation matrix from the root reference frame to
    *    the 0th frame (default=eye(4,4)).
    *  
    * \b numLinks <int>: specifies the expected number of links.
    *  
    * \b A <double>: specifies the link length [m] (default=0.0).
    *  
    * \b D <double>: specifies the link offset [m] (default=0.0).
    *  
    * \b alpha <double>: specifies the link twist [deg] 
    *    (default=0.0).
    *  
    * \b offset <double>: specifies the joint angle offset [deg] 
    *    (default=0.0).
    *  
    * \b min <double>: specifies the joint angle lower bound [deg]
    *    (default=0.0).
    *  
    * \b max <double>: specifies the joint angle upper bound [deg] 
    *    (default=0.0).
    *  
    * \b blocked <double>: blocks the link at the specified value 
    *    [deg] (default=released).
    *  
    * @note The list should look like as the following: 
    *  
    * @code 
    * type right 
    * H0 (1.0 2.0 3.0 ...) 
    * numLinks 4 
    * link_0 (option1 value1) (option2 value2) ... 
    * link_1 (option1 value1) ... 
    * ... 
    * @endcode 
    */
    bool fromLinksProperties(const yarp::os::Property &option);

    /**
    * Checks if the limb has been properly configured.
    * @return true iff correctly coinfigured.
    */
    bool isValid() { return configured; }

    /**
    * Copies a Limb object into the current one.
    * @param limb is a reference to an object of type iKinLimb.
    * @return a reference to the current object.
    */
    iDynLimb &operator=(const iDynLimb &limb);

    /**
    * Returns a pointer to the Limb seen as Chain object. 
    * Useful to to operate on the Links of Limb.
    * @return a pointer to a Chain object with the same Links of 
    *         Limb.
    */
	iDynChain *asChain() { return static_cast<iDynChain*>(this); }

    /**
    * Returns the Limb type as a string. 
    * @return the type as a string {"left", "right"}.
    */
    std::string getType() { return type; }

	/**
    * Destructor. 
    */
    virtual ~iDynLimb();


	// base methods - see iKin

    unsigned int      getN() const                                                    { return N;                                   }
	unsigned int      getNTOT()														  { return iDynChain::getNTOT();                }
	unsigned int      getDOF() const                                                  { return DOF;                                 }
    bool              blockLink(const unsigned int i, double Ang)                     { return iDynChain::blockLink(i,Ang);         }
    bool              blockLink(const unsigned int i)                                 { return iDynChain::blockLink(i);             }
    bool              setBlockingValue(const unsigned int i, double Ang)              { return iDynChain::setBlockingValue(i,Ang);  }
    bool              releaseLink(const unsigned int i)                               { return iDynChain::releaseLink(i);           }
    void              setAllConstraints(bool _constrained)                            { iDynChain::setAllConstraints(_constrained); }
    void              setConstraint(unsigned int i, bool _constrained)                { iDynChain::setConstraint(i,_constrained);   }
    bool              getConstraint(unsigned int i)                                   { return iDynChain::getConstraint(i);         }
    void              setAllLinkVerbosity(unsigned int _verbose)                      { iDynChain::setAllLinkVerbosity(_verbose);   }
    void              setVerbosity(unsigned int _verbose)                             { iDynChain::setVerbosity(_verbose);          }
    unsigned int      getVerbosity()                                                  { return iDynChain::getVerbosity();           }

	// kinematics methods - see iKin

	yarp::sig::Matrix getH(const unsigned int i, const bool allLink=false)            { return iDynChain::getH(i,allLink);          }
    yarp::sig::Matrix getH()                                                          { return iDynChain::getH();                   }
    yarp::sig::Matrix getH(const yarp::sig::Vector &q)                                { return iDynChain::getH(q);                  }
    yarp::sig::Vector Pose(const unsigned int i, const bool axisRep=true)             { return iDynChain::Pose(i,axisRep);          }
    yarp::sig::Vector EndEffPose(const bool axisRep=true)                             { return iDynChain::EndEffPose(axisRep);      }
    yarp::sig::Vector EndEffPose(const yarp::sig::Vector &q, const bool axisRep=true) { return iDynChain::EndEffPose(q,axisRep);    }
    yarp::sig::Matrix AnaJacobian(unsigned int col=3)                                 { return iDynChain::AnaJacobian(col);         }
    yarp::sig::Matrix AnaJacobian(const yarp::sig::Vector &q, unsigned int col=3)     { return iDynChain::AnaJacobian(q,col);       }
    yarp::sig::Matrix GeoJacobian()                                                   { return iDynChain::GeoJacobian();            }
    yarp::sig::Matrix GeoJacobian(const yarp::sig::Vector &q)                         { return iDynChain::GeoJacobian(q);           }
    yarp::sig::Vector Hessian_ij(const unsigned int i, const unsigned int j)          { return iDynChain::Hessian_ij(i,j);          }
    void              prepareForHessian()                                             { iDynChain::prepareForHessian();             }
    yarp::sig::Vector fastHessian_ij(const unsigned int i, const unsigned int j)      { return iDynChain::fastHessian_ij(i,j);      }

	// q, dq, d2q

    yarp::sig::Vector setAng(const yarp::sig::Vector &_q)                             { return iDynChain::setAng(_q);               }
    yarp::sig::Vector getAng()                                                        { return iDynChain::getAng();                 }
    double            setAng(const unsigned int i, double _q)                         { return iDynChain::setAng(i,_q);             }
    double            getAng(const unsigned int i)                                    { return iDynChain::getAng(i);                }

	yarp::sig::Vector setDAng(const yarp::sig::Vector &_dq)                           { return iDynChain::setDAng(_dq);             }
    yarp::sig::Vector getDAng()                                                       { return iDynChain::getDAng();                }
    double            setDAng(const unsigned int i, double _dq)                       { return iDynChain::setDAng(i,_dq);           }
    double            getDAng(const unsigned int i)                                   { return iDynChain::getDAng(i);               }

	yarp::sig::Vector setD2Ang(const yarp::sig::Vector &_ddq)                         { return iDynChain::setD2Ang(_ddq);           }
    yarp::sig::Vector getD2Ang()                                                      { return iDynChain::getD2Ang();               }
    double            setD2Ang(const unsigned int i, double _ddq)                     { return iDynChain::setD2Ang(i,_ddq);         }
    double            getD2Ang(const unsigned int i)                                  { return iDynChain::getD2Ang(i);              }

	// COM

	yarp::sig::Vector getLinAcc(const unsigned int i)								  { return iDynChain::getLinAcc(i);				}
	yarp::sig::Vector getLinAccCOM(const unsigned int i)							  { return iDynChain::getLinAccCOM(i);			}

	// force, moment, torque							  

	yarp::sig::Matrix getForces() const												  { return iDynChain::getForces();				}
	yarp::sig::Matrix getMoments() const											  { return iDynChain::getMoments();				}
	yarp::sig::Vector getTorques() const											  { return iDynChain::getTorques();				}
	yarp::sig::Vector getForce(const unsigned int iLink) const						  { return iDynChain::getForce(iLink);			}
	yarp::sig::Vector getMoment(const unsigned int iLink) const						  { return iDynChain::getMoment(iLink);			}
	double			  getTorque(const unsigned int iLink) const						  { return iDynChain::getTorque(iLink);			}

	// mass

	yarp::sig::Vector getMasses() const												  { return iDynChain::getMasses();				}
	bool			  setMasses(yarp::sig::Vector _m)								  { return iDynChain::setMasses(_m);			}
	double			  getMass(const unsigned int i) const							  { return iDynChain::getMass(i);				}
	bool			  setMass(const unsigned int i, const double _m)				  { return iDynChain::setMass(i,_m);			}

	// dynamic parameters

	bool setDynamicParameters(const unsigned int i, const double _m, const yarp::sig::Matrix &_HC, const yarp::sig::Matrix &_I, const double _kr, const double _Fv, const double _Fs, const double _Im) 
	{ return iDynChain::setDynamicParameters(i,_m,_HC,_I,_kr,_Fv,_Fs,_Im); }

	bool setDynamicParameters(const unsigned int i, const double _m, const yarp::sig::Matrix &_HC, const yarp::sig::Matrix &_I)
	{ return iDynChain::setDynamicParameters(i,_m,_HC,_I); }

	bool setStaticParameters(const unsigned int i, const double _m, const yarp::sig::Matrix &_HC)
	{ return iDynChain::setStaticParameters(i,_m,_HC); }

	// methods for Newton-Euler computation

	bool				computeNewtonEuler()										{ return iDynChain::computeNewtonEuler(); }
	void				setModeNewtonEuler(const NewEulMode NewEulMode_s=DYNAMIC)	{ iDynChain::setModeNewtonEuler(NewEulMode_s); }
	void				prepareNewtonEuler(const NewEulMode NewEulMode_s=DYNAMIC)	{ iDynChain::prepareNewtonEuler(NewEulMode_s); }
	yarp::sig::Matrix	getForcesNewtonEuler() const								{ return iDynChain::getForcesNewtonEuler();}
	yarp::sig::Matrix	getMomentsNewtonEuler() const								{ return iDynChain::getMomentsNewtonEuler();}
	yarp::sig::Vector	getTorquesNewtonEuler() const								{ return iDynChain::getTorquesNewtonEuler();}
	void				computeKinematicNewtonEuler() 								{ iDynChain::computeKinematicNewtonEuler();}
	void				computeWrenchNewtonEuler()									{ iDynChain::computeWrenchNewtonEuler();}

	bool computeNewtonEuler(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0, const yarp::sig::Vector &Fend, const yarp::sig::Vector &Muend)
	{ return iDynChain::computeNewtonEuler(w0,dw0,ddp0,Fend,Muend); }

	bool initNewtonEuler(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0, const yarp::sig::Vector &Fend, const yarp::sig::Vector &Muend)
	{ return iDynChain::initNewtonEuler(w0,dw0,ddp0,Fend,Muend); }

	bool initKinematicNewtonEuler(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0)
	{return iDynChain::initKinematicNewtonEuler(w0, dw0, ddp0);}
	 
	bool initWrenchNewtonEuler(const yarp::sig::Vector &Fend, const yarp::sig::Vector &Muend)
	{return iDynChain::initWrenchNewtonEuler(Fend,Muend);}

	void getKinematicNewtonEuler( yarp::sig::Vector &w, yarp::sig::Vector &dw, yarp::sig::Vector &ddp) 
	{iDynChain::getKinematicNewtonEuler(w,dw,ddp);}
	 
	void getFrameKinematic(unsigned int i, yarp::sig::Vector &w, yarp::sig::Vector &dw, yarp::sig::Vector &ddp)
	{iDynChain::getFrameKinematic(i,w,dw,ddp);}
	 
	void getFrameWrench(unsigned int i, yarp::sig::Vector &F, yarp::sig::Vector &Mu)
	{iDynChain::getFrameWrench(i,F,Mu);}

	void getWrenchNewtonEuler( yarp::sig::Vector &F, yarp::sig::Vector &Mu) 
	{iDynChain::getWrenchNewtonEuler(F,Mu);}





};


/**
* \ingroup iDyn
*
* A class for defining the 7-DOF iCub Arm in the iDyn framework
*/
class iCubArmDyn : public iDynLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubArmDyn();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" arm
    */
    iCubArmDyn(const std::string &_type, const ChainComputationMode _mode=KINFWD_WREBWD);

    /**
    * Creates a new Arm from an already existing Arm object.
    * @param arm is the Arm to be copied.
    */
    iCubArmDyn(const iCubArmDyn &arm);
};



/**
* \ingroup iDyn
*
* A class for defining the 7-DOF iCub Arm in the iDyn framework
*/
class iCubArmNoTorsoDyn : public iDynLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubArmNoTorsoDyn();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" arm
    */
    iCubArmNoTorsoDyn(const std::string &_type, const ChainComputationMode _mode=KINFWD_WREBWD);

    /**
    * Creates a new Arm from an already existing Arm object.
    * @param arm is the Arm to be copied.
    */
    iCubArmNoTorsoDyn(const iCubArmNoTorsoDyn &arm);
};


/**
* \ingroup iDyn
*
* A class for defining the 3-DOF iCub Torso in the iDyn framework
*/
class iCubTorsoDyn : public iDynLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubTorsoDyn();

    /**
    * Constructor. Note that the type is not influential at the moment, 
	* but the distinction could be useful for future developments.
    * @param _type is a string to discriminate between "upper" and "lower" torso
    */
    iCubTorsoDyn(const std::string &_type, const ChainComputationMode _mode=KINFWD_WREBWD);

    /**
    * Creates a new Torso from an already existing Torso object.
    * @param torso is the Torso to be copied.
    */
    iCubTorsoDyn(const iCubTorsoDyn &torso);
};


/**
* \ingroup iDyn
*
* A class for defining the 6-DOF iCub Leg
*/
class iCubLegDyn : public iDynLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubLegDyn();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" leg
    */
    iCubLegDyn(const std::string &_type,const ChainComputationMode _mode=KINFWD_WREBWD);

    /**
    * Creates a new Leg from an already existing Leg object.
    * @param leg is the Leg to be copied.
    */
    iCubLegDyn(const iCubLegDyn &leg);
};


/**
* \ingroup iDyn
*
* A class for defining the 5-DOF iCub Eye
*/
class iCubEyeDyn : public iDynLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubEyeDyn();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" eye
    */
    iCubEyeDyn(const std::string &_type,const ChainComputationMode _mode=KINFWD_WREBWD);

    /**
    * Creates a new Eye from an already existing Eye object.
    * @param eye is the Eye to be copied.
    */
    iCubEyeDyn(const iCubEyeDyn &eye);
};


/**
* \ingroup iDyn
*
* A class for defining the 5-DOF iCub Eye with the root 
* reference frame attached to the neck. 
*/
class iCubEyeNeckRefDyn : public iCubEyeDyn
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubEyeNeckRefDyn();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" eye
    */
    iCubEyeNeckRefDyn(const std::string &_type, const ChainComputationMode _mode=KINFWD_WREBWD);

    /**
    * Creates a new Eye from an already existing Eye object.
    * @param eye is the Eye to be copied.
    */
    iCubEyeNeckRefDyn(const iCubEyeNeckRefDyn &eye);
};


/**
* \ingroup iDyn
*
* A class for defining the 6-DOF Inertia Sensor Kinematics
*/
class iCubInertialSensorDyn : public iDynLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubInertialSensorDyn(const ChainComputationMode _mode=KINBWD_WREBWD);

    /**
    * Creates a new Inertial Sensor from an already existing object.
    * @param sensor is the object to be copied.
    */
    iCubInertialSensorDyn(const iCubInertialSensorDyn &sensor);
};

/**
* \ingroup iDyn
*
* A class for defining the 3-DOF Inertia Sensor Kinematics
*/
class iCubNeckInertialDyn : public iDynLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubNeckInertialDyn(const ChainComputationMode _mode=KINBWD_WREBWD);

    /**
    * Creates a new Inertial Sensor from an already existing object.
    * @param sensor is the object to be copied.
    */
    iCubNeckInertialDyn(const iCubInertialSensorDyn &sensor);
};


}//end namespace
#endif
