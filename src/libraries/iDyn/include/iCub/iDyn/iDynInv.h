/**
 * \defgroup RecursiveNewtonEuler RecursiveNewtonEuler 
 *    
 * @ingroup iDyn
 *  
 * Classes for force/torque computation using Newton-Euler recursive formulation,
 * both in the forward/backward sense.
 * 
 * \section intro_sec Description
 * 
 * OneLinkNewtonEuler is a class that provides Newton-Euler recursive formulation,
 * for the computation of forces, moments and joint torques in a kinematic chain; 
 * both classical and "inverse" formulation are included; the classe is the basis 
 * for the computation of torques given force/torque sensor measurements, given 
 * any sensor anywhere in the dynamic/kinematic chain. BaseLink-, FinalLink- and
 * SensorLink-NewtonEuler are derived class, to be used for computations.
 * OneChainNewtonEuler defines a chain of OneLinkNewtonEuler corresponding to an
 * iDynChain: it constructs a chain with a BaseLink, N OneLinks corresponding to 
 * the N iDynLinks of the iDynChain, and a FinalLink. BaseLink and FinalLink are 
 * used only to initialize the two phases of the Newton-Euler algorithm, thus 
 * they don't have a corresponding dynamic link but a virtual one.
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows
 *
 *
 * \author Serena Ivaldi, Matteo Fumagalli
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * 
 **/ 

#ifndef __IDYNINV_H__
#define __IDYNINV_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/ctrlMath.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iDyn/iDyn.h>
#include <deque>
#include <string>

namespace iDyn
{

// Newton-Euler types
// useful for iDyn, iDynInv, iDynFwd
enum NewEulMode {STATIC,DYNAMIC,DYNAMIC_W_ROTOR,DYNAMIC_CORIOLIS_GRAVITY};
const std::string NewEulMode_s[4] = {"static","dynamic","dynamic with motor/rotor","dynamic with only Coriolis and gravitational terms"};

// Kinematic and Wrench modes
enum ChainIterationMode { FORWARD, BACKWARD };
enum ChainComputationMode { KINFWD_WREFWD, KINFWD_WREBWD, KINBWD_WREFWD, KINBWD_WREBWD };
const std::string ChainIterationMode_s[2] = {"Forward (Base To End)","Backward (End To Base)"};
const std::string ChainComputationMode_s[4] = {"Kinematic Forward - Wrench Forward","Kinematic Forward - Wrench Backward","Kinematic Backward - Wrench Forward","Kinematic Backward - Wrench Backward"};

// verbosity levels
// useful for all classes
enum VerbosityLevel{ NO_VERBOSE, VERBOSE, MORE_VERBOSE};


	class iDynLink;
	class iDynChain;
	class iDynLimb;
	class iCubArmDyn;
	class iCubLegDyn;
	class iCubEyeDyn;
	class iCubEyeNeckRefDyn;
	class iCubInertialSensorDyn;
	class iFakeDyn;
	class iFakeDyn2GdL;
	class iDynSensor;
	class iDynSensorLeg;
	class iDynSensorArm;
	class iGenericFrame;
    class iFrameOnLink;
	class iFTransformation;



/**
* \ingroup RecursiveNewtonEuler
*
* A base class for computing forces and torques in a serial link chain
*/
class OneLinkNewtonEuler
{
protected:

	/// STATIC/DYNAMIC/DYNAMIC_W_ROTOR/DYNAMIC_CORIOLIS_GRAVITY
	NewEulMode mode;	
	///info or useful notes
	std::string	info;
	///verbosity flag
	unsigned int verbose;
	///z0=[0 0 1]'
	yarp::sig::Vector z0;
	///z^{i-1}_{m_{i}}		versor rotating solidally with link i, projected in frame i ==>> constant
	yarp::sig::Vector zm;	
	///the corresponding iDynLink 
	iDyn::iDynLink *link;	

	//~~~~~~~~~~~~~~~~~~~~~~
	//   set methods  
	//~~~~~~~~~~~~~~~~~~~~~~

	/**
	* Set the OneLink force: either the corresponding iDynLink force, or the one declared as member
	* in the child classes derived from OneLink, such as SensorLink
	* @param _F a (3x1) vector of forces
	* @return true if size is correct, false otherwise
	*/
	virtual bool setForce(const yarp::sig::Vector &_F);

	/**
	* Set the OneLink moment: either the corresponding iDynLink moment, or the one declared as member
	* in the child classes derived from OneLink, such as SensorLink
	* @param _Mu a 3x1 vector of moments
	* @return true if size is correct, false otherwise
	*/
	virtual bool setMoment(const yarp::sig::Vector &_Mu);

	/**
	* Set the OneLink torque, ie the corresponding iDynLink joint torque (nothing
	* in the child classes derived from OneLink, such as SensorLink)
	* @param _Tau a real torque value
	*/
	virtual void setTorque(const double _Tau);

	/**
	* Set the OneLink angular velocity (w), ie the corresponding iDynLink angular 
	* velocity (w) (in the child classes derived from OneLink, it depends)
	* @param _w angular velocity
	*/
	virtual bool setAngVel(const yarp::sig::Vector &_w);
	
	/**
	* Set the OneLink angular acceleration (dw), ie the corresponding iDynLink angular 
	* acceleration (dw) (in the child classes derived from OneLink, it depends)
	* @param _dw angular acceleration
	*/
	virtual bool setAngAcc(const yarp::sig::Vector &_dw);
	/**
	* Set the OneLink linear acceleration (ddp), ie the corresponding iDynLink linear 
	* acceleration (ddp) (in the child classes derived from OneLink, it depends)
	* @param _ddp linear acceleration
	*/
	virtual bool setLinAcc(const yarp::sig::Vector &_ddp);
	/**
	* Set the OneLink linear acceleration of the COM (ddpC), ie the corresponding 
	* iDynLink linear acceleration of the COM (ddpC) (nothing in the child classes 
	* derived from OneLink, except for SensorLink)
	* @param _ddpC linear acceleration of the COM
	*/
	virtual bool setLinAccC(const yarp::sig::Vector &_ddpC);
	/**
	* Set the OneLink angular acceleration of the motor (dwM), ie the corresponding 
	* iDynLink angular acceleration of the COM (dwM) (nothing in the child classes
	* derived from OneLink)
	* @param _dwM angular acceleration of the motor
	*/
	virtual bool setAngAccM(const yarp::sig::Vector &_dwM);

	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   core computation
	 //~~~~~~~~~~~~~~~~~~~~~~

	/**
     * [Forward Newton-Euler] compute angular velocity of the link
	  * @param prev the OneLinkNewtonEuler class of the previous link
     */
	 void computeAngVel( OneLinkNewtonEuler *prev);

	 /**
     * [Forward Newton-Euler] compute angular velocity of the previous link
	 * frame
	  * @param prev the OneLinkNewtonEuler class of the previous link
     */
	 void computeAngVelBackward( OneLinkNewtonEuler *prev);

	/**
     * [Forward Newton-Euler] compute angular acceleration of the link
	  * @param prev the OneLinkNewtonEuler class of the previous link
     */
	 void computeAngAcc( OneLinkNewtonEuler *prev);

	 /**
     * [Forward Newton-Euler] compute angular acceleration of the previous link frame
	  * @param prev the OneLinkNewtonEuler class of the previous link
     */
	 void computeAngAccBackward( OneLinkNewtonEuler *prev);

	/**
     * [Forward Newton-Euler] compute linear acceleration of the reference frame of the link
	  * @param prev the OneLinkNewtonEuler class of the previous link
     */
	 void computeLinAcc( OneLinkNewtonEuler *prev);
	 
	/**
     * [Forward Newton-Euler] compute linear acceleration of the reference frame of the previous link
	  * @param prev the OneLinkNewtonEuler class of the previous link
     */
	 void computeLinAccBackward( OneLinkNewtonEuler *prev);

	/**
     * [Forward Newton-Euler] compute linear acceleration of the center of mass
	  * @return ddpC
     */
	 void computeLinAccC();

	/**
     * [Forward Newton-Euler] compute angular acceleration of the rotor
	  * @param prev the OneLinkNewtonEuler class of the previous link
	  * @return dwM
     */
	 void computeAngAccM( OneLinkNewtonEuler *prev);

 	/**
     * [Backward Newton-Euler] compute force from the following link
	  * @param next the OneLinkNewtonEuler class of the following link
     */
	 void computeForceBackward( OneLinkNewtonEuler *next);

  	/**
     * [Inverse Newton-Euler] compute force from the previous link
	  * @param prev the OneLinkNewtonEuler class of the previous link
     */
	 void computeForceForward( OneLinkNewtonEuler *prev);

  	/**
     * [Backward Newton-Euler] compute moment from the following link
	  * @param next the OneLinkNewtonEuler class of the following link
     */
	 void computeMomentBackward( OneLinkNewtonEuler *next);

  	/**
     * [Inverse Newton-Euler] compute moment from the previous link
	  * @param prev the OneLinkNewtonEuler class of the previous link
     */
	 void computeMomentForward( OneLinkNewtonEuler *prev);


public:

	/**
    * Default constructor 
    */
	OneLinkNewtonEuler(iDyn::iDynLink *dlink=NULL);
  
	/**
     * Constructor, with initialization of some data
     */
	OneLinkNewtonEuler(const NewEulMode _mode, unsigned int verb = NO_VERBOSE, iDyn::iDynLink *dlink=NULL);
	 	 
	/**
     * Set everything to zero; R is set to an identity matrix
     */
	void zero();

	/**
     * Virtual method to set the frame as the base one: this is useful to initialize the forward phase of
	 * Newton-Euler's method. The BaseLink class is used to this scope.
     */
	virtual bool setAsBase(const yarp::sig::Vector &_w, const yarp::sig::Vector &_dw, const yarp::sig::Vector &_ddp);
	virtual bool setAsBase(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);

	/**
     * Set the frame as the final one: this is useful to initialize the backward phase of
	  * Newton-Euler's method, by setting F and Mu; R is an identity matrix
	  * @param _F the final force
	  * @param _Mu the final moment
     */
	virtual bool setAsFinal(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);

	/**
     * Set the frame as the final one: this is useful to initialize the backward phase of
	  * Euler's method, by setting w, dw and ddp; R is an identity matrix
	  * @param _w the final angular velocity
	  * @param _dw the final angular acceleration
	  * @param _ddp the final linear acceleration
     */
	virtual bool setAsFinal(const yarp::sig::Vector &_w, const yarp::sig::Vector &_dw, const yarp::sig::Vector &_ddp);

	/**
     * Set measured force and moment in a 'sensor' frame: this is useful to initialize the forward phase of the Inverse
	  * Newton-Euler's method, by setting F and Mu, measured by the corresponding F/T sensor
	  * @param _F the sensor force
	  * @param _Mu the sensor moment
     */
	virtual bool setMeasuredFMu(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);

	/**
     * Set measured torque in a joint torque sensor frame	  
	 * @param _Tau the sensor torque
     */
	virtual bool setMeasuredTorque(const double _Tau);
	
	/**
     * Useful to print some information..
     */
	virtual std::string toString() const;

 
	//~~~~~~~~~~~~~~~~~~~~~~
	//   set methods
	//~~~~~~~~~~~~~~~~~~~~~~

	/**
	* Set the verbosity level of comments during operations
	* @param verb, a boolean flag
	*/
	 void setVerbose(unsigned int verb = VERBOSE);
 	/**
	* Set the operation mode (static,dynamic etc)
	* @param _mode the NewEulMode defining the type of operations
	*/
	 void setMode(const NewEulMode _mode);
 	/**
	* Set the zM vector
	* @param _zm a (3x1) vector zM
	* @return true if size is correct, false otherwise
	*/
	 bool setZM(const yarp::sig::Vector &_zm);
 	/**
	* Set some information about this OneLink class
	* @param _info a string
	*/
	 void setInfo(const std::string &_info);


	 //~~~~~~~~~~~~~~~~~~~~~~
	 //   get methods
	 //~~~~~~~~~~~~~~~~~~~~~~

  	/**
	* @return the operation mode (static, dynamic etc)
	*/
	 NewEulMode				getMode()		const;
  	/**
	* @return zM vector (3x1)
	*/
	 yarp::sig::Vector		getZM()			const;
 	/**
	* @return the info string
	*/
	 std::string			getInfo()		const;
 	/**
	* @return w = angular velocity (3x1)
	*/
	 virtual yarp::sig::Vector	getAngVel()		const;
 	/**
	* @return dw = angular acceleration (3x1)
	*/
	 virtual yarp::sig::Vector	getAngAcc()		const;
 	/**
	* @return dwM = angular acceleration of the rotor (3x1)
	*/
	 virtual yarp::sig::Vector	getAngAccM()	const;
 	/**
	* @return ddp = d2p = linear acceleration (3x1)
	*/
	 virtual yarp::sig::Vector	getLinAcc()		const;
 	/**
	* @return ddpC = d2pC = linear acceleration of the center of mass (3x1)
	*/
	 virtual yarp::sig::Vector	getLinAccC()	const;
  	/**
	* @return F = force (3x1)
	*/
	 virtual yarp::sig::Vector	getForce()		const;
   	/**
	* @return Mu = moment (3x1)
	*/
	 virtual yarp::sig::Vector	getMoment()		const;
   	/**
	* @return Tau = torque (1x1)
	*/
	 virtual double				getTorque()		const;
    /**
	* @return Im
	*/
	 virtual double				getIm()		const;
    /**
	* @return Fs = static friction
	*/
 	 virtual double				getFs()		const;
    /**
	* @return Fv = viscous friction
	*/
 	 virtual double				getFv()		const;
    /**
	* @return ddq = d2q = joint acceleration
	*/
	 virtual double				getD2q()	const;
     /**
	* @return dq = dq = joint velocity
	*/
	 virtual double				getDq()		const;
    /**
	* @return Kr
	*/
	 virtual double				getKr()		const;
    /**
	* @return m = mass of the link
	*/
	 virtual double				getMass()	const;
    /**
	* @return I = inertia of the link
	*/
  	 virtual yarp::sig::Matrix	getInertia() const;
    /**
	* @return R = (3x3) rotational matrix from the Denavit-Hartenberg roto-translational matrix describing the link
	*/
 	 virtual yarp::sig::Matrix	getR();
    /**
	* @return RC = (3x3) rotational matrix from the roto-translational matrix of the link COM
	*/
	virtual yarp::sig::Matrix	getRC();
	/**
	* @return r = (3x1) distance vector from the Denavit-Hartenberg roto-translational matrix describing the distance vector between frames <i> and <i-1>
	*/
	 virtual yarp::sig::Vector	getr(bool proj=false);
     /**
	* @return rC = (3x1) distance vector from the roto-translational matrix of the link COM, describing the distance vector between COM and frame <i>
	*/
	 virtual yarp::sig::Vector	getrC(bool proj=false);


	 //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	 //   main computation methods
	 //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	/**
     * [Forward Newton-Euler] Compute w, dw, ddp, ddpC, dwM
	  * @param prev the OneLinkNewtonEuler class of the previous link 
     */
	 void ForwardKinematics( OneLinkNewtonEuler *prev);
 
	/**
     * [Backward Kinematic computation] Compute w, dw, ddp, ddpC, dwM
	  * @param prev the OneLinkNewtonEuler class of the previous link 
     */
	 void BackwardKinematics( OneLinkNewtonEuler *prev);

	/**
     * [Backward Newton-Euler] Compute F, Mu, Tau
	  * @param next the OneLinkNewtonEuler class of the following link 
     */
	 void BackwardWrench( OneLinkNewtonEuler *next);

	/**
     * [Inverse Newton-Euler] Compute F, Mu, Tau
	  * @param prev the OneLinkNewtonEuler class of the previous link 
     */
	 void ForwardWrench( OneLinkNewtonEuler *prev);
	
 	/**
     * [all] Compute joint torque; moment must be pre-computed
	  * @param next the OneLinkNewtonEuler class of the following link
     */
	 void computeTorque(OneLinkNewtonEuler *prev);

};

/**
* \ingroup RecursiveNewtonEuler
*
* A class for setting a virtual base link: this is useful to initialize the forward phase of
* Newton-Euler's method, by setting w, dw, ddp; H is an identity matrix, while ddpC=ddp;
* Note that this is a virtual link, since there's no iDynLink attached: it is just necessary to make 
* the recursive Newton-Euler computations 
*/
class BaseLinkNewtonEuler : public OneLinkNewtonEuler
{
protected:
	///initial angular velocity
	yarp::sig::Vector w;	
	///initial angular acceleration
	yarp::sig::Vector dw;
	///initial linear acceleration
	yarp::sig::Vector ddp;	
	///base roto-traslation (if necessary)
	yarp::sig::Matrix H0;
	///initial force
	yarp::sig::Vector F;
	///initial moment
	yarp::sig::Vector Mu;
	///corresponding torque
	double Tau;				

public:

	/**
    * Default constructor 
	* @param _H0 the base roto-translation
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	BaseLinkNewtonEuler(const yarp::sig::Matrix &_H0, const NewEulMode _mode, unsigned int verb=NO_VERBOSE);

	/**
    * Constructor, initializing the base data
	* @param _H0 the base roto-translation
	* @param _w	the initial angular velocity
	* @param _dw the initial angular acceleration
	* @param _ddp the initial linear acceleration
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	BaseLinkNewtonEuler(const yarp::sig::Matrix &_H0, const yarp::sig::Vector &_w, const yarp::sig::Vector &_dw, const yarp::sig::Vector &_ddp, const NewEulMode _mode, unsigned int verb=NO_VERBOSE);

	/**
     * Sets the base data
	 * @param _w	the initial angular velocity
	 * @param _dw the initial angular acceleration
	 * @param _ddp the initial linear acceleration
     */
	bool setAsBase(const yarp::sig::Vector &_w, const yarp::sig::Vector &_dw, const yarp::sig::Vector &_ddp);
		/**
     * Sets the base data
	 * @param _F	the initial angular velocity
	 * @param _Mu the initial angular acceleration
     */
	bool setAsBase(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);

	//~~~~~~~~~~~~~~~~~~~~~~
	//   get methods
	//~~~~~~~~~~~~~~~~~~~~~~

	 yarp::sig::Vector	getAngVel()		const;
	 yarp::sig::Vector	getAngAcc()		const;
	 yarp::sig::Vector	getAngAccM()	const;
	 yarp::sig::Vector	getLinAcc()		const;
	 yarp::sig::Vector	getLinAccC()	const;
	 	
	 // redefine the other unuseful methods 
	 // to avoid errors due to missing link

	 yarp::sig::Vector	getForce()		const;
	 yarp::sig::Vector	getMoment()		const;
	 double				getTorque()		const;
	 yarp::sig::Matrix	getR();		
	 yarp::sig::Matrix	getRC();	
	 double				getIm()		const;
	 double				getD2q()	const;
	 double				getDq()		const;
	 double				getKr()		const;
  	 double				getFs()		const;
 	 double				getFv()		const;
	 double				getMass()	const;
   	 yarp::sig::Matrix	getInertia()const;
	 yarp::sig::Vector	getr(bool proj=false);
	 yarp::sig::Vector	getrC(bool proj=false);

	//~~~~~~~~~~~~~~~~~~~~~~
	//   set methods
	//~~~~~~~~~~~~~~~~~~~~~~

	 bool setForce(const yarp::sig::Vector &_F);
	 bool setMoment(const yarp::sig::Vector &_Mu);
	 void setTorque(const double _Tau);
 	 bool setAngVel(const yarp::sig::Vector &_w);
	 bool setAngAcc(const yarp::sig::Vector &_dw);
	 bool setLinAcc(const yarp::sig::Vector &_ddp);
	 bool setLinAccC(const yarp::sig::Vector &_ddpC);
	 bool setAngAccM(const yarp::sig::Vector &_dwM);

	 // other methods

	 std::string toString() const;

};


/**
* \ingroup RecursiveNewtonEuler
*
* A class for setting a virtual final link: this is useful to initialize the backward phase of
* Newton-Euler's method, by setting F, Mu; H is an identity matrix, while ddpC=ddp;
* Note that this is a virtual link, since there's no iDynLink attached: it is just necessary to make 
* the recursive Newton-Euler computations 
*/
class FinalLinkNewtonEuler : public OneLinkNewtonEuler
{
protected:
	///initial angular velocity
	yarp::sig::Vector w;	
	///initial angular acceleration
	yarp::sig::Vector dw;
	///initial linear acceleration
	yarp::sig::Vector ddp;	
	///final force
	yarp::sig::Vector F;	
	///final moment
	yarp::sig::Vector Mu;	

public:

	/**
    * Default constructor 
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	FinalLinkNewtonEuler(const NewEulMode _mode, unsigned int verb=NO_VERBOSE);

	/**
    * Constructor, initializing the final frame data
	* @param _F the final force
	* @param _Mu the final moment
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	FinalLinkNewtonEuler(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu, const NewEulMode _mode, unsigned int verb=NO_VERBOSE);

	/**
     * Set the final frame data
	  * @param _F the final force
	  * @param _Mu the final moment
	  * @return true if dimensions are correct, false otherwise
     */
	bool setAsFinal(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);
	
	/**
     * Set the final frame data
	  * @param _w the final force
	  * @param _dw the final force
	  * @param _ddp the final moment
	  * @return true if dimensions are correct, false otherwise
     */
	bool setAsFinal(const yarp::sig::Vector &_w, const yarp::sig::Vector &_dw, const yarp::sig::Vector &_ddp);


	//~~~~~~~~~~~~~~~~~~~~~~
	//   get methods
	//~~~~~~~~~~~~~~~~~~~~~~
	yarp::sig::Vector	getForce()		const;
	yarp::sig::Vector	getMoment()		const;

	// redefine the other unuseful methods to avoid errors due to missing link
	yarp::sig::Vector	getAngVel()		const;
	yarp::sig::Vector	getAngAcc()		const;
	yarp::sig::Vector	getAngAccM()	const;
	yarp::sig::Vector	getLinAcc()		const;
	yarp::sig::Vector	getLinAccC()	const;
	double				getTorque()		const;
	yarp::sig::Matrix	getR();		
	yarp::sig::Matrix	getRC();	
	double				getIm()		const;
  	double				getFs()		const;
 	double				getFv()		const;
	double				getD2q()	const;
	double				getDq()		const;
	double				getKr()		const;
	double				getMass()	const;
   	yarp::sig::Matrix	getInertia()const;
	yarp::sig::Vector	getr(bool proj=false);
	yarp::sig::Vector	getrC(bool proj=false);
	bool setForce(const yarp::sig::Vector &_F);
	bool setMoment(const yarp::sig::Vector &_Mu);
	void setTorque(const double _Tau);
	bool setAngVel(const yarp::sig::Vector &_w);
	bool setAngAcc(const yarp::sig::Vector &_dw);
	bool setLinAcc(const yarp::sig::Vector &_ddp);
	bool setLinAccC(const yarp::sig::Vector &_ddpC);
	bool setAngAccM(const yarp::sig::Vector &_dwM);

	//other
	std::string toString() const;
};


/**
* \ingroup RecursiveNewtonEuler
*
* A class for setting a virtual sensor link. This class is used to initialize the forward and backward phase of
* Newton-Euler's method in the Inverse formulation, but also to have estimation of the FT measures by the sensor; 
* the sensor frame is defined with respect to the i-th link, where the sensor is attached.
* Note that this is a virtual link, since there's no iDynLink attached: it is just necessary to make 
* the recursive Inverse Newton-Euler computations; however, inertia, mass and COM are defined, for the portion of 
* link defined between sensor and i-th frame.
*/
class SensorLinkNewtonEuler : public OneLinkNewtonEuler
{
protected:
	/// measured or estimated force
	yarp::sig::Vector F;
	/// measured or estimated moment
	yarp::sig::Vector Mu;	
	
	/// angular velocity
	yarp::sig::Vector w;
	/// angular acceleration
	yarp::sig::Vector dw;
	/// linear acceleration
	yarp::sig::Vector ddp;	
	/// linear acceleration of the COM
	yarp::sig::Vector ddpC;	
	/// the roto-translational matrix from the i-th link to the sensor: it's the matrix describing the sensor position and orientation with respect to the frame of the link where the sensor is placed on
	yarp::sig::Matrix H;
	/// the roto-translational matrix of the COM of the semi-link (bewteen sensor and ith link frame)
	yarp::sig::Matrix COM;
	/// the semi-link inertia
	yarp::sig::Matrix I;	
	/// the semi-link mass (the portion of link defined by the sensor)
	double m;				

public:

	/**
    * Default constructor 
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	SensorLinkNewtonEuler(const NewEulMode _mode, unsigned int verb=NO_VERBOSE);

	/**
    * Constructor 
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	SensorLinkNewtonEuler(const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_COM, const double _m, const yarp::sig::Matrix &_I, const NewEulMode _mode, unsigned int verb=NO_VERBOSE);
	

	/**
     * Set the sensor measured force/moment - if measured by a FT sensor
	  * @param _F the final force
	  * @param _Mu the final moment
	  * @return true if dimensions are correct, false otherwise
     */
	bool setMeasuredFMu(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);

	/**
     * Set a new sensor or new sensor properties
	 * @param _H the roto-traslational matrix from the reference frame of the i-th link to the sensor
	 * @param _HC the roto-traslational matrix of the center of mass of the semi-link defined by the sensor in the i-th link
	 * @param _m the mass of the semi-link
	 * @param _I the inertia of the semi-link
     */
	bool setSensor(const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_HC, const double _m, const yarp::sig::Matrix &_I);

	/**
     * Compute w,dw,ddp,dppC given the reference frame of the link where the sensor is
	 * @param link the iDynLink class of the same link 
     */
	 void ForwardAttachToLink( iDynLink *link);

 	/**
     * Compute F,Mu given the reference frame of the link where the sensor is
	 * @param link the iDynLink class of the same link 
     */
	 void BackwardAttachToLink( iDynLink *link);

	 /**
     * Forward the sensor forces and moments, measured by the sensor, to the reference
	 * frame of the link where the sensor is: this method is the base for the inverse 
	 * Newton-Euler algorithm, since it forwards the sensor measurements to the iDynChain
	 * @param link the iDynLink class of the same link 
     */
	 void ForwardForcesMomentsToLink( iDynLink *link);

 	/**
     * Get the sensor force and moment in a single (6x1) vector
	  * @return a (6x1) vector where 0:2=force 3:5=moment
     */
	yarp::sig::Vector getForceMoment() const;


	//~~~~~~~~~~~~~~~~~~~~~~
	//   get methods
	//~~~~~~~~~~~~~~~~~~~~~~
	yarp::sig::Vector	getAngVel()		const;
	yarp::sig::Vector	getAngAcc()		const;
	yarp::sig::Vector	getLinAcc()		const;
	yarp::sig::Vector	getLinAccC()	const;

	yarp::sig::Vector	getForce()		const;
	yarp::sig::Vector	getMoment()		const;
	
	double				getIm()		const;
 	double				getFs()		const;
 	double				getFv()		const;
	double				getD2q()	const;
	double				getDq()		const;
	double				getKr()		const;
	yarp::sig::Vector	getAngAccM()const;
	double				getTorque()	const;
	yarp::sig::Matrix	getH()		const;

	double				getMass()	const;
  	yarp::sig::Matrix	getInertia()const;
 	yarp::sig::Matrix	getR();
	yarp::sig::Matrix	getRC();
	yarp::sig::Vector	getr(bool proj=false);
	yarp::sig::Vector	getrC(bool proj=false);

	bool setForce(const yarp::sig::Vector &_F);
	bool setMoment(const yarp::sig::Vector &_Mu);
	void setTorque(const double _Tau);
	bool setAngVel(const yarp::sig::Vector &_w);
	bool setAngAcc(const yarp::sig::Vector &_dw);
	bool setLinAcc(const yarp::sig::Vector &_ddp);
	bool setLinAccC(const yarp::sig::Vector &_ddpC);
	bool setAngAccM(const yarp::sig::Vector &_dwM);

	/**
     * Useful to print some information..
     */
	std::string toString() const;

	// redefinitions but using iDynLink instead of OneLinkNeutonEuler
	// that because it is used by IDynInvSensor

	 void computeAngVel	( iDynLink *link);
	 void computeAngAcc	( iDynLink *link);
	 void computeLinAcc	( iDynLink *link);
	 void computeLinAccC( );
	 void computeForce	( iDynLink *link);
	 void computeMoment	( iDynLink *link);

	 void computeForceToLink ( iDynLink *link);
	 void computeMomentToLink( iDynLink *link);

	 // virtual function to be called by specific sensorLinks
	 virtual std::string getType() const;

};


/**
* \ingroup RecursiveNewtonEuler
*
* A class for computing forces and torques in a iDynChain
*/
class OneChainNewtonEuler
{
	friend class iDynChain;

protected:

	/// the real kinematic/dynamic chain of the robot
	iDyn::iDynChain *chain;	
	/// the chain of links/frames for Newton-Euler computations
	OneLinkNewtonEuler ** neChain;	

	/// number of links
	unsigned int nLinks;		
	/// the index f the end-effector in the chain (the last frame)
	unsigned int nEndEff;		

	/// static/dynamic/dynamicWrotor
	NewEulMode mode;
	/// info or useful notes
	std::string	info;
	/// verbosity flag
	unsigned int verbose;

public:

  /**
    * Constructor (note: without FT sensor)
    */
	OneChainNewtonEuler(iDyn::iDynChain *_c, std::string _info, const NewEulMode _mode = STATIC, unsigned int verb = NO_VERBOSE);

	/**
	* Standard destructor
	*/
	~OneChainNewtonEuler();

	/**
     * Useful to print some information..
     */
	std::string toString() const;

	/**
     * Useful to debug, getting the intermediate computations after the forward phase
     */
	bool getVelAccAfterForward(unsigned int i, yarp::sig::Vector &w, yarp::sig::Vector &dw, yarp::sig::Vector &dwM, yarp::sig::Vector &ddp, yarp::sig::Vector &ddpC) const;

	/**
     * This method is used by iDynChain to retrieve kinematic information for connection with one or more
	 * iDynLimb, through iDynNode.
     */
	void getVelAccBase(yarp::sig::Vector &w, yarp::sig::Vector &dw,yarp::sig::Vector &ddp) const;
	
	/**
     * This method is used by iDynChain to retrieve kinematic information for connection with one or more
	 * iDynLimb, through iDynNode.
     */
	void getVelAccEnd(yarp::sig::Vector &w, yarp::sig::Vector &dw,yarp::sig::Vector &ddp) const;

	/**
     * This method is used by iDynChain to retrieve wrench information for connection with one or more
	 * iDynLimb, through iDynNode.
     */
	void getWrenchBase(yarp::sig::Vector &F, yarp::sig::Vector &Mu) const;
	
	/**
     * This method is used by iDynChain to retrieve wrench information for connection with one or more
	 * iDynLimb, through iDynNode.
     */
	void getWrenchEnd(yarp::sig::Vector &F, yarp::sig::Vector &Mu) const;

	//~~~~~~~~~~~~~~~~~~~~~~
	//   set methods
	//~~~~~~~~~~~~~~~~~~~~~~

	void setVerbose(unsigned int verb=VERBOSE);
	void setMode(const NewEulMode _mode);
 	void setInfo(const std::string _info);
	
	/**
	* [classic] Initialize the base with measured or known kinematics variables
	* @param w0 angular velocity
	* @param dw0 angular acceleration
	* @param ddp0 linear acceleration
	* @return true if succeeds, false otherwise
	*/
	bool initKinematicBase(const yarp::sig::Vector &w0,const yarp::sig::Vector &dw0,const yarp::sig::Vector &ddp0);
	
	/**
	* [inverse] Initialize the end-effector finalLink with measured or known kinematics variables
	* @param w0 angular velocity
	* @param dw0 angular acceleration
	* @param ddp0 linear acceleration
	* @return true if succeeds, false otherwise
	*/
	bool initKinematicEnd(const yarp::sig::Vector &w0,const yarp::sig::Vector &dw0,const yarp::sig::Vector &ddp0);
	
	/**
	* [classic] Initialize the end-effector finalLink with measured or known wrench
	* @param F0 force
	* @param Mu0 moment
	* @return true if succeeds, false otherwise
	*/
	bool initWrenchEnd(const yarp::sig::Vector &F0,const yarp::sig::Vector &Mu0);

	/**
	* [inverse] Initialize the base with measured or known wrench
	* @param F0 force
	* @param Mu0 moment
	* @return true if succeeds, false otherwise
	*/
	bool initWrenchBase(const yarp::sig::Vector &F0,const yarp::sig::Vector &Mu0);


	//~~~~~~~~~~~~~~~~~~~~~~
	//   get methods
	//~~~~~~~~~~~~~~~~~~~~~~

	/**
	* @return information about the chain
	*/
	std::string			getInfo()		const;

	/**
	* @return the computational mode: static/dynamic/etc
	*/
	NewEulMode			getMode()		const;


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//   main computation methods
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	/**
     * [classic/inverse] Base function for forward of classical Newton-Euler.
     */
	void ForwardKinematicFromBase();
	
	/**
     * [classic/inverse] Forward of classical Newton-Euler, after initializing the base link
     */
	void ForwardKinematicFromBase(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0);

	/**
     * [inverse] Base function for forward of classical Newton-Euler.
     */
	void BackwardKinematicFromEnd();
	
	/**
     * [inverse] Forward of classical Newton-Euler, after initializing the base link
     */
	void BackwardKinematicFromEnd(const yarp::sig::Vector &we, const yarp::sig::Vector &dwe, const yarp::sig::Vector &ddpe);

	/**
     * [classic] Base function for backward of classical Newton-Euler.
     */
	void BackwardWrenchFromEnd();

	/**
     * [classic] Backward of classical Newton-Euler, after initializing the final link
     */
	void BackwardWrenchFromEnd(const yarp::sig::Vector &F, const yarp::sig::Vector &Mu);

	/**
     * [inverse] Base function for inverse Newton-Euler: from the i-th link to the end, 
	 * forward of forces and moments using the inverse formula
     */
	void ForwardWrenchFromBase();

	/**
     * [inverse] Base function for inverse Newton-Euler: from the i-th link to the base, 
	 * backward of forces and moments using the classical formula
	 * @param lSens the i-th link, where the sensor is attached to
	 * @return true if the operation is successful, false otherwise (eg out of range index)
	 */
	void ForwardWrenchFromBase(const yarp::sig::Vector &F, const yarp::sig::Vector &Mu);

	/**
     * [inverse] Base function for inverse Newton-Euler: from the i-th link to the end, 
	 * forward of forces and moments using the inverse formula
	 * @param lSens the i-th link, where the sensor is attached to
	 * @return true if the operation is successful, false otherwise (eg out of range index)
     */
	bool ForwardWrenchToEnd(unsigned int lSens);

	/**
     * [classic/inverse] Base function for inverse Newton-Euler: from the i-th link to the base, 
	 * backward of forces and moments using the classical formula
	 * @param lSens the i-th link, where the sensor is attached to
	 * @return true if the operation is successful, false otherwise (eg out of range index)
	 */
	bool BackwardWrenchToBase(unsigned int lSens);


};




/**
 * \defgroup iDynInv iDynInv 
 *    
 * @ingroup iDyn
 *  
 * Classes for force/torque computation in a dynamic 
 * chain, where a single FT sensor is placed. 
 * Using Newton-Euler formula it is possible to 
 * compute an estimate of the FT sensor measurements. 
 * iDynInvSensor is the generic class which takes a 
 * iDynChain, sets a generic sensor attached to the 
 * chain, and manages the proper computations. 
 * iDynInvSensorArm and iDynInvSensorLeg
 * are specific classes performing computations for iCub
 * arms and legs: by choosing left/right part, they
 * automatically set the proper FT sensor parameters.
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows
 *
 * \section example_sec Example
 *
 * Set a iDynInvSensor for iCub's left arm, with 
 * force/moment computation in the static case (STATIC), and
 * verbose (VERBOSE==1) output:
 *
 * <tt> iCubArmDyn *arm = new iCubArmDyn("left"); </tt> \n
 * <tt> iDynInvSensorArm armWSensorSolver = new iDynInvSensorArm(arm,STATIC,VERBOSE); </tt> \n
 *
 * Note that by setting the arm as "left", the sensor is automatically set
 * as the left sensor of the arm.
 * Then start retrieving the sensor force/moment. First the chain must be updated with 
 * the current angle configuration:
 *
 * <tt> arm->setAng(q); </tt> \n
 * <tt> arm->setDAng(dq); </tt> \n
 * <tt> arm->setD2Ang(ddq); </tt> \n
 *
 * Then the arm dynamics must be solved, in order to find each links' force/moment, etc:
 *
 * <tt> arm->computeNewtonEuler(w0,dw0,d2p0,Fend,Mend); </tt> \n
 *
 * Finally we can compute the sensor force and moment, attaching the sensor to its link
 * (whose force/moment have been previously computed):
 *
 * <tt> armWSensorSolver->computeSensorForceMoment(); </tt> \n
 *
 * The sensor force/moment can be retrieved separately (3x1 vectors) or 
 * together (6x1 vector).
 *
 * <tt> Vector F = armWSensorSolver->getSensorForce(); </tt> \n
 * <tt> Vector Mu = armWSensorSolver->getSensorMoment(); </tt> \n
 * <tt> Vector FMu = armWSensorSolver->getSensorForceMoment(); </tt> \n
 *
 * \author Serena Ivaldi
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * 
 **/ 

/**
* \ingroup iDynInv
*
* A class for computing force/moment of a sensor placed anywhere in 
*  a kinematic chain; its position in the chain is defined wrt a certain link in the chain;
* this class can be usefult to estimate the FT measurements of the sensor
* 
*/
class iDynInvSensor
{
	friend class iFTransformation;
protected:

	/// the link where the sensor is attached to
	unsigned int lSens;		
	/// the sensor
	SensorLinkNewtonEuler * sens;	
	/// the iDynChain describing the robotic chain
	iDynChain * chain;	
	/// static/dynamic/etc..
	NewEulMode mode;
	/// verbosity flag
	unsigned int verbose;
	/// a string with useful information if needed
	std::string info;

public:

	/**
    * Constructor without FT sensor: the sensor must be set with setSensor()
	* @param _c a pointer to the iDynChain where the sensor is placed on
	* @param _info a string with information
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	iDynInvSensor(iDyn::iDynChain *_c, const std::string &_info, const NewEulMode _mode = STATIC, unsigned int verb = NO_VERBOSE);

	/**
    * Constructor with FT sensor
	* @param _c a pointer to the iDynChain where the sensor is placed on
	* @param i the i-th link to whom the sensor is attached
	* @param _H the roto-traslational matrix from the reference frame of the i-th link to the sensor
	* @param _HC the roto-traslational matrix of the center of mass of the semi-link defined by the sensor in the i-th link
	* @param _m the mass of the semi-link
	* @param _I the inertia of the semi-link
	* @param _info a string with information
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	iDynInvSensor(iDyn::iDynChain *_c, unsigned int i, const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_HC, const double _m, const yarp::sig::Matrix &_I, const std::string &_info, const NewEulMode _mode = STATIC, unsigned int verb = 0);

	/**
     * Set a new sensor or new sensor properties
	 * @param i the i-th link to whom the sensor is attached
	 * @param _H the roto-traslational matrix from the reference frame of the i-th link to the sensor
	 * @param _HC the roto-traslational matrix of the center of mass of the semi-link defined by the sensor in the i-th link
	 * @param _m the mass of the semi-link
	 * @param _I the inertia of the semi-link
	 * @return true if the operation is successful, false otherwise (eg if index is out of range)
     */
	bool setSensor(unsigned int i, const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_HC, const double _m, const yarp::sig::Matrix &_I);

	/**
     * Compute forces and moments at the sensor frame; this method calls special Forward and Backward methods of
	 * SensorLink, using Newton-Euler's formula applied in the link where the sensor is placed on; the link is 
	 * automatically found, being specified by the index in the chain and the chain itself
     */
	void computeSensorForceMoment();

	/**
     * Print some information
	 */
	std::string			toString()	const;

	/**
     * Returns the sensor estimated force
	 * @return the force at the sensor frame
	 */
	yarp::sig::Vector	getSensorForce()	const;

	/**
     * Returns the sensor estimated moment
	 * @return the moment at the sensor frame
	 */
	yarp::sig::Vector	getSensorMoment()	const;

 	/**
     * Get the sensor force and moment in a single (6x1) vector
	 * @return a (6x1) vector where 0:2=force 3:5=moment
     */
	yarp::sig::Vector getSensorForceMoment() const;
	
 	/**
     * Get the sensor roto-translational matrix defining its position/orientation wrt the link
	 * @return a (4x4) matrix
     */
	yarp::sig::Matrix getH() const;

	//~~~~~~~~~~~~~~
	// set methods
	//~~~~~~~~~~~~~~

	void setMode(const NewEulMode _mode = STATIC);
	void setVerbose(unsigned int verb=VERBOSE);
	void setInfo(const std::string &_info);
	void setSensorInfo(const std::string &_info);

	//~~~~~~~~~~~~~~
	// get methods
	//~~~~~~~~~~~~~~

	std::string getInfo() const;
	std::string getSensorInfo() const;
	unsigned int getSensorLink()	const;

	

};

/**
* \ingroup iDynInv
*
* A class for setting a virtual sensor link on the iCub 
* arm, for the arm FT sensor. The parameters are
* automatically set after choosing "left" or "right"
* part. By default, the CAD parameters are set.
*/
class iCubArmSensorLink : public SensorLinkNewtonEuler
{
protected:
	
	/// the arm type: left/right
	std::string type;

public:

	/**
    * Constructor: the sensor parameters are automatically set with "right" or "left" choice
	* @param _type a string "left"/"right" 
	* @param _mode the analysis mode (static/dynamic/etc)
	* @param verb flag for verbosity
    */
	iCubArmSensorLink(const std::string &_type, const NewEulMode _mode = STATIC, unsigned int verb = NO_VERBOSE);

	/**
	* @return type the arm type: left/right
	*/
	std::string getType() const;

};

/**
* \ingroup iDynInv
*
* A class for setting a virtual sensor link on the iCub
* leg, for the leg FT sensor. The parameters are
* automatically set after choosing "left" or "right"
* part. By default, the CAD parameters are set.
*/
class iCubLegSensorLink : public SensorLinkNewtonEuler
{
protected:
	
	/// the leg type: left/right
	std::string type;

	public:

	/**
    * Constructor: the sensor parameters are automatically set with "right" or "left" choice
	* @param _type a string "left"/"right" 
	* @param _mode the analysis mode (static/dynamic/etc)
	* @param verb flag for verbosity
    */
	iCubLegSensorLink(const std::string _type, const NewEulMode _mode = STATIC, unsigned int verb = NO_VERBOSE);

	/**
	* @return type the leg type: left/right
	*/
	std::string getType() const;

};


/**
* \ingroup iDynInv
*
* A class for computing force/moment of the FT sensor placed
* in the middle of the iCub's left or right arm. The sensor
* parameters are automatically set by chosing left or right
* during initialization of the iCubArmDyn.
* 
*/
class iDynInvSensorArm : public iDynInvSensor
{

public:

	/**
    * Constructor: the sensor is automatically set with "right" or "left" choice
	* @param _c a pointer to the iCubArmDyn where the sensor is placed on
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	iDynInvSensorArm(iDyn::iCubArmDyn *_c, const NewEulMode _mode = STATIC, unsigned int verb = NO_VERBOSE);

	/**
    * Constructor: the sensor is automatically set with "right" or "left" choice; note that in this case 
	* there is not a specification of the iCubArmDyn, but the part must be specified
	* @param _c a pointer to the iDynChain where the sensor is placed on
	* @param _type a string setting the arm type
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	iDynInvSensorArm(iDyn::iDynChain *_c, const std::string _type, const NewEulMode _mode = STATIC, unsigned int verb = NO_VERBOSE);

	/**
	* @return type the arm type: left/arm
	*/
	std::string getType() const;


};

/**
* \ingroup iDynInv
*
* A class for computing force/moment of the FT sensor placed
* in the middle of the iCub's left or right leg. The sensor
* parameters are automatically set by chosing left or right
* during initialization of iCubLegDyn.
* 
*/
class iDynInvSensorLeg : public iDynInvSensor
{

public:

	/**
    * Constructor: the sensor is automatically set with "right" or "left" choice
	* @param _c a pointer to the iCubLegDyn where the sensor is placed on
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	iDynInvSensorLeg(iDyn::iCubLegDyn *_c, const NewEulMode _mode = STATIC, unsigned int verb = NO_VERBOSE);

	/**
    * Constructor: the sensor is automatically set with "right" or "left" choice
	* @param _c a pointer to the iDynChain where the sensor is placed on
	* @param _type a string setting the leg type
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	iDynInvSensorLeg(iDyn::iDynChain *_c, const std::string _type, const NewEulMode _mode = STATIC, unsigned int verb = NO_VERBOSE);

	/**
	* @return type the leg type: left/arm
	*/
	std::string getType() const;


};


}//end namespace

#endif