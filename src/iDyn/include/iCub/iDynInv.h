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
 * \section example_sec Example
 *
 * See iDynChain examples.
 *
 * \author Serena Ivaldi
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
#include <iCub/ctrlMath.h>
#include <iCub/iKinFwd.h>
#include <iCub/iDyn.h>
#include <deque>
#include <string>

enum NewEulMode {NE_static,NE_dynamic,NE_dynamicWrotor,NE_dynamicCoriolisGravity};
const std::string NE_Mode[4] = {"static","dynamic","dynamic with motor/rotor","dynamic with only Coriolis and gravitational terms"};

namespace iDyn{

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
	class iFTransform;
    class iSFrame;
	class iFB;


/**
* \ingroup RecursiveNewtonEuler
*
* A base class for computing forces and torques in a serial link chain
*/
class OneLinkNewtonEuler
{
protected:

	///NE_static/NE_dynamic/NE_dynamicWrotor/NE_dynamicCoriolisGravity
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

	virtual bool setAngVel(const yarp::sig::Vector &_w);
	virtual bool setAngAcc(const yarp::sig::Vector &_dw);
	virtual bool setLinAcc(const yarp::sig::Vector &_ddp);
	virtual bool setLinAccC(const yarp::sig::Vector &_ddpC);
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
     * [Forward Newton-Euler] compute angular acceleration of the link
	  * @param prev the OneLinkNewtonEuler class of the previous link
     */
	 void computeAngAcc( OneLinkNewtonEuler *prev);

	/**
     * [Forward Newton-Euler] compute linear acceleration of the reference frame of the link
	  * @param prev the OneLinkNewtonEuler class of the previous link
     */
	 void computeLinAcc( OneLinkNewtonEuler *prev);

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
	OneLinkNewtonEuler(const NewEulMode _mode, unsigned int verb=0, iDyn::iDynLink *dlink=NULL);
	 	 
	/**
     * Set everything to zero; R is set to an identity matrix
     */
	void zero();

	/**
     * Virtual method to set the frame as the base one: this is useful to initialize the forward phase of
	 * Newton-Euler's method. The BaseLink class is used to this scope.
     */
	virtual bool setAsBase(const yarp::sig::Vector &_w, const yarp::sig::Vector &_dw, const yarp::sig::Vector &_ddp);

	/**
     * Set the frame as the final one: this is useful to initialize the backward phase of
	  * Newton-Euler's method, by setting F and Mu; R is an identity matrix
	  * @param _F the final force
	  * @param _Mu the final moment
     */
	virtual bool setAsFinal(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);

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
	 void setVerbose(unsigned int verb=1);
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
	 void setInfo(const std::string _info);


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
	 void ForwardNewtonEuler( OneLinkNewtonEuler *prev);
 
	/**
     * [Backward Newton-Euler] Compute F, Mu, Tau
	  * @param next the OneLinkNewtonEuler class of the following link 
     */
	 void BackwardNewtonEuler( OneLinkNewtonEuler *next);

	/**
     * [Inverse Newton-Euler] Compute F, Mu, Tau
	  * @param prev the OneLinkNewtonEuler class of the previous link 
     */
	 void InverseNewtonEuler( OneLinkNewtonEuler *prev);
	
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
	BaseLinkNewtonEuler(const yarp::sig::Matrix &_H0, const NewEulMode _mode, unsigned int verb=0);

	/**
    * Constructor, initializing the base data
	* @param _H0 the base roto-translation
	* @param _w	the initial angular velocity
	* @param _dw the initial angular acceleration
	* @param _ddp the initial linear acceleration
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	BaseLinkNewtonEuler(const yarp::sig::Matrix &_H0, const yarp::sig::Vector &_w, const yarp::sig::Vector &_dw, const yarp::sig::Vector &_ddp, const NewEulMode _mode, unsigned int verb=0);

	/**
     * Sets the base data
	 * @param _w	the initial angular velocity
	 * @param _dw the initial angular acceleration
	 * @param _ddp the initial linear acceleration
     */
	bool setAsBase(const yarp::sig::Vector &_w, const yarp::sig::Vector &_dw, const yarp::sig::Vector &_ddp);

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
	FinalLinkNewtonEuler(const NewEulMode _mode, unsigned int verb=0);

	/**
    * Constructor, initializing the final frame data
	* @param _F the final force
	* @param _Mu the final moment
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	FinalLinkNewtonEuler(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu, const NewEulMode _mode, unsigned int verb=0);

	/**
     * Set the final frame data
	  * @param _F the final force
	  * @param _Mu the final moment
	  * @return true if dimensions are correct, false otherwise
     */
	bool setAsFinal(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu);

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
* the recursive Inverse Newton-Euler computations; however, inertia, mass and COM are defined, for the portion of link defined between sensor and i-th frame
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
	/// the semi-link 
	yarp::sig::Matrix I;	
	/// the semi-link mass (the portion of link defined by the sensor)
	double m;				

public:

	/**
    * Default constructor 
	* @param _sensor the sensor frame class
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	SensorLinkNewtonEuler(const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_COM, const double _m, const yarp::sig::Matrix &_I, const NewEulMode _mode, unsigned int verb=0);
	

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

	/// the real kinematic chain of the robot
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
    * Constructor without FT sensor
    */
	OneChainNewtonEuler(iDyn::iDynChain *_c, std::string _info, const NewEulMode _mode = NE_static, unsigned int verb = 0);

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


	//~~~~~~~~~~~~~~~~~~~~~~
	//   set methods
	//~~~~~~~~~~~~~~~~~~~~~~

	void setVerbose(unsigned int verb=1);
	void setMode(const NewEulMode _mode);
 	void setInfo(const std::string _info);
	bool initNewtonEuler(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0, const yarp::sig::Vector &Fend, const yarp::sig::Vector &Muend);


	//~~~~~~~~~~~~~~~~~~~~~~
	//   get methods
	//~~~~~~~~~~~~~~~~~~~~~~

	std::string			getInfo()		const;
	NewEulMode			getMode()		const;


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//   main computation methods
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	/**
     * [classic/inverse] Base function for forward of classical Newton-Euler.
     */
	void ForwardFromBase();
	
	/**
     * [classic/inverse] Forward of classical Newton-Euler, after initializing the base link
     */
	void ForwardFromBase(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0);

	/**
     * [classic] Base function for backward of classical Newton-Euler.
     */
	void BackwardFromEnd();

	/**
     * [classic] Backward of classical Newton-Euler, after initializing the final link
     */
	void BackwardFromEnd(const yarp::sig::Vector &F, const yarp::sig::Vector &Mu);

	/**
     * [inverse] Base function for inverse Newton-Euler: from the i-th link to the end, 
	 * forward of forces and moments using the inverse formula
	 * @param lSens the i-th link, where the sensor is attached to
	 * @return true if the operation is successful, false otherwise (eg out of range index)
     */
	bool InverseToEnd(unsigned int lSens);

	/**
     * [inverse] Base function for inverse Newton-Euler: from the i-th link to the base, 
	 * backward of forces and moments using the classical formula
	 * @param lSens the i-th link, where the sensor is attached to
	 * @return true if the operation is successful, false otherwise (eg out of range index)
	 */
	bool InverseToBase(unsigned int lSens);


};




/**
 * \defgroup iDynInv iDynInv 
 *    
 * @ingroup iDyn
 *  
 * Classes for force/torque computation in a dynamic chain, where a single FT sensor is placed.
 * Computations from  using Newton-Euler recursive formulation
 * 
 * \section intro_sec Description
 * 
 * OneLinkNewtonEuler is a class that provides Newton-Euler recursive formulation, for the computation of forces, moments and
 * joint torques in a kinematic chain; both classical and "inverse" formulation are included; the classe is the basis for the computation of torques
 * given force/torque sensor measurements, given any sensor anywhere in the dynamic/kinematic chain
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows
 *
 * \section example_sec Example
 *
 * Exe
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
	iDynInvSensor(iDyn::iDynChain *_c, std::string _info, const NewEulMode _mode = NE_static, unsigned int verb = 0);

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
	iDynInvSensor(iDyn::iDynChain *_c, unsigned int i, const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_HC, const double _m, const yarp::sig::Matrix &_I, std::string _info, const NewEulMode _mode = NE_static, unsigned int verb = 0);

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

	//~~~~~~~~~~~~~~
	// set methods
	//~~~~~~~~~~~~~~

	void setMode(const NewEulMode _mode = NE_static);
	void setVerbose(unsigned int verb=1);
	void setInfo(std::string _info);
	void setSensorInfo(std::string _info);

	//~~~~~~~~~~~~~~
	// get methods
	//~~~~~~~~~~~~~~

	std::string getInfo() const;
	std::string getSensorInfo() const;

	

};


}//end namespace

#endif