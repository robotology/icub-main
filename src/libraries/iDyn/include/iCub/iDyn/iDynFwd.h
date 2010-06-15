/**
 * \defgroup iDynFwd iDynFwd 
 *    
 * @ingroup iDyn
 *  
 * Classes for link/joint force/moment/torque computation 
 * using Newton-Euler recursive formulation, given single 
 * FT sensor measurements.
 * 
 * \section intro_sec Description
 * 
 * iDynSensor is a class which attachs a FT sensor into a 
 * iDynChain; exploiting the sensor force/moment measurements, 
 * and applying the inverse formula of Newton-Euler recursive 
 * algorithm, it is possible to compute joint torques given
 * single FT measurements. The dynamical parameters necessary 
 * to the algorithm are read automatically from the iDynChain 
 * (consisting of iDynLinks), whereas the FT sensor must be 
 * set from iDynSensor constructor. 
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows
 *
 * \section example_sec Example
 *
 * Set a iDynSensor for iCub's left arm, with 
 * force/moment computation in the static case (STATIC), and
 * verbose (VERBOSE==1) output:
 *
 * <tt> iCubArmDyn *arm = new iCubArmDyn("left"); </tt> \n
 * <tt> iDynSensorArm armWSensorSolver = new iDynSensorArm(arm,STATIC,VERBOSE); </tt> \n
 *
 * Note that by setting the arm as "left", the sensor is automatically set
 * as the left sensor of the arm.
 * The chain must be updated with the current angle configuration, and the sensor 
 * measurements must be put into two (3x1) vectors or one (6x1).
 *
 * <tt> arm->setAng(q); </tt> \n
 * <tt> Vector FM = measureFromSensor();</tt> \n
 * <tt> Vector F = measureForceFromSensor();</tt> \n
 * <tt> Vector M = measureMomentFromSensor();</tt> \n
 *
 * In both cases, we can give the compute method our measurements:
 *
 * <tt> armWSensorSolver->computeFromSensorNewtonEuler(FM); </tt> \n
 * <tt> armWSensorSolver->computeFromSensorNewtonEuler(F,M); </tt> \n
 *
 * The arm joints force/moment/torque can be now retrieved:
 *
 * <tt> Matrix F = armWSensorSolver->getForces(); </tt> \n
 * <tt> Matrix Mu = armWSensorSolver->getMoments(); </tt> \n
 * <tt> Vector Tau = armWSensorSolver->getTorques(); </tt> \n
 *
 * \author Serena Ivaldi
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * 
 **/ 

#ifndef __IDYNFWD_H__
#define __IDYNFWD_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/ctrlMath.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/iDyn/iDyn.h>
#include <deque>
#include <string>


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
	class OneLinkNewtonEuler;
	class BaseLinkNewtonEuler;
	class FinalLinkNewtonEuler;
	class SensorLinkNewtonEuler;
	class OneChainNewtonEuler;
	class iDynInvSensor;
	class iGenericFrame;
    class iFrameOnLink;
	class iFTransformation;


/**
* \ingroup iDynFwd
*
* A class for computing forces and torques in a iDynChain, 
* when a force/torque sensor is placed in the middle of the 
* kinematic chain and it is the only available sensor for 
* measuring forces and moments; the sensor position in the 
* chain must be set; the computation of joint forces, moments 
* and torques is performed by an Inverse Newton-Euler method.
* 
*/
class iDynSensor: public iDynInvSensor
{

public:

	/**
    * Constructor without FT sensor: the sensor must be set with setSensor()
	* @param _c a pointer to the iDynChain where the sensor is placed on
	* @param _info a string with information
	* @param _mode the analysis mode (static/dynamic)
	* @param verb flag for verbosity
    */
	iDynSensor(iDyn::iDynChain *_c, std::string _info, const NewEulMode _mode = DYNAMIC, unsigned int verb = NO_VERBOSE);


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
	iDynSensor(iDyn::iDynChain *_c, unsigned int i, const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_HC, const double _m, const yarp::sig::Matrix &_I, std::string _info, const NewEulMode _mode = DYNAMIC, unsigned int verb = NO_VERBOSE);

	/**
	 * Set the sensor measured force and moment
     * @param F the sensor force (3x1)
	 * @param Mu the sensor moment (3x1)
	 * @return true if the operation is successful, false otherwise (ie wrong vector size)
	 */
	bool setSensorMeasures(const yarp::sig::Vector &F, const yarp::sig::Vector &Mu);

	/**
	 * Set the sensor measured force and moment at once. The measure vector (6x1) is made
	 * of 0:2=force 3:5=moment
     * @param FM the sensor force and moment (6x1)
	 * @return true if the operation is successful, false otherwise (ie wrong vector size)
	 */
	bool setSensorMeasures(const yarp::sig::Vector &FM);

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//   main computation methods
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	/**
	 * The main computation method: given the FT sensor measurements, compute forces moments 
	 * and torques in the iDynChain. A forward pass of the classical Newton-Euler method is 
	 * run, to retrieve angular and linear accelerations. Then, from sensor to end-effector
	 * the inverse Newton-Euler formula is applied to retrieve joint forces and torques, while
	 * from sensor to base the classical backward pass is run.
     * @param F the sensor force (3x1)
	 * @param Mu the sensor moment (3x1)
	 * @return true if the operation is successful, false otherwise (ie wrong vector size)
	 */
	bool computeFromSensorNewtonEuler(const yarp::sig::Vector &F, const yarp::sig::Vector &Mu);

	/**
	 * The main computation method: given the FT sensor measurements, compute forces moments 
	 * and torques in the iDynChain. A forward pass of the classical Newton-Euler method is 
	 * run, to retrieve angular and linear accelerations. Then, from sensor to end-effector
	 * the inverse Newton-Euler formula is applied to retrieve joint forces and torques, while
	 * from sensor to base the classical backward pass is run.
     * @param FMu the sensor force and moment (6x1)
	 * @return true if the operation is successful, false otherwise (ie wrong vector size)
	 */
	bool computeFromSensorNewtonEuler(const yarp::sig::Vector &FMu);

	/**
	 * The main computation method: given the FT sensor measurements, compute forces moments 
	 * and torques in the iDynChain. A forward pass of the classical Newton-Euler method is 
	 * run, to retrieve angular and linear accelerations. Then, from sensor to end-effector
	 * the inverse Newton-Euler formula is applied to retrieve joint forces and torques, while
	 * from sensor to base the classical backward pass is run.
	 * This method only perform the computations: the force and moment measured on the sensor
	 * must be set before calling this method using setSensorMeasures()
	 */
	void computeFromSensorNewtonEuler();

	/**
	 * The main computation method: given the FT sensor measurements, compute forces moments 
	 * and torques in the iDynChain. The kinematic pass is already performed. Only the wrench 
	 * computation are performed here: from sensor to end-effector
	 * the inverse Newton-Euler formula is applied to retrieve joint forces and torques, while
	 * from sensor to base the classical backward pass is run.
	 * This method only perform the computations: the force and moment measured on the sensor
	 * must be set before calling this method using setSensorMeasures()
	 * This method is called by iDynSensorNode.
	 */
	void computeWrenchFromSensorNewtonEuler();

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//  get methods, overloaded from iDyn
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
    * @return a Nx1 vector with the torques
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
    * Returns the end-effector force-moment as a single (6x1) vector
    * @return a 6x1 vector with the the end-effector force-moment
    */
	yarp::sig::Vector getForceMomentEndEff() const;


	/**
	* Destructor
	*/
	~iDynSensor();
};




/**
* \ingroup iDynFwd
*
* A class for computing joint force/moment/torque of an iCub
* arm (left/right) given the FT measurements of the sensor placed
* in the middle of the arm. The sensor
* parameters are automatically set by chosing left or right
* during initialization of the iCubArmDyn.
* 
*/
class iDynSensorArm : public iDynSensor
{

public:

	/**
    * Constructor: the sensor is automatically set with "right" or "left" choice
	* @param _c a pointer to the iCubArmDyn where the sensor is placed on
	* @param _mode the analysis mode (static/dynamic/etc)
	* @param verb flag for verbosity
    */
	iDynSensorArm(iDyn::iCubArmDyn *_c, const NewEulMode _mode = DYNAMIC, unsigned int verb = NO_VERBOSE);

	/**
	* @return type the arm sensor type: left/arm
	*/
	std::string getType() const;


};


/**
* \ingroup iDynFwd
*
* A class for computing joint force/moment/torque of an iCub
* arm (left/right) given the FT measurements of the sensor placed
* in the middle of the arm. The sensor
* parameters are automatically set by chosing left or right
* during initialization of the iCubArmDyn.
* 
*/
class iDynSensorArmNoTorso : public iDynSensor
{

public:

	/**
    * Constructor: the sensor is automatically set with "right" or "left" choice
	* @param _c a pointer to the iCubArmDyn where the sensor is placed on
	* @param _mode the analysis mode (static/dynamic/etc)
	* @param verb flag for verbosity
    */
	iDynSensorArmNoTorso(iDyn::iCubArmNoTorsoDyn *_c, const NewEulMode _mode = DYNAMIC, unsigned int verb = NO_VERBOSE);

	/**
	* @return type the arm sensor type: left/arm
	*/
	std::string getType() const;


};

/**
* \ingroup iDynFwd
*
* A class for computing joint force/moment/torque of an iCub
* leg (left/right) given the FT measurements of the sensor placed
* in the middle of the leg. The sensor
* parameters are automatically set by chosing left or right
* during initialization of iCubLegDyn.
* 
*/
class iDynSensorLeg : public iDynSensor
{

public:

	/**
    * Constructor: the sensor is automatically set with "right" or "left" choice
	* @param _c a pointer to the iCubLegDyn where the sensor is placed on
	* @param _mode the analysis mode (static/dynamic/etc)
	* @param verb flag for verbosity
    */
	iDynSensorLeg(iDyn::iCubLegDyn *_c, const NewEulMode _mode = DYNAMIC, unsigned int verb = NO_VERBOSE);

	/**
	* @return type the leg sensor type: left/arm
	*/
	std::string getType() const;


};















}//end namespace

#endif