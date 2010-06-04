/* 
 * Copyright (C) <2010> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Serena Ivaldi
 * email:   serena.ivaldi@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/** 
* @ingroup icub_tools
*
* \defgroup torqueObserver torqueObserver
*
* This module takes force/moment measurements from FT sensors placed 
* in the middle of the iCub limbs, and compute joint torques for each 
* link in the kinematic chain. Forces and moments can be also retrieved.
* 
* Version information: currently under development.
* Date: 01/06/2010
*
* \section intro_sec Description
* 
* This module takes force/moment measurements from FT sensors placed 
* in the middle of the iCub limbs, and compute joint torques for each 
* link in the kinematic chain. Forces and moments can be also retrieved.
* The 6-axis force/torque (FT) sensor's measurements are read through 
* an input YARP port. This module compute the joint torques by calling 
* Newton-Euler algorithm (see iDyn library) applied to the dynamic 
* chain, and provides joint torques and end-effector force/moment to 
* proper output YARP ports.
* All the computations are perfomed relying on rigid body dynamics 
* and CAD parameters are used.
*
* \section lib_sec Libraries
*
* - YARP 
* - iDyn
* - ctrlLib
*
* \section parameters_sec Parameters
* 
* <b>Command-line Parameters</b>
* 
* The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
* (e.g. --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
*
* --name \e moduleName \n
* The parameter \e moduleName is used to tag all the opened ports, 
* which will have the prefix \e /moduleName/. \n
* Default value: \e ftObs
* 
* --robot \e robotName \n
* The parameter \e robotName is used to find the device ports, which will have the 
* prefix \e robotName. \n
* Default value: \e icub
*
* --part \e robotPart \n
* The parameter \e robotPart is used to find the device ports, which will have the tag
* \e robotPart after \e robotName, i.e. \e /robotName/robotPart/. \n
* Default value: \e left_arm
*
* --rate \e r \n
* The parameter \e r defines the rate of thread, in \e ms. The minimum safe rate is \e 20ms. \n
* Default value: \e 100ms 
*
* <b> Configuration file Parameters </b>
*
* None.
*
* \section portsa_sec Ports Accessed
* 
* Input port must be manually attached to the one sending the FT sensor
* measurements. As an example, if the port sending FT measures is 
* \e /icub/right_arm/analog:o , the yarp command to connect the 
* analog port of the sensor to the input one created here, e.g. 
* \e /ftObs/right_arm/FT:i , is : \n
* \code yarp connect /icub/right_arm/analog:o /ftObs/right_arm/FT:i \endcode \n
* See the example section for more details.
*                      
* \section portsc_sec Ports Created
*
* Input ports
*
*  - \e /moduleName/robotPart/FT:i \n
*    The port reading FT sensor measurements
*
* Output ports
*
*  - \e /moduleName/robotPart/torques:o \n
*    The port sending the \e robotPart joint torques, as a vector \e (Nx1) , where \e N is the number of DOF of the \e robotPart
*
*  - \e /moduleName/robotPart/FTendeff:o \n
*    The port sending the \e robotPart 's end-effector force and moment, as a \e (6x1) vector
*    
* \section in_files_sec Input Data Files
*
*  None
*
* \section out_data_sec Output Data Files
*
*  None
*
* \section conf_file_sec Configuration Files
*
*  None
* 
* \section tested_os_sec Tested OS
*
* Windows
*
* \section example_sec Example Instantiation of the Module
* 
* Here is an example of command-line instantiation, where we want
* to observe icub's right arm, at 50ms rate: 
*
* \code torqueObserver --name ftObs --robot icub --part right_arm --rate 50  \endcode 
*
* The following listening port, for reading the FT sensor measurements, is created: \n
* \e /ftObs/right_arm/FT:i \n
* In order to read properly the sensor measurements, the input port must be connected to the 
* analog port of the sensor: \n
* \code yarp connect /icub/right_arm/analog:o /ftObs/right_arm/FT:i \endcode 
*
* The following output ports, for sending joint torques and end effector force/moment, are created: \n
* \e /ftObs/right_arm/torques:o \n
* \e /ftObs/right_arm/FTendeff:o \n
*  
* 
*
* \author Serena Ivaldi
* 
* Copyright (C) 2010 RobotCub Consortium
* 
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
* 
* 
*
**/ 

#ifndef __TORQUEOBSERVER_H__
#define __TORQUEOBSERVER_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iFC.h>

#include <iostream>
#include <iomanip>
#include <string.h>



/**
* A thread reading FT sensor measurements and computing joint torques for a certain limb.
**/
class SensToTorques: public yarp::os::RateThread
{
private:

	yarp::dev::PolyDriver	*pd_limb;
	yarp::dev::PolyDriver	*pd_torso;
	yarp::dev::IEncoders	*iencs;
	yarp::dev::IEncoders	*iencs_torso;

	std::string part;
	std::string type;

	yarp::sig::Vector *FTmeasure;
	yarp::sig::Vector sensorOffset;

	yarp::os::BufferedPort<yarp::sig::Vector> *port_i_FT;
	yarp::os::BufferedPort<yarp::sig::Vector> *port_o_Torques;
	yarp::os::BufferedPort<yarp::sig::Vector> *port_o_FTendef;

	ctrl::AWLinEstimator   *linEst;
    ctrl::AWQuadEstimator  *quadEst;
	ctrl::AWPolyElement		estElement;

	iDyn::iDynLimb *limb;
	iDyn::iDynSensor *sens;

	yarp::sig::Vector encoders;
	yarp::sig::Vector encodersTorso;
	yarp::sig::Vector q;
	yarp::sig::Vector dq;
	yarp::sig::Vector d2q;	
	yarp::sig::Vector w0;
	yarp::sig::Vector dw0;
	yarp::sig::Vector d2p0;
	yarp::sig::Vector Fend;
	yarp::sig::Vector Mend;
	yarp::sig::Vector FTsensor;
	yarp::sig::Vector Tau;
	yarp::sig::Vector FTendeff;

	yarp::sig::Vector updateVelocity(const yarp::sig::Vector &v);
	yarp::sig::Vector updateAcceleration(const yarp::sig::Vector &a);
	void calibrateOffset(const unsigned int Ntrials);

	void readAndUpdate();

	    
public:

	SensToTorques(int _rate, yarp::dev::PolyDriver *_pd_leg, yarp::os::BufferedPort<yarp::sig::Vector> *port_FT, yarp::os::BufferedPort<yarp::sig::Vector> *port_Torques, yarp::os::BufferedPort<yarp::sig::Vector> *port_FTendef, std::string _type);
	
	SensToTorques(int _rate, yarp::dev::PolyDriver *_pd_arm, yarp::dev::PolyDriver *_pd_torso, yarp::os::BufferedPort<yarp::sig::Vector> *port_FT, yarp::os::BufferedPort<yarp::sig::Vector> *port_Torques, yarp::os::BufferedPort<yarp::sig::Vector> *port_FTendef, std::string _type);
	  
	bool threadInit();
	  
	void run();
	  
	void threadRelease();


	  
};


class TorqueObserver: public yarp::os::RFModule
{
private:

	yarp::os::Property OptionsLimb;
	yarp::os::Property OptionsTorso;
	yarp::dev::PolyDriver *pd_limb;
	yarp::dev::PolyDriver *pd_torso;

	SensToTorques *sens2torques;

	yarp::os::BufferedPort<yarp::sig::Vector> port_i_FT;
	yarp::os::BufferedPort<yarp::sig::Vector> port_o_Torques;
	yarp::os::BufferedPort<yarp::sig::Vector> port_o_FTendef;

	std::string handlerPortName;
	yarp::os::Port handlerPort;      //a port to handle messages 

	/*
	* Check driver consistency
	* @return true if driver is working properly and encoders are correctlty viewed
	*/
	bool checkDriver(yarp::dev::PolyDriver *pd_limb);
	
public:

	/*
	* Constructor
	*/
	TorqueObserver();
	/*
	* Configure the module parameters
	* @return true if successful
	*/
	bool configure(yarp::os::ResourceFinder &rf); 
	/*
	* Interrupt, e.g. the ports
	* @return true if successful
	*/
   bool interruptModule();                     
   /*
   * Close and shut down the module
   * @return true if successful
   */
   bool close();     
   /*
   * Respond if necessary
   * @return true if successful, false otherwise (e.g. bad command)
   */
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   /*
   * @return period
   */
   double getPeriod(); 
   /*
   * Update module
   * @return true if successful
   */
   bool updateModule();



};



#endif //__TORQUEOBSERVER_H__


	
	

		
