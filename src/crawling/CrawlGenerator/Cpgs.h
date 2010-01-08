/** @file Cpgs.h Header file the Cpgs class.
*
* Version information : 1.0
*
* Date 04/05/2009
*
*/
/*
* Copyright (C) 2009 Sarah Degallier, EPFL
* RobotCub Consortium, European Commission FP6 Project IST-004370
* email: sarah.degallier@epfl.ch
* website: www.robotcub.org
*
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2
* or any later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*/

#ifndef CPGS_H
#define CPGS_H

#ifndef M_PI
#define M_PI        3.14159265358979323846   /**< the pi number */
#endif

#include <math.h>
#include <vector>
#include <yarp/String.h>

using namespace std;

/**
* A class to handle CPGs. 
* Encapsulates a CPG and its derivation methods for discrete and rythmic movements.
*/
class Cpgs
{

public:

    /**
    * Constructor of the Cpgs class.
    * Initialise open and fixed parameters of the CPG.
    * @param myNbDOFs Number of degrees of freedom of the part controlled by this CPG.
    * @param myNbLIMBs Number of limbs coupled to the part controlled by this CPG.
    */
	Cpgs(int myNbDOFs, int myNbLIMBs);

    /**
    * Destructor of the Cpgs class.
    * Frees the memory.
    */
	~Cpgs();
 
    /**
    * Integrates one step of the system.
    * Defines the equations of the CPG and integrate them using a simple euler integration method.
    * @param y MISSING.
    * @param at_states MISSING.
    */
	void integrate_step(double *y, double *at_states);

    /**
    * Prints the value of all the member variables.
    */
	void printInternalVariables();

	/**
    * Gets the angles of the arms.
    */
	void getArmAngles();

	/**
    * Returns the value of the derivatives dt.
    * @return the value of the derivatives dt.
    */
	double get_dt();

public:

	//parameters of the equations
	double om_stance, om_swing;/**< Frequency of the oscillations */ 
	double *ampl; /**< Output amplitude of the oscillations	*/
	double *parameters; /**< 2 parameters per dof: mu and g */	
	double **epsilon; /**< Internal coupling strength */
	double **theta; /**< Coupling phase in radians */
	double **theta_init; /**< Initial coupling phase in radians */
	double *external_coupling; /**< Coupling between the limbs */
	double *next_external_coupling; /**< next coupling values between the limbs */
	double *dydt; /**< Derivatives */
	double *r; /**< Radius */

	//open parameters
	double *g; /**< Targets */ 
	double *m; /**< Amplitudes */
	yarp::String partName;  /**< Name of the part beeing controlled */
        
    double turnAngle;/**<angle of rotation of the torso roll */
    
    bool feedbackable; /**<Has this part a feedback? (head, torso: no)*/
    bool feedback_on; /**<If feedbackable, is the feedback information available*/
    double contact[2]; /**<Touch sensor information  for on arm (leg) and the other*/

	
private:

	int nbDOFs; /**< Number of degrees of freedom of the part controlled by this CPG */
	int nbLIMBs; /**< Number of limbs coupled to the part controlled by this CPG*/
	int Cpgs_size; /**< Size of the CPG */
	int controlled_param; /**< MISSING */

	//equations parameters 
	const double a; /**< rate of convergence of the rhythmic system */
	const double b; /**< rate of convergence of the discrete system */
	const double m_off; /**< value to turn of the oscillations */
	const double m_on; /**< value to turn on the oscillations */
	const double b_go; /**< rate of convergence of the go command */
	const double u_go; /**< max value of the go command */
	const double dt; /**< integration step in seconds */
	const double c; /**< param for sw/st switching */
};

#endif //CPGS_H

