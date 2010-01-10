/** @file Potential.h Header file the Potential class.
*
* Version information : 1.0
*
* Date 04/05/2009
*
*/
/*
* Copyright (C) 2009 Sebastien Gay, EPFL
* RobotCub Consortium, European Commission FP6 Project IST-004370
* email: sebastien.gay@epfl.ch
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

#ifndef POTENTIAL__H
#define POTENTIAL__H


#define d0 1
#define kp 2
#define kn 2
#define POSITIVE_GRADIENT_EXPRESSION (1/distance - 1/d0)*(1 / pow(distance,2)) // the gradient at point P is inversely proportional to distance^kp.
#define NEGATIVE_GRADIENT_EXPRESSION (1 / pow(distance, kn)) // the gradient at point P is inversely proportional to distance^kn.
#define EPSILON_LENGTH 0.01

#include <yarp/sig/Vector.h>
using namespace yarp::sig;

class Potential
{
private:
	Vector position;
    double radius;
	double potential;

	bool reachingMode;
    //bool reached;
public:

	/**
    * Default constructor of the GeneratorThread class.
    * Does nothing
    */
	Potential(void);

	/**
    * Constructor of the GeneratorThread class.
    * Defines a potentian at position x, y with radius myRadius and potential myPotential
    */
	Potential(double x, double y, double myRadius, double myPotential);

	/**
    * Destructor of the GeneratorThread class.
    * Does nothing
    */
	~Potential(void);

	/**
    * Returns the position of the potential in the reference frame of the robot.
    */
	const Vector &GetPosition(void) const;

	/**
    * Returns the radius of the potential
    */
    double GetRadius(void) const;

	/**
    * Returns the potential value of the potential
    */
    double GetPotential(void) const;

	/**
    * Computes the force vector defining the influence of the potential on the robot.
    */
	Vector GetPotentialVector(void) const;

	/**
    * Translates the potential in the robot reference frame
    */
	void Translate(const Vector &t);

	/**
    * Rotates the potential around the robot position.
    */
	void Rotate(double angle);
	
	/**
	* Sets the potential in the reached state.
	*/
    void SetReached(void);
	
	/**
	* Moves the potential to the specified position in the robot reference frame.
	*/
	void MoveTo(double x, double y);
};

#endif //POTENTIAL__H
