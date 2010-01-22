/**
 * \defgroup optimalControl optimalControl 
 *    
 * @ingroup ctrlLib
 *  
 * Class for optimal control based on Discrete Algebraic Riccati Equation (DARE).
 * 
 * \section intro_sec Description
 * 
 * The class Riccati can be used for optimal control based on Discrete Algebraic Riccati Equation (DARE).
 * The DARE is used to compute the linear gain matrix of the feedback controls. 
 * In the following, the basics of a LQR (linear quadratic regulation) problem are reported.
 *
 * Given the linear system:
 * \f[ x_{i+1} = A x_i + B u_i \ , \ i=0,1,\ldots,N-1 \f]
 * with the known initial state \f$ x_0 = \hat{x} \f$, and the 
 * quadratic cost \f$ J \f$: 
 * \f[ J = \sum^{N-1}_{i=0} \left[ x^\top_i V x_i + u^\top_i P u_i \right] + x^\top_N V_N x_N \f]
 * with \f$ V=V^\top \geq 0 \f$, \f$ V_N=V^\top_N \geq 0 \f$, \f$ P=P^\top>0 \f$, the problem is to find the sequence of 
 * optimal controls \f$ u^\circ_0, \ldots, u^\circ_{N-1} \f$ minimizing \f$J\f$. 
 * The optimal controls can be found via dynamic programming, and a closed form solution can be found.
 * At time instant \f$i\f$ the optimal cost-to-go and control are:
 * \f[ \begin{array}J^\circ(x_{i}) = x^\top_{i} \ T_{i} \ x_{i} 
 * \\ u^\circ_{i} = - L_i \ x_{i} \end{array} \f] where \f$ L_i 
 * \f$ is: 
 * \f[ L_i = (P+B^\top T_{i+1} B)^{-1} B^\top T_{i+1} A \f]
 * whilst \f$ T_i \f$ is computed after the <b>discrete time 
 * algebraic Riccati equation</b>: 
 * \f[ T_N = V_N \ , \ T_i = V + A^\top [ T_{i+1} - T_{i+1} B (P+B^\top T_{i+1} B)^{-1} B^\top T_{i+1} ] A \f]
 *
 *
 * \section code_example_sec Example
 *  
 * \code 
 * Riccati r(A,B,V,P,VN,true);
 * r.solveRiccati(steps);
 * for(i=0; i<steps; i++)
 * {
 *    u=r.doLQcontrol(i,x);
 *    x=A*x+B*u;
 * } 
 * \endcode 
 * 
 * \author Serena Ivaldi
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 **/ 

#ifndef __RICCATI_H__
#define __RICCATI_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrlMath.h>


namespace ctrl
{

/**
* \ingroup optimalControl
*
* Classic Riccati recursive formula for optimal control in a LQ problem
*/
class Riccati
{
protected:
    yarp::sig::Matrix A, At;
    yarp::sig::Matrix B, Bt;
    yarp::sig::Matrix V, VN;
    yarp::sig::Matrix P;

    yarp::sig::Matrix TN, lastT;
	yarp::sig::Matrix *Ti;
	yarp::sig::Matrix *Li;
    
    yarp::sig::Vector x;

    size_t n;
    size_t m;
	int N;

	bool verbose;

public:
    /**
     * Constructor, with initialization of algebraic Riccati equation
     * 
     * @param _A  State transition matrix
     * @param _B  Control matrix
     * @param _V  State cost matrix
     * @param _P  Control cost matrix 
     * @param _VN Final state cost matrix
     */
	 Riccati(const yarp::sig::Matrix &_A, const yarp::sig::Matrix &_B,
             const yarp::sig::Matrix &_V, const yarp::sig::Matrix &_P,
             const yarp::sig::Matrix &_VN, bool verb=false);

	 /**
     * Get stored L_i matrix; call this function only after solveRiccati()
     * 
     * @param step The time index of the i-th matrix
     */
	 yarp::sig::Matrix L(int step);

	 /**
     * Get stored T_i matrix; call this function only after solveRiccati()
     * 
     * @param step The time index of the i-th matrix
     */
	 yarp::sig::Matrix T(int step);

    /**
     * Initialization of algebraic Riccati equation
     * 
     * @param _A  State transition matrix
     * @param _B  Control matrix
     * @param _V  State cost matrix
     * @param _P  Control cost matrix 
     * @param _VN Final state cost matrix
     */
	 void setProblemData(const yarp::sig::Matrix &_A, const yarp::sig::Matrix &_B,
                         const yarp::sig::Matrix &_V, const yarp::sig::Matrix &_P,
                         const yarp::sig::Matrix &_VN);

	 /**
     * Solve recursively discrete algebraic Riccati equation (DARE) 
     * and stores matrices Ti and Li, where i=0:N-1 is the time 
     * index 
     * 
     * @param steps The number N of steps of the finite horizon controller
     */
	 void solveRiccati(int steps);

	 /**
     * Compute the LQ feedback control, in the form: ret= - L(i) * x 
     * 
     * @param step The time index i
	 * @param x The state vector
     */
	 yarp::sig::Vector doLQcontrol(int step, const yarp::sig::Vector &x);

	 /**
     * Compute the LQ feedback control, in the form: u= - L(i) * x 
     * 
     * @param step The time index i
	 * @param x The state vector
	 * @param ret The control vector
     */
	 void doLQcontrol(int step, const yarp::sig::Vector &x, yarp::sig::Vector &ret);

	 /**
     * Enable or disable verbose feedback(that is, printing 
     * additional information) 
     * 
     * @param verb Flag for verbose mode
     */
	 void setVerbose(bool verb=true);
};

}


#endif


