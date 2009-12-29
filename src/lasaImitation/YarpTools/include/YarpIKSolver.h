#ifndef YARPIKSOLVER_H_
#define YARPIKSOLVER_H_

#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>

namespace MathLib{
	class IKSolver;
}
using namespace MathLib;

class YarpIKSolver
{
private:
    IKSolver *impl;

public:
    YarpIKSolver();
    ~YarpIKSolver();
    
    /// Allows to print out debug message
    void    SetVerbose(bool verbose=true);

    /// Initialization: prepare the system to process n DOFs and m constraints
    void    SetSizes(int dofs, int constraintsSize);
    /// Set the thresholds for 
    void    SetThresholds(double loose, double cut);    

    /// Removes all constraints limits on the outputs
    void    ClearLimits();
    
    /**
     * \brief Sets the constraints limits on the putput
     * \param low      Vector for low bounds values (should be <=0)
     * \param high     Vector for high bounds values (should be >=0)
     */  
    void    SetLimits(yarp::sig::Vector &low, yarp::sig::Vector &high);
    
    /// Sets the jacobian to the system
    void    SetJacobian(yarp::sig::Matrix & j);
    
    /// Sets the weights on the degrees of freedom (useful if the redundancies are on the DOFs)
    void    SetDofsWeights(yarp::sig::Vector &v);
    /// Sets the weights on the constraints (useful if the redundancies are on the constraints)
    void    SetConstraintsWeights(yarp::sig::Vector &v);
    /// Selects which constraints should be used of not (all of them by default...)
    void    SetValidConstraints(yarp::sig::Vector &constr);

    /// Sets the target values to reach (Size given by the number of constraints)
    void    SetTarget(yarp::sig::Vector &v);
    /// Sets the target for the null space (Size given by the number of DOFs)
    void    SetNullTarget(yarp::sig::Vector &null);

    /// Compute the inverse kinematic solution
    void    Solve();
    
    /// Get the result
    void        GetOutput(yarp::sig::Vector &output);
    /// Get the actual target values produces
    void        GetTargetOutput(yarp::sig::Vector &output);
    /// Get the error between produced target and the requested one
    void        GetTargetError(yarp::sig::Vector &error);
    /// Get the squared norm of the error between produced target and the requested one
    double      GetTargetError();
};


/*
    void    HackSetPose7(yarp::sig::Vector &pose);
    void    HackSetTargetPose7(yarp::sig::Vector &pose);
    void    HackAddTargetPose6(yarp::sig::Vector &pose);
    void    HackGetTargetDelta(yarp::sig::Vector &delta);
*/

#endif /*IKSOLVER_H_*/
