#ifndef IKSUBSOLVER_H_
#define IKSUBSOLVER_H_

#include "MathLib.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

/**
 * \class IKSubSolver
 * 
 * \ingroup MathLib
 * 
 * \brief A class for solving inverse kinematic in a pseudo-inverse fashion with optimization
 * 
 * This is a loose inverse kinematic algorithm... By loose, I mean that constraint are only satisfied if possible...
 * This way some singularities can be solved pretty well...
 */

class IKSubSolver
{
public:
    friend class IKGroupSolver;
public:
            /// Constructor
            IKSubSolver();
            /// Destructor
    virtual ~IKSubSolver();
    
            /// Allows to print out debug message
            void    SetVerbose(bool verbose=true);
        
            /// Initialization: prepare the system to process n DOFs and m constraints
            void    SetSizes(int dofs, int constraintsSize);
            /// Set the thresholds for 
            void    SetThresholds(REALTYPE loose, REALTYPE cut);    

            /// Sets the jacobian to the system
            void    SetJacobian(Matrix & j);
            
            /// Sets the weights on the degrees of freedom (useful if the redundancies are on the DOFs)
            void    SetDofsWeights(Matrix &m);
            void    SetDofsWeights(Vector &v);
            /// Sets the weights on the constraints (useful if the redundancies are on the constraints)
            void    SetConstraintsWeights(Matrix &m);
            void    SetConstraintsWeights(Vector &v);

            /// Sets the target values to reach (Size given by the number of constraints)
            void    SetTarget(Vector &v);
        
            /// Compute the inverse kinematic solution
            void    Solve();
            
            /// Get the result
            Vector&     GetOutput();
            /// Get the actual target that output values produces
            Vector&     GetTargetOutput();
            /// Get the error between produced target and the requested one
            Vector&     GetTargetError();
            /// Get the squared norm of the error between produced target and the requested one
            REALTYPE    GetTargetErrorNorm();
            REALTYPE    GetTargetErrorNorm2();
            /// Get the null space
            Matrix&     GetNullSpace();
            /// Get the jacobian
            Matrix&     GetJacobian();

protected:
            void    Resize();  

protected:
    bool                bVerbose;

    int                 mConstraintsSize;
    int                 mDofs;

    Matrix              mJacobian;

    Matrix              mDofsWeights;
    Matrix              mConstrWeights;

    Vector              mDesiredTarget;
    Vector              mOutputTarget;
    Vector              mErrorTarget;

    Vector              mOutput;

    REALTYPE            mLooseThreshold;
    REALTYPE            mCutThreshold;

    Matrix              mJW;
    Matrix              mWtJt;
    Matrix              mWtJtJW;

    Matrix              mTriMatrix;

    Matrix              mEigenVectors;
    Matrix              mRedEigenVectors;
    Matrix              mNullEigenVectors;
    Matrix              mEigenVectorsTranspose;
    Matrix              mRedEigenVectorsTranspose;
    Matrix              mNullEigenVectorsTranspose;

    Vector              mEigenValues;
    Vector              mRedEigenValues;
    Vector              mRedInvEigenValues;

    Vector              mCondNumbersVector;

    int                 mEigenSteps;

    int                 mRank;

    Matrix              mRedPseudoInverseTmpMatrix;
    Matrix              mRedPseudoInverse;
    Matrix              mWeightedRedPseudoInverse;

};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif

#endif /*IKSOLVER_H_*/
