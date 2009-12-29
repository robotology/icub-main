#ifndef IKSOLVER_H_
#define IKSOLVER_H_

#include "MathLib.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

/**
 * \class IKSolver
 * 
 * \ingroup MathLib
 * 
 * \brief A class for solving inverse kinematic in a pseudo-inverse fashion with optimization
 * 
 * This is a loose inverse kinematic algorithm... By loose, I mean that constraint are only satisfied if possible...
 * This way some singularities can be solved pretty well...
 */

class IKSolver
{
public:
			/// Constructor
            IKSolver();
			/// Destructor
    virtual ~IKSolver();
	
			/// Allows to print out debug message
			void    SetVerbose(bool verbose=true);

			/// Initialization: prepare the system to process n DOFs and m constraints
            void    SetSizes(int dofs, int constraintsSize);
			/// Set the thresholds for 
            void    SetThresholds(REALTYPE loose, REALTYPE cut);    

			/// Removes all constraints limits on the outputs
            void    ClearLimits();
			
			/**
			 * \brief Sets the constraints limits on the putput
			 * \param low      Vector for low bounds values (should be <=0)
			 * \param high     Vector for high bounds values (should be >=0)
			 */  
            void    SetLimits(Vector &low, Vector &high);
            
			/// Sets the jacobian to the system
            void    SetJacobian(Matrix & j);
			
			/// Sets the weights on the degrees of freedom (useful if the redundancies are on the DOFs)
            void    SetDofsWeights(Vector &v);
			/// Sets the weights on the constraints (useful if the redundancies are on the constraints)
            void    SetConstraintsWeights(Vector &v);
			/// Selects which constraints should be used of not (all of them by default...)
            void    SetValidConstraints(Vector &constr);

			/// Sets the target values to reach (Size given by the number of constraints)
            void    SetTarget(Vector &v);
			/// Sets the target for the null space (Size given by the number of DOFs)
            void    SetNullTarget(Vector &null);
        
			/// Compute the inverse kinematic solution
            void    Solve();
			
			/// Get the result
            void        GetOutput(Vector &output);
            /// Get the actual target values produces
			void        GetTargetOutput(Vector &output);
			/// Get the error between produced target and the requested one
            void        GetTargetError(Vector &error);
			/// Get the squared norm of the error between produced target and the requested one
            REALTYPE    GetTargetError();
        
  
  
protected:
            void    Resize();  
            void    StepSolve();  

protected:
    bool                bVerbose;

    IndicesVector       mValidConstraints;
    Vector              mLimits[2]; 
  Vector              mCurrLimits[2]; 

  Matrix              mFullJacobian;
  Matrix              mJacobian;
  
  Matrix              mInputConstrWeights;
  Matrix              mInputDofsWeights;
  Matrix              mWeights;
  Matrix              mWeightsTranspose;
  Matrix              mConstrWeights;

  Vector              mFullDesiredTarget;
  Vector              mDesiredTarget;
  Vector              mOffsetTarget;
  Vector              mActualTarget;

  Vector              mOutputTarget;
  Vector              mFullOutputTarget;

  Vector              mNullTarget;
  bool                bUseNullTarget;
  
  Matrix              mJWt;
  Matrix              mWJt;
  Matrix              mWJtJWt;
  
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
  int                 mConstraintsSize;
  int                 mDofs;
  int                 mValidConstraintsSize;
  REALTYPE            mLooseThreshold;
  REALTYPE            mCutThreshold;
  
  Matrix              mRedPseudoInverseTmpMatrix;
  Matrix              mRedPseudoInverse;
  Matrix              mWeightedRedPseudoInverse;
  
  Vector              mStepOutput;
  Vector              mOutput;
  Vector              mOutputOffset;

  
  Vector              mOutputLimitsError;  
  
  IndicesVector       mValidJoints;
  IndicesVector       mJointMapping;
  IndicesVector       mInverseJointMapping;
  
  Matrix              mOutputNullSpace;
  
};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif

#endif /*IKSOLVER_H_*/
