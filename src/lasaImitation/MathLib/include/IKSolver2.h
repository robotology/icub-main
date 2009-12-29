#ifndef IKSOLVER2_H_
#define IKSOLVER2_H_

#include "MathLib.h"

class IKSolver2
{
public:
            IKSolver2();
    virtual ~IKSolver2();
    
            void    Solve();
        
            void    ClearLimits();
            void    SetLimits(Vector &low, Vector &high);
            
            void    SetJacobian(Matrix & j);
            void    SetDofsWeights(Vector &v);
            void    SetConstraintsWeights(Vector &v);
            
            void    SetTarget(Vector &v);
            
            void    SetValidConstraints(Vector &constr);
        
            void    GetOutput(Vector &output);
        
            void    SetSizes(int dofs, int constraintsSize);
            void    SetThresholds(REALTYPE loose, REALTYPE cut);    
            void    SetNullTarget(Vector &null);
  
  
protected:
            void    Resize();  
            void    StepSolve();  

protected:
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


#endif /*IKSOLVER_H_*/
