#include "IKSolver.h"
#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

IKSolver::IKSolver(){    
    SetSizes(0,0);
    SetThresholds(0.005,0.001);
    ClearLimits();
    bVerbose = false;
}
IKSolver::~IKSolver(){
    SetSizes(0,0);   
}
void    IKSolver::SetVerbose(bool verbose){
    bVerbose = verbose;
}

void  IKSolver::Solve(){
    if(bVerbose) cerr << "IKSolver: Solving"<<endl;
        
    mWeights       = mInputDofsWeights;
    mCurrLimits[0] = mLimits[0];
    mCurrLimits[1] = mLimits[1];
    mOutput.Zero();
    
    if(mValidConstraintsSize<=0){
        mFullOutputTarget.Zero();
        return;    
    }
    
    mFullJacobian.GetRowSpace(mValidConstraints,mJacobian);
    mFullDesiredTarget.GetSubVector(mValidConstraints,mDesiredTarget);

    mInputConstrWeights.GetMatrixSpace(mValidConstraints,mValidConstraints,mConstrWeights);

    mLimitsOffset.Zero();
    for(int i=0;i<mDofs;i++){
        if(mCurrLimits[0][i]<mCurrLimits[1][i]){
            if(mCurrLimits[0][i]>0.0){
                mCurrLimits[1][i]  -= mCurrLimits[0][i];
                mLimitsOffset[i]    = mCurrLimits[0][i];
                mCurrLimits[0][i]   = 0.0;
            }else if(mCurrLimits[1][i]<0.0){
                mCurrLimits[0][i]  -= mCurrLimits[1][i];
                mLimitsOffset[i]    = mCurrLimits[1][i];
                mCurrLimits[1][i]   = 0.0;
            }
        }
    }
    mJacobian.Mult(mLimitsOffset,mLimitsOffsetTarget);
    mDesiredTarget -= mLimitsOffsetTarget;
    mOutput += mLimitsOffset;
    
    int stepCnt =0;
    while(1){
        if(bVerbose) cerr << "IKSolver: Pass <"<< stepCnt <<">"<<endl;
        
        StepSolve();

        // Checking Limits
        for(int i=0;i<mDofs;i++){
            mOutputLimitsError[i] = 1.0;        
            if(mCurrLimits[0][i]<mCurrLimits[1][i]){            
                if(mStepOutput[i]<mCurrLimits[0][i])
                    mOutputLimitsError[i] = fabs(mCurrLimits[0][i]/mStepOutput[i]);
                else if(mStepOutput[i]>mLimits[1][i])
                    mOutputLimitsError[i] = fabs(mCurrLimits[1][i]/mStepOutput[i]);             
            }
        }   
        double minOutputLimitError=1.0;         
        int minId=-1;
        for(int i=0;i<mDofs;i++){
            if(minOutputLimitError>mOutputLimitsError[i]){
                minOutputLimitError = mOutputLimitsError[i];
                minId = i;
            }
        }
        if(minId>=0){
            if(bVerbose) cerr << "IKSolver: Pass <"<< stepCnt <<"> Limits reached..."<<endl;
            for(int i=0;i<mDofs;i++){
                if(minOutputLimitError == mOutputLimitsError[i]){
                    if(bVerbose) cerr << "IKSolver: Pass <"<< stepCnt <<"> Locking DOF <"<<i<<">"<<endl;
                    mWeights(i,i) = 0.0;
                }
            }
        }
        mStepOutput *= minOutputLimitError;
        for(int i=0;i<mDofs;i++){
            if     (mStepOutput[i]>0) mCurrLimits[1][i] -= mStepOutput[i]; 
            else if(mStepOutput[i]<0) mCurrLimits[0][i] -= mStepOutput[i];
            mCurrLimits[1][i] = MAX(0,mCurrLimits[1][i]); 
            mCurrLimits[0][i] = MIN(0,mCurrLimits[0][i]); 
        }        

        if(minOutputLimitError==1.0){
            mOutput += mStepOutput;
            if(bVerbose) cerr << "IKSolver: Pass <"<< stepCnt <<"> Success"<<endl;
            break;
        }else{
            stepCnt ++;
            mDesiredTarget -= mJacobian * mStepOutput;    
            mOutput += mStepOutput;
        }
    }
    
    mOutput+=(mNullEigenVectors*(mNullEigenVectorsTranspose*mNullTarget));
    
    mJacobian.Mult(mOutput,mOutputTarget);
    mFullOutputTarget.Zero();
    mFullOutputTarget.SetSubVector(mValidConstraints,mOutputTarget);
    
    if(bVerbose) cerr << "IKSolver: Done"<<endl;
}

void  IKSolver::StepSolve(){
  Matrix tmp;

  // W = Vn' * W;  
  mWeights.Transpose(mWeightsTranspose);
  // y = x - J*off  
  mOffsetTarget.Zero();

  //mDesiredTarget.Sub(mJacobian.Mult(mOffsetTarget,mActualTarget),mActualTarget);
  mActualTarget = mDesiredTarget; 
  // A = J*W'
  mConstrWeights.Mult(mJacobian.Mult(mWeightsTranspose,tmp),mJWt);
  
  // AtA = A'*A
  mJWt.Transpose(mWJt);
  mWJt.Mult(mJWt,mWJtJWt);
  //mWJtJWt.Print();
  //mWJtJWt.Print();
  // [V D] = EigenSort(AtA);
  mWJtJWt.Tridiagonalize(mTriMatrix,mEigenVectors);
  mEigenSteps = 20;
  mTriMatrix.TriEigen(mEigenValues, mEigenVectors, mEigenSteps);
  mEigenVectors.SortColumnAbs(mEigenValues);
  mEigenVectors.Transpose(mEigenVectorsTranspose);
  
  // Condition numbers
  if(!Matrix::IsInverseOk()){
    if(bVerbose) cerr << "IKSolver:    Unable to perform eigen value decomposition: too many iterations"<<endl;
    mCondNumbersVector.Zero();
  }else{  
      if((fabs(mEigenValues(0))<EPSILON)||isnan(mEigenValues(0))){
        mCondNumbersVector.Zero();  
      }else{
        mCondNumbersVector[0] = 1.0f;
        for(int i=1;i<mConstraintsSize;i++)
          mCondNumbersVector[i] = fabs(mEigenValues(i)/mEigenValues(0)); 
      }  
  }
  //mCondNumbersVector.Print();
  //mCondNumbersVector.Print();
  // Rank
  int maxRank = MIN(mConstraintsSize,mDofs);
  mRank = maxRank;
  for(int i=0;i<maxRank;i++){
    if(mCondNumbersVector[i]<mCutThreshold){
      mRank = i;
      break;
    }
  }
  if(bVerbose){
    if(maxRank != mRank)
     cerr << "IKSolver:   Jacobian is not full rank: "<<mRank<<"/"<<maxRank<<endl;
  }   


  //if(mRank!=mConstraintsSize) cerr<<mRank<<endl;
  //cerr << "Rank: "<<mRank<<" "<<mConstraintsSize<<endl;
  // If positive rank... do what's possible
  if(mRank>0){      
    // Vr  = V(1:rank)
    mEigenVectors.GetColumnSpace(0,mRank,mRedEigenVectors);
    mEigenVectorsTranspose.GetRowSpace(0,mRank,mRedEigenVectorsTranspose);

    // Dr  = D(1:rank,1:rank)
    mRedEigenValues.Set(mEigenValues);
    mRedEigenValues.Resize(mRank);
    mRedInvEigenValues.Resize(mRank,false);
    for(int i=0;i<mRank;i++){
      mRedInvEigenValues(i) = R_ONE/mRedEigenValues(i);
    }

    // Smoothing when close to singular  
    for(int i=1;i<mRank;i++){
      if(mCondNumbersVector(i) < mLooseThreshold){
        if(bVerbose) cerr << "IKSolver:   Loose mode set: Multiplying eigenvalue <"<< mRedInvEigenValues(i) <<"> with condition number <"<<mCondNumbersVector(i)<<"> with <"<< (mCondNumbersVector(i)-mCutThreshold)/(mLooseThreshold-mCutThreshold) <<">"<<endl;
        mRedInvEigenValues(i) *= (mCondNumbersVector(i)-mCutThreshold)/(mLooseThreshold-mCutThreshold);
        //cerr <<"loosing"<<endl;
        //exit(0); 
      }
    }
     
    // th  = W'*V1r*inv(D)*V1r'*A'*y;
    //mOutput = weightsTranspose*redEigenVectors*invEigenValuesDiag*redEigenVectorsTranspose*WJt*virtualTarget;
    //mRedPseudoInverseTmpMatrix;
    
    mRedEigenVectors.MultColumn(mRedInvEigenValues,mRedPseudoInverseTmpMatrix);
    mRedPseudoInverseTmpMatrix.Mult(mRedEigenVectorsTranspose,mRedPseudoInverse);
    mWeightsTranspose.Mult(mRedPseudoInverse,mRedPseudoInverseTmpMatrix);
    mRedPseudoInverseTmpMatrix.Mult(mWJt,mWeightedRedPseudoInverse);    
    //mWeightedRedPseudoInverse.Mult(mActualTarget,mStepOutput);
    mWeightedRedPseudoInverse.Mult(mConstrWeights.Mult(mActualTarget),mStepOutput);
    
    //mWeightedRedPseudoInverse.Print();

  }else{  
    if(bVerbose) cerr << "IKSolver:   Jacobian is of rank 0. Zeroing output."<<endl;
    mStepOutput.Zero();
  }  
  
  bool bSanityCheck = true;
  for(int i=0;i<mDofs;i++){
    if(mStepOutput(i) != mStepOutput(i)){
      bSanityCheck = false;
      break; 
    }
  }
  if(!bSanityCheck){
    if(bVerbose) cerr << "IKSolver:   Sanity check failed. Nan values found. Zeroing output."<<endl;
    mStepOutput.Zero(); 
  }

  // Get the null space
  mEigenVectors.GetColumnSpace(mRank,mEigenVectors.ColumnSize()-mRank,mNullEigenVectors);
  mEigenVectorsTranspose.GetRowSpace(mRank,mEigenVectorsTranspose.RowSize()-mRank,mNullEigenVectorsTranspose);
  //if(bVerbose) mNullEigenVectors.Print();
}
void    IKSolver::ClearLimits(){
    mLimits[0].Zero();  mLimits[0] += 1.0;
    mLimits[1].Zero();  mLimits[1] -= 1.0;
}

void    IKSolver::SetLimits(Vector &low,Vector &high){
    int len;
    mLimits[0].Zero();
    mLimits[0].SetSubVector(0,low);
    mLimits[1].Zero();
    mLimits[1].SetSubVector(0,high);
}
void    IKSolver::SetTarget(Vector &v){
    mFullDesiredTarget.Zero();
    mFullDesiredTarget.SetSubVector(0,v);
}
void    IKSolver::SetValidConstraints(Vector &constr){
    mValidConstraints.clear();
    int cnt=0;
    int len = MIN(constr.Size(),mConstraintsSize);
    for(int i=0;i<len;i++){
        if(constr(i)>0){
            mValidConstraints.push_back(i);
            cnt++;
        }         
    }    
    mValidConstraintsSize = cnt;;  
}

void    IKSolver::GetTargetOutput(Vector &output){
    output = mFullOutputTarget;
}

void    IKSolver::GetOutput(Vector &output){
    output = mOutput;
}
void    IKSolver::GetTargetError(Vector &error){
    mFullDesiredTarget.Sub(mFullOutputTarget,error);   
}
REALTYPE IKSolver::GetTargetError(){
    Vector tmp;
    mFullDesiredTarget.Sub(mFullOutputTarget,tmp);
    return tmp.Norm2();   
}

void  IKSolver::SetJacobian(Matrix & jac){
    int r = MIN(jac.RowSize(),mConstraintsSize);
    int c = MIN(jac.ColumnSize(),mDofs);
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            mFullJacobian(i,j) = jac(i,j);
        }
    }
}

void  IKSolver::SetSizes(int dofs, int constraintsSize){
  mDofs            = MAX(0,dofs);
  mConstraintsSize = MAX(0,constraintsSize);
  mValidConstraints.clear();
  for(int i=0;i<mConstraintsSize;i++){
    mValidConstraints.push_back(i);
  }
  mValidConstraintsSize = mConstraintsSize; 
  Resize();  
}
void    IKSolver::SetThresholds(REALTYPE loose, REALTYPE cut){    
  mLooseThreshold   = MAX(0.0,loose);
  mCutThreshold     = MAX(0.0,cut);
}

void    IKSolver::SetDofsWeights(Vector &v){
    mInputDofsWeights.Zero();    
    int len = MIN(v.Size(),mDofs);
    for(int i=0;i<len;i++){
        mInputDofsWeights(i,i) = v(i);         
    }
}
void    IKSolver::SetConstraintsWeights(Vector &v){
    mInputConstrWeights.Zero();    
    int len = MIN(v.Size(),mConstraintsSize);
    for(int i=0;i<len;i++){
        mInputConstrWeights(i,i) = v(i);         
    }
}

void    IKSolver::SetNullTarget(Vector &null){
    mNullTarget.Zero();    
    mNullTarget.SetSubVector(0,null);
}


void IKSolver::Resize(){  
  
    mLimits[0].Resize(mDofs);
    mLimits[1].Resize(mDofs);
    mLimitsOffset.Resize(mDofs);

    mFullJacobian.Resize(mConstraintsSize,mDofs);

    mInputDofsWeights.Resize(mDofs,mDofs,false);
    mInputConstrWeights.Resize(mConstraintsSize,mConstraintsSize,false);
    mWeights.Resize(mDofs,mDofs,false);
    mWeightsTranspose.Resize(mDofs,mDofs,false);
    mInputDofsWeights.Identity();
    mInputConstrWeights.Identity();

    mFullDesiredTarget.Resize(mConstraintsSize);
    mOffsetTarget.Resize(mConstraintsSize);
    mActualTarget.Resize(mConstraintsSize);
    mFullOutputTarget.Resize(mConstraintsSize);

    mNullTarget.Resize(mDofs);
    mNullTarget.Zero();

    mJWt.Resize(mConstraintsSize,mDofs,false);
    mWJt.Resize(mDofs,mConstraintsSize,false);
    mWJtJWt.Resize(mDofs,mDofs,false);

    mTriMatrix.Resize(3,mDofs);

    mEigenVectors.Resize(mDofs,mDofs,false);
    mEigenVectorsTranspose.Resize(mDofs,mDofs,false);

    mEigenValues.Resize(mDofs,false);
    mCondNumbersVector.Resize(mDofs,false);

    mOutputLimitsError.Resize(mDofs,false);


    mOutput.Resize(mDofs,false);
    mOutputOffset.Resize(mDofs,false);
    mStepOutput.Resize(mDofs,false);
}










