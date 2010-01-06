#include "IKSubSolver.h"
#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

IKSubSolver::IKSubSolver(){    
    SetSizes(0,0);
    SetThresholds(0.005,0.001);
    bVerbose = false;
}
IKSubSolver::~IKSubSolver(){
    SetSizes(0,0);   
}

void  IKSubSolver::Solve(){
    if(bVerbose) cerr << "IKSubSolver: Solving"<<endl;
    Matrix tmp;

    // W = Vn' * W;

    // A = J*W'
    mConstrWeights.Mult(mJacobian.Mult(mDofsWeights,tmp),mJW);
    //cout << "CW:  "<<mConstrWeights.RowSize()<<","<<mConstrWeights.ColumnSize()<<endl;
    //cout << "J:   "<<mJacobian.RowSize()<<","<<mJacobian.ColumnSize()<<endl;
    //cout << "DW:  "<<mDofsWeights.RowSize()<<","<<mDofsWeights.ColumnSize()<<endl;
    //mDofsWeights.Print();
    // AtA = A'*A
    mJW.Transpose(mWtJt);
    mWtJt.Mult(mJW,mWtJtJW);
    //cout << "AtA: "<<mWtJtJW.RowSize()<<","<<mWtJtJW.ColumnSize()<<endl;
    
    // [V D] = EigenSort(AtA);
    mWtJtJW.Tridiagonalize(mTriMatrix,mEigenVectors);
    mEigenSteps = 20;
    mTriMatrix.TriEigen(mEigenValues, mEigenVectors, mEigenSteps);
    mEigenVectors.SortColumnAbs(mEigenValues);
    mEigenVectors.Transpose(mEigenVectorsTranspose);

    // Condition numbers
    if(!Matrix::IsInverseOk()){
        if(bVerbose) cerr << "IKSubSolver:    Unable to perform eigen value decomposition: too many iterations"<<endl;
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
            cerr << "IKSubSolver:   Jacobian is not full rank: "<<mRank<<"/"<<maxRank<<endl;
        //else
          //  cerr << "IKSubSolver:   Jacobian rank is: "<<mRank<<"/"<<maxRank<<endl;
    }
            


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
            if(bVerbose) cerr << "IKSubSolver:   Loose mode set: Multiplying eigenvalue <"<< mRedInvEigenValues(i) <<"> with condition number <"<<mCondNumbersVector(i)<<"> with <"<< (mCondNumbersVector(i)-mCutThreshold)/(mLooseThreshold-mCutThreshold) <<">"<<endl;
            mRedInvEigenValues(i) *= (mCondNumbersVector(i)-mCutThreshold)/(mLooseThreshold-mCutThreshold);
          }
        }
         
        // th  = W'*V1r*inv(D)*V1r'*A'*y;
        mRedEigenVectors.MultColumn(mRedInvEigenValues,mRedPseudoInverseTmpMatrix);
        mRedPseudoInverseTmpMatrix.Mult(mRedEigenVectorsTranspose,mRedPseudoInverse);
        mDofsWeights.Mult(mRedPseudoInverse,mRedPseudoInverseTmpMatrix);
        mRedPseudoInverseTmpMatrix.Mult(mWtJt,mWeightedRedPseudoInverse);    
        mWeightedRedPseudoInverse.Mult(mConstrWeights.Mult(mDesiredTarget),mOutput);


    }else{  
        if(bVerbose) cerr << "IKSubSolver:   Jacobian is of rank 0. Zeroing output."<<endl;
        mOutput.Zero();
    }  

    bool bSanityCheck = true;
    for(int i=0;i<mDofs;i++){
        if(mOutput(i) != mOutput(i)){
          bSanityCheck = false;
          break; 
        }
    }
    if(!bSanityCheck){
        if(bVerbose) cerr << "IKSubSolver:   Sanity check failed. Nan values found. Zeroing output."<<endl;
        mOutput.Zero(); 
    }

    // Get the null space
    mEigenVectors.GetColumnSpace(mRank,mEigenVectors.ColumnSize()-mRank,mNullEigenVectors);
    mEigenVectorsTranspose.GetRowSpace(mRank,mEigenVectorsTranspose.RowSize()-mRank,mNullEigenVectorsTranspose);
    // Computing target output
    mJacobian.Mult(mOutput,mOutputTarget);
    // Computing target output error
    mDesiredTarget.Sub(mOutputTarget,mErrorTarget);

    if(bVerbose) cerr << "IKSubSolver: Done"<<endl;
}

void    IKSubSolver::SetVerbose(bool verbose){
    bVerbose = verbose;
}

void    IKSubSolver::SetTarget(Vector &v){
    mDesiredTarget.SetSubVector(0,v);
}

Vector& IKSubSolver::GetTargetOutput(){
    return mOutputTarget;
}
Vector& IKSubSolver::GetOutput(){
    return mOutput;
}
Matrix& IKSubSolver::GetNullSpace(){
    return mNullEigenVectors;
}
Vector& IKSubSolver::GetTargetError(){    
    return mErrorTarget;
}
REALTYPE IKSubSolver::GetTargetErrorNorm(){
    return sqrt(GetTargetErrorNorm2());
}
REALTYPE IKSubSolver::GetTargetErrorNorm2(){
    return mErrorTarget.Norm2();   
}

void  IKSubSolver::SetJacobian(Matrix & jac){
    int r = MIN(jac.RowSize(),mConstraintsSize);
    int c = MIN(jac.ColumnSize(),mDofs);
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            mJacobian(i,j) = jac(i,j);
        }
    }
}

void  IKSubSolver::SetSizes(int dofs, int constraintsSize){
  mDofs            = MAX(0,dofs);
  mConstraintsSize = MAX(0,constraintsSize);
  Resize();  
}
void    IKSubSolver::SetThresholds(REALTYPE loose, REALTYPE cut){    
  mLooseThreshold   = MAX(0.0,loose);
  mCutThreshold     = MAX(0.0,cut);
}

void    IKSubSolver::SetDofsWeights(Vector &v){
    mDofsWeights.Resize(mDofs,mDofs,false);
    mDofsWeights.Zero();    
    int len = MIN(v.Size(),mDofs);
    for(int i=0;i<len;i++){
        mDofsWeights(i,i) = v(i);         
    }
}

void    IKSubSolver::SetDofsWeights(Matrix &m){
    mDofsWeights = m;
}

void    IKSubSolver::SetConstraintsWeights(Vector &v){
    mConstrWeights.Resize(mConstraintsSize,mConstraintsSize,false);
    mConstrWeights.Zero();    
    int len = MIN(v.Size(),mConstraintsSize);
    for(int i=0;i<len;i++){
        mConstrWeights(i,i) = v(i);         
    }
}
void    IKSubSolver::SetConstraintsWeights(Matrix &m){
    mConstrWeights = m;
}
Matrix& IKSubSolver::GetJacobian(){
    return mJacobian;
}
void IKSubSolver::Resize(){  
  
    mJacobian.Resize(mConstraintsSize,mDofs);

    mDofsWeights.Resize(mDofs,mDofs,false);
    mConstrWeights.Resize(mConstraintsSize,mConstraintsSize,false);
    mDofsWeights.Identity();
    mConstrWeights.Identity();

    mDesiredTarget.Resize(mConstraintsSize);
    mOutputTarget.Resize(mConstraintsSize);
    mErrorTarget.Resize(mConstraintsSize);

    mJW.Resize(mConstraintsSize,mDofs,false);
    mWtJt.Resize(mDofs,mConstraintsSize,false);
    mWtJtJW.Resize(mDofs,mDofs,false);

    mTriMatrix.Resize(3,mDofs);

    mEigenVectors.Resize(mDofs,mDofs,false);
    mEigenVectorsTranspose.Resize(mDofs,mDofs,false);

    mEigenValues.Resize(mDofs,false);
    mCondNumbersVector.Resize(mDofs,false);

    mOutput.Resize(mDofs,false);
}










