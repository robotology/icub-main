#include "IKGroupSolver.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

IKGroupSolver::IKGroupSolver(){
    mIKItems.clear();
    mSortedPriorityIds.clear();
    
    mConstraintsSize    = 0;
    mDofs               = 0;
    bVerbose            = false;
    bComputePriorities  = false;
}

IKGroupSolver::~IKGroupSolver(){
    for(size_t i=0;i<mIKItems.size();i++)
        if(mIKItems[i].mIKSolver!=NULL) delete mIKItems[i].mIKSolver;
    mIKItems.clear();
    mSortedPriorityIds.clear();
}

void    IKGroupSolver::SetVerbose(bool verbose){
    bVerbose = verbose;
}

int     IKGroupSolver::AddSolverItem(IKSolver* solver, const vector<int> & dofsIndex, int priority){
    mIKItems.resize(mIKItems.size()+1);
    IKSolverItem &item = mIKItems[mIKItems.size()-1];
    item.mIKSolver = solver;
    item.mPriority = priority;    
    for(size_t i=0;i<dofsIndex.size();i++)
        item.mDofsIndex.push_back(dofsIndex[i]);
    bComputePriorities = true;
}

void    IKGroupSolver::SetPriority(int solverId, int priority){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        mIKItems[solverId].mPriority = priority;
        bComputePriorities = true;
    }
}

void    IKGroupSolver::ComputePriorities(){
    if(bVerbose) cerr << "IKGroupSolver: Computing Priorities"<<endl;
    mSortedPriorityIds.clear();
    for(size_t i=0;i<mIKItems.size();i++)
        mSortedPriorityIds.push_back(i);

    bComputePriorities = false;
}


void    IKGroupSolver::Solve(){
    if(bVerbose) cerr << "IKGroupSolver: Solving"<<endl;
    
    if(bComputePriorities)
        ComputePriorities();
    
    mWeights       = mInputDofsWeights;
    mCurrLimits[0] = mLimits[0];
    mCurrLimits[1] = mLimits[1];

    int cCnt;

    // Compute constraints size
    mConstraintsSize = 0;
    for(size_t i=0;i<mSortedPriorityIds.size();i++){
        IKSolver* cSolver = mIKItems[mSortedPriorityIds[i]].mIKSolver;
        mConstraintsSize += cSolver->mConstraintsSize;
    }
    
    // Set global target vector
    cCnt = 0;
    for(size_t i=0;i<mSortedPriorityIds.size();i++){
        IKSolver* cSolver = mIKItems[mSortedPriorityIds[i]].mIKSolver;
        mDesiredTarget.SetSubVector(cCnt,cSolver->mFullDesiredTarget);    
        cCnt += cSolver->mConstraintsSize;
    }
    
    mOutput.Zero();
    mOutputTarget.Zero();

    // Check initial limits
    bool bHasInitialLimits = false;
    mLimitsOffset.Zero();
    for(int i=0;i<mDofs;i++){
        if(mCurrLimits[0][i]<mCurrLimits[1][i]){
            if(mCurrLimits[0][i]>0.0){
                mCurrLimits[1][i]  -= mCurrLimits[0][i];
                mLimitsOffset[i]    = mCurrLimits[0][i];
                mCurrLimits[0][i]   = 0.0;
                bHasInitialLimits   = true;
            }else if(mCurrLimits[1][i]<0.0){
                mCurrLimits[0][i]  -= mCurrLimits[1][i];
                mLimitsOffset[i]    = mCurrLimits[1][i];
                mCurrLimits[1][i]   = 0.0;
                bHasInitialLimits   = true;
            }
        }
    }
    
    Vector tmpCsV(mConstraintsSize);
    Vector tmpDsV(mDofs);
    
    // Apply initial limits
    if(bHasInitialLimits){
        mLimitsOffsetTarget.Zero();
        for(size_t i=0;i<mSortedPriorityIds.size();i++){
            IKSolver* cSolver = mIKItems[mSortedPriorityIds[i]].mIKSolver;
            mLimitsOffsetTarget += cSolver->mJacobian.Mult(mLimitsOffset,tmpCsV);
        }
        mDesiredTarget  -= mLimitsOffsetTarget;
        mOutput         += mLimitsOffset;
    }
    
    int stepCnt =0;
    while(1){
        if(bVerbose) cerr << "IKSolver: Pass <"<< stepCnt <<">"<<endl;
        
        // Solve each sub system
        for(size_t i=0;i<mSortedPriorityIds.size();i++){
            IKSolverItem* cItem = &mIKItems[mSortedPriorityIds[i]];
            cItem->mIKSolver->StepSolve();
            // Retrieve each sub output
            tmpDsV.Zero();
            tmpDsV.SetSubVector(cItem->mDofsIndex, cItem->mIKSolver->mStepOutput);
            mStepOutput += tmpDsV;
        }
        
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
            if(bVerbose) cerr << "IKGroupSolver: Pass <"<< stepCnt <<"> Limits reached..."<<endl;
            for(int i=0;i<mDofs;i++){
                if(minOutputLimitError == mOutputLimitsError[i]){
                    if(bVerbose) cerr << "IKGroupSolver: Pass <"<< stepCnt <<"> Locking DOF <"<<i<<">"<<endl;
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
            if(bVerbose) cerr << "IKGroupSolver: Pass <"<< stepCnt <<"> Success"<<endl;
            break;
        }else{
            stepCnt ++;
            for(size_t i=0;i<mSortedPriorityIds.size();i++){
                IKSolverItem* cItem = &mIKItems[mSortedPriorityIds[i]];
                mStepOutput.GetSubVector(cItem->mDofsIndex,cItem->mIKSolver->mStepOutput);
                cItem->mIKSolver->mDesiredTarget -= cItem->mIKSolver->mJacobian * cItem->mIKSolver->mStepOutput;    
            }            
            // Update desired target
            mOutput += mStepOutput;
        }
    }
    
    if(bVerbose) cerr << "IKGroupSolver: Done"<<endl;
}

void IKGroupSolver::Resize(){  
  
    mLimits[0].Resize(mDofs);
    mLimits[1].Resize(mDofs);
    mLimitsOffset.Resize(mDofs);

    mInputDofsWeights.Resize(mDofs,mDofs,false);
    mWeights.Resize(mDofs,mDofs,false);
    mInputDofsWeights.Identity();


    mOutputLimitsError.Resize(mDofs,false);


    mOutput.Resize(mDofs,false);
    mOutputOffset.Resize(mDofs,false);
    mStepOutput.Resize(mDofs,false);
}
