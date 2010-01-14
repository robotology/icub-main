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
    mIKItems.clear();
    mSortedPriorityIds.clear();
}

void    IKGroupSolver::SetVerbose(bool verbose){
    bVerbose = verbose;
    for(size_t i=0;i<mIKItems.size();i++){
        IKSolverItem &item = mIKItems[i];
        item.mSolver.SetVerbose(verbose);
    }
}
void    IKGroupSolver::SetSizes(int dofs){
    mDofs = MAX(0,dofs);
    Resize();
}

int     IKGroupSolver::AddSolverItem(const int constraintsSize){
    mIKItems.resize(mIKItems.size()+1);
    IKSolverItem &item = mIKItems[mIKItems.size()-1];
    
    item.mSolver.SetSizes(mDofs,constraintsSize);
    item.mDofsIndex.clear();
    item.mPriority = 0;
    item.mDesiredTarget.Resize(constraintsSize,false);
    item.mDesiredTarget.Zero();
    item.mOutputTarget.Resize(constraintsSize,false);
    item.mOutputTarget.Zero();
    item.mActualTarget.Resize(constraintsSize,false);
    item.mActualTarget.Zero();
    item.mErrorTarget.Resize(constraintsSize,false);
    item.mErrorTarget.Zero();
    
    item.mOutput.Resize(mDofs,false);
    item.mOutput.Zero();
    item.bEnabled   = true;
    item.bSuspended = false;
    bComputePriorities = true;
}

void    IKGroupSolver::SetDofsIndices(const vector<unsigned int> & dofsIndex, int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        item.mDofsIndex.clear();
        for(size_t i=0;i<dofsIndex.size();i++)
            item.mDofsIndex.push_back(dofsIndex[i]);
    }
}
void    IKGroupSolver::SetPriority(int priority, int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        item.mPriority = priority;
        bComputePriorities = true;
    }
}

void    IKGroupSolver::SetThresholds(REALTYPE loose, REALTYPE cut, int solverId){
    if(solverId<0){
        for(size_t i=0;i<mIKItems.size();i++){
            mIKItems[i].mSolver.SetThresholds(loose,cut);
        }
    }else if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        item.mSolver.SetThresholds(loose,cut);
    }    
}

void    IKGroupSolver::SetJacobian(const Matrix & j, int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        if(item.mDofsIndex.size()>0){
            item.mSolver.GetJacobian().Zero();
            item.mSolver.GetJacobian().SetColumnSpace(item.mDofsIndex, j);
            //item.mSolver.GetJacobian().Print();
        }else
            item.mSolver.GetJacobian()=j;
    }
}
void    IKGroupSolver::SetConstraintsWeights(Matrix &m, int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        item.mSolver.SetConstraintsWeights(m);
    }
}
void    IKGroupSolver::SetConstraintsWeights(Vector &v, int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        item.mSolver.SetConstraintsWeights(v);
    }
}
void    IKGroupSolver::SetTarget(const Vector &v, int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        item.mDesiredTarget = v;
    }
}
void    IKGroupSolver::SetNullTarget(const Vector &null){
    mNullTarget.Zero();    
    mNullTarget.SetSubVector(0,null);
}

void    IKGroupSolver::Enable(bool enable, int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        if(item.bEnabled != enable){
            bComputePriorities  = true;
            item.bEnabled       = enable;
        }
        if(enable){
            if(item.bSuspended){
                bComputePriorities  = true;
                item.bSuspended     = false;
            }
        }else{
            if(!item.bSuspended){
                bComputePriorities  = true;
                item.bSuspended     = true;
            }
        }
    }
}
bool    IKGroupSolver::IsEnabled(int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        return item.bEnabled;
    }
    return false;
}

void    IKGroupSolver::Suspend(bool suspend, int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        if(item.bSuspended != suspend){
            bComputePriorities = true;
            item.bSuspended = suspend;
        }
    }
}


void    IKGroupSolver::ComputePriorities(){
    if(bVerbose) cerr << "IKGroupSolver: Computing Priorities"<<endl;
    mSortedPriorityIds.clear();
    for(size_t i=0;i<mIKItems.size();i++){
        if((mIKItems[i].bEnabled)&&(!mIKItems[i].bSuspended))
            mSortedPriorityIds.push_back(i);
    }
    for(size_t i=0;i<mSortedPriorityIds.size();i++){
        for(size_t j=0;j<i;j++){
            if(mIKItems[mSortedPriorityIds[i]].mPriority<mIKItems[mSortedPriorityIds[j]].mPriority){
                int tmp               = mSortedPriorityIds[i];
                mSortedPriorityIds[i] = mSortedPriorityIds[j];
                mSortedPriorityIds[j] = tmp;
            }
        }
    }
    if(bVerbose){
        for(size_t i=0;i<mSortedPriorityIds.size();i++){
            cerr << "  "<<i<<": Solver <"<<mSortedPriorityIds[i]<<">"<<endl;
        }
    } 

    bComputePriorities = false;
}

/*void    IKGroupSolver::SetDofsWeights(Matrix &m){
    mDofsWeights = m;
}*/
void    IKGroupSolver::SetDofsWeights(Vector &v){
    mDofsWeights.Resize(mDofs,mDofs,false);
    mDofsWeights.Zero();    
    mInvDofsWeights.Resize(mDofs,mDofs,false);
    mInvDofsWeights.Zero();    
    int len = MIN(v.Size(),mDofs);
    for(int i=0;i<len;i++){
        mDofsWeights(i,i) = v(i);
        //mInvDofsWeights(i,i) = 1.0/v(i);
    }
}

void    IKGroupSolver::Solve(){
    if(bVerbose) cerr << "IKGroupSolver: Solving"<<endl;
    
    if(bComputePriorities)
        ComputePriorities();
    
    mCurrDofsWeights  = mDofsWeights;
    
    mCurrLimits[0] = mLimits[0];
    mCurrLimits[1] = mLimits[1];

    int cCnt;

    // Compute constraints size
    mConstraintsSize = 0;
    for(size_t i=0;i<mSortedPriorityIds.size();i++){
        IKSubSolver& cSolver = mIKItems[mSortedPriorityIds[i]].mSolver;
        mConstraintsSize += cSolver.mConstraintsSize;
    }
    
    // Set target vectors
    for(size_t i=0;i<mSortedPriorityIds.size();i++){
        IKSolverItem &item = mIKItems[mSortedPriorityIds[i]];
        item.mSolver.SetTarget(item.mDesiredTarget);
    }
    
    mOutput.Zero();

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
            IKSubSolver& cSolver = mIKItems[mSortedPriorityIds[i]].mSolver;
            cSolver.mJacobian.Mult(mLimitsOffset,mLimitsOffsetTarget);
            cSolver.mDesiredTarget  -= mLimitsOffsetTarget;
        }
        mOutput         += mLimitsOffset;
    }
    
    int stepCnt =0;
    while(1){
        if(bVerbose) cerr << "IKGroupSolver: Pass <"<< stepCnt <<">"<<endl;
        
        mStepOutput.Zero();
        
        mCurrWeights   = mCurrDofsWeights;

        // Solve each sub system
        for(size_t i=0;i<mSortedPriorityIds.size();i++){
            IKSolverItem &item = mIKItems[mSortedPriorityIds[i]];
            item.mActualTarget = item.mSolver.mDesiredTarget;
            if(i>0){
                item.mSolver.mDesiredTarget -= item.mSolver.mJacobian * mStepOutput;
            }
            item.mSolver.SetDofsWeights(mCurrWeights);
            item.mSolver.Solve();
            // Retrieve each sub output
            mStepOutput += item.mSolver.mOutput;
            
            Matrix tmp(mCurrWeights);
            tmp.Mult(item.mSolver.GetNullSpace(),mCurrWeights);
            
            item.mSolver.mDesiredTarget = item.mActualTarget;
        }
        mCurrWeights.Transpose(mCurrWeightsTranspose);
        //mStepOutput += mCurrWeights*(mCurrWeightsTranspose * ((mInvDofsWeights*mInvDofsWeights)*(mNullTarget-mOutput-mStepOutput)));
        //mStepOutput += mCurrWeights*(mCurrWeightsTranspose * ((mNullTarget-mOutput-mStepOutput)));
        mStepOutput += mCurrWeights*(mCurrWeightsTranspose * ((mNullTarget)));
        //;
        //(mCurrDofsWeights*mCurrDofsWeights).Print();
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
                    for(int j=0;j<mDofs;j++){
                        mCurrDofsWeights(i,j)   = 0.0;
                    }
                    
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
                IKSolverItem &item = mIKItems[mSortedPriorityIds[i]];
                item.mSolver.mDesiredTarget -= item.mSolver.mJacobian * mStepOutput;    
            }            
            // Update desired target
            mOutput += mStepOutput;
        }
    }
    
    if(bVerbose) cerr << "IKGroupSolver: Done"<<endl;
}
void    IKGroupSolver::ClearLimits(){
    mLimits[0].Zero();  mLimits[0] += 1.0;
    mLimits[1].Zero();  mLimits[1] -= 1.0;
}

void    IKGroupSolver::SetLimits(const Vector &low,const Vector &high){
    int len;
    mLimits[0].Zero();
    mLimits[0].SetSubVector(0,low);
    mLimits[1].Zero();
    mLimits[1].SetSubVector(0,high);
}
Vector&     IKGroupSolver::GetOutput(){
    return mOutput;
}
Vector&     IKGroupSolver::GetTargetError(int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        item.mOutputTarget = item.mSolver.mJacobian * mOutput;    
        item.mErrorTarget  = item.mDesiredTarget - item.mOutputTarget;
        return item.mErrorTarget;
    }
    return mIKItems[0].mErrorTarget;
}
Vector&     IKGroupSolver::GetTargetOutput(int solverId){
    if((solverId>=0)&&(solverId<int(mIKItems.size()))){
        IKSolverItem &item = mIKItems[solverId];
        item.mOutputTarget = item.mSolver.mJacobian * mOutput;    
        item.mErrorTarget  = item.mDesiredTarget - item.mOutputTarget;
        return item.mOutputTarget;
    }
    return mIKItems[0].mOutputTarget;
}
REALTYPE    IKGroupSolver::GetTargetErrorNorm(){
    return 0;
}
REALTYPE    IKGroupSolver::GetTargetErrorNorm2(){
    return 0;    
}

void IKGroupSolver::Resize(){  
  
    mLimits[0].Resize(mDofs);
    mLimits[1].Resize(mDofs);
    mLimitsOffset.Resize(mDofs);

    mDofsWeights.Resize(mDofs,mDofs,false);
    mInvDofsWeights.Resize(mDofs,mDofs,false);
    mCurrDofsWeights.Resize(mDofs,mDofs,false);
    mCurrWeights.Resize(mDofs,mDofs,false);
    mDofsWeights.Identity();
    mInvDofsWeights.Identity();

    mOutputLimitsError.Resize(mDofs,false);

    mNullTarget.Resize(mDofs,false);
    mNullTarget.Zero();

    mOutput.Resize(mDofs,false);
    mStepOutput.Resize(mDofs,false);
    mOutputOffset.Resize(mDofs,false);
    mStepOutput.Resize(mDofs,false);
}
