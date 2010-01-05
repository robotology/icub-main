#ifndef IKGROUPSOLVER_H_
#define IKGROUPSOLVER_H_

#include "MathLib.h"
#include "IKSolver.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

    
/**
 * \class IKGroupSolver
 * 
 * \ingroup MathLib
 * 
 * \brief A class for solving inverse kinematic in a pseudo-inverse fashion with optimization
 * 
 * This is a loose inverse kinematic algorithm... By loose, I mean that constraint are only satisfied if possible...
 * This way some singularities can be solved pretty well...
 */

class IKGroupSolver
{
public:
        
            /// Constructor
            IKGroupSolver();
            /// Destructor
    virtual ~IKGroupSolver();

            /// Allows to print out debug message
            void    SetVerbose(bool verbose=true);

            int     AddSolverItem(IKSolver* solver, const vector<int> & dofsIndex, int priority);

            void    SetPriority(int solverId, int priority);
        
            void    ComputePriorities();
        
            void    Solve();
        
            void    Resize();
        
protected:
    typedef struct{
        IKSolver*       mIKSolver;
        IndicesVector   mDofsIndex;
        int             mPriority;
    }IKSolverItem;
    
    vector<IKSolverItem>    mIKItems;

    vector<int>             mSortedPriorityIds;
    
    bool                    bVerbose;

    bool                    bComputePriorities;
        
    int                     mConstraintsSize;
    int                     mDofs;
        
    Vector              mLimits[2]; 
    Vector              mCurrLimits[2];
    Vector              mLimitsOffset;

    Matrix              mInputDofsWeights;
    Matrix              mWeights;        

    Vector              mDesiredTarget;
    Vector              mLimitsOffsetTarget;

    Vector              mOutputTarget;
    Vector              mStepOutput;
    Vector              mOutput;
    Vector              mOutputOffset;
    Vector              mOutputLimitsError;  
};




#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif