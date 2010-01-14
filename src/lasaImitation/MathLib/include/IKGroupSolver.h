#ifndef IKGROUPSOLVER_H_
#define IKGROUPSOLVER_H_

#include "MathLib.h"
#include "IKSubSolver.h"

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

            void    SetSizes(int dofs);
        
            /// Sets the weights on the degrees of freedom (useful if the redundancies are on the DOFs)
            void    SetDofsWeights(Vector &v);
            //void    SetDofsWeights(Matrix &m);

            /// Add a solver item with given constraints size and return the sovler id
            int     AddSolverItem(const int constraintsSize);
            /// Set jacobian dofs indices for the given solver
            void    SetDofsIndices(const vector<unsigned int> & dofsIndex, int solverId = 0);
            /// Set the priority of the given solver
            void    SetPriority(int priority, int solverId = 0);
            /// Set the thresholds for the given solver (all by default)
            void    SetThresholds(REALTYPE loose, REALTYPE cut, int solverId = -1);

            /// Sets the jacobian for the given solver (if provided, dofs indices will be used)
            void    SetJacobian(const Matrix & j, int solverId = 0);
            
            /// Sets the weights on the constraints for the given solver (useful if the redundancies are on the constraints)
            void    SetConstraintsWeights(Matrix &m, int solverId = 0);
            void    SetConstraintsWeights(Vector &v, int solverId = 0);

            /// Sets the target values to reach for the given solver
            void    SetTarget(const Vector &v, int solverId = 0);
            /// Enable or disable the given solver
            void    Enable(bool enable=true, int solverId = 0);
            /// Suspend or resume the given solver (used in conjunction with Enable)
            void    Suspend(bool suspend=true, int solverId = 0);
            /// Get if the given solver is enabled
            bool    IsEnabled(int solverId = 0);
        

            /// Sets the target for the null space (Size given by the number of DOFs)
            void    SetNullTarget(const Vector &null);

            /// Removes all constraints limits on the outputs
            void    ClearLimits();
            
            /**
             * \brief Sets the constraints limits on the putput
             * \param low      Vector for low bounds values
             * \param high     Vector for high bounds values
             */  
            void    SetLimits(const Vector &low, const Vector &high);
        
            void    ComputePriorities();
        
            void    Solve();
        
            void    Resize();
            
            /// Get the result
            Vector&     GetOutput();        
            /// Get the error between produced target and the requested one
            Vector&     GetTargetError(int solverId = 0);        
            /// Get the actual target that output values produces
            Vector&     GetTargetOutput(int solverId = 0);
            /// Get the squared norm of the error between produced target and the requested one
            REALTYPE    GetTargetErrorNorm();
            REALTYPE    GetTargetErrorNorm2();
protected:
    typedef struct{
        IKSubSolver     mSolver;
        IndicesVector   mDofsIndex;
        int             mPriority;
        Vector          mDesiredTarget;
        Vector          mActualTarget;
        Vector          mOutputTarget;
        Vector          mErrorTarget;
        Vector          mOutput;
        bool            bEnabled;
        bool            bSuspended;
    }IKSolverItem;
    
    vector<IKSolverItem>    mIKItems;

    vector<int>             mSortedPriorityIds;
    
    bool                    bVerbose;

    bool                    bComputePriorities;
        
    int                     mConstraintsSize;
    int                     mDofs;
    
    Vector                  mNullTarget;
    
    Vector                  mLimits[2]; 
    Vector                  mCurrLimits[2];
    Vector                  mLimitsOffset;

    Matrix                  mDofsWeights;
    Matrix                  mInvDofsWeights;
    Matrix                  mCurrDofsWeights;        
    Matrix                  mCurrWeights;        
    Matrix                  mCurrWeightsTranspose;        
    
    Vector              mLimitsOffsetTarget;

    Vector              mOutput;
    Vector              mStepOutput;
    Vector              mOutputOffset;
    Vector              mOutputLimitsError;  
};




#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif
