#include "MathLib/MathLib.h"
#include "MathLib/IKSolver.h"
#include "MathLib/IKGroupSolver.h"
#include "StdTools/Timer.h"

using namespace MathLib;

#include <iostream>
using namespace std;
#define DEMOID 2

int main(int argc, char **argv){

    RANDOMIZE;

#if DEMOID==1
    Vector limitsLow;
    Vector limitsHig;
    Matrix jacobian;
    Vector dofWeights;
    Vector ctrWeights;
    Vector target;
    Vector result;

    IKSolver  IK;
    IK.SetVerbose(true);

    IK.SetSizes(8,6);
    IK.SetThresholds(0.00005,0.00001);    
    
    limitsLow.Resize(8); limitsLow.Zero(); limitsLow -= 1;
    limitsHig.Resize(8); limitsHig.Zero(); limitsHig += 1;
    IK.SetLimits(limitsLow,limitsHig);
    
    jacobian.Resize(6,8);
    jacobian.Random();
    jacobian-=0.5;
    IK.SetJacobian(jacobian);

    for(int i=0;i<2;i++){

        dofWeights.Resize(8);
        dofWeights.One();
        if(i==1){
            //dofWeights(0) = 0.1;
            //dofWeights(3) = 0.1;
            //dofWeights(1) = 0.0;            

            limitsLow[0] =  0.1;
            limitsHig[1] = -0.0;
            IK.SetLimits(limitsLow,limitsHig);
        }
        IK.SetDofsWeights(dofWeights);
    
        ctrWeights.Resize(6);
        ctrWeights.One();
        IK.SetConstraintsWeights(ctrWeights);
    
        target.Resize(6); target.Zero(); target += 0.1;
        IK.SetTarget(target);
    
        IK.Solve();   
        cout << "****************************************"<<endl;
        cout << "Output"<<endl;
        IK.GetOutput(result);
        result.Print();
    
        cout << "Error: "<<IK.GetTargetError()<<endl;
        IK.GetTargetError(result);
        result.Print();
        cout << "****************************************"<<endl;
    }    
#endif

#if DEMOID==2
    Vector limitsLow;
    Vector limitsHig;
    Matrix jacobian;
    Matrix jacobian1,jacobian2;
    Vector dofWeights;
    Vector ctrWeights;
    Vector target;
    Vector result;

    int dof = 17;
    int c1 = 6;
    int c2 = 6;
    int c0 = c1+c2;
    
    limitsLow.Resize(dof); limitsLow.Zero(); limitsLow -= 1;
    limitsHig.Resize(dof); limitsHig.Zero(); limitsHig += 1;

    jacobian1.Resize(c1,dof);
    jacobian1.Random();
    jacobian1-=0.5;

    jacobian2.Resize(c2,dof);
    jacobian2.Random();
    jacobian2-=0.5;

    jacobian.Resize(c0,dof);
    jacobian.SetRowSpace(jacobian1,0);
    jacobian.SetRowSpace(jacobian2,c1);
    


    IKSolver  IK;
    IK.SetVerbose(true);
    IK.SetSizes(dof,c0);
    IK.SetThresholds(0.00005,0.00001);        
    IK.SetLimits(limitsLow,limitsHig);
    IK.SetJacobian(jacobian);

    IKGroupSolver IKG;
    IKG.SetSizes(dof);
    IKG.AddSolverItem(c1);
    IKG.AddSolverItem(c2);
    IKG.SetVerbose(false);
    IKG.SetThresholds(0.00005,0.00001);        
    IKG.SetLimits(limitsLow,limitsHig);
    IKG.SetJacobian(jacobian1,0);
    IKG.SetJacobian(jacobian2,1);
    
    dofWeights.Resize(dof);
    dofWeights.One();
    IK.SetDofsWeights(dofWeights);
    IKG.SetDofsWeights(dofWeights);
    
    ctrWeights.Resize(c0);
    ctrWeights.One();
    IK.SetConstraintsWeights(ctrWeights);
    ctrWeights.Resize(c1);
    ctrWeights.One();    
    IKG.SetConstraintsWeights(ctrWeights,0);
    ctrWeights.Resize(c2);
    ctrWeights.One();
    IKG.SetConstraintsWeights(ctrWeights,1);
    
    target.Resize(c0); target.Zero(); target += 0.1;
    IK.SetTarget(target);
    target.Resize(c1); target.Zero(); target += 0.1;
    IKG.SetTarget(target,0);
    target.Resize(c2); target.Zero(); target += 0.1;
    IKG.SetTarget(target,1);
    
    Chrono mChrono;
    mChrono.Start();
    int NI = 1000;
    for(int i=0;i<NI;i++){
        jacobian1.Resize(c1,dof);
        jacobian1.Random();
        jacobian1-=0.5;

        jacobian2.Resize(c2,dof);
        jacobian2.Random();
        jacobian2-=0.5;

        jacobian.Resize(c0,dof);
        jacobian.SetRowSpace(jacobian1,0);
        jacobian.SetRowSpace(jacobian2,c1);
        
        //IK.Solve();   
        IKG.Solve();   
    }
    double time = double(mChrono.ElapsedTimeUs());
    cout <<time/1000000.0<<" "<<time/1000000.0/double(NI)<<endl;
    return 0;
    cout << "****************************************"<<endl;
    cout << "Output"<<endl;
    IK.GetOutput(result);
    result.Print();
    IKG.GetOutput().Print();
    cout << "****************************************"<<endl;
    cout << "Error: "<<IK.GetTargetError()<<endl;
    IK.GetTargetError(result);
    result.Print();
    IKG.GetTargetError(0).Print();
    IKG.GetTargetError(1).Print();
    cout << "****************************************"<<endl;    
    IK.GetTargetOutput(result);
    result.Print();
    IKG.GetTargetOutput(0).Print();
    IKG.GetTargetOutput(1).Print();
    cout << "****************************************"<<endl;
#endif
    return 0;
}
