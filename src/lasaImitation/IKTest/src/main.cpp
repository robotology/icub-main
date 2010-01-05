#include "MathLib/MathLib.h"
#include "MathLib/IKSolver.h"

using namespace MathLib;

#include <iostream>
using namespace std;
#define DEMOID 1

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
    IK.SetThresholds(0.0005,0.0001);    
    
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
    
        cout << "Error"<<endl;
        IK.GetTargetError(result);
        result.Print();
        cout << "****************************************"<<endl;
    }    
#endif
    return 0;
}
