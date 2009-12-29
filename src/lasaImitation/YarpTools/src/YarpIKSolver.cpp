#include "YarpIKSolver.h"

#include "YarpMathLibInterface.h"
#include "MathLib/MathLib.h"
#include "MathLib/IKSolver.h"

using namespace MathLib;

YarpIKSolver::YarpIKSolver()                                                      {impl = new IKSolver();}
YarpIKSolver::~YarpIKSolver()                                                     {delete impl;}

void    YarpIKSolver::SetVerbose(bool verbose)                                    {impl->SetVerbose(verbose);}
void    YarpIKSolver::SetSizes(int dofs, int constraintsSize)                     {impl->SetSizes(dofs,constraintsSize);}
void    YarpIKSolver::SetThresholds(double loose, double cut)                     {impl->SetThresholds(loose,cut);}    
void    YarpIKSolver::ClearLimits()                                               {impl->ClearLimits();}
void    YarpIKSolver::SetLimits(yarp::sig::Vector &low, yarp::sig::Vector &high)  {Vector v1,v2; impl->SetLimits(YarpVectorToVector(low,v1),YarpVectorToVector(high,v2));}
void    YarpIKSolver::SetJacobian(yarp::sig::Matrix & j)                          {Matrix m; impl->SetJacobian(YarpMatrixToMatrix(j,m));}
void    YarpIKSolver::SetDofsWeights(yarp::sig::Vector &v)                        {Vector v1; impl->SetDofsWeights(YarpVectorToVector(v,v1));}
void    YarpIKSolver::SetConstraintsWeights(yarp::sig::Vector &v)                 {Vector v1; impl->SetConstraintsWeights(YarpVectorToVector(v,v1));}
void    YarpIKSolver::SetValidConstraints(yarp::sig::Vector &constr)              {Vector v1; impl->SetValidConstraints(YarpVectorToVector(constr,v1));}
void    YarpIKSolver::SetTarget(yarp::sig::Vector &v)                             {Vector v1; impl->SetTarget(YarpVectorToVector(v,v1));}
void    YarpIKSolver::SetNullTarget(yarp::sig::Vector &null)                      {Vector v1; impl->SetNullTarget(YarpVectorToVector(null,v1));}
void    YarpIKSolver::Solve()                                                     {impl->Solve();}
void    YarpIKSolver::GetOutput(yarp::sig::Vector &output)                        {Vector v; impl->GetOutput(v);VectorToYarpVector(v,output);}
void    YarpIKSolver::GetTargetOutput(yarp::sig::Vector &output)                  {Vector v; impl->GetTargetOutput(v);VectorToYarpVector(v,output);}
void    YarpIKSolver::GetTargetError(yarp::sig::Vector &error)                    {Vector v; impl->GetTargetError(v);VectorToYarpVector(v,error);}
double  YarpIKSolver::GetTargetError()                                            {return impl->GetTargetError();}
