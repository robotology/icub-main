#ifndef Differentiator_H_
#define Differentiator_H_

#include <vector>
using namespace std;

#include "MathLibCommon.h"
#include "Matrix.h"


#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

class Differentiator
{
protected:
  int             mOrder;
  int             mVariablesCount;
  vector<Vector>  mVariables;

  Vector          mDts;
  Vector          mInvDts;

  REALTYPE        mPastDt[2];
  int             mStepCount;
public:  
  bool            mStepBack;
  
public:
          Differentiator();
  virtual ~Differentiator();
  
  virtual void        Init(int nbVariables, int order);
  virtual void        Free();
  
          void        Reset();
  virtual bool        IsValid();
          int         GetOrder();
          int         GetVariablesCount();
          void        SetInput(int index, const REALTYPE input);
          void        SetInput(const Vector& vector);
  virtual void        Update(REALTYPE dt);
          REALTYPE    GetOutput(int index, int order);
          Vector      GetOutput(int order);
          Vector&     GetOutput(int order, Vector& result);
  
protected:
  virtual void        Resize(int nbVariables);
};


class EulerDifferentiator : public Differentiator
{
protected:
  vector<Vector>    mPastBuffer;
  Vector            mWorkingBuffer;

public:
          EulerDifferentiator();
  virtual ~EulerDifferentiator();

  virtual void        Init(int order, int nbVariables);
  virtual void        Free();
  virtual bool        IsValid();
  
  virtual void  Update(REALTYPE dt);
protected:
  virtual void        Resize(int nbVariables);
  
};

typedef EulerDifferentiator EDifferentiator;




class Integrator
{
protected:
  int               mOrder;
  int               mVariablesCount;
  vector<Vector*>   mVariables;
  vector<Vector*>   mPastBuffer;
  Vector            mWorkingBuffer;

  Vector            mDts;

  int               mStepCount;
  
public:
          Integrator();
  virtual ~Integrator();
  
  virtual void        Init(int nbVariables, int order);
  virtual void        Free();
  
          void        Reset();
  virtual bool        IsValid();
          int         GetOrder();
          int         GetVariablesCount();
          void        SetInput(int index, int order, const REALTYPE input);
          void        SetInput(int order, const Vector& vector);
  virtual void        Update(REALTYPE dt);
          REALTYPE    GetOutput(int index, int order);
          Vector      GetOutput(int order);
          Vector&     GetOutput(int order, Vector& result);
  
protected:
  virtual void        Resize(int nbVariables);
};




class ViteDynamicalSystem : public Integrator
{
public:
  Vector  mA;
  Vector  mB;
  Vector  mTarget;
  
public:
          ViteDynamicalSystem();
  virtual ~ViteDynamicalSystem();
  
  virtual void        Init(int nbVariables, int order);
  virtual void        Free();
  
  virtual void        Update(REALTYPE dt);
          void        SetTarget(const Vector& vector);
          void        SetFactors(const Vector& a,const Vector& b);
  
protected:
  virtual void        Resize(int nbVariables);
};

















class DigitalFilter
{
protected:
  int       mOrder;
  int       mVariablesCount;

  Vector    mFwdCoefs;
  Vector    mBkwCoefs;
  
  Matrix    mFwdBuffer;
  Matrix    mBkwBuffer;
  
  Vector    mWorkingVector;
  Vector    mResult;
/*
  REALTYPE  *mFwdCoefs;  
  REALTYPE  *mBkwCoefs;
  
  REALTYPE  *mFwdBuffer;
  REALTYPE  *mBkwBuffer;
*/
  

  int        mIndex;  
  //int        mOutputIndex;

  REALTYPE   mDt;
      
public:
          DigitalFilter();
  virtual ~DigitalFilter();

  virtual void        Init(int nbVariables, int order);
  virtual void        Free();
  virtual bool        IsValid();
  virtual void        Update();

          void        SetInput(int index, const REALTYPE input);
          REALTYPE    GetOutput(int index);

          void        SetInput(const Vector& vector);
          void        SetZeroInput();
          Vector&     GetOutput();
          Vector&     GetOutput(Vector& result);


          void        SetSamplingPeriod(REALTYPE dt);

          void        SetButterworth(REALTYPE cutOffFreq);
          void        SetMovingAverage();
          void        SetGaussian();
          void        SetHalfGaussian();

          void        Print();
protected:
          void        Resize(int nbVariables);
};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif

#endif
