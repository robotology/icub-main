#ifndef __GDCLASS_H__
#define __GDCLASS_H__

#define GDESCENT_ERROR 1e10

class GradientFunction
{
          GradientFunction();
  virtual ~GradientFunction();
public:
  virtual float GetErrorValue(float *params);
};


class GradientDescent
{
public:
  GradientFunction   *m_Func;
  int                 m_NbParams;
  float              *m_CParams;
  float              *m_dCParams;
  float              *m_mdCParams;
  float              *m_ParamsStep;
  float              *m_tParams;
  float              *m_ParamsRangeMin;
  float              *m_ParamsRangeMax;
  float              *m_ParamsRate;
  
  
  float               m_CError;
  float               m_Rate;
  float               m_Momentum;

public:
  GradientDescent();
  ~GradientDescent();

  void  Init(int nbParams, float *pSteps, float rate, float moment, GradientFunction * func);
  void  SetRange(float *rangeMin,float *rangeMax);
  void  SetRate(float *rate);
  void  Clear();

  float Start(float *params);
  float Step(int cnt = 1);

protected:
  void  CheckRange();
};

#endif
