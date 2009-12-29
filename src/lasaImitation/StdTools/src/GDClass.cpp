#include "GDClass.h"

#include <stdlib.h>

GradientFunction::GradientFunction(){
}
GradientFunction::~GradientFunction(){
}
float GradientFunction::GetErrorValue(float *params){ 
  return GDESCENT_ERROR;
}



GradientDescent::GradientDescent(){
  m_Func       = NULL;
  m_NbParams   = 0;
  m_CParams    = NULL;
  m_dCParams   = NULL;
  m_mdCParams  = NULL;
  m_ParamsStep = NULL;
  m_tParams    = NULL;
  m_CError     = 0;
  m_Rate       = 0;
  m_Momentum   = 0;
  m_ParamsRangeMin = NULL;
  m_ParamsRangeMax = NULL;
  m_ParamsRate = NULL;
}

GradientDescent::~GradientDescent(){
  Clear();
}

void GradientDescent::Init(int nbParams, float *pSteps, float rate, float moment, GradientFunction * func){

  if(nbParams<=0)
    return;

  if(func==NULL)
    return;

  if(m_Func!=NULL)
    return;
  
  Clear();

  
  m_Func       = func;
  m_NbParams   = nbParams;
  m_CParams    = new float[m_NbParams];
  m_dCParams   = new float[m_NbParams];
  m_mdCParams  = new float[m_NbParams];
  m_ParamsStep = new float[m_NbParams];
  m_tParams    = new float[m_NbParams];
  m_ParamsRangeMin    = new float[m_NbParams];
  m_ParamsRangeMax    = new float[m_NbParams];
  m_ParamsRate = new float[m_NbParams];

  m_Rate       = rate;
  m_Momentum   = moment;
  int i;
  for(i=0;i<m_NbParams;i++){
    m_ParamsStep[i] = pSteps[i];
    m_ParamsRangeMin[i] = 0;
    m_ParamsRangeMax[i] = 0;
    m_ParamsRate[i] = m_Rate;
  }
}

void GradientDescent::Clear(){
  if(m_CParams != NULL)
    delete [] m_CParams;
  if(m_dCParams != NULL)
    delete [] m_dCParams;
  if(m_mdCParams != NULL)
    delete [] m_dCParams;
  if(m_ParamsStep != NULL)
    delete [] m_ParamsStep;
  if(m_tParams != NULL)
    delete [] m_tParams;
  if(m_ParamsRangeMin != NULL)
    delete [] m_ParamsRangeMin;
  if(m_ParamsRangeMax != NULL)
    delete [] m_ParamsRangeMax;
  if(m_ParamsRate != NULL)
    delete [] m_ParamsRate;
  
  m_Func       = NULL;
  m_NbParams   = 0;
  m_CParams    = NULL;
  m_dCParams   = NULL;
  m_mdCParams  = NULL;
  m_ParamsStep = NULL;
  m_tParams    = NULL;
  m_CError     = 0;
  m_Rate       = 0;
  m_Momentum   = 0;
  m_ParamsRangeMin = NULL;
  m_ParamsRangeMax = NULL;
  m_ParamsRate = NULL;


}
void  GradientDescent::SetRange(float *rangeMin,float *rangeMax){
  if(m_Func==NULL)
    return;
  int i;
  for(i=0;i<m_NbParams;i++){
    m_ParamsRangeMin[i] = rangeMin[i];
    m_ParamsRangeMax[i] = rangeMax[i];
  }
}

void  GradientDescent::SetRate(float *rate){
  if(m_Func==NULL)
    return;
  int i;
  for(i=0;i<m_NbParams;i++){
    m_ParamsRate[i] = rate[i];
  }
}

void  GradientDescent::CheckRange(){
  int i;
  for(i=0;i<m_NbParams;i++){
    if(m_ParamsRangeMin[i]<m_ParamsRangeMax[i]){
      m_CParams[i] = (m_CParams[i]<m_ParamsRangeMin[i]?m_ParamsRangeMin[i]:(m_CParams[i]>m_ParamsRangeMax[i]?m_ParamsRangeMax[i]:m_CParams[i]));
    }
  }
}

float GradientDescent::Start(float *params){
  if(m_Func==NULL)
    return GDESCENT_ERROR;
  int i;
  for(i=0;i<m_NbParams;i++){
    m_CParams[i]   = params[i];
    m_mdCParams[i] = 0;
  }
  m_CError = m_Func->GetErrorValue(m_CParams);
  return m_CError;
}

float GradientDescent::Step(int cnt){
  if(m_Func==NULL)
    return GDESCENT_ERROR;

  int i,j;
  for(j=0;j<cnt;j++){
    for(i=0;i<m_NbParams;i++){
      m_tParams[i] = m_CParams[i];
    }
    for(i=0;i<m_NbParams;i++){
      if(m_ParamsStep[i]==0){
        m_dCParams[i] = 0;
      }else{
        m_tParams[i]  += m_ParamsStep[i];
        m_dCParams[i]  = m_Func->GetErrorValue(m_tParams);
        m_tParams[i]  -= 2*m_ParamsStep[i];
        m_dCParams[i] -= m_Func->GetErrorValue(m_tParams);
        m_dCParams[i] /= 2*m_ParamsStep[i];
        m_tParams[i]  += m_ParamsStep[i];
      }
    }
    for(i=0;i<m_NbParams;i++){
      m_CParams[i]   = m_CParams[i] - m_ParamsRate[i]*m_dCParams[i] - m_Momentum*m_mdCParams[i];
      m_mdCParams[i] = m_dCParams[i];
    }
    CheckRange();
  }
  m_CError = m_Func->GetErrorValue(m_CParams);
  return m_CError;

}
