#include "Differentiator.h"

#include "Macros.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif


Differentiator::Differentiator(){  
  mOrder            = 0;
  mVariablesCount   = 0;
  mStepCount        = 0;
  mPastDt[0]=mPastDt[1]= 0.001;
  mStepBack = false;
  Init(1,0);
}
Differentiator::~Differentiator(){
  Free();
}

void Differentiator::Init(int nbVariables, int order){
  Free();
  mOrder           = (order>0?order:0);
  mVariablesCount  = (nbVariables>0?nbVariables:0);
  if((mOrder>=0)&&(mVariablesCount>0)){
    
    mVariables.resize(mOrder+1);
    for(int i=0;i<mOrder+1;i++){
      mVariables[i].Resize(mVariablesCount,false);mVariables[i].Zero();
    }
    mDts.Resize(mOrder,false);
    mDts.One(); mDts *=0.001;
    mInvDts.Resize(mOrder,false);
    mInvDts.Zero();
  }else{
    Free();  
  }
  mStepCount = 0;
}
  
void Differentiator::Free(){
  mOrder           = 0;
  mVariablesCount  = 0;
  mDts.Resize(0);  
  mInvDts.Resize(0);
}

void Differentiator::Reset(){
  mStepCount = 0;  
}
int   Differentiator::GetOrder(){
  return mOrder;  
}
int   Differentiator::GetVariablesCount(){
  return mVariablesCount;  
}
void        Differentiator::SetInput(int index, const REALTYPE input){
  if((index>=0)&&(index<mVariablesCount))
    mVariables[0](index) = input;
}
void        Differentiator::SetInput(const Vector& vector){
  //printf("*********** Diff: Getting Input Vector size\n");
  //printf("*********** Diff: Getting Input Vector size: %d\n",int(vector.Size()));	
  if(int(vector.Size())!=mVariablesCount) Resize(int(vector.Size()));
    mVariables[0].Set(vector); 
  //printf("*********** Diff: Set input done\n",int(vector.Size()));	
}
REALTYPE    Differentiator::GetOutput(int index, int order){
  if((index>=0)&&(index<mVariablesCount)&&(order>=0)&&(order<=mOrder))
    return mVariables[order](index);
  return R_ZERO;
}
Vector      Differentiator::GetOutput(int order){
  order = (order>=0?order:0);
  order = (order<=mOrder?order:mOrder);
  return mVariables[order];
}
Vector&     Differentiator::GetOutput(int order, Vector& result){
  order = (order>=0?order:0);
  order = (order<=mOrder?order:mOrder);
  result.Set(mVariables[order]);
  return result;
}

void Differentiator::Update(REALTYPE dt){}

bool Differentiator::IsValid(){return false;}

void Differentiator::Resize(int nbVariables){
  if(nbVariables==mVariablesCount) return;
  //printf("*********** Diff: Resizing from %d to %d\n",mVariablesCount,nbVariables);
  
  mVariablesCount  = (nbVariables>0?nbVariables:0);

  for(int i=0;i<mOrder+1;i++){
    mVariables[i].Resize(mVariablesCount);
  }
}


EulerDifferentiator::EulerDifferentiator():Differentiator(){
}
EulerDifferentiator::~EulerDifferentiator(){}

void EulerDifferentiator::Init(int nbVariables, int order){
  Differentiator::Init(nbVariables,order);
  
  if((mOrder>=0)&&(mVariablesCount>0)){
    mPastBuffer.resize(mOrder);
    for(int i=0;i<mOrder;i++){
      mPastBuffer[i].Resize(mVariablesCount,false);mPastBuffer[i].Zero();
    }
    mWorkingBuffer.Resize(mVariablesCount,false);
    mWorkingBuffer.Zero();
  }
}
void EulerDifferentiator::Free(){
  Differentiator::Free();
}
  
void EulerDifferentiator::Update(REALTYPE dt){

  mDts.ShiftRight();
  if(mStepBack){
    mDts(0) = mPastDt[0];
  }else{
    mDts(0) = dt;//mPastDt[0];    
  }
  mPastDt[0] = dt;
  for(int i=0;i<mOrder;i++){
    mInvDts(i) = mDts(0);
    for(int j=1;j<=i;j++){
      mInvDts(i) += mDts(j);
    }
    mInvDts(i) = REALTYPE(i+1)/mInvDts(i);
  }
  
  mPastDt[1] = mPastDt[0];

  for(int i=0;i<mOrder;i++){
    mVariables[i].Sub(mPastBuffer[i],mWorkingBuffer);
    mWorkingBuffer *= mInvDts(i);
    mVariables[i+1].Set(mWorkingBuffer);
  }
  for(int i=0;i<mOrder;i++){
    mPastBuffer[i].Set(mVariables[i]);
  }
  

  if(mStepCount<=mOrder){
    for(int i=mStepCount+1;i<=mOrder;i++){    
      mVariables[i].Zero();
    }
    for(int i=mStepCount;i<mOrder;i++){    
      mVariables[i].Set(mVariables[i]);
    }
    mStepCount++;        
  }

}

bool EulerDifferentiator::IsValid(){return mStepCount>mOrder;}

void EulerDifferentiator::Resize(int nbVariables){
  if(nbVariables==mVariablesCount) return;

  Differentiator::Resize(nbVariables);

  for(int i=0;i<mOrder;i++){
    mPastBuffer[i].Resize(mVariablesCount);
  }
  mWorkingBuffer.Resize(mVariablesCount);
  //mVariables.Resize(mOrder+1,mVariablesCount,true);
}


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

Integrator::Integrator(){  
  mOrder            = 0;
  mVariablesCount   = 0;
  Init(1,0);
}
Integrator::~Integrator(){
  Free();
}

void Integrator::Init(int nbVariables, int order){
  Free();
  mOrder           = (order>0?order:0);
  mVariablesCount  = (nbVariables>0?nbVariables:0);
  if((mOrder>=0)&&(mVariablesCount>0)){
    for(unsigned int i=0;i<mVariables.size();i++) delete mVariables[i];      
    mVariables.clear();
    for(int i=0;i<=mOrder;i++){
      mVariables.push_back(new Vector(mVariablesCount));
    }
    mDts.Resize(mOrder,false);
    mDts.One(); mDts *=0.001;
    
    for(unsigned int i=0;i<mPastBuffer.size();i++) delete mPastBuffer[i];      
    mPastBuffer.clear();
    for(int i=0;i<=mOrder;i++){
      mPastBuffer.push_back(new Vector(mVariablesCount));
    }
    mWorkingBuffer.Resize(mVariablesCount,false);
    mWorkingBuffer.Zero();
    
  }else{
    Free();  
  }
}
  
void Integrator::Free(){
  mOrder           = 0;
  mVariablesCount  = 0;
  mDts.Resize(0);  
}

void Integrator::Reset(){
}
int   Integrator::GetOrder(){
  return mOrder;  
}
int   Integrator::GetVariablesCount(){
  return mVariablesCount;  
}
void        Integrator::SetInput(int index, int order, const REALTYPE input){
  if((index>=0)&&(index<mVariablesCount)&&(order>=0)&&(order<=mOrder)){
    if(order == mOrder){
      (*mVariables[order])(index) = input;      
    }else{
      (*mPastBuffer[order])(index) = input;
    }
  }
}
void        Integrator::SetInput(int order, const Vector& vector){
  if((order>=0)&&(order<=mOrder)){
    if(int(vector.Size())!=mVariablesCount) Resize(int(vector.Size()));
    if(order == mOrder){
      mVariables[order]->Set(vector); 
    }else{
      mPastBuffer[order]->Set(vector);
    }
  }
}
REALTYPE    Integrator::GetOutput(int index, int order){
  if((index>=0)&&(index<mVariablesCount)&&(order>=0)&&(order<=mOrder))
    return (*mVariables[order])(index);
  return R_ZERO;
}
Vector      Integrator::GetOutput(int order){
  order = (order>=0?order:0);
  order = (order<=mOrder?order:mOrder);
  return (*mVariables[order]);
}
Vector&     Integrator::GetOutput(int order, Vector& result){
  order = (order>=0?order:0);
  order = (order<=mOrder?order:mOrder);
  result.Set(*mVariables[order]);
  return result;
}

void Integrator::Resize(int nbVariables){
  if(nbVariables==mVariablesCount) return;
  mVariablesCount  = (nbVariables>0?nbVariables:0);

  for(int i=0;i<=mOrder;i++){
    mVariables[i]->Resize(mVariablesCount);
  }
  for(int i=0;i<=mOrder;i++){
    mPastBuffer[i]->Resize(mVariablesCount);
  }
  mWorkingBuffer.Resize(mVariablesCount);
}
  
void Integrator::Update(REALTYPE dt){

  mDts.ShiftRight();
  mDts(0) = dt;

  for(int i=mOrder;i>0;i--){
    mVariables[i]->Add(*mPastBuffer[i],mWorkingBuffer);
    mWorkingBuffer *= 0.5*mDts(0);
    mPastBuffer[i-1]->Add(mWorkingBuffer,*mVariables[i-1]);
  }
  for(int i=0;i<=mOrder;i++){
    mPastBuffer[i]->Set(*mVariables[i]);
  }
}

bool Integrator::IsValid(){return true;}



ViteDynamicalSystem::ViteDynamicalSystem():Integrator(){
}
ViteDynamicalSystem::~ViteDynamicalSystem(){
}

void        ViteDynamicalSystem::Init(int nbVariables, int order){
  Integrator::Init(nbVariables,2); 
}
void        ViteDynamicalSystem::Free(){
  Integrator::Free();
}
  
void        ViteDynamicalSystem::Update(REALTYPE dt){
  
  //ddX = a*(b*(D-X)-dX);
  
  mTarget.Sub(*mPastBuffer[0],*mVariables[2]);
  (*mVariables[2]) ^= mB;
  (*mVariables[2]) -= (*mPastBuffer[1]);
  (*mVariables[2]) ^= mA;
  //mA.Print();
  //mVariables[2]->Print();
  Integrator::Update(dt);
}
  
void        ViteDynamicalSystem::SetTarget(const Vector& vector){
  if(int(vector.Size())!=mVariablesCount) Resize(int(vector.Size()));  
  mTarget.Set(vector); 
}

void        ViteDynamicalSystem::SetFactors(const Vector& a,const Vector& b){
  Resize(a.Size());
  mA.Set(a);
  mB.Set(b);
}


void        ViteDynamicalSystem::Resize(int nbVariables){

  if(nbVariables==mVariablesCount) return;

  mTarget.Resize(nbVariables);
  //mA.Resize(nbVariables);  
  //mB.Resize(nbVariables);

  Integrator::Resize(nbVariables);
}




///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////


DigitalFilter::DigitalFilter(){
  mOrder          = 0;
  mVariablesCount = 0;
  /*
  mFwdCoefs       = NULL;
  mBkwCoefs       = NULL;
  mFwdBuffer      = NULL;
  mBkwBuffer      = NULL;
  */
  mIndex          = 0;
  //mOutputIndex    = 0;
  mDt             = R_ONE;
}
DigitalFilter::~DigitalFilter(){
  Free();
}

void DigitalFilter::Init(int nbVariables, int order){
  Free();
  mOrder           = (order>0?order:0);
  mVariablesCount  = (nbVariables>0?nbVariables:0);

  if((mOrder>0)&&(mVariablesCount>0)){
    mFwdBuffer.Resize(mOrder+1,mVariablesCount,false);
    mBkwBuffer.Resize(mOrder+1,mVariablesCount,false);
    mFwdCoefs.Resize(mOrder+1,false);
    mBkwCoefs.Resize(mOrder+1,false);
    mWorkingVector.Resize(mVariablesCount,false);
    mResult.Resize(mVariablesCount,false);
    mFwdBuffer.Zero();
    mBkwBuffer.Zero();
    mFwdCoefs.Zero();
    mBkwCoefs.Zero();
    mWorkingVector.Zero();
    mResult.Zero();
    /*
    mFwdBuffer  = new REALTYPE[mVariablesCount*(mOrder+1)];
    mBkwBuffer  = new REALTYPE[mVariablesCount*(mOrder+1)];
    mFwdCoefs   = new REALTYPE[mOrder+1];
    mBkwCoefs   = new REALTYPE[mOrder+1];
    memset(mFwdBuffer,0,mVariablesCount*(mOrder+1)*sizeof(REALTYPE));
    memset(mBkwBuffer,0,mVariablesCount*(mOrder+1)*sizeof(REALTYPE));
    memset(mFwdCoefs,0,(mOrder+1)*sizeof(REALTYPE));
    memset(mBkwCoefs,0,(mOrder+1)*sizeof(REALTYPE));
    */
  }
}

void DigitalFilter::Resize(int nbVariables){
  if(nbVariables==mVariablesCount) return;
  mVariablesCount  = (nbVariables>0?nbVariables:0);

  mFwdBuffer.Resize(mOrder+1,mVariablesCount,true);
  mBkwBuffer.Resize(mOrder+1,mVariablesCount,true);
  mWorkingVector.Resize(mVariablesCount,true);
  mResult.Resize(mVariablesCount,true);
}

void DigitalFilter::Free(){
  
  mOrder            = 0;
  mVariablesCount   = 0;
  mIndex            = 0;
  //mOutputIndex      = 0;
  mFwdBuffer.Resize(0,0);
  mBkwBuffer.Resize(0,0);
  mFwdCoefs.Resize(0);
  mBkwCoefs.Resize(0);
  mWorkingVector.Resize(0);
  mResult.Resize(0);
  /*
  if(mFwdCoefs  != NULL) delete [] mFwdCoefs;     mFwdCoefs     = NULL;
  if(mBkwCoefs  != NULL) delete [] mBkwCoefs;     mBkwCoefs     = NULL;
  if(mFwdBuffer != NULL) delete [] mFwdBuffer;    mFwdBuffer    = NULL;
  if(mBkwBuffer != NULL) delete [] mBkwBuffer;    mBkwBuffer    = NULL;
  */
}
  
void DigitalFilter::Update(){

  //int outputOffset = mVariablesCount*mIndex;
  mBkwBuffer.SetRow(R_ZERO,mIndex);
  /*
  for(int j=0;j<mVariablesCount;j++){
    mBkwBuffer[outputOffset+j] = R_ZERO;
  }
  */
  
  mWorkingVector.Zero();
  mFwdBuffer.MultTranspose(mFwdCoefs,mResult);
  //mResult.Print();
  mBkwBuffer.MultTranspose(mBkwCoefs,mWorkingVector);
  mResult-=(mWorkingVector);
  
  /*
  for(int i=0; i<=mOrder;i++){    
    int idx        = (mOrder+1+mIndex-i)%(mOrder+1);
    int currOffset = mVariablesCount*idx;
    
    for(int j=0;j<mVariablesCount;j++){      
      //printf("out += %f * %f - %f * %f = ",mFwdCoefs[i],mFwdBuffer[currOffset+j],mBkwCoefs[i],mBkwBuffer[currOffset+j]);
      mBkwBuffer[outputOffset+j] += +mFwdCoefs[i] * mFwdBuffer[currOffset+j]
                                    -mBkwCoefs[i] * mBkwBuffer[currOffset+j];
      //printf("%f\n",mBkwBuffer[outputOffset+j]);
    }
  }
  */

  //mOutputIndex = mIndex;  
  mBkwBuffer.SetRow(mResult,mIndex);
  mFwdCoefs.ShiftLeft();
  mBkwCoefs.ShiftLeft();
  
  mIndex       = (mOrder+1+mIndex-1)%(mOrder+1);
}

void DigitalFilter::Print(){
  printf("Index: %d\n",mIndex);
  
  for(int i=0; i<mOrder+1;i++){        
    if(i==mIndex) printf("->");
    else printf("  ");
    for(int j=0;j<mVariablesCount;j++){
      /*printf("%f ",mFwdBuffer[i*mVariablesCount+j]);*/
      printf("%f ",mFwdBuffer(i,j));
    }
    printf("       ");
    for(int j=0;j<mVariablesCount;j++){
      /*printf("%f ",mBkwBuffer[i*mVariablesCount+j]);*/
      printf("%f ",mBkwBuffer(i,j));
    }


    //if(i==mOutputIndex) printf("->");
    printf("\n");
  }  
}


bool DigitalFilter::IsValid(){return true;}
void DigitalFilter::SetSamplingPeriod(REALTYPE dt){
  mDt = dt;  
}

#define BW_MAXORDER 5
void DigitalFilter::SetButterworth(REALTYPE cutOffFreq){

  REALTYPE CoefsA[BW_MAXORDER+1][BW_MAXORDER+1] = { { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000} ,
                                                    { 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000} ,
                                                    { 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000} ,
                                                    { 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000} ,
                                                    { 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000} ,
                                                    { 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000} };
                                     
  REALTYPE CoefsB[BW_MAXORDER+1][BW_MAXORDER+1] = { { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000} ,
                                                    { 1.0000000, 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000} ,
                                                    { 1.0000000, 1.4142136, 1.0000000, 0.0000000, 0.0000000, 0.0000000} ,
                                                    { 1.0000000, 2.0000000, 2.0000000, 1.0000000, 0.0000000, 0.0000000} ,
                                                    { 1.0000000, 2.6131259, 3.4142136, 2.6131259, 1.0000000, 0.0000000} ,
                                                    { 1.0000000, 3.2360680, 5.2360680, 5.2360680, 3.2360680, 1.0000000} };




  REALTYPE A[BW_MAXORDER+1];
  REALTYPE B[BW_MAXORDER+1];
  for(int i=0;i<=BW_MAXORDER;i++){
    A[i] = CoefsA[mOrder][i];
    B[i] = CoefsB[mOrder][i];
  }
  //printf("--------------------- %f\n",mDt);
  for(int i=0;i<=BW_MAXORDER;i++){
    //printf("A(%d): %f   B(%d): %f\n",i,A[i],i,B[i]);
  }

  REALTYPE C[BW_MAXORDER+1];
  C[0] = 1.0;
  C[1] = 1.0/tan(PI* mDt * cutOffFreq);
  for(int i=2;i<=BW_MAXORDER;i++) C[i] = C[i-1]*C[1];

  for(int i=0;i<=BW_MAXORDER;i++){
    A[i] *= C[i];  
    B[i] *= C[i];  
  } 
  for(int i=0;i<=BW_MAXORDER;i++){
    //printf("A(%d): %f   B(%d): %f \n",i,A[i],i,B[i]);
  }

  REALTYPE ratio = R_ZERO;
  for(int i=0;i<=BW_MAXORDER;i++) ratio += B[i];
  ratio = R_ONE/ratio;

  switch(mOrder){
  case 0:
    mFwdCoefs[0] = 1.0;
    mBkwCoefs[0] = 0.0;
    break;
  case 1:
    mFwdCoefs[0] = ratio * (A[0] + A[1]);
    mBkwCoefs[0] = 0.0;
    mFwdCoefs[1] = ratio * (A[0] - A[1]);
    mBkwCoefs[1] = ratio * (B[0] - B[1]);
    break;
  case 2:
    mFwdCoefs[0] = ratio * (A[0] + A[1] + A[2]);
    mBkwCoefs[0] = 0.0;
    mFwdCoefs[1] = ratio * (2.0*A[0] - 2.0*A[2]);
    mBkwCoefs[1] = ratio * (2.0*B[0] - 2.0*B[2]);
    mFwdCoefs[2] = ratio * (A[0] - A[1] + A[2]);
    mBkwCoefs[2] = ratio * (B[0] - B[1] + B[2]);
    break;
  case 3:
    mFwdCoefs[0] = ratio * (A[0] + A[1] + A[2] + A[3]);
    mBkwCoefs[0] = 0.0;
    mFwdCoefs[1] = ratio * (3.0*A[0] + A[1] - A[2] - 3.0*A[3]);
    mBkwCoefs[1] = ratio * (3.0*B[0] + B[1] - B[2] - 3.0*B[3]);
    mFwdCoefs[2] = ratio * (3.0*A[0] - A[1] - A[2] + 3.0*A[3]);
    mBkwCoefs[2] = ratio * (3.0*B[0] - B[1] - B[2] + 3.0*B[3]);
    mFwdCoefs[3] = ratio * (A[0] - A[1] + A[2] - A[3]);
    mBkwCoefs[3] = ratio * (B[0] - B[1] + B[2] - B[3]);
    break;
  case 4:
    mFwdCoefs[0] = ratio * (A[0] + A[1] + A[2] + A[3] + A[4]);
    mBkwCoefs[0] = 0.0;
    mFwdCoefs[1] = ratio * (4.0*A[0] + 2.0*A[1] - 2.0*A[3] - 4.0*A[4]);
    mBkwCoefs[1] = ratio * (4.0*B[0] + 2.0*B[1] - 2.0*B[3] - 4.0*B[4]);
    mFwdCoefs[2] = ratio * (6.0*A[0] - 2.0*A[2] + 6.0*A[4]);
    mBkwCoefs[2] = ratio * (6.0*B[0] - 2.0*B[2] + 6.0*B[4]);
    mFwdCoefs[3] = ratio * (4.0*A[0] - 2.0*A[1] + 2.0*A[3] - 4.0*A[4]);
    mBkwCoefs[3] = ratio * (4.0*B[0] - 2.0*B[1] + 2.0*B[3] - 4.0*B[4]);
    mFwdCoefs[4] = ratio * (A[0] - A[1] + A[2] - A[3] + A[4]);
    mBkwCoefs[4] = ratio * (B[0] - B[1] + B[2] - B[3] + B[4]);
    break;
  case 5:
    mFwdCoefs[0] = ratio * ( 1.0*A[0] + 1.0*A[1] + 1.0*A[2] + 1.0*A[3] + 1.0*A[4] + 1.0*A[5]);
    mBkwCoefs[0] = 0.0;
    mFwdCoefs[1] = ratio * ( 5.0*A[0] + 3.0*A[1] + 1.0*A[2] - 1.0*A[3] - 3.0*A[4] - 5.0*A[5]);
    mBkwCoefs[1] = ratio * ( 5.0*B[0] + 3.0*B[1] + 1.0*B[2] - 1.0*B[3] - 3.0*B[4] - 5.0*B[5]);
    mFwdCoefs[2] = ratio * (10.0*A[0] + 2.0*A[1] - 2.0*A[2] - 2.0*A[3] + 2.0*A[4] +10.0*A[5]);
    mBkwCoefs[2] = ratio * (10.0*B[0] + 2.0*B[1] - 2.0*B[2] - 2.0*B[3] + 2.0*B[4] +10.0*B[5]);
    mFwdCoefs[3] = ratio * (10.0*A[0] - 2.0*A[1] - 2.0*A[2] + 2.0*A[3] + 2.0*A[4] -10.0*A[5]);
    mBkwCoefs[3] = ratio * (10.0*B[0] - 2.0*B[1] - 2.0*B[2] + 2.0*B[3] + 2.0*B[4] -10.0*B[5]);
    mFwdCoefs[4] = ratio * ( 5.0*A[0] - 3.0*A[1] + 1.0*A[2] + 1.0*A[3] - 3.0*A[4] + 5.0*A[5]);
    mBkwCoefs[4] = ratio * ( 5.0*B[0] - 3.0*B[1] + 1.0*B[2] + 1.0*B[3] - 3.0*B[4] + 5.0*B[5]);
    mFwdCoefs[5] = ratio * ( 1.0*A[0] - 1.0*A[1] + 1.0*A[2] - 1.0*A[3] + 1.0*A[4] - 1.0*A[5]);
    mBkwCoefs[5] = ratio * ( 1.0*B[0] - 1.0*B[1] + 1.0*B[2] - 1.0*B[3] + 1.0*B[4] - 1.0*B[5]);
    break;
  default:
    break;
  }
  mIndex = 0;
}
#include <iostream>
using namespace std;

void        DigitalFilter::SetMovingAverage(){
  for(int i=0;i<=mOrder;i++)
    mBkwCoefs[i] = 0.0;
    
  REALTYPE ratio = 1.0/REALTYPE(mOrder+1);
  for(int i=0;i<=mOrder;i++)
    mFwdCoefs[i] = ratio;

  REALTYPE msum=0.0;
  for(int i=0;i<=mOrder;i++) msum += mFwdCoefs[i];
  //cout <<"***SUM** "<<msum<<endl;
}
void        DigitalFilter::SetGaussian(){
  for(int i=0;i<=mOrder;i++)
    mBkwCoefs[i] = 0.0;
    
  REALTYPE ratio = 1.0/REALTYPE(mOrder+1);
  for(int i=0;i<=mOrder;i++)
    mFwdCoefs[i] = ratio;

}
void        DigitalFilter::SetHalfGaussian(){
  for(int i=0;i<=mOrder;i++)
    mBkwCoefs[i] = 0.0;
      
  REALTYPE s = sqrt(REALTYPE(mOrder));
  for(int i=0;i<=mOrder;i++){
    REALTYPE x = REALTYPE(i);
    mFwdCoefs[i] = exp(-(x*x)/(2*s*s));;
  }
  REALTYPE sum=0.0;
  for(int i=0;i<=mOrder;i++) sum += mFwdCoefs[i];
  for(int i=0;i<=mOrder;i++) mFwdCoefs[i]/=sum;
  REALTYPE msum=0.0;
  for(int i=0;i<=mOrder;i++) msum += mFwdCoefs[i];
  //cout <<"***SUM** "<<msum<<endl;
} 


void        DigitalFilter::SetInput(int index, const REALTYPE input){
  if((index>=0)&&(index<mVariablesCount)){
    /*
    mFwdBuffer[mVariablesCount*mIndex+index] = input;
    */
    mFwdBuffer(mIndex,index) = input;
  } 
}
REALTYPE    DigitalFilter::GetOutput(int index){
  if((index>=0)&&(index<mVariablesCount)){
    /*
    return mBkwBuffer[mVariablesCount*mOutputIndex+index];
    */
    return mResult(index);    
  }else{
    return R_ZERO;
  }
}

Vector&     DigitalFilter::GetOutput(Vector& result){
  result.Set(mResult);
  return result;
}
void        DigitalFilter::SetZeroInput(){
  mFwdBuffer.SetRow(R_ZERO,mIndex);
}
void        DigitalFilter::SetInput(const Vector& vector){
  if(int(vector.Size())!=mVariablesCount) Resize(int(vector.Size()));
  mFwdBuffer.SetRow(vector,mIndex);    
}
Vector&     DigitalFilter::GetOutput(){
  return mResult;
}
