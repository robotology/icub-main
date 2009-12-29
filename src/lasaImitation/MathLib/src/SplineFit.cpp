#include "Quaternion.h"

#include "SplineFit.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

Matrix& SplineFit::HermitteSplineFit(const Matrix& inputData, Matrix& outputData){
  const int dataCount   = inputData.RowSize(); 
  const int fieldCount  = inputData.ColumnSize()-1;
  const int outputCount = outputData.RowSize();
   
  if((fieldCount  <= 0)||
     (dataCount   <= 0)||
     (outputCount <= 0)) return outputData;
  
  outputData.Resize(outputCount, fieldCount+1);

  const REALTYPE startTime = inputData.At(0,0);
  const REALTYPE stopTime  = inputData.At(dataCount-1,0);
  
  REALTYPE currTime     = outputData.At(0,0);
  REALTYPE pastCurrTime = currTime-R_ONE;
  REALTYPE prevTime     = startTime;
  REALTYPE nextTime     = startTime;
  int      prev         = 0;
  int      next         = 0;

  for(int i=0;i<outputCount;i++){
    currTime = outputData.At(i,0);
    
    // strictly increasing time constraint not satisfied...
    if(pastCurrTime>currTime) return outputData;
    
    // Find the nearest data pair    
    if(currTime<=startTime){
      outputData.InsertSubRow(i,1,inputData,0,1,fieldCount);
    }else if(currTime>=stopTime){
      outputData.InsertSubRow(i,1,inputData,dataCount-1,1,fieldCount);
    }else{ 
      const int searchStartPos = prev+1;
      for(int j=searchStartPos;j<dataCount;j++){
        next      = j;
        nextTime  = inputData.At(j,0);
        if((currTime>=prevTime)&&(currTime<nextTime)) break;        
        prev      = next;
        prevTime  = nextTime; 
      }

      const REALTYPE nphase = (currTime-prevTime)/(nextTime-prevTime);
      const REALTYPE s1 = nphase;
      const REALTYPE s2 = s1*nphase;
      const REALTYPE s3 = s2*nphase; 
      const REALTYPE h1 =  2.0f*s3 - 3.0f*s2 + 1.0f;
      const REALTYPE h2 = -2.0f*s3 + 3.0f*s2;
      const REALTYPE h3 =       s3 - 2.0f*s2 + s1;
      const REALTYPE h4 =       s3 -      s2;
      for(int j=1;j<=fieldCount;j++){
        const REALTYPE p0 = (prev>0?inputData.At(prev-1,j):inputData.At(prev,j));
        const REALTYPE p1 = inputData.At(prev,j);
        const REALTYPE p2 = inputData.At(next,j);      
        const REALTYPE p3 = (next<dataCount-1?inputData.At(next+1,j):inputData.At(next,j));
        const REALTYPE t1 = REALTYPE(0.5)*(p2-p0);
        const REALTYPE t2 = REALTYPE(0.5)*(p3-p1);
        outputData(i,j) = p1*h1+p2*h2+t1*h3+t2*h4;
      }      
    }
    pastCurrTime = currTime;
  }   
  return outputData;
}


// ****************************************************
// ****************************************************
// ****************************************************

Matrix& SplineFit::HermitteSplineFitQuat(const Matrix& inputData, Matrix& outputData){
  const int dataCount   = inputData.RowSize(); 
  const int fieldCount  = inputData.ColumnSize()-1;
  const int outputCount = outputData.RowSize();
   
  if((fieldCount  <= 0)||
     (dataCount   <= 0)||
     (outputCount <= 0)) return outputData;
  
  if(fieldCount>4) return outputData;
  
  outputData.Resize(outputCount, fieldCount+1);

  const REALTYPE startTime = inputData.At(0,0);
  const REALTYPE stopTime  = inputData.At(dataCount-1,0);
  
  REALTYPE currTime     = outputData.At(0,0);
  REALTYPE pastCurrTime = currTime-R_ONE;
  REALTYPE prevTime     = startTime;
  REALTYPE nextTime     = startTime;
  int      prev         = 0;
  int      next         = 0;

  for(int i=0;i<outputCount;i++){
    currTime = outputData.At(i,0);
    
    // strictly increasing time constraint not satisfied...
    if(pastCurrTime>currTime) return outputData;
    
    // Find the nearest data pair    
    if(currTime<=startTime){
      outputData.InsertSubRow(i,1,inputData,0,1,fieldCount);
    }else if(currTime>=stopTime){
      outputData.InsertSubRow(i,1,inputData,dataCount-1,1,fieldCount);
    }else{ 
      const int searchStartPos = prev+1;
      for(int j=searchStartPos;j<dataCount;j++){
        next      = j;
        nextTime  = inputData.At(j,0);
        if((currTime>=prevTime)&&(currTime<nextTime)) break;        
        prev      = next;
        prevTime  = nextTime; 
      }
      

      const REALTYPE nphase = (currTime-prevTime)/(nextTime-prevTime);
      const REALTYPE s1 = nphase;
      const REALTYPE s2 = s1*nphase;
      const REALTYPE s3 = s2*nphase; 
      const REALTYPE h1 =  2.0f*s3 - 3.0f*s2 + 1.0f;
      const REALTYPE h2 = -2.0f*s3 + 3.0f*s2;
      const REALTYPE h3 =       s3 - 2.0f*s2 + s1;
      const REALTYPE h4 =       s3 -      s2;
      
      Quaternion p1,p2,res;
      p1.Set(inputData.At(prev,1),inputData.At(prev,2),inputData.At(prev,3),inputData.At(prev,4));
      p2.Set(inputData.At(next,1),inputData.At(next,2),inputData.At(next,3),inputData.At(next,4));
      
      p1.Slerp(p2,nphase,res);
      res.Normalize();
      outputData(i,1) = res.X();
      outputData(i,2) = res.Y();
      outputData(i,3) = res.Z();
      outputData(i,4) = res.W();

    }
    pastCurrTime = currTime;
  }   
  return outputData;
}

// ****************************************************
// ****************************************************
// ****************************************************
// ****************************************************








Vector& SplineFit::HermitteSplineFit(const Matrix& inputData, Vector& outputData, int *startRowGuess, REALTYPE *phase){
  const int dataCount   = inputData.RowSize(); 
  const int fieldCount  = inputData.ColumnSize()-1;
  
  int startRow = (startRowGuess!=NULL?*startRowGuess:0);
  
  if(phase) *phase = R_ZERO;
   
  if((fieldCount <= 0)||
     (dataCount  <= 0)||
     (startRow   >=dataCount)){
      if(startRowGuess!=NULL) *startRowGuess = -1;
      return outputData;
  }
  
  if(startRow<0) startRow=0; 
  
  outputData.Resize(fieldCount+1);
  
  const REALTYPE startTime = inputData.At(startRow,0);
  const REALTYPE stopTime  = inputData.At(dataCount-1,0);
  
  REALTYPE currTime     = outputData.At(0);
  REALTYPE prevTime     = startTime;
  REALTYPE nextTime     = startTime;
  int      prev         = startRow;
  int      next         = startRow;
        
  // Find the nearest data pair    
  if(currTime<=startTime){    
    outputData.InsertSubVector(1,inputData.GetRow(startRow),1,fieldCount);
  }else if(currTime>=stopTime){
    startRow = dataCount-1;
    outputData.InsertSubVector(1,inputData.GetRow(startRow),1,fieldCount);
  }else{ 
    for(int j=startRow+1;j<dataCount;j++){
      next      = j;
      nextTime  = inputData.At(j,0);
      if((currTime>=prevTime)&&(currTime<nextTime)) break;        
      prev      = next;
      prevTime  = nextTime; 
    }
    startRow = prev;

    const REALTYPE nphase = (currTime-prevTime)/(nextTime-prevTime);
    const REALTYPE s1 = nphase;
    const REALTYPE s2 = s1*nphase;
    const REALTYPE s3 = s2*nphase; 
    const REALTYPE h1 =  2.0f*s3 - 3.0f*s2 + 1.0f;
    const REALTYPE h2 = -2.0f*s3 + 3.0f*s2;
    const REALTYPE h3 =       s3 - 2.0f*s2 + s1;
    const REALTYPE h4 =       s3 -      s2;
    if(phase) *phase = nphase;
    for(int j=1;j<=fieldCount;j++){
      const REALTYPE p0 = (prev>0?inputData.At(prev-1,j):inputData.At(prev,j));
      const REALTYPE p1 = inputData.At(prev,j);
      const REALTYPE p2 = inputData.At(next,j);      
      const REALTYPE p3 = (next<dataCount-1?inputData.At(next+1,j):inputData.At(next,j));
      const REALTYPE t1 = REALTYPE(0.5)*(p2-p0);
      const REALTYPE t2 = REALTYPE(0.5)*(p3-p1);
      outputData(j) = p1*h1+p2*h2+t1*h3+t2*h4;
    }      
  }

  if(startRowGuess!=NULL) *startRowGuess = startRow;
  return outputData;
}

Matrix  SplineFit::mKinenaticSplineFitMatrix;

Matrix& SplineFit::KinematicSplineCoefs(const Vector& pos, const Vector& vel, const Vector& acc, const Vector& targetPos, const Vector& targetVel, const Vector& targetAcc, const REALTYPE targetTime, Matrix& resultCoefs){
  if(mKinenaticSplineFitMatrix.RowSize()==0){
    Matrix timeCoefs(6,6);
    mKinenaticSplineFitMatrix.Resize(6,6,false);
    mKinenaticSplineFitMatrix.Zero();
    timeCoefs(0,0) = R_ONE;  
    timeCoefs(1,1) = R_ONE;  
    timeCoefs(2,2) = 2.0*R_ONE;  
    for(int i=0;i<6;i++) timeCoefs(3,i) = R_ONE;
    for(int i=1;i<6;i++) timeCoefs(4,i) = REALTYPE(i);
    for(int i=2;i<6;i++) timeCoefs(5,i) = REALTYPE(i*(i-1));
    timeCoefs.Inverse(mKinenaticSplineFitMatrix);
    //timeCoefs.Print();    
    //mKinenaticSplineFitMatrix.Print();
  }
  REALTYPE  targetTime2 = targetTime *targetTime;
  Matrix stateMatrix(6,pos.Size());
  stateMatrix.SetRow(pos,0);
  stateMatrix.SetRow(vel,1); 
  stateMatrix.SetRow(acc,2); 
  stateMatrix.SetRow(targetPos,3);
  stateMatrix.SetRow(targetVel,4); 
  stateMatrix.SetRow(targetAcc,5);
  Vector timeCoefs(6);
  timeCoefs(0) = R_ONE;
  timeCoefs(1) = targetTime;
  timeCoefs(2) = targetTime2;
  timeCoefs(3) = R_ONE;
  timeCoefs(4) = targetTime;
  timeCoefs(5) = targetTime2;  
  
  stateMatrix.SMultRow(timeCoefs);
  //stateMatrix.Print();
  //mKinenaticSplineFitMatrix.Print();
  mKinenaticSplineFitMatrix.Mult(stateMatrix,resultCoefs);
  //resultCoefs.Print();
  
  return resultCoefs;
}    

void    SplineFit::KinematicSplineFit(const Matrix& coefs, const REALTYPE targetTime, const REALTYPE time, Vector* pos, Vector* vel, Vector* acc){
  REALTYPE t     = R_ONE;
  REALTYPE nTime = time/targetTime;
  if(nTime>1.0) return;
  
  const unsigned int size = coefs.ColumnSize();
  Vector timeVector(6);
  for(int i=0;i<5;i++){
    timeVector(i) = t;
    t*=nTime;
  }
  timeVector(5) = t;

  //timeVector.Print();
  //coefs.Print();
  if(pos){
    coefs.MultTranspose(timeVector,*pos);
  }
  
  if((vel)||(acc)){
    REALTYPE  invTargetTime = R_ONE/targetTime;
    timeVector.ShiftRight(); timeVector(0) = R_ZERO;
    for(int i=1;i<6;i++) timeVector(i) *= REALTYPE(i);
    if(vel){   
      coefs.MultTranspose(timeVector,*vel);
      *vel *= invTargetTime;
    }
    if(acc){
      REALTYPE  invTargetTime2 = invTargetTime*invTargetTime;
      timeVector.ShiftRight(); timeVector(0) = R_ZERO;
      for(int i=2;i<6;i++) timeVector(i) *= REALTYPE(i);  
      coefs.MultTranspose(timeVector,*acc);
      *acc *= invTargetTime2;
    }
  }
}


Matrix  SplineFit::mKinenaticSplineFit2Matrix;

Matrix& SplineFit::KinematicSplineCoefs2(const Vector& pos, const Vector& vel, const Vector& targetPos, const Vector& targetVel, const REALTYPE targetTime, Matrix& resultCoefs){
  if(mKinenaticSplineFitMatrix.RowSize()==0){
    Matrix timeCoefs(4,4);
    mKinenaticSplineFitMatrix.Resize(4,4,false);
    mKinenaticSplineFitMatrix.Zero();
    timeCoefs(0,0) = R_ONE;  
    timeCoefs(1,1) = R_ONE;  
    for(int i=0;i<4;i++) timeCoefs(2,i) = R_ONE;
    for(int i=1;i<4;i++) timeCoefs(3,i) = REALTYPE(i);
    timeCoefs.Inverse(mKinenaticSplineFit2Matrix);
    //timeCoefs.Print();    
    mKinenaticSplineFit2Matrix.Print();
  }
  REALTYPE  targetTime2 = targetTime *targetTime;
  Matrix stateMatrix(4,pos.Size());
  stateMatrix.SetRow(pos,0);
  stateMatrix.SetRow(vel,1); 
  stateMatrix.SetRow(targetPos,2);
  stateMatrix.SetRow(targetVel,3); 
  Vector timeCoefs(4);
  timeCoefs(0) = R_ONE;
  timeCoefs(1) = targetTime;
  timeCoefs(2) = R_ONE;
  timeCoefs(3) = targetTime;
  
  stateMatrix.SMultRow(timeCoefs);
  //stateMatrix.Print();
  //mKinenaticSplineFitMatrix.Print();
  mKinenaticSplineFit2Matrix.Mult(stateMatrix,resultCoefs);
  //resultCoefs.Print();
  
  return resultCoefs;
}    

void    SplineFit::KinematicSplineFit2(const Matrix& coefs, const REALTYPE targetTime, const REALTYPE time, Vector* pos, Vector* vel, Vector* acc){
  REALTYPE t     = R_ONE;
  REALTYPE nTime = time/targetTime;
  if(nTime>1.0) return;
  
  const unsigned int size = coefs.ColumnSize();
  Vector timeVector(4);
  for(int i=0;i<3;i++){
    timeVector(i) = t;
    t*=nTime;
  }
  timeVector(3) = t;

  //timeVector.Print();
  //coefs.Print();
  if(pos){
    coefs.MultTranspose(timeVector,*pos);
  }
  
  if((vel)||(acc)){
    REALTYPE  invTargetTime = R_ONE/targetTime;
    timeVector.ShiftRight(); timeVector(0) = R_ZERO;
    for(int i=1;i<4;i++) timeVector(i) *= REALTYPE(i);
    if(vel){   
      coefs.MultTranspose(timeVector,*vel);
      *vel *= invTargetTime;
    }
    if(acc){
      REALTYPE  invTargetTime2 = invTargetTime*invTargetTime;
      timeVector.ShiftRight(); timeVector(0) = R_ZERO;
      for(int i=2;i<4;i++) timeVector(i) *= REALTYPE(i);  
      coefs.MultTranspose(timeVector,*acc);
      *acc *= invTargetTime2;
    }
  }
}


void    SplineFit::KinematicSplineFitMax2(const Matrix& coefs, const REALTYPE targetTime, Vector* pos, Vector* vel, Vector* acc){
  REALTYPE t     = R_ONE;
  //REALTYPE nTime = time/targetTime;
  //if(nTime>1.0) return;
  
  const unsigned int size = coefs.ColumnSize();

  //if(*vel){
    Vector vmin;
    Vector vmax;
    Vector v0;
    Vector vT;
    coefs.GetRow(1,v0);
    vT = coefs.GetRow(1) + (coefs.GetRow(2)*2.0) + (coefs.GetRow(3)*3.0);
    
    vmin.Min(v0,vT);
    vmax.Max(v0,vT);

    Vector vmid(v0);
    for(int i=0;i<size;i++){
      if(coefs.At(3,i)!=0.0){
        REALTYPE t = -1.0/3.0*(coefs.At(2,i)/coefs.At(3,i));
        if((t>=0.0)&&(t<=1.0)){
          vmid(i) = 3.0*coefs.At(3,i)*(t*t) + 2.0*coefs.At(2,i)*(t) + coefs.At(1,i);  
        }  
      }
    }
    vmin.Min(vmin,vmid);
    vmax.Max(vmax,vmid);
    

    //cout <<"minmax "<<coefs.RowSize()<<" "<<coefs.ColumnSize()<<endl;
    //vmin.Print();
    //vmax.Print();
  if(vel){
    vel->Max(vmin.Abs(),vmax.Abs());
    (*vel)*=1.0/targetTime;
  }
  
  if(acc){
    Vector a0;
    Vector aT;
    a0 = (coefs.GetRow(2)*2.0);
    aT = (coefs.GetRow(2)*2.0) + (coefs.GetRow(3)*6.0);
    acc->Max(a0.Abs(),aT.Abs());
    (*acc)*=1.0/(targetTime*targetTime);
  }
  
  /*
  //timeVector.Print();
  //coefs.Print();
  if(pos){
    coefs.MultTranspose(timeVector,*pos);
  }
  
  if((vel)||(acc)){
    REALTYPE  invTargetTime = R_ONE/targetTime;
    timeVector.ShiftRight(); timeVector(0) = R_ZERO;
    for(int i=1;i<4;i++) timeVector(i) *= REALTYPE(i);
    if(vel){   
      coefs.MultTranspose(timeVector,*vel);
      *vel *= invTargetTime;
    }
    if(acc){
      REALTYPE  invTargetTime2 = invTargetTime*invTargetTime;
      timeVector.ShiftRight(); timeVector(0) = R_ZERO;
      for(int i=2;i<4;i++) timeVector(i) *= REALTYPE(i);  
      coefs.MultTranspose(timeVector,*acc);
      *acc *= invTargetTime2;
    }
  }
  */
}
