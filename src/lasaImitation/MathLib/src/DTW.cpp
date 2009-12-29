#include "DTW.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

float DTW::Distance(const Vector &x,const Vector &y)
{
  Vector temp = x - y;
  return temp.Norm2();
}

Matrix & DTW::Distance(const Matrix &x,const Matrix &y, Matrix &result){
  
  Matrix mx(x);
  Matrix my(y);
  
  const int varSize = mx.ColumnSize();
  const int xSize   = mx.RowSize();
  const int ySize   = my.RowSize();
  
  Vector meanX(varSize);
  Vector meanY(varSize);
  Vector minX(varSize);
  Vector minY(varSize);
  Vector maxX(varSize);
  Vector maxY(varSize);
  
  mx.SumRow(meanX);
  my.SumRow(meanY);
  meanX /= float(xSize);
  meanY /= float(ySize);
 
  result.Resize(xSize,ySize,false);
  result.Zero();

  Vector val;
  for(int i=0;i<varSize;i++){
    mx.GetColumn(i,val); minX(i) = val.Min(); maxX(i) = val.Max();
    my.GetColumn(i,val); minY(i) = val.Min(); maxY(i) = val.Max();
  }
  
  Vector vx,vy;  
  for(int i=0;i<xSize;i++){
    for(int j=0;j<ySize;j++){
      mx.GetRow(i,vx);
      my.GetRow(j,vy);
      vx -= meanX;
      vy -= meanY;      
      vx(0) = 0.0f;
      vy(0) = 0.0f;
      result(i,j)= Distance(vx,vy);
    }
  }
 
  return result; 
}

float*   DTW::SmoothPath(int* pathIn, float *pathOut, const int size, const int window){
  const int hw = window/2;
  const int ws = hw*2;

  for(int i=0;i<hw;i++){
    float mean = float(pathIn[0]) * float(hw-i);
    for(int j=-i;j<hw-1;j++){
      mean += pathIn[i+j];
    } 
    mean/= float(ws);
    pathOut[i] = mean;
  }
   
  for(int i=hw;i<size-hw;i++){
    float mean = 0.0f;
    for(int j=-hw;j<hw-1;j++){
      mean += pathIn[i+j];
    } 
    mean/= float(ws);
    pathOut[i] = mean;
  }

  for(int i=0;i<hw-1;i++){
    float mean = float(pathIn[size-1]) * float(i);
    for(int j=-hw;j<i;j++){
      mean += pathIn[size-hw+i+j];
    } 
    mean/= float(ws);
    pathOut[size-hw+i] = mean;
  }


}


Matrix & DTW::Process(Matrix &Ref, Matrix &In, Matrix &Out, Matrix& Dist, Matrix& Cost)
{
  std::cout << Ref.RowSize()<< " <-In Out-> " << In.RowSize()<<std::endl;

  // parametres jouables : alpha du passebas
  // "zone de distortion maxi dans le calcul du path
  int     *path;
  float   *smoothpath;
  int refsize = Ref.RowSize();
  int insize  = In.RowSize();
  
  Out.Resize(Ref.RowSize(),Ref.ColumnSize(),false);
  
  // float maxConstraint = refsize/10; // Max distortion ..     
  Dist.Resize(refsize,insize);
  Cost.Resize(refsize,insize);
  path        = new  int[insize];
  smoothpath  = new float[insize];
  
  float alpha = 0.5;
  float totalDistance = 0.0;
  float linearDistance = 0.0;
  
  //cout << "RefSize "<< refsize <<" InSize "<<insize<<endl;

  /* Computing distance matrix */

  Distance(Ref,In, Dist);
  
  /*
  Vector v1,v2;  
  for(int i = 0;i<refsize;i++){
    for(int j=0;j<insize;j++) {
      Ref.GetRow(i,v1);
      In.GetRow(j,v2);
      v1(0) = 0.0f;
      v2(0) = 0.0f;
      Dist(i,j)= DTWDistance(v1,v2);
    }
  }*/

  /*Then the cost matrix */
  Cost(0,0)=Dist(0,0);

  /* initialising firsts rows */
  for(int i=1;i<refsize;i++){
    Cost(i,0)=Dist(i,0)+Cost(i-1,0);
  }

  for(int j=1;j<insize;j++){
    Cost(0,j)=Dist(0,j)+Cost(0,j-1);
  }
  
  /* iterative Cost computation (see FastDTW for optimisation)*/ 
  for(int i=1;i<refsize;i++) {
    for(int j=1;j<insize;j++) {
      float prevDist = Cost(i-1,j-1);
      if(Cost(i-1,j) < prevDist) prevDist = Cost(i-1,j);
      if(Cost(i,j-1) < prevDist) prevDist = Cost(i,j-1);
      Cost(i,j)=Dist(i,j) + prevDist;
    }
  }

  /* finding optimal path */  
  path[insize-1]=refsize-1;
  
  // experimental path finding
  int c       = insize-1;
  int cursorx = insize-1;
  int cursory = refsize-1;
  int nextx   = 0;
  int nexty   = 0;
  float min   = 0.0f;
  while(c>0){
    if(cursorx < c) {
      //std::cout << c << std::endl;
      c       = cursorx;
      path[c] = cursory;
    }
    min = Cost(cursory,cursorx-1);
    nextx = cursorx - 1;
    nexty = cursory;
    if(Cost(cursory-1,cursorx-1) < min) {
      nextx = cursorx -1;
      nexty = cursory -1;
      min = Cost(cursory-1,cursorx-1);
    }
    if(Cost(cursory-1,cursorx) < min) {
      nextx = cursorx;
      nexty = cursory-1;
      min = Cost(cursory,cursorx-1);
    }
    cursorx = nextx;
    cursory = nexty;
  }

  /* computing score */ 
  for(int i=0;i<insize;i++) {
    linearDistance += Dist((int) (refsize*i/insize),i);
    totalDistance += Dist(path[i],i); 
  }
  std::cout << "Total Distance before warping : " << linearDistance << std::endl;
  std::cout << "Total Distance after warping : " << totalDistance << std::endl;
  
  /* smoothing path with a low pass filter */  
  smoothpath[0] = 0.0;
  for(int i=1;i<insize;i++) {
    smoothpath[i]=alpha*path[i]+(1-alpha)*smoothpath[i-1];
  }

  SmoothPath(path,smoothpath,insize,insize/32);
  
  /* Filling out the Out matrix */
  Vector inTimeBase(insize);
  Vector outTimeBase(insize);
  for(int i=0;i<insize;i++) {
    inTimeBase(i) =  smoothpath[i];
    outTimeBase(i) =float(i);    
  }
  
  Regression::HermitteSplineFit(In,inTimeBase,outTimeBase,Out);
  //for(int i=0;i<insize;i++) {
  //  Out(i,0) = i;
  //}

  //inTimeBase.Print();
  //outTimeBase.Print();
  //Out.SetRow(In.GetRow(path[i]),i);
  std::cout << In.RowSize()<< " <-In Out-> " << Out.RowSize()<<std::endl;
  std::cout << inTimeBase.Size()<< " <-In Out-> " << outTimeBase.Size()<<std::endl;
  
  //std::ofstream pathfile("rawpath.csv");
  //for(int i=0;i<insize;i++)
  //pathfile << path[i] << std::endl;

  delete [] path;
  delete [] smoothpath;


  return Out; 
}

