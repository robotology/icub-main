#include "MathLib.h"
//#define MYMAIN
#ifdef MYMAIN

#include <string>
#include <iostream>
using namespace std;
#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

int main(int argc, char *argv[]){


  Matrix3 m1,m2,m3;
  Matrix3 res1,res2,res3;  
  Vector3 v;

  RANDOMIZE;
        REALTYPE f1,f2,f3;
        f1 = (RND(TWOPI)-PI);
        f2 = (RND(TWOPI)-PI);
        f3 = (RND(TWOPI)-PI);
        cout << RAD2DEG(f1) <<" "<<RAD2DEG(f2) <<" "<<RAD2DEG(f3) <<" "<<endl;
    
  for(int r1=0;r1<3;r1++){
    for(int r2=0;r2<3;r2++){
      if(r1==r2) continue;
      for(int r3=0;r3<3;r3++){
        if(r2==r3) continue;

        cout << "-------------------------------------------"<<endl;

        for(int p=0;p<=1;p++){
          switch(r1){
          case 0: m1.RotationX(f1); break;
          case 1: m1.RotationY(f1); break;
          case 2: m1.RotationZ(f1); break;
          }      
          switch(r2){
          case 0: m2.RotationX(f2); break;
          case 1: m2.RotationY(f2); break;
          case 2: m2.RotationZ(f2); break;
          }      
          switch(r3){
          case 0: m3.RotationX(f3); break;
          case 1: m3.RotationY(f3); break;
          case 2: m3.RotationZ(f3); break;
          }      
          res1 = m3*m2*m1;
          switch(r1){ case 0: cout <<"X"; break; case 1: cout <<"Y"; break;case 2: cout <<"Z"; break;}
          switch(r2){ case 0: cout <<"X"; break; case 1: cout <<"Y"; break;case 2: cout <<"Z"; break;}
          switch(r3){ case 0: cout <<"X"; break; case 1: cout <<"Y"; break;case 2: cout <<"Z"; break;}
          cout << " ";
          res1.GetEulerAngles(r1,r2,r3,false,v,p);
          //res1.Print();
          //v.Print();
          
          switch(r1){
          case 0: m1.RotationX(v[0]); break;
          case 1: m1.RotationY(v[0]); break;
          case 2: m1.RotationZ(v[0]); break;
          }      
          switch(r2){
          case 0: m2.RotationX(v[1]); break;
          case 1: m2.RotationY(v[1]); break;
          case 2: m2.RotationZ(v[1]); break;
          }      
          switch(r3){
          case 0: m3.RotationX(v[2]); break;
          case 1: m3.RotationY(v[2]); break;
          case 2: m3.RotationZ(v[2]); break;
          }      
          res2 = m3*m2*m1;
  
          cout << RAD2DEG(v[0]) <<" " <<RAD2DEG(v[1])<<" "<< RAD2DEG(v[2])<<   endl;
    /*        
     * 
          cout << "********("<<r1<<","<<r2<<","<<r3<<") *********"<<endl;
          cout << "*************************"<<endl;
          */
  
    
          res3 = res1 - res2;
          if(res3.SAbs().Sum()>0.0001){
            cout << "********("<<r1<<","<<r2<<","<<r3<<") *********"<<endl;
  /*          cout << RAD2DEG(v[0]) <<" "<<RAD2DEG(v[1]) <<" "<<RAD2DEG(v[2]) <<""<<endl;
            for(int p=0;p<3;p++) {
              cout << res1(p,0) << " "<< res1(p,1) << " "<< res1(p,2);  
              cout << "          ";  
              cout << res2(p,0) << " "<< res2(p,1) << " "<< res2(p,2);
              cout <<endl;  
            }
  //        cout << f1 <<" "<<f2 <<" "<<f3 <<" "<<endl;
    //      cout << v[0] <<" "<<v[1] <<" "<<v[2] <<""<<endl;
            cout << "*************************"<<endl;
            //cout << "********("<<r1<<","<<r2<<","<<r3<<") *********"<<endl;
            //cout << f1 <<""<<f2 <<""<<f3 <<""<<endl;
            //cout << v[0] <<""<<v[1] <<""<<v[2] <<""<<endl;
            //res1.Print();
            //res2.Print();*/
          }
        }
      }
    }
  }
    

  




  return 0;
}
#if 0
  Vector p(4);
  Vector v(4);
  Vector a(4);
  Vector tp(4);
  Vector tv(4);
  Vector ta(4);
  Matrix res;

  p(0)  = 0;
  v(0)  = 0;
  a(0)  = 1;
  tp(0) = 0;
  tv(0) = -1;
  ta(0) = 0;

  p(3)  = 0;
  v(3)  = -1;
  a(3)  = 1;
  tp(3) = 0;
  tv(3) = -1;
  ta(3) = 0;

  SplineFit::KinematicSplineCoefs(p,v,a,tp,tv,ta,res);
  for(float t=0;t<1.0;t+=0.01){
    SplineFit::KinematicSplineFit(res,t,p,v,a);
    for(int i=0;i<4;i++){
      cout << p(i)<<" ";  
    }
    cout <<endl;
    
  }

return 0;  
  double dt = 0.01;
  double in;
  double fOut0;
  double fOut1;
  double fOut2;
  
  double out01,out02;
  double out11,out12;
  double out21,out22;

  double s0,s1,s2;

  EulerDifferentiator diff0,diff1,diff2;  
  DigitalFilter       df0,df1,df2;
  
  
  diff0.Init(1,2);
  diff1.Init(1,2);
  diff2.Init(1,2);
  
  int order = 5;
  int window = 10;
  df0.Init(1,order);
  df1.Init(1,window);
  df2.Init(1,window*8);
  df0.SetSamplingPeriod(dt);
  df1.SetSamplingPeriod(dt);
  df2.SetSamplingPeriod(dt);
  
  df0.SetButterworth(1.0/(double(window)*dt));
  df1.SetMovingAverage();  
  df2.SetHalfGaussian();  
  
  while(!feof(stdin)){
  //for(double t=0.0;t<20.0;t+=dt){
/*    s0 =  cos(t) + 0.2*cos(7*t);
    s1 = -sin(t) - 0.2*7*sin(7*t);
    s2 = -cos(t) - 0.2*49*cos(7*t);
        
    in  = s0 +(RND(1.0)-0.5)*0.01;
*/
    float inp;
    fscanf(stdin,"%f",&inp);
    in = double(inp);
    //printf("%f\n",inp);

    df0.SetInput(0,in);
    df0.Update();
    fOut0 = df0.GetOutput(0);
    
    df1.SetInput(0,in);
    df1.Update();
    fOut1 = df1.GetOutput(0);

    df2.SetInput(0,in);
    df2.Update();
    fOut2 = df2.GetOutput(0);

    diff0.SetInput(0,fOut0);
    diff0.Update(dt);
    out01 = diff0.GetOutput(0,1);
    out02 = diff0.GetOutput(0,2);

    diff1.SetInput(0,fOut1);
    diff1.Update(dt);
    out11 = diff1.GetOutput(0,1);
    out12 = diff1.GetOutput(0,2);

    diff2.SetInput(0,fOut2);
    diff2.Update(dt);
    out21 = diff2.GetOutput(0,1);
    out22 = diff2.GetOutput(0,2);
    
    
    printf("%f %f %f %f %f %f %f %f %f %f \n",in,fOut0,out01,out02,fOut1,out11,out12,fOut2,out21,out22);
  }
  
  return 0;
}





#if 0

  double dt = 0.01;
  double in;
  double fOut0;
  double fOut1;
  double fOut2;
  
  double out01,out02;
  double out11,out12;
  double out21,out22;

  double s0,s1,s2;

  EulerDifferentiator diff0,diff1,diff2;  
  DigitalFilter       df0,df1,df2;
  
  
  diff0.Init(1,2);
  diff1.Init(1,2);
  diff2.Init(1,2);
  
  int order = 5;
  int window = 10;
  df0.Init(1,order);
  df1.Init(1,window);
  df2.Init(1,window*8);
  df0.SetSamplingPeriod(dt);
  df1.SetSamplingPeriod(dt);
  df2.SetSamplingPeriod(dt);
  
  df0.SetButterworth(1.0/(double(window)*dt));
  df1.SetMovingAverage();  
  df2.SetHalfGaussian();  
  
  for(double t=0.0;t<20.0;t+=dt){
    s0 =  cos(t) + 0.2*cos(7*t);
    s1 = -sin(t) - 0.2*7*sin(7*t);
    s2 = -cos(t) - 0.2*49*cos(7*t);
        
    in  = s0 +(RND(1.0)-0.5)*0.01;



    df0.SetInput(0,in);
    df0.Update();
    fOut0 = df0.GetOutput(0);
    
    df1.SetInput(0,in);
    df1.Update();
    fOut1 = df1.GetOutput(0);

    df2.SetInput(0,in);
    df2.Update();
    fOut2 = df2.GetOutput(0);

    diff0.SetInput(0,fOut0);
    diff0.Update(dt);
    out01 = diff0.GetOutput(0,1);
    out02 = diff0.GetOutput(0,2);

    diff1.SetInput(0,fOut1);
    diff1.Update(dt);
    out11 = diff1.GetOutput(0,1);
    out12 = diff1.GetOutput(0,2);

    diff2.SetInput(0,fOut2);
    diff2.Update(dt);
    out21 = diff2.GetOutput(0,1);
    out22 = diff2.GetOutput(0,2);
    
    printf("%f %f %f %f %f %f %f %f %f %f %f %f %f \n",s0,s1,s2,in,fOut0,out01,out02,fOut1,out11,out12,fOut2,out21,out22);
  }
  


#endif




#if 0

  int K = 2;
  int N = 20;
  int M = 50;
  Matrix inData(N,K);
  Vector inTime(N);
  Matrix outData(M,K);
  Vector outTime(M);
  
  for(int i=0;i<N;i++){
    float t = float(i)/float(N);
    inTime(i) = t;
    inData(i,0) = sin(PIf*t);   
    inData(i,1) = cos(PIf*t);   
  }
  
  for(int i=0;i<M;i++){
    float t = float(i)/float(M);
    outTime(i) = t;
  }

  //HermitteSplineFit(inData, inTime, outTime, outData);
  HermitteSplineFit(inData, M, outData);
  inData.Transpose().Print();
  outData.Transpose().Print();
    
  return 0;
  /*
  Matrix G(10,10);        // row,column
  G.Random();           // Randomize [0,1]
  G = G*2-1;            // Rescale between [-1,+1]
  G.SetColumn(G.GetColumn(1),0);
  G.SetColumn(G.GetColumn(3),2);
  G.SetColumn(G.GetColumn(5),4);
  G.SetColumn(G.GetColumn(5),9);
  Vector unit(10);
  unit(0)=1;
  G.Print();
  G.GramSchmidt(unit);
  G.RemoveZeroColumns();
  G.Print();
  Matrix H;
  H = G^G;
  H.SumRow().Print();

  return 0;
  */
  
  Matrix A(5,5);        // row,column
  A.Random();           // Randomize [0,1]
  A = A*2-1;            // Rescale between [-1,+1]

  A.Random();           // Randomize [0,1]
  A.Mult(2,A).Sub(1,A); // Rescale between [-1,+1]
  A.Print();
  
  Matrix InvA,Id;
  A.Inverse(InvA);      // Inverse A
  if(Matrix::IsInverseOk()){
    (A*InvA).Print();   // Show Identity
    A.Mult(InvA,Id);    // Multiply
    Id.Print();         // Show identity
  }else{
    printf("Error in inverse...\n");
  }
  
  
  Matrix B(5,5);
  Matrix C,D;             // Empty matrices
  Matrix E(A);            // Copy constructor
  B.Identity();           // Identity();
  B(0,0) = 10.0f;         // Assignment
  B.Print();              // Print
  B.GetColumnSpace(0,2,C);// Get the two first column
  C.Print();
    
  A.Transpose(D);         // Transpose
  D.Print();
  
  E = A+D;
  
  Vector v1(5),v2,v3;     // Vectors
  v1.Random();
  v2 = A*v1;
  v3 = InvA*v2;
  v1.Print();
  v2.Print();
  v3.Print();
  
  
    
}
#endif
#endif
#endif
