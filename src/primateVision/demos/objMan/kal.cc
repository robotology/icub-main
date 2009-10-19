#include "kal.h"
#include <stdio.h>
#include <iostream>
#include <sys/time.h>


using namespace std;

iCub::contrib::primateVision::Kal::Kal(int period,double X_,double Y_,double Z_,double proc_noise_cov, double meas_noise_cov):
  RateThread(period)
{
  
  kalA=eye(3,3);		  // the state is 3d pos
  kalH=eye(3,3);		  // 3d pos is measured
  
  kalQ=proc_noise_cov*eye(3,3); // Process noise covariance
  kalR=meas_noise_cov*eye(3,3); // Measurement noise covariance
  
  //initial pos:
  kalx0.resize(3); 
  //kalx0=0.0;
  kalx0(0)=X_;
  kalx0(1)=Y_;
  kalx0(2)=Z_;
 
  kalP0=10*eye(3,3);
  
  kalPos=new Kalman(kalA,kalH,kalQ,kalR);
  kalPos->init(kalx0,kalx0,kalP0);
  
  done = true;
}

Vector iCub::contrib::primateVision::Kal::update(double x_,double y_,double z_){
  //latch on these input measurements: 
  kalx0(0)=x_;
  kalx0(1)=y_;
  kalx0(2)=z_; 
  //wait for an update:
  while(!done){
    usleep(1000);
  }
  
  return estX;
}
