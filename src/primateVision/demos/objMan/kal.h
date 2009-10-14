#include <iCub/kalman.h>
#include <yarp/math/Math.h>

using namespace ctrl;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class kal
{
  
 public:

  kal(double proc_noise_cov=0.04, double meas_noise_cov=0.08){
    
    kalA=eye(3,3);		// the state is 3d pos
    kalH=eye(3,3);		// 3d pos is measured
    
    kalQ=proc_noise_cov*eye(3,3);		// Process noise covariance
    kalR=meas_noise_cov*eye(3,3);		// Measurement noise covariance
    
    //initial pos:
    kalx0.resize(3); kalx0=0.0;
    kalP0=10*eye(3,3);
    
    kalPos=new Kalman(kalA,kalH,kalQ,kalR);
    kalPos->init(kalx0,kalx0,kalP0);
    
  }
  
  ~kal(){
    delete kalPos;
  }
   

  Vector update(double x,double y,double z){
    
    //input next measurements: 
    kalx0(0)=x;
    kalx0(1)=y;
    kalx0(2)=z;
    
    estX=kalPos->filt(kalx0);

    return estX;
  }
  
  
 private:
  Matrix kalA,kalH,kalQ,kalR;
  Matrix kalP0;
  Vector kalx0;
  Kalman* kalPos;
  Vector estX;
  
};
