// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#include <iostream>
#include <string>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <YARPConfigurationVariables.h>
#include <ManipulatorOrientationJacobian.h>
#include <EndEffectorCartesianOrientation.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;
using namespace CB;

int main(int argc, char *argv[]) {

  string robot_name = "/icubSim/right_arm";
  Vector Vrot;
  Vector Vgoal;
  Vector Vcur;
  Vector Vref;
  Vector Vdiff;
  Vector Vq;
  Vector Vx;
  Matrix Mx;
  Matrix Mq;
  Matrix Jrot;
  Vector Jc;
  double phi;
  double gain;

  YARPConfigurationVariables robot(robot_name, "");
  robot.loadConfig("/home/hart/research/code/iCub/app/controlBasis/conf/right_arm.dh");
  robot.startResource();
  Time::delay(1);

  EndEffectorCartesianOrientation robotOrientation(robot_name);
  robotOrientation.startResource();
  Time::delay(1);

  ManipulatorOrientationJacobian robotJacobian;
  robotJacobian.setDevice(robot_name);
  robotJacobian.startJacobian();
  Time::delay(1);

  Vgoal.resize(4);
  Vref.resize(3);
  Vcur.resize(3);
  Vdiff.resize(3);

  Vgoal[0] = 1.0;
  Vgoal[1] = 0.0;
  Vgoal[2] = 0.0;
  Vgoal[3] = 0.5;

  while(true) {

    Vrot = robotOrientation.getResourceData();
    ///    Jrot = robotJacobian.getJacobianTranspose();
    Jrot = robotJacobian.getJacobianInverse();

    cout << "ref  -   cur (initial)" << endl;
    for(int i=0; i<Vrot.size(); i++) {
      cout << Vgoal[i] << "     " << Vrot[i] << endl;
    }
    cout << endl;
    
    /*
    cout << "Jacobian: " << endl;
    for(int i=0; i<Jrot.rows(); i++) {
      for(int j=0; j<Jrot.cols(); j++) {
          cout << Jrot[i][j] << " ";
      }
      cout << endl;
    }
    cout << endl;
    */

    for(int i=0; i<3; i++) {
        Vcur[i] = Vrot[i]*Vrot[3];
        Vref[i] = Vgoal[i]*Vgoal[3];
        Vdiff[i] = Vref[i]-Vcur[i];
    }

    cout << "ref  -   cur   =  diff  (scaled)" << endl;
    for(int i=0; i<Vref.size(); i++) {
      cout << Vref[i] << "     " << Vcur[i] << "     " << Vdiff[i] << endl;
    }
    cout << endl;

    Jc = 1.0*Vdiff;
    phi = 0.5*dot(Vdiff,Vdiff);
    cout << endl << "Potential: " << phi << endl << endl;

    gain = 1.0;
    Vx = Jc*phi*gain;
    //Vx = Jc*gain;

    /*
    cout << "Vx: " << endl;
    for(int i=0; i<Vx.size(); i++) {
      cout << Vx[i] << endl;
    }
    cout << endl;
    */

    Mx.resize(3,1);
    for(int i=0; i<Vx.size(); i++) {
      Mx[i][0] = Vx[i];
    }
    Mq = Jrot*Mx;

    
    Vq.resize(7);
    //cout << "Tau: " << endl;
    for(int i=0; i<Vq.size(); i++) {
      Vq[i] = Mq[i][0];
      //cout << Vq[i] << endl;
    }
    //    cout << endl;
    

    robot.setLock(false);
    robot.setDesiredIncrement(Vq);
    //    robot.setLock(true);
    Time::delay(.05);
  } 
  return 1;
}
