 /*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "vor.h"

#include <stdio.h>
#include <iostream>
#include <yarp/os/Module.h>
#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace std; 


iCub::contrib::primateVision::VORServer::VORServer(string*cfg_)
{
  
  cfg=cfg_;

  start();

}

iCub::contrib::primateVision::VORServer::~VORServer()
{

  stop();

}


void iCub::contrib::primateVision::VORServer::stop()
{

  double vels[6];
  vels[0]=0.0; 
  vels[1]=0.0; 
  vels[2]=0.0; 
  vels[3]=0.0; 
  vels[4]=0.0; 
  vels[5]=0.0; 

  vel_ctrl->velocityMove(vels);
  vel_ctrl->stop();

}


void iCub::contrib::primateVision::VORServer::run()
{
  

  Network yarp;


  Property prop;
  prop.fromConfigFile(cfg->c_str());
  double k_rol = prop.findGroup("VOR").find("K_ROL").asDouble();
  double k_pan = prop.findGroup("VOR").find("K_PAN").asDouble();
  double k_tlt = prop.findGroup("VOR").find("K_TLT").asDouble();
  double d_rol = prop.findGroup("VOR").find("D_ROL").asDouble();
  double d_pan = prop.findGroup("VOR").find("D_PAN").asDouble();
  double d_tlt = prop.findGroup("VOR").find("D_TLT").asDouble();
  


  //instantiate remotecontrolboard for VOR motion:
  Property options_rcb;
  options_rcb.put("device", "remote_controlboard");
  options_rcb.put("local",  "/vorserver/rcb_client"); 
  options_rcb.put("remote", "/icub/head");            
  PolyDriver *VORMotion = new PolyDriver(options_rcb);
  if (!VORMotion->isValid()) {
    printf("VORServer: PolyDriver motion device not available.\n");
  }
  VORMotion->view(vel_ctrl);
  if (vel_ctrl==NULL){     
    VORMotion->close();
    printf("VORServer: No head controller device found.\n");
  }
  

  //to read inertial port:
  BufferedPort<yarp::sig::Vector> *inertial = new BufferedPort<yarp::sig::Vector>;
  inertial->open("/vorserver/input/inertial");
  Network::connect("/icub/inertial" , "/vorserver/input/inertial");
  Vector *gyro = NULL;


  double vels[6];
  vels[0]=0.0; 
  vels[1]=0.0; 
  vels[2]=0.0; 
  vels[3]=0.0; 
  vels[4]=0.0; 
  vels[5]=0.0; 

  double gyro_vel[3];
  gyro_vel[0] = 0.0;
  gyro_vel[1] = 0.0;
  gyro_vel[2] = 0.0;



  //start at rest:
  vel_ctrl->velocityMove(vels);


  while (1){


    //get gyro accelleration data:
    gyro = inertial->read(false); //false = non-blocking
    if (gyro!=NULL){
      
      
      /*inertial signal 12-chan as follows:
       * (*gyro)[0];//r
       * (*gyro)[1];//p
       * (*gyro)[2];//y
       * 
       * (*gyro)[3];//ax
       * (*gyro)[4];//ay
       * (*gyro)[5];//az
       * 
       * (*gyro)[6];//gx
       * (*gyro)[7];//gy
       * (*gyro)[8];//gz
       *
       * (*gyro)[9]; //mx
       * (*gyro)[10];//my
       * (*gyro)[11];//mz
       */
      

      //integrate to get vel:
      gyro_vel[0] += (*gyro)[6];//gx
      gyro_vel[1] += (*gyro)[7];//gy
      gyro_vel[2] += (*gyro)[8];//gz
      
      //tend towards zero to eliminate residual velocity drift:
      gyro_vel[0] *= d_rol;
      gyro_vel[1] *= d_tlt;
      gyro_vel[2] *= d_pan;
      
      //corrective motion cmd (only vestibular axes)
      vels[1] =  k_rol*(gyro_vel[0]); //roll correction
      vels[3] = -k_tlt*(gyro_vel[1]); //tilt correction
      vels[4] =  k_pan*(gyro_vel[2]); //pan correction
      
#if 1
      //send motion cmd:
      vel_ctrl->velocityMove(vels);
#endif

      //usleep(2000);
    }
    else{
      //usleep(1000);//don't blow out port
    }
   
 
  }
  
}


