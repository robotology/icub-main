#include "recmot.h"



iCub::contrib::primateVision::RecHandleMotionRequest::RecHandleMotionRequest(int period):
  RateThread(period)
{
  inPort_mot.open( "/recserver/input/motion"   );
  angles[0]=0.0;
  angles[1]=0.0;
  angles[2]=0.0;
  angles[3]=0.0;
  angles[4]=0.0;
  angles[5]=0.0;
  suspend = 0;
  locked_to = NO_LOCK;



   //to read inertial port:
  inertial = new BufferedPort<yarp::sig::Vector>;
  inertial->open("/recserver/input/inertial");
  Network::connect("/icub/inertial" , "/recserver/input/inertial");

  vor_vels[0]=0.0; 
  vor_vels[1]=0.0; 
  vor_vels[2]=0.0; 
  vor_vels[3]=0.0; 
  vor_vels[4]=0.0; 
  vor_vels[5]=0.0; 

  gyro_vel[0] = 0.0;
  gyro_vel[1] = 0.0;
  gyro_vel[2] = 0.0;

 
}




void iCub::contrib::primateVision::RecHandleMotionRequest::run(){
  
  
  if (motion){

    if (suspend>0){
      suspend--;
      printf("RecServer: Suspended %d.\n",suspend);
    }
    



    //incorporate motion requests:
    rmq = inPort_mot.read(false);
    //skip updating desired target pos for "suspend" * period:
    if (rmq!=NULL && suspend==0){
      
      if (locked_to!=NO_LOCK){
	printf("locked to: %d\n",locked_to);
      }

      if (locked_to == rmq->content().lockto || locked_to == NO_LOCK){
	//accept command..
	//printf("acting.\n");
	angles[0] = rmq->content().deg_p;                
	angles[1] = rmq->content().deg_r;                
	angles[2] = rmq->content().deg_y;                
	angles[3] = -rmq->content().pix_y*pix2degy;      
	angles[4] = (rmq->content().pix_xl+rmq->content().pix_xr)*pix2degx/2.0;
	angles[5] = (rmq->content().pix_xl-rmq->content().pix_xr)*pix2degx/2.0;
	relative  = rmq->content().relative;
	suspend   = rmq->content().suspend;
	locked_to = rmq->content().lockto;
	if (rmq->content().unlock == 1){
	  locked_to = NO_LOCK;
	}
	
	if (!fake){	
	  if (relative){//relative move requested:
	    //convert to absolute:
	    for (int i=0;i<6;i++){
	      angles[i] += enc[i];
	    }
	  }
	  if(angles[5]<0.0){
	    angles[5]=0.0;
	  }
	}

	//fake:
	else {//if (fake){      
	  if (relative){//relative move requested:
	    //convert to absolute:
	    for (int i=0;i<6;i++){
	      enc[i] += angles[i]; 
	    }
	  }
	  else{
	    for (int i=0;i<6;i++){
	      enc[i] = angles[i]; 
	    }
	  }
	}
	

      }
    }
    
    

    //incorporate VOR:
    if (vor_on){

      //get gyro accelleration data:
      gyro = inertial->read(false); //false = non-blocking
      if (gyro!=NULL){
	
	//printf("RecServer: VOR Data.\n");
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
	gyro_vel[0] *= vor_d_rol;
	gyro_vel[1] *= vor_d_tlt;
	gyro_vel[2] *= vor_d_pan;
	
	//corrective motion cmd (only vestibular axes)
	vor_vels[0] =  vor_k_rol*(gyro_vel[0]); //roll correction
	vor_vels[1] =  -vor_k_tlt*(gyro_vel[1]); //tilt correction
	vor_vels[2] =  vor_k_pan*(gyro_vel[2]); //pan correction
      }
      //else{
      //printf("RecServer: No VOR Data.\n");
      //	vor_vels[0]=0.0;
      //	vor_vels[1]=0.0;
      //	vor_vels[2]=0.0;
      //}
      
    }


    

    //motion command:
    if (!fake){


      //get encoder data:
      if (encs->getEncoders(enc_tmp)){
	for (int i=0;i<6;i++){
	  enc[i] = enc_tmp[i];
	}
      }
  
      vels[0] = k_vel_pitch*(angles[0]-enc[0]);
      vels[1] = k_vel_roll*(angles[1]-enc[1]);
      vels[2] = k_vel_yaw*(angles[2]-enc[2]);
      vels[3] = k_vel_tilt*(angles[3]-enc[3]);
      vels[4] = k_vel_vers*(angles[4]-enc[4]);
      vels[5] = k_vel_verg*(angles[5]-enc[5]);

      //incorporate VOR corrections:
      if (vor_on){
	vels[1] += vor_vels[0];  //head roll
	vels[3] += vor_vels[1];  //eye tilt
	vels[4] += vor_vels[2];  //eye pan
      }


      //send!
      vel->velocityMove(vels);
    }
   


 
  }
  
  
  
    
}



