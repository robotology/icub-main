/**
 * Client of objRec.  Manages objects in the world. 
 * Outputs list of objects to be drawn/moved (to the sim).
 *
 */ 

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <iostream>
//YARP2 INCLUDES
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
//IPP
#include <ippi.h>

//for filters:
#include "kal.h"

//for obj struct:
#include "objManio.h"

#define MAX_OBJS 100
#define MIN_DIST 0.5 //meters!


using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;
using namespace ctrl;



int main( int argc, char **argv )
{


  obj *objList[MAX_OBJS];
  kal *kalman[MAX_OBJS]; 

  //get objRec server params:
  Port inPort_s;
  inPort_s.open("/objMan/input/serv_params");
  Network::connect("/objMan/input/serv_params","/objRec/output/serv_params");
  Network::connect("/objRec/output/serv_params","/objMan/input/serv_params");
  BinPortable<ObjManServerParams> server_response;
  Bottle empty;
  inPort_s.write(empty,server_response);
  RecServerParams rsp = server_response.content();
  std::cout << "ObjRecServer Probe Response: " << rsp.toString() << std::endl;
  

  //Port to get object postion and label:
  BufferedPort<Bottle> inPort_obj; 
  inPort_obj.open("/objMan/input/obj"); 
  Network::connect("/objRec/output/obj" , "/objMan/input/obj");
  Bottle *inBot_obj;
  
  // create port that outputs objList
  BufferedPort<Bottle> outPort_objList;
  outPort_objList.open("/objMan/output/objlist"); 

  double newRadius,newPosX,newPosY,newPosZ,newVal;
  string newLabel;
  Ipp8u*newImage;
  IppiSize imsize={rsp.width,rsp.height};
  int psb_in = rsp.psb;
  int numObjs = 0;
  bool handled;
  
  
  
  printf("ObjMan: begin..\n");
  //main event loop:
  while (1){
    
    
    //get data from objRec server:
    inBot_obj = inPort_obj.read(); //blocking
    if (inBot_obj!=NULL){
      
      newRadius =  inBot_obj->get(0).asDouble();
      newVal    =  inBot_obj->get(1).asDouble();
      newPosX   = -inBot_obj->get(2).asDouble();
      newPosY   = -inBot_obj->get(3).asDouble() + 0.928; //height of head offset
      newPosZ   =  inBot_obj->get(4).asDouble();
      newLabel  =  inBot_obj->get(5).asString();
      newImage  = (Ipp8u*) inBot_obj->get(6).asBlob();

      //Check if it is similar to an existing label at similar pos, we move the object.
      handled = false;
      for (int k=0;k<numObjs;k++){
	//if label exists	
	if (objList[k]->label==newLabel &&
	    //AND Malhonobis dist. is low:
	    sqrt( (newPosX-objList[k]->x)*(newPosX-objList[k]->x)+
		  (newPosY-objList[k]->y)*(newPosY-objList[k]->y)+
		  (newPosZ-objList[k]->z)*(newPosZ-objList[k]->z) ) < MIN_DIST
	    //AND OBJECTS LOOK SIMILAR:
	    //ADD ME****
	    ){
	  //not new object, so just update pos,radius,image of this object:	
	  objList[k]->radius = newRadius;
	  objList[k]->confidence = newVal;
	  //KALMAN FILTER POSITION:
	  Vector v = kalman[k]->update(newPosX,newPosY,newPosZ);
	  objList[k]->x = v[0];
	  objList[k]->y = v[1];
	  objList[k]->z = v[2];
	  //print out diff between measurement and Kalman filtered output:
	  printf("ObjMan: Object:%s (%f) - Measured: (%f,%f,%f)  Kalman: (%f,%f,%f)\n", 
		 objList[k]->label.c_str(),objList[k]->confidence,
		 newPosX,newPosY,newPosZ,
		 v[0],v[1],v[2]);
	  //cache image:
	  ippiCopy_8u_C1R(newImage,psb_in,objList[k]->texture,psb,imsize); //overwrite image
	  handled = true;
	  break; //eject the for loop
	}
      }
      
      if (!handled){
	//we have a new object!
	if (numObjs<MAX_OBJS){
	  objList[numObjs] = new obj();
	  objList[numObjs]->label  = newLabel;
	  objList[numObjs]->x      = newPosX;
	  objList[numObjs]->y      = newPosY;
	  objList[numObjs]->z      = newPosZ;
	  objList[numObjs]->radius = newRadius;
	  objList[numObjs]->confidence = newVal;
	  objList[numObjs]->texture = ippiMalloc_8u_C1(imsize.width,imsize.height,&psb); //malloc
	  kal[numObjs] = new kal(newPosX,newPosY,newPosZ);//spawn a Kalman filter with default settings
	  ippiCopy_8u_C1R(newImage,psb_in,objList[numObjs]->texture,psb,imsize); //copy in
	  //increment object counter
	  numObjs++;	
	}
	else{
	  printf("ObjMan: DATABASE FULL\n");
	}
      }
      

      printf("ObjMan: %d Objects in Database\n",numObjs);

      //prepare list of objects to send to clients:
      Bottle& bot = outPort_objList.prepare();
      bot.clear();
      bot.addInt(numObjs);
      for (int k=0;k<numObjs;k++){
	bot.add(Value::makeBlob(&objList[k],sizeof(obj)));
	//copy in image:
	
      }
      //send!
      outPort_objList.write();
      
    }//if received
    


  } //while

  //never here! 
}
