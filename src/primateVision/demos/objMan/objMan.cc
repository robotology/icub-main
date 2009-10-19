#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>

//MY INCLUDES
//client of:
#include <objRecio.h>

#include "objMan.h"
//for Kalman filters:
#include "kal.h"


#define MIN_DIST 0.5 //meters!

using namespace std;
using namespace iCub::contrib::primateVision;



iCub::contrib::primateVision::ObjManServer::ObjManServer( string*cfg_ )
{
  
  cfg=cfg_;
  start();

}

iCub::contrib::primateVision::ObjManServer::~ObjManServer()
{
  
}


void iCub::contrib::primateVision::ObjManServer::run()
{


  Property prop;
  prop.fromConfigFile(cfg->c_str());
  //double param = prop.findGroup("OBJMAN").find("PARAM").asDouble();



  ObjManServerList *objList;
  KalmanList *kalList; 


  //Probe objRec server:
  Port inPort_s;
  inPort_s.open("/objManServer/input/serv_params");
  Network::connect("/objManServer/input/serv_params","/objRecServer/output/serv_params");
  Network::connect("/objRecServer/output/serv_params","/objManServer/input/serv_params");
  BinPortable<ObjRecServerParams> server_response;
  Bottle empty;
  inPort_s.write(empty,server_response);
  ObjRecServerParams rsp = server_response.content();
  std::cout << "ObjRecServer Probe Response: " << rsp.toString() << std::endl;
  int width = rsp.width;
  int height = rsp.height;
  int psb_in  = rsp.psb;
  IppiSize imsize={width,height};


  //To get data from ObjRecServer:
  BufferedPort<ObjRecServerData > inPort_objData;  
  inPort_objData.open("/objManServer/input/objData");
  Network::connect("/objRecServer/output/objData" , "/objManServer/input/objData");
  ObjRecServerData *newObjData;
 
  //create port to output objList:
  Port outPort_objList;
  outPort_objList.open("/objManServer/output/objList"); 


  double newPosX,newPosY,newPosZ;
  int numObjs = 0;
  bool handled;
  

   
  printf("ObjManServer: begin..\n");
  //main event loop:
  while (1){
    
    
    //get data from objRec server:
    newObjData = inPort_objData.read(); //blocking
      
    //update:
    newPosX = -newObjData->x;
    newPosY = -newObjData->y + 0.928; //offset by head height
    newPosZ =  newObjData->z;

    //Check if it is similar to an existing label at a similar pos:
    handled = false;
    for (int k=0;k<numObjs;k++){
      //if label exists	
      if (objList->get(k)->label == newObjData->label &&
	  //AND Malhonobis dist. is low:
	  sqrt( (newPosX-objList->get(k)->x)*(newPosX-objList->get(k)->x)+
		(newPosY-objList->get(k)->y)*(newPosY-objList->get(k)->y)+
		(newPosZ-objList->get(k)->z)*(newPosZ-objList->get(k)->z) ) < MIN_DIST
	  //AND OBJECTS LOOK SIMILAR:
	  //ADD ME****SHOULD REALLY ONLY DO THIS!
	  ){
	//not new object, so update params only:
	objList->get(k)->radius     = newObjData->radius;
	objList->get(k)->confidence = newObjData->confidence;
	objList->get(k)->mos_x      = newObjData->mos_x;
	objList->get(k)->mos_y      = newObjData->mos_y;
	//KALMAN FILTER POSITION:
	Vector v = kalList->get(k)->update(newPosX,newPosY,newPosZ);
	objList->get(k)->x = v[0];
	objList->get(k)->y = v[1];
	objList->get(k)->z = v[2];
	//print out diff between measurement and Kalman filtered output:
	printf("ObjManServer: Object:%s (%f) - Measured: (%f,%f,%f)  Kalman: (%f,%f,%f)\n", 
	       objList->get(k)->label.c_str(),objList->get(k)->confidence,
	       newPosX,newPosY,newPosZ,
	       v[0],v[1],v[2]);
	//update image:
	ippiCopy_8u_C1R((Ipp8u*) newObjData->tex.getRawImage(),newObjData->tex.getRowSize(),
			objList->get(k)->tex.getRawImage(),objList->get(k)->tex.getRowSize(),imsize); 
	handled = true;
	break; //eject the for loop
      }
    }
    
    if (!handled){
      //Create temporary holder:
      ObjRecServerData* obj = new ObjRecServerData();
      //malloc im space:
      obj->resize(width, height);
      //copy in new image:
      ippiCopy_8u_C1R((Ipp8u*) newObjData->tex.getRawImage(),newObjData->tex.getRowSize(),
			obj->tex.getRawImage(),obj->tex.getRowSize(),imsize); 
      obj->label      = newObjData->label;
      obj->mos_x      = newObjData->mos_x;
      obj->mos_y      = newObjData->mos_y;
      obj->radius     = newObjData->radius;
      obj->confidence = newObjData->confidence;
      obj->x          = newPosX;
      obj->y          = newPosY;
      obj->z          = newPosZ;
      //push this temporary object onto list:
      numObjs = objList->add(obj); //increments object counter

      //spawn a Kalman filter with default settings:
      Kal*kalman = new Kal(newPosX,newPosY,newPosZ);
      numObjs = kalList->add(kalman);
    }
    
    
    printf("ObjManServer: %d Objects in Database\n",numObjs+1);
    //send entire list of objects to clients:
    outPort_objList.write(*objList);
    


  } //while

  //never here! 
}
