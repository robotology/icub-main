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

#define MAX_OBJS 20
#define MIN_DIST 0.5 //meters!

using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


struct obj{
  string label;
  double x;
  double y;
  double z;
  double radius;
  Ipp8u *image;
};

obj objList[MAX_OBJS];


int main( int argc, char **argv )
{
  
  //Port to get object postion and label:
  BufferedPort<Bottle> inPort_obj; 
  inPort_obj.open("/objMan/input/obj"); 
  Network::connect("/objRec/output/obj" , "/objMan/input/obj");
  Bottle *inBot_obj;
  
  // create port that will connect with the simulator
  BufferedPort<Bottle> simPort;
  simPort.open("/objMan/output/world"); 
  Network::connect("/objMan/output/world", "/icubSim/world"); 
  
  // Make a port for YARP Image
  BufferedPort<ImageOf<PixelMono> > outPort_yarpimg;
  outPort_yarpimg.open("/objMan/output/yarpimg");
  Network::connect("/objMan/output/yarpimg", "/icubSim/texture"); 

  double newRadius,newPosX,newPosY,newPosZ;
  string newLabel;
  Ipp8u*newImage;
  IppiSize imsize={100,100};
  int psb;
  int numObjs = 0;
  bool handled;
  
  
  
  printf("begin..\n");
  //main event loop:
  while (1){
    
    
    //get data from objRec server:
    inBot_obj = inPort_obj.read(); //blocking
    if (inBot_obj!=NULL){
      
      newRadius = inBot_obj->get(0).asDouble();
      newPosX   = -inBot_obj->get(1).asDouble();
      newPosY   = -inBot_obj->get(2).asDouble() + 0.928; //height of head offset
      newPosZ   = inBot_obj->get(3).asDouble();
      newLabel  = inBot_obj->get(4).asString();
      newImage  = (Ipp8u*) inBot_obj->get(5).asBlob();

      //Check if it is similar to an existing label at similar pos, we move the object.
      handled = false;
      for (int k=0;k<numObjs;k++){
	//if label exists	
	if (objList[k].label==newLabel &&
	    //AND Malhonobis dist. is low:
	    sqrt( (newPosX-objList[k].x)*(newPosX-objList[k].x)+
		  (newPosY-objList[k].y)*(newPosY-objList[k].y)+
		  (newPosZ-objList[k].z)*(newPosZ-objList[k].z) ) < MIN_DIST
	    //AND OBJECTS LOOK SIMILAR:
	    //ADD ME****
	    ){
	  //not new object, so just update pos,radius,image of this object:	
	  objList[k].x = newPosX;
	  objList[k].y = newPosY;
	  objList[k].z = newPosZ;
	  //KALMAN FILTER THIS POSITION!
	  //ADD ME*****
	  objList[k].radius = newRadius;
	  ippiCopy_8u_C1R(newImage,psb,objList[k].image,psb,imsize); //overwrite image
	  handled = true;
	  break; //eject the for loop
	}
      }
      
      if (!handled){
	//we have a new object!
	if (numObjs<MAX_OBJS){
	  objList[numObjs].label = newLabel;
	  objList[numObjs].x = newPosX;
	  objList[numObjs].y = newPosY;
	  objList[numObjs].z = newPosZ;
	  objList[numObjs].radius = newRadius;
	  objList[numObjs].image = ippiMalloc_8u_C1(100,100,&psb); //malloc
	  ippiCopy_8u_C1R(newImage,psb,objList[numObjs].image,psb,imsize); //copy in
	  
	  //draw in sim immediately.  
	  Bottle& bot = simPort.prepare();
	  bot.clear();
	  bot.addString ("world");
	  bot.addString ("mk");
	  bot.addString ("labl");
	  bot.addDouble(objList[numObjs].radius);
	  bot.addDouble(objList[numObjs].x);
	  bot.addDouble(objList[numObjs].y);
	  bot.addDouble(objList[numObjs].z);
	  bot.addString(objList[numObjs].label.c_str());
	  simPort.write();
	  //and send texture:
	  ImageOf<PixelMono>& tmp_yarpimg = outPort_yarpimg.prepare();
	  tmp_yarpimg.resize(100,100);
	  for (int y=0;y<imsize.height;y++){
	    memcpy(tmp_yarpimg.getRowArray()[y],&objList[numObjs].image[y*psb],imsize.width);
	  }
	  outPort_yarpimg.write();

	  //increment object counter
	  numObjs++;	
	}
	else{
	  printf("DATABASE FULL!!\n");
	}
      }
      
      //now loop over all numObjs in database and update positions in sim!:
      for (int k=0;k<numObjs;k++){
	Bottle& bot = simPort.prepare();
	bot.clear();
	bot.addString ("world");
	bot.addString ("set");
	bot.addString ("labl");
	bot.addInt(k);
	bot.addDouble(objList[k].x);
	bot.addDouble(objList[k].y);
	bot.addDouble(objList[k].z);
	bot.addDouble(objList[k].radius);
	simPort.write();
	//and send texture:
	ImageOf<PixelMono>& tmp_yarpimg = outPort_yarpimg.prepare();
	tmp_yarpimg.resize(100,100);
	for (int y=0;y<imsize.height;y++){
	  memcpy(tmp_yarpimg.getRowArray()[y],&objList[k].image[y*psb],imsize.width);
	}
	outPort_yarpimg.write();
      }
      

      printf("ObjMan: %d Objects in Database\n",numObjs);
      
      
    }//if received
    
  } //while

  //never here! 
}
