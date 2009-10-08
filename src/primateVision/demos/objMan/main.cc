/**
 * Client of objRec.  Manages the world. 
 * Outputs localised classifications (primarily to the sim).
 *
 */ 


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
//YARP2 INCLUDES
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>


#define MAX_OBJS 200
#define MIN_DIST 0.5 //meters!


using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


//using namespace iCub::contrib::primateVision;


struct obj{
	string label;
	double x;
	double y;
	double z;
	double radius;
};

obj objList[MAX_OBJS];


int main( int argc, char **argv )
{

  //Port to get object postion and label:
  BufferedPort<Bottle> inPort_obj; 
  inPort_obj.open("/objMan/input/obj"); 
  Network::connect("/objRec/output/obj" , "/objMan/input/obj");
  Bottle *inBot_obj;
 
  BufferedPort<Bottle> simPort;// create port that will connect with the simulator
  simPort.open("/objMan/output/world"); 
  Network::connect("/objMan/output/world", "/icubSim/world"); 


	double newRadius,newPosX,newPosY,newPosZ;
	string newLabel;
	int numObjs = 0;
	bool handled;




  printf("begin..\n");
  //main event loop:
  while (1){
    

    //get data from zdf server:
    inBot_obj = inPort_obj.read(false); //non-blocking
    if (inBot_obj!=NULL){
			
		  newRadius = inBot_obj->get(0).asDouble();
      newPosX   = inBot_obj->get(1).asDouble();
      newPosY   = inBot_obj->get(2).asDouble();
      newPosZ   = inBot_obj->get(3).asDouble();
      newLabel  = inBot_obj->get(4).asString();

      handled = false;


	//Check if it is similar to an existing label at similar pos, we move the object.
	for (int k=0;k<numObjs;k++){
	  //if label exists	
	  if (objList[k].label==newLabel &&
			//AND Malhonobis dist. is close:
			sqrt( (newPosX-objList[k].x)*(newPosX-objList[k].x)+
			      (newPosY-objList[k].y)*(newPosY-objList[k].y)+
         	  (newPosZ-objList[k].z)*(newPosZ-objList[k].z) ) < MIN_DIST
	       ){
		     		//not new object, so just update pos & radius of this object:	
						objList[k].x = newPosX;
						objList[k].y = newPosY;
						objList[k].z = newPosZ;
	          objList[k].radius = newRadius;
		        handled = true;
	    }
   }

  if (!handled){
   //we have a new object!
   numObjs++;
		if (numObjs>=MAX_OBJS){
      printf("DATABASE FULL!!\n");
    }
    else{
	   objList[numObjs].label = newLabel;
     objList[numObjs].x = newPosX;
     objList[numObjs].y = newPosY;
     objList[numObjs].z = newPosZ;
     objList[numObjs].radius = newRadius;
		
		 // draw immediately in sim !!!  
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
   }
  }


 //now loop over existing numObjs and update positions in sim!:
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
 }











    }
    if (inBot_obj==NULL){
      // printf("No Input\n");
      usleep(5000);// don't blow out port
    }

  } //while



  //never here! 
}
