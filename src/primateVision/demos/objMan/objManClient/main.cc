/**
 * Client of objMan.  
 * Displays all segmented and classified objects in the ObjMan database
 * in 1st person egocentric perspective.
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

//for display:
#include <qapplication.h>
#include <mosaic.h>
#include <objManio.h>

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);


  //get objMan server params:
  Port inPort_s;
  inPort_s.open("/objManClient/input/serv_params");
  Network::connect("/objManClient/input/serv_params","/objManServer/output/serv_params");
  Network::connect("/objManServer/output/serv_params","/objManClient/input/serv_params");
  BinPortable<ObjManServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  ObjManServerParams rsp = server_response.content();
  std::cout << "ObjManServer Probe Response: " << rsp.toString() << std::endl;
  int width = rsp.width;
  int height = rsp.height;
  int mos_width = rsp.mos_width;
  int mos_height = rsp.mos_height;
  int psb = rsp.psb;
  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;
  IppiSize mossize;
  mossize.width = mos_width;
  mossize.height = mos_height;

  int numObjs;

  //Port to get online object list:
  BufferedPort<ObjManServerList> inPort_objList; 
  inPort_objList.open("/objManServer/output/objList"); 
  Network::connect("/objManServer/output/objList" , "/objManClient/input/objList");
  ObjManServerList *objList;
  

  //display mosaics:
  Mosaic *ml = new Mosaic(mossize,srcsize,psb,D_8U_NN,"ObjManClient L");
  Mosaic *mr = new Mosaic(mossize,srcsize,psb,D_8U_NN,"ObjManClient R");


  printf("ObjManClient: begin..\n");
  //main event loop:
  while (1){
    
    
    //get data from objMan server:
    objList = inPort_objList.read(); //blocking
    
    //draw all sent objects in the mosaic:      
    numObjs = objList->size();
    for (int i=0;i<numObjs;i++){
      //draw ith object in left and right mosaics: 
      ml->display(objList->get(i)->tex.getRawImage(),
		  objList->get(i)->mos_xl,objList->get(i)->mos_yl);
      mr->display(objList->get(i)->tex.getRawImage(),
		  objList->get(i)->mos_xr,objList->get(i)->mos_yr);
    }
    

  } //while

  //never here! 
}
