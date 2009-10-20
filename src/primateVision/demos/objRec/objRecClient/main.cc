/**
 * Client of objRec.  
 * Displays objRec segmentation
 * in 1st person egocentric perspective mosaic.
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
#include <objRecio.h>

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);


  //get objRec server params:
  Port inPort_s;
  inPort_s.open("/objRecClient/input/serv_params");
  Network::connect("/objRecClient/input/serv_params","/objRecServer/output/serv_params");
  Network::connect("/objRecServer/output/serv_params","/objRecClient/input/serv_params");
  BinPortable<ObjRecServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  ObjRecServerParams rsp = server_response.content();
  std::cout << "ObjRecServer Probe Response: " << rsp.toString() << std::endl;
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

  //Port to get online object data:
  BufferedPort<ObjRecServerData> inPort_objData; 
  inPort_objData.open("/objRecServer/output/objData"); 
  Network::connect("/objRecServer/output/objData" , "/objRecClient/input/objData");
  ObjRecServerData *objData;
  

  //display mosaics:
  Mosaic *ml = new Mosaic(mossize,srcsize,psb,D_8U_NN,"ObjRecClient L");
  Mosaic *mr = new Mosaic(mossize,srcsize,psb,D_8U_NN,"ObjRecClient R");


  printf("ObjRecClient: begin..\n");
  //main event loop:
  while (1){
    
    
    //get data from objMan server:
    objData = inPort_objData.read(); //blocking
    
    //draw object in left and right mosaics: 
    ml->display(objData->tex.getRawImage(),
		objData->mos_xl,objData->mos_yl);
    mr->display(objData->tex.getRawImage(),
		objData->mos_xr,objData->mos_yr);
    printf("ObjRecClient: %s\n",objData->toString().c_str());
    
  } //while
  
  //never here! 
}
