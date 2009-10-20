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
#include <display.h>
#include <objRecio.h>

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);


  QString arg1 = argv[1];
  QString arg2 = argv[2];

  bool mos = false;
  if (arg1=="mosaic" || arg1=="mos" || arg2=="mosaic" || arg2=="mos"){
    mos = true;
  }
  bool save = false;
  if (arg1=="save" || arg2=="save"){
    save = true;
  }

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
  
  //display:
  Mosaic *ml;
  Mosaic *mr;
  iCub::contrib::primateVision::Display * dl;
  iCub::contrib::primateVision::Display * dr;

  if (mos){
    ml = new Mosaic(mossize,srcsize,psb,D_8U_NN,"ObjRecClient L");
    mr = new Mosaic(mossize,srcsize,psb,D_8U_NN,"ObjRecClient R");
  }
  else{
    dl = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"ObjRecClient L");
    dr = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"ObjRecClient R");
  }


  int k=0;



  printf("ObjRecClient: begin..\n");
  //main event loop:
  while (1){
    
    k++;
    
    //get data from objMan server:
    objData = inPort_objData.read(); //blocking
    
    if (mos){
      //draw object in left and right mosaics: 
      ml->display(objData->tex.getRawImage(),
		  objData->mos_xl,objData->mos_yl);
      mr->display(objData->tex.getRawImage(),
		  objData->mos_xr,objData->mos_yr);
      if (save){
	ml->save(objData->tex.getRawImage(),"objrec"+QString::number(k)+".jpg");
      }
    }
    else{
      dl->display(objData->tex.getRawImage());
      dr->display(objData->tex.getRawImage());
      if (save){
	dl->save(objData->tex.getRawImage(),"objrec"+QString::number(k)+".jpg");
      }
    }

    printf("ObjRecClient: %s\n",objData->toString().c_str());
    
  } //while
  
  //never here! 
}
