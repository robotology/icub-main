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
#include <objManio.h>
#include <ippi.h>
#include <multiFrameViewer.h>



#define MAX_OBJS 100


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


  //Port to get online object list:
  BufferedPort<ObjManServerList> inPort_objList; 
  inPort_objList.open("/objManClient/input/objList"); 
  Network::connect("/objManServer/output/objList" , "/objManClient/input/objList");
  ObjManServerList *objList;
  

  //setup viewer:
  multiFrameViewer *mfv = new multiFrameViewer(mos_width,mos_height);
  mfv->setCaption("ObjManClient");
  mfv->show();



  int locations[MAX_OBJS*2];
  QImage *qims[MAX_OBJS];
  for (int i=0;i<MAX_OBJS;i++) {
    qims[i] = new QImage(width, height, 8, 256);
    for(unsigned int ui=0;ui<256;ui++){
      qims[i]->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
    }    
  }




  printf("ObjManClient: begin..\n");
  //main event loop:
  while (1){
    
    //get data from objManServer:
    objList = inPort_objList.read(); //blocking
    

    if (objList->size()<=MAX_OBJS){

      //copy data into QImages for multiFrameViewer display:
      for (int i=objList->size()-1;i>=0;i--){ //inverted order looks nicer for display
	ippiCopy_8u_C1R((Ipp8u*)objList->get(i)->tex.getRawImage(),
			objList->get(i)->tex.getRowSize(),
			qims[i]->bits(),qims[i]->width(),srcsize);
	locations[i*2]   = objList->get(i)->mos_xl + mos_width/2 - width/2;
	locations[i*2+1] = objList->get(i)->mos_yl + mos_height/2 - height/2;


	//THIS IS HOW YOUR CLIENTS CAN ACCESS THE OBJECT MANAGER DATA:
	//objList->size()         //num of objs in list (int)
	//objList->get(i)->label  //ith object label (string)
	//objList->get(i)->tex    //ith object image (yarp img)
	//objList->get(i)->x      //3D coords of ith object for Sim display (floats)
	//objList->get(i)->y
	//objList->get(i)->z
	//objList->get(i)->mos_xl //2D mosaic position of ith object for mosaic clients (ints)
	//objList->get(i)->mos_xr //

      }
      
      //display:
      mfv->showViews(objList->size(),(QImage**) qims,(int*) locations);

    }
    else{
      printf("too many objects!!\n");
    }


  } //while

  //never here! 
}
