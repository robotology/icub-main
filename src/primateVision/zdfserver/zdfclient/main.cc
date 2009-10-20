/**
 * @ingroup icub_primatevision_zdfserver
 * \defgroup icub_primatevision_zdfserver_zdfclient ZDFClient
 *
 *This endclient module ZDFClient displays segmentation output from the ZDFServer, and initates motion commands to maintain track in the segmented target via the RecServer motion port. It also provides sample code showing how to make a useful ZDFServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b zdfclient \b\n
 *\n 
 * Ports accessed:\n
 * <ul>
 * <li> /zdfserver/output/tune
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /zdfclient/input/tune
 * </ul>
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/zdfserver/zdfclient/main.cc 
 * 
 */


/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>
#include <ippi.h>

//MY INCLUDES
#include <zdfio.h>
#include <multiFrameViewer.h>
#include <mosaic.h>


using namespace iCub::contrib::primateVision;



int main( int argc, char **argv )
{

  QApplication *a = new QApplication(argc, argv);

  QString arg1 = argv[1];
  bool save = false;
  if (arg1=="save"){
    save = true;
  }


  Port inPort_p;
  inPort_p.open("/zdfclient/input/serv_params");   
  Network::connect("/zdfclient/input/serv_params" , "/zdfserver/output/serv_params");
  Network::connect("/zdfserver/output/serv_params", "/zdfclient/input/serv_params");
  BinPortable<ZDFServerParams> response; 
  Bottle empty;
  inPort_p.write(empty,response);
  ZDFServerParams zsp = response.content();
  std::cout << "ZDFServer Probe Response: " << zsp.toString() << std::endl; 
  int width = zsp.width;
  int height = zsp.height;
  IppiSize msize={width,height};


  BufferedPort<ZDFServerTuneData > inPort_tune;  
  inPort_tune.open("/zdfclient/input/zdfdata");
  Network::connect("/zdfserver/output/tune" , "/zdfclient/input/zdfdata");
  ZDFServerTuneData *zdfData;



  //memory space for display images:

  QImage*qim1 = new QImage(width, height, 8, 256); 
  QImage*qim2 = new QImage(width, height, 8, 256); 
  QImage*qim3 = new QImage(width, height, 8, 256); 
  QImage*qim4 = new QImage(width, height, 8, 256); 
  //set to grey:
  for(unsigned int ui=0;ui<256;ui++) //set to B&W
    {
      qim1->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
      qim2->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
      qim3->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
      qim4->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
    }    

  QImage *qims[4]={qim1,qim2,qim3,qim4};
  int locations[4*2]={0,0,width,0,0,height,width,height};

  //setup viewer:
  multiFrameViewer *mfv = new multiFrameViewer(width*2,height*2);
  mfv->setCaption("ZDFClient");
  mfv->show();



  int k = 0;


  printf("ZDFClient: begin..\n");
  while (1){

    //get input:
    zdfData = inPort_tune.read(); //blocking

    //copy data into QImages for multiFrameViewer display:
    ippiCopy_8u_C1R((Ipp8u*)zdfData->tex.getRawImage(),zdfData->tex.getRowSize(),qim4->bits(),qim4->width(),msize);
    ippiCopy_8u_C1R((Ipp8u*)zdfData->prob.getRawImage(),zdfData->prob.getRowSize(),qim3->bits(),qim3->width(),msize);
    ippiCopy_8u_C1R((Ipp8u*)zdfData->left.getRawImage(),zdfData->left.getRowSize(),qim1->bits(),qim1->width(),msize);
    ippiCopy_8u_C1R((Ipp8u*)zdfData->right.getRawImage(),zdfData->right.getRowSize(),qim2->bits(),qim2->width(),msize);
    
    if (save){
      k++;
      qim4->save("tex"+QString::number(k)+".jpg","JPEG",100);
    }

    //display:
    mfv->showViews(4,qims,locations);
    
  }
  
  //never here!
  return a->exec();
}

