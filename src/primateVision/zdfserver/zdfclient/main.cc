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

using namespace iCub::contrib::primateVision;



int main( int argc, char **argv )
{

  QApplication *a = new QApplication(argc, argv);

  Port inPort_p;
  inPort_p.open("/zdfclient/input/serv_params");   
  Network::connect("/zdfclient/input/serv_params" , "/zdfserver/output/serv_params");
  Network::connect("/zdfserver/output/serv_params", "/zdfclient/input/serv_params");
  BinPortable<ZDFServerParams> response; 
  Bottle empty;
  inPort_p.write(empty,response);
  ZDFServerParams zsp = response.content();
  std::cout << "ZDFServer Probe Response: " << zsp.toString() << std::endl; 
  int m_size = zsp.m_size;

  IppiSize msize={m_size,m_size};

  BufferedPort<ZDFServerTuneData > inPort_tune;  
  inPort_tune.open("/zdfclient/input/zdfdata");
  Network::connect("/zdfserver/output/tune" , "/zdfclient/input/zdfdata");
  ZDFServerTuneData *zdfData;



  //memory space for display images:
  QImage*qim1 = new QImage(m_size, m_size, 8, 256); 
  QImage*qim2 = new QImage(m_size, m_size, 8, 256); 
  QImage*qim3 = new QImage(m_size, m_size, 8, 256); 
  QImage*qim4 = new QImage(m_size, m_size, 8, 256); 
  //set to grey:
  for(unsigned int ui=0;ui<256;ui++) //set to B&W
    {
      qim1->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
      qim2->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
      qim3->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
      qim4->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
    }    

  QImage *qims[4]={qim1,qim2,qim3,qim4};
  int locations[4*2]={0,0,m_size,0,0,m_size,m_size,m_size};

  //setup viewer:
  multiFrameViewer *mfv = new multiFrameViewer(m_size*2,m_size*2);
  mfv->setCaption("ZDFClient");
  mfv->show();



  printf("begin..\n");
  while (1){

    //get input:
    zdfData = inPort_tune.read(); //blocking

    //copy data into QImages for multiFrameViewer display:
    ippiCopy_8u_C1R((Ipp8u*)zdfData->tex.getRawImage(),zdfData->tex.getRowSize(),qim4->bits(),m_size,msize);
    ippiCopy_8u_C1R((Ipp8u*)zdfData->prob.getRawImage(),zdfData->prob.getRowSize(),qim3->bits(),m_size,msize);
    ippiCopy_8u_C1R((Ipp8u*)zdfData->left.getRawImage(),zdfData->left.getRowSize(),qim1->bits(),m_size,msize);
    ippiCopy_8u_C1R((Ipp8u*)zdfData->right.getRawImage(),zdfData->right.getRowSize(),qim2->bits(),m_size,msize);
    
    //display:
    mfv->showViews(4,qims,locations);
    
  }
  
  //never here!
  return a->exec();
}

