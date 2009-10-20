/**
 * @ingroup icub_primatevision_zdfserver
 * \defgroup icub_primatevision_zdfserver_zdfclientMos ZDFClientMos
 *
 *This endclient module ZDFClient displays segmentation output from the ZDFServer, and initates motion commands to maintain track in the segmented target via the RecServer motion port. It also provides sample code showing how to make a useful ZDFServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b zdfclient \b\n
 *\n 
 * Ports accessed:\n
 * <ul>
 * <li> /zdfserver/output/data
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /zdfclient/input/data
 * </ul>
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/zdfserver/zdfclient/zdfmos.cc 
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
#include <mosaic.h>


using namespace iCub::contrib::primateVision;



int main( int argc, char **argv )
{

  QApplication *a = new QApplication(argc, argv);

  Port inPort_p;
  inPort_p.open("/zdfclientMos/input/serv_params");   
  Network::connect("/zdfclientMos/input/serv_params" , "/zdfserver/output/serv_params");
  Network::connect("/zdfserver/output/serv_params", "/zdfclientMos/input/serv_params");
  BinPortable<ZDFServerParams> response; 
  Bottle empty;
  inPort_p.write(empty,response);
  ZDFServerParams zsp = response.content();
  std::cout << "ZDFServer Probe Response: " << zsp.toString() << std::endl; 
  int width = zsp.width;
  int height = zsp.height;
  int psb = zsp.psb;
  int mos_width = zsp.mos_width;
  int mos_height = zsp.mos_height;

  IppiSize msize={width,height};
  IppiSize mossize={mos_width,mos_height};


  BufferedPort<ZDFServerData > inPort_data;  
  inPort_data.open("/zdfclientMos/input/zdfdata");
  Network::connect("/zdfserver/output/data" , "/zdfclientMos/input/zdfdata");
  ZDFServerData *zdfData;

  //display:
  Mosaic *ml = new Mosaic(mossize,msize,psb,D_8U_NN,"ZDFClientMos L");
  Mosaic *mr = new Mosaic(mossize,msize,psb,D_8U_NN,"ZDFClientMos R");




  printf("ZDFClientMos: begin..\n");
  while (1){

    //get input:
    zdfData = inPort_data.read(); //blocking

    //draw object in left and right mosaics: 
    ml->display(zdfData->tex.getRawImage(),
		zdfData->mos_xl,zdfData->mos_yl);
    mr->display(zdfData->tex.getRawImage(),
		zdfData->mos_xr,zdfData->mos_yr);
    
  }
  
  //never here!
  return a->exec();
}

