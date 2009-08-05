/**
 * @ingroup icub_primatevision_depthflowserver
 * \defgroup icub_primatevision_depthflowserver_depthflowclient DepthlowClient
 *
 *This endclient module DepthlowClient displays output from the DepthflowServer, as well as providing sample code showing how to make a useful DepthflowServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b depthflowclient \b\n
 *\n 
 * Ports accessed:\n
 * <ul>
 * <li> /depthflowserver/output/disp
 * <li> /depthflowserver/output/mdisp
 * <li> /depthflowserver/output/depth
 * <li> /depthflowserver/output/depthflow
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /depthflowclient/input/disp
 * <li> /depthflowclient/input/mdisp
 * <li> /depthflowclient/input/depth
 * <li> /depthflowclient/input/depthflow
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> NONE! It's an endClient!!
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/depthflowserver/depthflowclient/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <string>
#include <iostream>
#include <qapplication.h>

//MY INCLUDES
#include <display.h>
#include <depthflowsio.h>
 

using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication a(argc, argv);

 
  Port inPort_s;
  inPort_s.open("/depthflowclient/input/serv_params");     // Give it a name on the network.
  Network::connect("/depthflowclient/input/serv_params", "/depthflowserver/output/serv_params");
  Network::connect("/depthflowserver/output/serv_params", "/depthflowclient/input/serv_params");
  BinPortable<DepthflowServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  DepthflowServerParams dfsp = server_response.content();
  std::cout << "Depthflowserver Probe Response: " << dfsp.toString() << std::endl;


  int width = dfsp.width;
  int height = dfsp.height;
  int psb = dfsp.psb;
  int psb32f = dfsp.psb32f;




  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;


  BufferedPort<Bottle> inPort_disp;      // Create a ports
  inPort_disp.open("/depthflowclient/input/disp");     // Give it a name on the network.
  Network::connect("/depthflowserver/output/disp" , "/depthflowclient/input/disp");
  Bottle *inBot_disp;
  Ipp8u* disp;

  BufferedPort<Bottle> inPort_depth;      // Create a ports
  inPort_depth.open("/depthflowclient/input/depth");     // Give it a name on the network.
  Network::connect("/depthflowserver/output/depth" , "/depthflowclient/input/depth");
  Bottle *inBot_depth;
  Ipp32f* depth;

  BufferedPort<Bottle> inPort_depthflow;      // Create a ports
  inPort_depthflow.open("/depthflowclient/input/depthflow");     // Give it a name on the network.
  Network::connect("/depthflowserver/output/depthflow" , "/depthflowclient/input/depthflow");
  Bottle *inBot_depthflow;
  Ipp32f* depthflow;

  BufferedPort<Bottle> inPort_sal;      // Create a ports
  inPort_sal.open("/depthflowclient/input/sal");     // Give it a name on the network.
  Network::connect("/depthflowserver/output/sal" , "/depthflowclient/input/sal");
  Bottle *inBot_sal;
  Ipp8u* sal;




  DepthflowResultParams*df_res_params;



  iCub::contrib::primateVision::Display *d_d = new iCub::contrib::primateVision::Display(srcsize,psb32f,D_32F,"Depth");
  iCub::contrib::primateVision::Display *d_df = new iCub::contrib::primateVision::Display(srcsize,psb32f,D_32F,"DepthFlow");
  iCub::contrib::primateVision::Display *d_sal = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"Sal");
  iCub::contrib::primateVision::Display *d = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"Disp");
 



  
  while (1){


    //inBot_sal = inPort_sal.read(false);
    inBot_depthflow = inPort_depthflow.read(false);
    inBot_depth = inPort_depth.read(false);
    inBot_disp = inPort_disp.read();


    if (inBot_disp!=NULL){
      disp = (Ipp8u*) inBot_disp->get(0).asBlob();
      df_res_params = (DepthflowResultParams*) inBot_disp->get(1).asBlob();
      d->display(disp);
    }

    if (inBot_depth!=NULL){
      depth = (Ipp32f*) inBot_depth->get(0).asBlob();
      df_res_params = (DepthflowResultParams*) inBot_depth->get(1).asBlob();
      d_d->display(depth);
    }

    if (inBot_depthflow!=NULL){
      depthflow = (Ipp32f*) inBot_depthflow->get(0).asBlob();
      df_res_params = (DepthflowResultParams*) inBot_depthflow->get(1).asBlob();
      d_df->display(depthflow);
    }

    //if (inBot_sal!=NULL){
    //  sal = (Ipp8u*) inBot_sal->get(0).asBlob();
    //  d_sal->display(sal);
    //}


    //don't blow out ports:
    if (inBot_depth     == NULL && 
	inBot_depthflow == NULL &&
	inBot_disp      == NULL && 
	inBot_sal       == NULL ){
      printf("No Input\n");
      usleep(5000);
    }

  }

  //never here!
  
}
