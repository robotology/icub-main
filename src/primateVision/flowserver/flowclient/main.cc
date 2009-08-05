/**
 * @ingroup icub_primatevision_flowserver
 * \defgroup icub_primatevision_flowserver_flowclient FlowClient
 *
 *This endclient module FlowClient displays output from the FlowServer, as well as providing sample code showing how to make a useful FlowServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b flowclient l \b\n
 *\n where 'l' starts a FlowClient of the left FlowServer.
 * Ports accessed:\n
 * <ul>
 * <li> /flowserverL/output/flowx
 * <li> /flowserverL/output/flowy
 * <li> /flowserverL/output/flowxs
 * <li> /flowserverL/output/flowys
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /flowclientL/input/flowx
 * <li> /flowclientL/input/flowy
 * <li> /flowclientL/input/flowxs
 * <li> /flowclientL/input/flowys
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
 * This file can be edited at src/primateVision/flowserver/flowclient/main.cc 
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

//MY INCLUDES
#include <display.h>
#include <flowsio.h>
 

using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  if (argc!=2){
    printf("usage: %s [l,r]\n\n",argv[0]);
    exit(0);
  }
  QString input = argv[1];

 
  Port inPort_s;
  inPort_s.open("/flowclient_"+input+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/flowclient_"+input+"/input/serv_params", "/flowserver_"+input+"/output/serv_params");
  Network::connect("/flowserver_"+input+"/output/serv_params", "/flowclient_"+input+"/input/serv_params");
  BinPortable<FlowServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  FlowServerParams fsp = server_response.content();
  std::cout << "FlowServer Probe Response: " << fsp.toString() << std::endl;

  int width = fsp.width;
  int height = fsp.height;
  int psb = fsp.psb;

  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;



  BufferedPort<Bottle> inPort_y;      // Create a ports
  inPort_y.open("/flowclient_"+input+"/input/y");     // Give it a name on the network.
  Network::connect("/flowserver_"+input+"/output/y" , "/flowclient_"+input+"/input/y");
  Bottle *inBot_y;
  Ipp8u* y;

  BufferedPort<Bottle> inPort_x;      // Create a ports
  inPort_x.open("/flowclient_"+input+"/input/x");     // Give it a name on the network.
  Network::connect("/flowserver_"+input+"/output/x" , "/flowclient_"+input+"/input/x");
  Bottle *inBot_x;
  Ipp8u* x;


  BufferedPort<Bottle> inPort_y_filt;      // Create a ports
  inPort_y_filt.open("/flowclient_"+input+"/input/y_filt");     // Give it a name on the network.
  Network::connect("/flowserver_"+input+"/output/y_filt" , "/flowclient_"+input+"/input/y_filt");
  Bottle *inBot_y_filt;
  Ipp8u* y_filt;

  BufferedPort<Bottle> inPort_x_filt;      // Create a ports
  inPort_x_filt.open("/flowclient_"+input+"/input/x_filt");     // Give it a name on the network.
  Network::connect("/flowserver_"+input+"/output/x_filt" , "/flowclient_"+input+"/input/x_filt");
  Bottle *inBot_x_filt;
  Ipp8u* x_filt;

  BufferedPort<Bottle> inPort_sal;      // Create a ports
  inPort_sal.open("/flowclient_"+input+"/input/sal");     // Give it a name on the network.
  Network::connect("/flowserver_"+input+"/output/sal" , "/flowclient_"+input+"/input/sal");
  Bottle *inBot_sal;
  Ipp8u* sal;




  iCub::contrib::primateVision::Display *d_y = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"Y "+input);
  iCub::contrib::primateVision::Display *d_x = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"X "+input);
  iCub::contrib::primateVision::Display *d_y_filt = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"Y_filt "+input);
  iCub::contrib::primateVision::Display *d_x_filt = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"X_filt "+input);
  iCub::contrib::primateVision::Display *d_sal = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"Sal "+input);



  
  while (1){    
    

    inBot_sal = inPort_sal.read(false);
    inBot_x_filt = inPort_x_filt.read(false);
    inBot_y_filt = inPort_y_filt.read(false);
    inBot_x = inPort_x.read(false);
    inBot_y = inPort_y.read();


    if (inBot_y!=NULL){
      y = (Ipp8u*) inBot_y->get(0).asBlob();
      ippiThreshold_LTVal_8u_C1IR(y,psb,srcsize,(Ipp8u)1,(Ipp8u)4);
      ippiSubC_8u_C1IRSfs(1,y,psb,srcsize,0);
      ippiMulC_8u_C1IRSfs(40,y,psb,srcsize,0);
      d_y->display(y);
    }
    
    if (inBot_x!=NULL){
      x = (Ipp8u*) inBot_x->get(0).asBlob();
      ippiThreshold_LTVal_8u_C1IR(x,psb,srcsize,(Ipp8u)1,(Ipp8u)4);
      ippiSubC_8u_C1IRSfs(1,x,psb,srcsize,0);
      ippiMulC_8u_C1IRSfs(40,x,psb,srcsize,0);
      d_x->display(x);
    }
    
    if (inBot_y_filt!=NULL){
      y_filt = (Ipp8u*) inBot_y_filt->get(0).asBlob();
      ippiThreshold_LTVal_8u_C1IR(y_filt,psb,srcsize,(Ipp8u)1,(Ipp8u)4);
      ippiSubC_8u_C1IRSfs(1,y_filt,psb,srcsize,0);
      ippiMulC_8u_C1IRSfs(40,y_filt,psb,srcsize,0);
      d_y_filt->display(y_filt);
    }

    if (inBot_x_filt!=NULL){
      x_filt = (Ipp8u*) inBot_x_filt->get(0).asBlob();
      ippiThreshold_LTVal_8u_C1IR(x_filt,psb,srcsize,(Ipp8u)1,(Ipp8u)4);
      ippiSubC_8u_C1IRSfs(1,x_filt,psb,srcsize,0);
      ippiMulC_8u_C1IRSfs(40,x_filt,psb,srcsize,0);
      d_x_filt->display(x_filt);
    }

    if (inBot_sal!=NULL){
      sal = (Ipp8u*) inBot_sal->get(0).asBlob();
      d_sal->display(sal);
    }

    
    if (inBot_sal==NULL && 
	inBot_x_filt==NULL && inBot_y_filt==NULL &&
	inBot_x==NULL && inBot_y==NULL)
      {
	printf("No Input\n");
	usleep(5000); //dont blow out port
      }
    
  }


  //never here!
}
