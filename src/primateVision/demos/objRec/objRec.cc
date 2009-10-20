#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>

//MY INCLUDES
//client of:
#include <zdfio.h>
#include "objRec.h"

#define NCLASSES 3
#define NN_WIDTH 50
#define NN_HEIGHT 50

using namespace nnfw;
using namespace std;
using namespace iCub::contrib::primateVision;



iCub::contrib::primateVision::ObjRecServer::ObjRecServer( string*cfg_ )
{
  
  cfg=cfg_;
  start();

}

iCub::contrib::primateVision::ObjRecServer::~ObjRecServer()
{
  
}

void iCub::contrib::primateVision::ObjRecServer::run()
{

  Property prop;
  prop.fromConfigFile(cfg->c_str());
  double threshold = prop.findGroup("OBJREC").find("THRESHOLD").asDouble();
  momentum  = prop.findGroup("OBJREC").find("MOMENTUM").asDouble();
  learnRate = prop.findGroup("OBJREC").find("LEARNRATE").asDouble();
  int max_unknowns = prop.findGroup("OBJREC").find("MAX_UNKNOWNS").asInt();

  //probe ZDFServer:
  Port inPort_s;
  inPort_s.open("/objRecServer/input/serv_params"); 
  Network::connect("/objRecServer/input/serv_params", "/zdfserver/output/serv_params");
  Network::connect("/zdfserver/output/serv_params", "/objRecServer/input/serv_params");
  BinPortable<ZDFServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  ZDFServerParams zsp = server_response.content();
  std::cout << "ZDFServer Probe Response: " << zsp.toString() << std::endl;
  int width = zsp.width;
  int height = zsp.height;
  int psb  = zsp.psb;
  IppiSize msize={width,height};
  
  

  //To get data from zdfserver:
  BufferedPort<ZDFServerData > inPort_zdfdata;  
  inPort_zdfdata.open("/objRecServer/input/zdfdata");
  Network::connect("/objRecServer/output/data" , "/objRecServer/input/zdfdata");
  ZDFServerData *zdfData;
 

  // set up NN
  int seed =time(0);
  nnfw::Random::setSeed(seed); // initialise the network with a random seed
  net = loadXML( "/home/andrew/src/iCub/src/primateVision/demos/objRec/data/learnedModel.xml" ); //load the saved model
  const ClusterVec& cl = net->clusters();  //configure the NN clusters
  for( nnfw::u_int i=0; i<cl.size(); i++ ){                
    cl[i]->inputs().zeroing();
    cl[i]->outputs().zeroing();                                
  }
  loadNet();//configures the neural network
  


  //make online output port:
  BufferedPort<ObjRecServerData> outPort_objData; 
  outPort_objData.open("/objRecServer/output/objData"); 
  
  
  //used to convert dog img to opencv:
  segImg = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);
  //resizing for NN query:
  temp = cvCreateImage(cvSize(NN_WIDTH,NN_HEIGHT), IPL_DEPTH_8U , 1 );
  

  const char *label;
  double bestval = 0.0;
  double radius = 0.03;
  int unknowns = 0;
  
  
  
  //make server probe replier:
  //Server params:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/objRecServer/output/serv_params");
  ObjRecServerParams rsp;
  rsp.width = width;
  rsp.height = height;
  rsp.mos_width = zsp.mos_width;
  rsp.mos_height = zsp.mos_height;
  rsp.nclasses = NCLASSES;
  rsp.psb = psb;
  //Replier:
  ObjRecServerReplyParamProbe server_replier;
  server_replier.reply=rsp;
  outPort_s.setReplier(server_replier); 





  
  printf("ObjRecServer: begin..\n");
  //main event loop:
  while (1){
    
    
    //get data from zdf server
    zdfData = inPort_zdfdata.read(); //blocking

    //resize dog image into openCV container "temp" for NN query:
    cvResize((IplImage*) zdfData->dog.getIplImage(), temp, CV_INTER_LINEAR);


    
    //**************************************
    //CLASSIFY:
    //set image as inputs to NN:
    extractPixels(temp);
    //run the NN:
    RealVec outputs(out->numNeurons());
    for (int input = 0; input< (int)in->numNeurons(); input++ )
      in->setInput( input, pixelValNorm[input][0] ); 
    net->step();
    //get outputs:
    outputs = out->outputs();	
    cout << outputs[0] << " " <<  outputs[1] << " " << outputs[2] << endl;
        
    //latch label from previous classification.
    //Then...
    bestval = 0.0;
    
    if (outputs[0] > bestval){
      label = "bottle";
      bestval = outputs[0];
    }
    if (outputs[1] > bestval){
      label = "fags";
      bestval = outputs[1];
    }
    if (outputs[2] > bestval ){
      label = "coke";
      bestval = outputs[2];
    }
    if (bestval<threshold){
      //only revert to 'unknown' if we've had lost of low-conf classifications.
      unknowns++;
      if (unknowns>=max_unknowns){
	label = "unknown";
	unknowns = 0;
      }
    }
    
    printf("ObjRecServer: %s:%f (%f,%f,%f)\n",label,bestval,zdfData->x,zdfData->y,zdfData->z);
    //**************************************
    
    
    

    //OUTPUT

    
    //SEND as portable <ObjRecServerData>:
    ObjRecServerData& objData = outPort_objData.prepare();
    objData.resize(width,height);
    ippiCopy_8u_C1R((Ipp8u*)zdfData->tex.getRawImage(),zdfData->tex.getRowSize(),
		    (Ipp8u*)objData.tex.getRawImage(),objData.tex.getRowSize(), msize);
    objData.label = label;        
    objData.radius = radius;      
    objData.confidence = bestval; 
    objData.x = zdfData->x;
    objData.y = zdfData->y;
    objData.z = zdfData->z;
    objData.mos_xl = zdfData->mos_xl;
    objData.mos_yl = zdfData->mos_yl;
    objData.mos_xr = zdfData->mos_xr;
    objData.mos_yr = zdfData->mos_yr;
    //send:
    outPort_objData.write();
    

    //}//if update
    
    
  } //while
  
  //never here! 
}







void iCub::contrib::primateVision::ObjRecServer::loadNet(){
  //load the net
  
  in = (BiasedCluster*)net->getByName("in");
  out = (BiasedCluster*)net->getByName("out");
  numInputs = in->numNeurons();
  numOutputs = out->numNeurons();
  hid = (BiasedCluster*)net->getByName("hid");
  in2hid = (DotLinker*)net->getByName("in2hid");
  hid2out = (DotLinker*)net->getByName("hid2out");
  numHiddens = hid->numNeurons();
  //reverse the order	
  UpdatableVec rev_ord(net->order().size());
  rev_ord.assign_reverse(net->order());
  learnNet = new BackPropagationAlgo (net, rev_ord, learnRate);
  learnNet->enableMomentum();
  learnNet->setMomentum(momentum);
}


void iCub::contrib::primateVision::ObjRecServer::extractPixels(IplImage* img){

  int height,width;
  char path[100] ;
  CvScalar s;
  for (int i=0; i<img->height; i++){
    for (int j=0; j<img->width; j++){	

      //get the (i,j) pixel intensity
      s=cvGet2D(img,i,j); 
      //convert to range 0.0->1.0:
      pixelValNorm[inc][0] = s.val[0]/255;		
     // printf("intensity = %lf  %d\n", pixelValNorm[inc][0], inc);
      inc ++;
    }
  }
  inc = 0;

}



