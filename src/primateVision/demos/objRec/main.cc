/**
 * Client of zdfserver.  
 * 
 * Classify ZDF DOG output.
 * If confident, send commands to recognised objects in iCubSIM
 *
 */ 
//OPENCV INCLUDES
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>
//MY INCLUDES
#include <display.h>
//client of:
#include <zdfio.h>
//NNFW includes
#include "nnfw.h"
#include "biasedcluster.h"
#include "simplecluster.h"
#include "dotlinker.h"
#include "copylinker.h"
#include "liboutputfunctions.h"
#include "backpropagationalgo.h"
#include "random.h"
#include <algorithm>


#define learnRate 2.0
#define momentum 0.9

#define SAVE 0
#define THRESHOLD 0.8

using namespace nnfw;
using namespace std;
using namespace iCub::contrib::primateVision;

//NN Functions
void loadNet();
void extractPixels(IplImage* img);

//NN Parameters
BiasedCluster *in, *hid, *out;
DotLinker *in2hid, *hid2out, *in2out;
BackPropagationAlgo* learnNet; 
BaseNeuralNet* net;
double pixelValNorm[3000][50];
int numInputs, numOutputs, numHiddens, inc;
IplImage* segImg = 0;
IplImage* temp = 0;




int main( int argc, char **argv )
{
  QApplication *a = new QApplication(argc, argv);

  //probe ZDFServer:
  Port inPort_s;
  inPort_s.open("/objRec/input/serv_params"); 
  Network::connect("/objRec/input/serv_params", "/zdfserver/output/serv_params");
  Network::connect("/zdfserver/output/serv_params", "/objRec/input/serv_params");
  BinPortable<ZDFServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  ZDFServerParams zsp = server_response.content();
  std::cout << "ZDFServer Probe Response: " << zsp.toString() << std::endl;
  int m_size = zsp.m_size;
  int m_psb  = zsp.m_psb;
  IppiSize msize={m_size,m_size};

  //Get DOG output from ZDF Server:
  BufferedPort<Bottle> inPort_seg_dog; 
  inPort_seg_dog.open("/objRec/input/seg_dog"); 
  Network::connect("/zdfserver/output/seg_dog" , "/objRec/input/seg_dog");
  Bottle *inBot_seg_dog;
  Ipp8u  *zdf_im_seg_dog;

  //Display ZDF DOG output that will be classified:
  iCub::contrib::primateVision::Display *d_seg_dog  = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"ZDF_SEG_DOG");




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
 
  BufferedPort<Bottle> simPort;// create port that will connect with the simulator
  simPort.open("/objRec/output/sim"); 
  Network::connect("/objRec/output/sim", "/icubSim/world"); //connect the output target to the simulator FOR NOW THIS CAN BE DONE MANUALLY




  //used to convert dog img to opencv:
  segImg = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);
  //for resizing to 50x50:
  temp = cvCreateImage(cvSize(50,50), IPL_DEPTH_8U , 1 );

  int k=0;
  double posX=0.0,posY=0.0,posZ=0.0;
  bool update = false;

  const char *label;
  const char *oldLabel;
  oldLabel = "unknown";

  printf("begin..\n");
  //main event loop:
  while (1){
    

    //get data from zdf server:
    inBot_seg_dog = inPort_seg_dog.read(false);
    if (inBot_seg_dog!=NULL){
      zdf_im_seg_dog = (Ipp8u*) inBot_seg_dog->get(0).asBlob();
      posX = inBot_seg_dog->get(1).asDouble();
      posY = inBot_seg_dog->get(2).asDouble();
      posZ = inBot_seg_dog->get(3).asDouble();
      update = (bool) inBot_seg_dog->get(4).asInt();



      //only classify if we know it's a nice segmentation:

      if (update){
	//put image in openCV container
	ippiCopy_8u_C1R( zdf_im_seg_dog, m_psb, (Ipp8u*)segImg->imageData, segImg->widthStep, msize);   
	//resize to 50x50:
	cvResize(segImg, temp, CV_INTER_LINEAR);
	

	//**************************************
	//CLASSIFY:
	//set image as inputs to NN:
	extractPixels(temp);
	//run the NN:
	RealVec outputs(out->numNeurons());
	
	for (int input = 0; input< (int)in->numNeurons(); input++ )
	  in->setInput( input, pixelValNorm[input][0] ); 
	
	net->step();
	outputs = out->outputs();	
	cout << outputs[0] << " " <<  outputs[1] << " " << outputs[2] << endl;

	label = "unknown";
	if (outputs[0] > THRESHOLD){
	  label = "bottle";
	}
	else if (outputs[1] > THRESHOLD){
	  label = "fags";
	}
	else if (outputs[2] > THRESHOLD){
	  label = "coke";
	}


	printf("%s, location: (%f,%f,%f)\n",label,posX,posY,posZ);
		
	if (oldLabel!=label){
	  printf("UPDATING WORLD\n");
	  //OUTPUT DISPLAY TO SIM:
	  Bottle& bot = simPort.prepare();
	  bot.clear();
	  bot.addString ("world");
	  bot.addString ("mk");
	  bot.addString ("labl");
	  bot.addDouble(0.025);//size
	  bot.addDouble( nnfw::Random::flatReal ((Real) 0.0, (Real) 1.0));//x
	  bot.addDouble( nnfw::Random::flatReal ((Real) 0.0, (Real) 1.0));//y
	  bot.addDouble( nnfw::Random::flatReal ((Real) 0.0, (Real) 1.0));//z
	  bot.addString(label);
	  simPort.write();	
	  //**************************************
	  oldLabel=label;

#if SAVE
	  k++;
	  d_seg_dog->save(zdf_im_seg_dog,"im"+QString::number(k)+".jpg");
#endif

	}

 
      } //update
      
      
      //always:
      //DISPLAY:
      d_seg_dog->display(zdf_im_seg_dog);
    }
    if (inBot_seg_dog==NULL){
      // printf("No Input\n");
      usleep(5000);// don't blow out port
    }

  } //while



  //never here! 
}




void loadNet(){
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


void extractPixels(IplImage* img){

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
printf("finished extracting pixels\n");
}



