/**
 * Client of zdfserver.  
 * 
 * Query if ZDF DOG output is classified.
 * Perhaps only query when cog close to origin.
 * Hence, perhaps re-instantiate drift towards CoG 
 * in ZDFServer?
 * If so, send commands to draw it in iCubSIM
 *
 */ 

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
//YARP INCLUDES
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
//OPENCV INCLUDES
#include <cv.h>
#include <highgui.h>

using namespace nnfw;
using namespace iCub::contrib::primateVision;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;

void loadNet();
void extractPixels(IplImage* img);

//NN parameters
BiasedCluster *in, *hid, *out;
DotLinker *in2hid, *hid2out, *in2out;
BackPropagationAlgo* learnNet; 
BaseNeuralNet* net;
double pixelValNorm[10001][50];


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
  int t_size = zsp.t_size;
  int t_psb  = zsp.t_psb;

  IppiSize tsize={t_size,t_size};
  IppiSize msize={m_size,m_size};
  IppiSize osize={320,240};

  BufferedPort<Bottle> inPort_seg_dog;      // Create a port
  inPort_seg_dog.open("/objRec/input/seg_dog");     // Give it a name on the network.
  Network::connect("/zdfserver/output/seg_dog" , "/objRec/input/seg_dog");
  Bottle *inBot_seg_dog;
  Ipp8u  *zdf_im_seg_dog;

  iCub::contrib::primateVision::Display *d_seg_dog  = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"ZDF_SEG_DOG");

  // setting up NN
  int seed =time(0);
  nnfw::Random::setSeed( seed); // initialise it with a random seed
  BufferedPort <Bottle> _targetPort;// create port that will connect with the simulator
  //Network::connect("/vtikha/target", "/icubSim/world"); //connect the output target to the simulator FOR NOW THIS CAN BE DONE MANUALLY

  // setting up images for opencv
  segImg = cvCreateImage(cvSize(320, 240, IPL_DEPTH_8U, 1); // manually
    

  net = loadXML( "data/learnedModel.xml" ); //load the saved model

  const ClusterVec& cl = net->clusters();  //configure the NN clusters
  for( nnfw::u_int i=0; i<cl.size(); i++ ){                
    cl[i]->inputs().zeroing();
    cl[i]->outputs().zeroing();                                
  }
  
  loadNet();//configures the net
  

  printf("begin..\n");

  //main event loop:
  while (1){
    
    inBot_seg_dog = inPort_seg_dog.read(false);
    

    if (inBot_seg_dog!=NULL){
      zdf_im_seg_dog = (Ipp8u*) inBot_seg_dog->get(0).asBlob();

      //DISPLAY:
      d_seg_dog->display(zdf_im_seg_dog);

      //CLASSIFY:
      printf("CHECKING CLASSIFICATION...\n");
      
	  ippiCopy_8u_C1(zdf_im_seg_dog, psb, (Ipp8u*)segImg->imageData, m_psb, msize);
	  //get the image
	  temp = cvCreateImage(cvSize(30,30), 8, 3 );
	  cvResize(segImg, temp,CV_INTER_LINEAR);
	  extractPixels(temp);

	  //run the NN
	  RealVec outputs(out->numNeurons());
	  for (int input = 0; input< (int)in->numNeurons(); input++ )
	  	in->setInput( input, pixelValNorm[input][0] ); 

	  net->step();
	  outputs = out->outputs();      

	  cout << outputs[0] << " " << endl;
	  
	  //OUTPUT DISPLAY TO SIM:
	  //if (outputs[0] < THRESHOLD )
		//label = 1....
	  
	  /*int label = 0;
	  Bottle& bot = _targetPort.prepare();
	  bot.clear();
	  bot.addString ("world");
	  bot.addString ("mk");
	  bot.addString ("labl");
	  bot.addDouble( nnfw::Random::flatReal ((Real) 0.01, (Real) 0.8));
	  bot.addDouble( nnfw::Random::flatReal ((Real) -2.0, (Real) 2.0));
	  bot.addDouble( nnfw::Random::flatReal ((Real) 0.0, (Real) 2.0));
	  bot.addDouble( nnfw::Random::flatReal ((Real) 0.2, (Real) 2.0));
	  */
	
	  //bot.addInt(label);
	  //_targetPort.write();	
      


      printf("DONE.\n");
    }

   
    if (inBot_seg_dog==NULL){
      printf("No Input\n");
      usleep(5000);// don't blow out port
    }
   
  }
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

  	IplImage* gray = 0; 
	int height,width;
	char path[100] ;
	cout << "\nLOADING image\n"<< endl;

	gray = cvCreateImage(cvGetSize(img), 8, 1 );
	cvCvtColor(img,gray,CV_BGR2GRAY); 

	CvScalar s;
	for (int i=0; i<img->height; i++){
		for (int j=0; j<img->width; j++){	
			s=cvGet2D(gray,i,j); // get the (i,j) pixel value
			pixelValNorm[inc][0] = s.val[0]/255;		
			//printf("intensity = %lf  %d\n", pixelValNorm[inc][0], inc);
			inc ++;
		}
	}
	inc = 0;
}



