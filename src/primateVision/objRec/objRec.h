/** 
 *  \ingroup icub_primatevision_demos_objrec
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 */
#ifndef OBJREC_H
#define OBJREC_H

#include <qthread.h>
#include <ipp.h>
#include <string>
//MY INCLUDES
#include "objRecio.h"

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

//OPENCV INCLUDES
#include <cv.h>
#include <highgui.h>



using namespace nnfw;
using namespace std;
using namespace iCub::contrib::primateVision;

namespace iCub {
  namespace contrib {
    namespace primateVision {


     /** 
       * ObjRecServer
       */
      class ObjRecServer : public QThread
      {
	
      public:
	
	/** Constructor.
	 * @param c_ A string referencing the config file.
	 */
	ObjRecServer(string*c_);

	/** Destructor.
	 */
	~ObjRecServer();

	/** Main event loop.
	 */
	void run();

      private:
	string *cfg; /**< A string referencing the config file. */

	//NN Functions:
	void loadNet();
	void extractPixels(IplImage* img);
	
//Orient/scale inv. functions:	
void polar2cart_8u(Ipp8u*pol,int psb_pol,IppiSize pol_size, Ipp8u*cart,int psb_cart,IppiSize cart_size);
void cart2polar_8u(Ipp8u*cart,int psb_cart,IppiSize cart_size,  Ipp8u* pol,int psb_pol,IppiSize pol_size, int cx,int cy);
void getMaxRightXY_8u(Ipp8u* in, int psb_in,IppiSize sz,int*X,int*Y);
void rollUp_8u(int n,Ipp8u*in,int psb_in, IppiSize sz,Ipp8u*out,int psb_out);
void stretchRight_8u(int max_right, int max_width, Ipp8u* in, int psb_in,IppiSize sz_o, Ipp8u* out, int psb_out);


	//NN Parameters
	BiasedCluster *NN_in, *NN_hid, *NN_out;
	DotLinker *in2hid, *hid2out, *in2out;
	BackPropagationAlgo* learnNet; 
	BaseNeuralNet* net;
	double pixelValNorm[3000][50];
	int numInputs, numOutputs, numHiddens, inc;
	IplImage *temp,*inv;
	double momentum;
	double learnRate;
      };



      /**
       * A class that responds to client ObjRecServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class ObjRecServerReplyParamProbe : public PortReader {
	
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<ObjRecServerParams> outBin;
	  outBin.content() = reply;
	  bool ok = foo.read(connection);
	  if (!ok){printf("read fail!\n"); return false;}
	  ConnectionWriter *returnToSender = connection.getWriter();
	  if (returnToSender!=NULL) {
	    outBin.write(*returnToSender);
	  }
	  return true;
	}

      public:

	/** Reply sent to the probing client. Contains server
	 *  parameters that the client uses for initialisation. */
	ObjRecServerParams reply;
      };
          
    }
  }
}

#endif
 
