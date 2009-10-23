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
	
	//NN Parameters
	BiasedCluster *in, *hid, *out;
	DotLinker *in2hid, *hid2out, *in2out;
	BackPropagationAlgo* learnNet; 
	BaseNeuralNet* net;
	double pixelValNorm[3000][50];
	int numInputs, numOutputs, numHiddens, inc;
	IplImage* temp;
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
 
