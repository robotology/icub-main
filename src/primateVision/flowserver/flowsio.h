/**
 * \ingroup icub_primatevision_flowserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef FLOWSIO_H
#define FLOWSIO_H

//YARP2 INCLUDES
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>


using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


namespace iCub {
  namespace contrib {
    namespace primateVision {


      //STATIC SERVER CONFIG OUTPUT:
#include <yarp/os/begin_pack_for_net.h>
      class FlowServerParams {
      public:
	FlowServerParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 4;
	}
	string toString(){
	  char buffer[256];
	  sprintf(buffer, "%d %d %d %d",width,height,psb,subflow);
	  return buffer;
	}
	int listTag;
	int lenTag;

	//this Server's Response Params:
	int width;/**< Server image width. */
	int height;/**< Server image height. */
	int psb;/**< Step width (in bytes) through image data. */
	int subflow;/**< Downsampling before processing. */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>
      
    }
  }
}
#endif
