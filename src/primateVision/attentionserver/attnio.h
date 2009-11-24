/**
 * \ingroup icub_primatevision_attnserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef ATTNIO_H 
#define ATTNIO_H

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
      
      /** A container class for handling AttnServer parameters
       *  sent over port in response to paramProbe.
       */
      class AttnServerParams {
      public:
	/** Constructor. */
	AttnServerParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 3;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[256];
	  sprintf(buffer, "%d %d %d",width,height,psb);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//this Server's Response Params:
	int width;/**< Server image width. */
	int height;/**< Server image height. */
	int psb;/**< Step width (in bytes) through image data. */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>
      
    }
  }
}
#endif
      
