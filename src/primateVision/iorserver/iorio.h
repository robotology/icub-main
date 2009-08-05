/**
 * \ingroup icub_primatevision_iorserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef IORIO_H 
#define IORIO_H

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
      
      /** A container class for handling YCSServer parameters
       *  sent over port in response to paramProbe.
       */
      class IORServerParams {
      public:
	/** Constructor. */
	IORServerParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 5;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%d %d %d %d %d",width,height,psb,mos_width,mos_height);
	  return buffer;
  }
	int listTag;
	int lenTag;
	
	//this Server's Response Params:
	int width;/**< Server image width. */
	int height;/**< Server image height. */
	int psb;/**< Step width (in bytes) through image data. */
	int mos_width; /**< Server mosaic width. */
	int mos_height; /**< Server mosaic height. */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>
      
    }
  }
}     
#endif
