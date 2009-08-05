/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef SPTZDFIO_H
#define SPTZDFIO_H

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
      /** A container class for handling ZDFServer parameters
       *  sent over port in response to paramProbe.
       */
      class SpTZDFServerParams {
      public:
	/** Constructor. */
	SpTZDFServerParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 4;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%d %d %d %d",m_size,t_size,m_psb,t_psb);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//this Server's Response Params:
	int m_psb;  /**< Step width (in bytes) through output image data. */
	int m_size; /**< Server image output width, height. */
	int t_size; /**< Server template image width, height. */
	int t_psb;  /**< Step width (in bytes) through template image data. */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>
      
    }
  }
}
#endif
