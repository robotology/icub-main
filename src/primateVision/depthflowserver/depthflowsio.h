/**
 * \ingroup icub_primatevision_depthflowserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef DEPTHFLOWSIO_H
#define DEPTHFLOWSIO_H

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
      
      /** A container class for handling DepthflowServer parameters
       *  sent over port in response to paramProbe.
       */
      class DepthflowServerParams {
      public:
	/** Constructor. */
	DepthflowServerParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 7;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[256];
	  sprintf(buffer, "%d %d %d %d %d %d %f",
		  width,height,psb,psb32f,range,offset,baseline);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//this Server's Response Params:
	int width; /**< Server image width. */
	int height;/**< Server image height. */
	int psb;   /**< Step width (in bytes) through 8u image data. */
	int psb32f;/**< Step width (in bytes) through 32f image data. */
	int range; /**< Server disparity search range. */
	int offset;/**< Server disparity search offset. */
	double baseline;/**< Server baseline parameter. */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>
      
      
      //ONLINE PROC OUTPUT:
#include <yarp/os/begin_pack_for_net.h>
      /** A container class for handling DepthflowServer online processing results
       */
      class DepthflowResultParams {
public:
	/** Constructor. */
	DepthflowResultParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 5;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[256];
	  sprintf(buffer, "%d %d %d %f %f",
		  px,py,hd, mind,maxd);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//Proc Results:
	int px; /**< Horizontal position. */
	int py; /**< Vertical position. */
	int hd; /**< Horizontal disparity. */
	double mind; /**< Minimum depth */
	double maxd; /**< Maximum depth */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>
      
    }
  }
}
#endif
