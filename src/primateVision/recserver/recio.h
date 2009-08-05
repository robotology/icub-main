/**
 * \ingroup icub_primatevision_recserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef RECIO_H
#define RECIO_H

//YARP2 INCLUDES
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
#include <string>


#define NO_LOCK  0
#define ZDF_LOCK 1
#define ATN_LOCK 2


using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


namespace iCub {
  namespace contrib {
    namespace primateVision {


      //STATIC SERVER CONFIG OUTPUT:
#include <yarp/os/begin_pack_for_net.h>
      /** A container class for handling RecServer parameters
       *  sent over port in response to paramProbe.
       */
      class RecServerParams {
      public:
	/** Constructor. */
	RecServerParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 6;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%d %d %d %d %d %d",
		  width,height,psb,mos_width,mos_height,focus);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//this Server's Response Params:
	int width; /**< Server image width. */
	int height;/**< Server image height. */
	int mos_width; /**< Server mosaic width. */
	int mos_height; /**< Server mosaic height. */
	int psb; /**< Step width (in bytes) through image data. */
	int focus; /**< Stores camera focus (in pixels). */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>
      
      
      //ONLINE PROC OUTPUT:
#include <yarp/os/begin_pack_for_net.h>
      /** A container class for handling online calculated 
       *  RecServer rectification parameters to be
       *  sent over ports.
       */
      class RecResultParams {
      public:
	/** Constructor. */
	RecResultParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 11;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%d %d %d %d %f %f %f %f %f %f %f",
		  lx,rx,ly,ry,deg_lx,deg_rx,deg_ly,deg_ry,head_r,head_p,head_y);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//Rec Results:
	int lx; /**< Left image mosaic x-coord. */
	int rx; /**< Right image mosaic x-coord. */
	int ly; /**< Left image mosaic y-coord. */
	int ry; /**< Right image mosaic y-coord. */
	double deg_lx; /**< Current x-pointing angle reported by left cam encoder. */
	double deg_rx; /**< Current x-pointing angle reported by right cam encoder. */
	double deg_ly; /**< Current y-pointing angle reported by left cam encoder. */
	double deg_ry; /**< Current y-pointing angle reported by right cam encoder. */
	double head_r; /**< Current head roll angle reported by neck roll encoder. */
	double head_p; /**< Current head pitch angle reported by neck pitch encoder. */
	double head_y; /**< Current head yaw angle reported by neck yaw encoder. */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>
      
      
      
      
      
      //ONLINE POS REQUEST INPUT:
#include <yarp/os/begin_pack_for_net.h>
      /** A container class that clients fill and send to the
       *  RecServer to request motion of the head and/or eyes.
       */
      class RecMotionRequest {
      public:
	/** Constructor. */
	RecMotionRequest() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 10;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%d %d %d %f %f %f %d %d %d %d",
		  pix_xl,pix_xr,pix_y,deg_r,deg_p,deg_y,(bool)relative,suspend,lockto,unlock);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//Params:
	int pix_y; /**< Desired vertical shift in pixels. */
	int pix_xl;/**< Desired left camera horizontal shift in pixels. */
	int pix_xr;/**< Desired right camera horizontal shift in pixels. */
	double deg_r;/**< Desired neck roll angle in degrees. */
	double deg_p;/**< Desired neck pitch angle in degrees. */
	double deg_y;/**< Desired neck yaw angle in degrees. */
	bool relative;/**< Specifies if it should be a relative or absolute motion. */
	int suspend; /**< Number of cycles (processed frames) the RecServer should stop responding to other motion commands after initiating this request. */
	int lockto; /**< which RecServer client server (if any) should motion commands be exclusively accepted from?*/
	int unlock; /**< Enable command locking to the 'lockto' server to be disengaged. */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>
      
      
      

    }
  }
}


#endif
