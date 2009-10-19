/**
 * \ingroup icub_primatevision_demos_objrec
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef OBJRECIO_H
#define OBJRECIO_H

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
      class ObjRecServerParams {
      public:
	/** Constructor. */
	ObjRecServerParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 4;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%d %d %d %d",width,height,psb,nclasses);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//this Server's Response Params:
	int width; /**< Server image output width. */
	int height; /**< Server image output height. */
	int psb;  /**< Server step in bytes through output image. */
	int nclasses; /**< Server num known classes. */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>



      //ONLINE PROC OUTPUT:
      /** A container class for handling online calculated 
       *  ObjRecServer output sent over ports.
       */
      class ObjRecServerData : public Portable{
      public:
	/** Constructor. */
	ObjRecServerData() {
	}
	void resize(int w,int h){
	  tex.resize(w,h);
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%f %f %f %d %d &d &d %f %f %s",
		  x,y,z,mos_xl,mos_yl,mos_xr,mos_yr,radius,confidence,label.c_str());
	  return buffer;
	}
	
	bool write(ConnectionWriter& con){
	  con.appendInt(BOTTLE_TAG_LIST);
	  con.appendInt(11);
	  tex.write(con);
	  con.appendDouble(x);
	  con.appendDouble(y);
	  con.appendDouble(z);
	  con.appendInt(mos_xl);
	  con.appendInt(mos_yl);
	  con.appendInt(mos_xr);
	  con.appendInt(mos_yr);
	  con.appendDouble(radius);
	  con.appendDouble(confidence);
	  con.appendString(label.c_str());
	  return true;
	}
	bool read(ConnectionReader& con){
	  if (!con.isValid()) {
	    return false;
	  }

	  con.convertTextMode();
	  int header=con.expectInt();
	  int len = con.expectInt();
	  if (header != BOTTLE_TAG_LIST || len!=11){
	    return false;
	  }
	  tex.read(con);
	  x = con.expectDouble();
	  y = con.expectDouble();
	  z = con.expectDouble();
	  mos_xl = con.expectInt();
	  mos_yl = con.expectInt();
	  mos_xr = con.expectInt();
	  mos_yr = con.expectInt();
	  radius = con.expectDouble();
	  confidence = con.expectDouble();
	  label = con.expectText();
	  return true;
	}

	//ObjRecServer Results:
	double x;
	double y;
	double z;
	int mos_xl;
	int mos_yl;
	int mos_xr;
	int mos_yr;
	string label;
	double radius;
	double confidence;
	ImageOf<PixelMono> tex;
      };


     
    }
  }
}
#endif
