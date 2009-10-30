/**
 * \ingroup icub_primatevision_zdfserver
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef ZDFIO_H
#define ZDFIO_H

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
      class ZDFServerParams {
      public:
	/** Constructor. */
	ZDFServerParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 5;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%d %d %d %d %d",width,height,mos_width,mos_height,psb);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//this Server's Response Params:
	int width; /**< Server image output width. */
	int height; /**< Server image output height. */
	int mos_width; /**< Mosaic width, forwarded from RecServer. */
	int mos_height; /**< Mosaic height, forwarded from RecServer. */
	int psb; /**< Server step in bytes through output image. */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>



      //ONLINE PROC OUTPUT:
      /** A container class for handling online calculated 
       *  zdf output sent over ports.
       */
      class ZDFServerData : public Portable{
      public:
	/** Constructor. */
	ZDFServerData(){

	}
	void resize(int w,int h){
	  dog.resize(w,h);
	  tex.resize(w,h);
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%f %f %f %d %d %d %d %d %d %d %d",
		  x,y,z,mos_xl,mos_yl,mos_xr,mos_yr,cog_x,cog_y,area,(int)update);
	  return buffer;
	}
	
	bool write(ConnectionWriter& con){
	  con.appendInt(BOTTLE_TAG_LIST);
	  con.appendInt(13);
	  dog.write(con);
	  tex.write(con);
	  con.appendDouble(x);
	  con.appendDouble(y);
	  con.appendDouble(z);
	  con.appendInt(mos_xl);
	  con.appendInt(mos_yl);
	  con.appendInt(mos_xr);
	  con.appendInt(mos_yr);
	  con.appendInt(cog_x);
	  con.appendInt(cog_y);
	  con.appendInt(area);
	  con.appendInt((int) update);
	  return true;
	}
	bool read(ConnectionReader& con){
	  if (!con.isValid()) {
	    return false;
	  }

	  con.convertTextMode();
	  int header = con.expectInt();
	  int len = con.expectInt();
	  if (header != BOTTLE_TAG_LIST || len!=13){
	    return false;
	  }
	  dog.read(con);
	  tex.read(con);
	  x = con.expectDouble();
	  y = con.expectDouble();
	  z = con.expectDouble();
	  mos_xl = con.expectInt();
	  mos_yl = con.expectInt();
	  mos_xr = con.expectInt();
	  mos_yr = con.expectInt();
	  cog_x = con.expectInt();
	  cog_y = con.expectInt();
	  area = con.expectInt();
	  update = (bool) con.expectInt();
	  return true;
	}

	//ZDFServer Results:
	double x;
	double y;
	double z;
	int mos_xl;
	int mos_yl;
	int mos_xr;
	int mos_yr;
	int cog_x;
	int cog_y;
	int area;
	bool update;
	ImageOf<PixelMono> dog;
	ImageOf<PixelMono> tex;
      };


      //ONLINE TUNING OUTPUT:
      /** A container class for handling images 
       *  sent to zdfclient for tuning.
       */
      class ZDFServerTuneData : public Portable {
      public:
	/** Constructor. */
	ZDFServerTuneData(){

	}
	void resize(int w,int h){
	  prob.resize(w,h);
	  tex.resize(w,h);
	  left.resize(w,h);
	  right.resize(w,h);
	}
	bool write(ConnectionWriter& con){
	  con.appendInt(BOTTLE_TAG_LIST);
	  con.appendInt(4);
	  prob.write(con);
	  tex.write(con);
	  left.write(con);
	  right.write(con);
	  return true;
	}
	bool read(ConnectionReader& con){
	  if (!con.isValid()) {
	    return false;
	  }

	  con.convertTextMode();
	  int header = con.expectInt();
	  int len = con.expectInt();
	  if (header != BOTTLE_TAG_LIST || len!=4){
	    return false;
	  }
	  prob.read(con);
	  tex.read(con);
	  left.read(con);
	  right.read(con);
	  return true;
	}
	
	//ZDFServer tuning stuff:
	ImageOf<PixelMono> prob;
	ImageOf<PixelMono> tex;
	ImageOf<PixelMono> left;
	ImageOf<PixelMono> right;
      };

    }
  }
}
#endif
