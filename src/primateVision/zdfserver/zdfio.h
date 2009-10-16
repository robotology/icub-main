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
	  lenTag = 1;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%d",m_size);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//this Server's Response Params:
	int m_size; /**< Server image output width, height. */
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
	ZDFServerData() {
	}
	void resize(int w,int h){
	  dog.resize(w,h);
	  tex.resize(w,h);
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%f %f %f %d %d",
		  x,y,z,mos_x,mos_y);
	  return buffer;
	}
	
	bool write(ConnectionWriter& con){
	  con.appendInt(BOTTLE_TAG_LIST);
	  con.appendInt(7);
	  dog.write(con);
	  tex.write(con);
	  con.appendDouble(x);
	  con.appendDouble(y);
	  con.appendDouble(x);
	  con.appendInt(mos_x);
	  con.appendInt(mos_y);
	}
	bool read(ConnectionReader& con){
	  if (!con.isValid()) {
	    return false;
	  }

	  con.convertTextMode();
	  int header=con.expectInt();
	  int len = con.expectInt();
	  if (header != BOTTLE_TAG_LIST || len!=7){
	    return false;
	  }
	  dog.read(con);
	  tex.read(con);
	  x = con.expectDouble();
	  y = con.expectDouble();
	  z = con.expectDouble();
	  mos_x = con.expectInt();
	  mos_y = con.expectInt();
	}

	//ZDFServer Results:
	double x;
	double y;
	double z;
	int mos_x;
	int mos_y;
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
	ZDFServerTuneData()  {

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
