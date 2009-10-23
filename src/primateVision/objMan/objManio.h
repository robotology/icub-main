/**
 * \ingroup icub_primatevision_demos_objman
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef OBJMANIO_H
#define OBJMANIO_H

//YARP2 INCLUDES
#include <vector>
#include <objRecio.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;


namespace iCub {
  namespace contrib {
    namespace primateVision {


      //STATIC SERVER CONFIG OUTPUT:
#include <yarp/os/begin_pack_for_net.h>
      
      /** A container class for handling ObjManServer parameters
       *  sent over port in response to paramProbe.
       */
      class ObjManServerParams {
      public:
	/** Constructor. */
	ObjManServerParams() {
	  listTag = BOTTLE_TAG_LIST + BOTTLE_TAG_INT;
	  lenTag = 6;
	}
	/** Converstion to string of parameters for printing. */
	string toString(){
	  char buffer[50];
	  sprintf(buffer, "%d %d %d %d %d %d",width,height,mos_width,mos_height,psb,nclasses);
	  return buffer;
	}
	int listTag;
	int lenTag;
	
	//this Server's Response Params:
	int width; /**< Server image width. */
	int height;/**< Server image height. */
	int mos_width; /**< Mosaic width forwarded from RecServer. */
	int mos_height;/**< Mosaic height forwarded from RecServer. */
	int psb;   /**< Step width (in bytes) through image data. */
	int nclasses; /**< Number of classes ObjMan server knows about. */
	//
	
      } PACKED_FOR_NET;
#include <yarp/os/end_pack_for_net.h>
      




      //OBJMANLIST IS A LIST OF "N" OBJRECDATA.
      //IT IS SENT IN ENTIRITY EACH TIME (for now):
      class ObjManServerList : public Portable {
      private:
	std::vector<ObjRecServerData*> list;

      public:	
	ObjManServerList() {

	}
	~ObjManServerList() {
	  clear();
	}
	
	void add(ObjRecServerData* obj_) {
	  //NOTE: ObjManServerList takes responsibility over pointer
	  list.push_back(obj_);
	}

	ObjRecServerData* get(int i) {
	  return list[i];
	}

	void clear() {
	  for(int i = 0; i < list.size(); i++) {
	    delete list[i];
	  }
	  list.clear();
	}
	
	int size(){
	  return list.size();
	}

	bool write(ConnectionWriter& con) {
	  con.appendInt(BOTTLE_TAG_LIST);
	  con.appendInt(list.size());
	  for(int i = 0; i < list.size(); i++) {
	    list[i]->write(con);
	  }
	  return true;
	}
      
	bool read(ConnectionReader& con) {
	  if (!con.isValid()) {
	    return false;
	  }
	  int header = con.expectInt();
	  int len = con.expectInt();
	  if(header != BOTTLE_TAG_LIST) {
	    return false;
	  }
	  // shouldn't really be necessary
	  clear();
	  for(int i = 0; i < len; i++) {
	    ObjRecServerData* obj = new ObjRecServerData();
	    obj->read(con);
	    add(obj);
	  }
	  return true;
	}
      
      };


    }
  }
}
#endif

