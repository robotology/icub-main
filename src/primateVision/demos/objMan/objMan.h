/** 
 *  \ingroup icub_primatevision_demos_objman
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 */
#ifndef OBJMAN_H
#define OBJMAN_H

#include <qthread.h>
#include <ipp.h>
#include <string>
#include <vector>
//MY INCLUDES
#include "objManio.h"
#include "kal.h"


using namespace std;
using namespace iCub::contrib::primateVision;

namespace iCub {
  namespace contrib {
    namespace primateVision {


     /** 
       * ObjManServer
       */
      class ObjManServer : public QThread
      {
	
      public:
	
	/** Constructor.
	 * @param c_ A string referencing the config file.
	 */
	ObjManServer(string*c_);

	/** Destructor.
	 */
	~ObjManServer();

	/** Main event loop.
	 */
	void run();

      private:
	string *cfg; /**< A string referencing the config file. */

      };



      /**
       * A class that responds to client ObjManServer probes, 
       * such that the clients can obtain server parameters 
       * essential to their operation.
       */
      class ObjManServerReplyParamProbe : public PortReader {
	
	/** Port reader for server parameter probes.
	 *  @param connection Pointer to Yarp2 ConnectionReader.
	 *  @return Returns true upon completion.
	 */
	virtual bool read(ConnectionReader& connection) {
	  Bottle foo;
	  BinPortable<ObjManServerParams> outBin;
	  outBin.content() = reply;
	  bool ok = foo.read(connection);
	  if (!ok){printf("read fail!\n"); return false;}
	  ConnectionWriter *returnToSender = connection.getWriter();
	  if (returnToSender!=NULL) {
	    outBin.write(*returnToSender);
	  }
	  return true;
	}

      public:

	/** Reply sent to the probing client. Contains server
	 *  parameters that the client uses for initialisation. */
	ObjManServerParams reply;
      };
          



     //KALMANLIST IS A LIST OF "N" KALMAN FILTERS.
      //IT IS SENT IN ENTIRITY EACH TIME (for now):
      class KalmanList {
      private:
	std::vector<Kal*> list;

      public:	
	KalmanList() {

	}
	~KalmanList() {
	  clear();
	}
	
	int add(Kal* data) {
	  //NOTE: KalmanList takes responsibility over pointer
	  int position = list.size();
	  list.push_back(data);
	  return position;
	}

	Kal* get(int i) {
	  return list[i];
	}

	int size() {
	  return list.size();
	}

	void clear() {
	  for(int i = 0; i < list.size(); i++) {
	    delete list[i];
	  }
	  list.clear();
	}
	      
      };



    }
  }
}

#endif
 
