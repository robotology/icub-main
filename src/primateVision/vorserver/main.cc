/**
 *
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_vorserver VORServer
 *
 * \brief A test server that implements a basic Vestibulo-Ocular Reflex (VOR) 
 * to stabilise the gaze using gyro feedback. Used for development of the VOR in the RecServer.
 *
 * When in use, removes blur due to high-frequency head shake/perturbation. 
 * Compliments the ZDFServers as they provide a low-frequency (frame-rate) 
 * visual target tracking input while the VOR takes care of sub-framerate 
 * stabilisation. 
 *
 * This version has no output ports and does not support clients.
 *
 * \author Andrew Dankers
 *
 * This file can be edited at src/primateVision/vorserver/main.cc 
 * 
 */



/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
//YARP:
#include <yarp/os/Module.h>
#include <yarp/os/Property.h>
#include "vor.h"

using namespace std; 
using namespace yarp;
using namespace yarp::os;
using namespace iCub::contrib::primateVision;





/*!
  This Application uses the usual Yarp2 Module features.
*/
class CtrlModule: public Module
{
protected:
  VORServer*v;
  string fname;

  
public:
  CtrlModule() { }
  
    virtual bool open(Searchable &s)
  {
      
      Property options(s.toString());
      
      fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/vor.cfg");
      

      printf("Using config file: %s\n",fname.c_str());

      if (options.check("config"))
	fname=options.find("config").asString();
      
      v = new VORServer(&fname);      
      
    
        return true;
    }

 

    virtual bool close()
    {

      v->stop();
        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};




/*!
  Instantiates Yarp2 network components, the QApplication, and begins the RecServer module.
*/
int main(int argc, char *argv[])
{
 

  Network yarp;
  
  if (!yarp.checkNetwork())
    return false;
  
  CtrlModule mod;
  
  return mod.runModule(argc,argv);


}

