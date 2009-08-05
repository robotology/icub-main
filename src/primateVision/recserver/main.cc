/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_recserver RecServer
 *
 *This module applies barrel rectification and calibrates stereo imagery to a plane behind the baseline, providing epipolar rectified images, and their mosaic coordinates.  Provides barrel and epipolar rectified left and right Y,U,V channel images, and associated parameters. Most of the other PrimateVision modules and clients depend directly or indirectly on the rectified output of the RecServer.

The module also implements a vestubulo-ocular reflex (VOR) and velocity control thread that accepts commands from clients and sends suitable commands to the control axes of the robot head such that the VOR and desired motions can occur simultaneously.  In this manner, all clients of the RecServer have the ability to move the robot head without any additional dependencies, and can obtain VOR stabilised images. This also allows the Recserver to prioritise motion requests internally.  For example, motion control can be locked to the ZDFServer so that the AttnServer does not cause attentional saccade during the execution of a task that requires concentrated/continuous fixation and tracking of a visual target.
 *
 * This module should be called in the following way:\n \b recserver \b -c \a rec.cfg\n
 * where rec.cfg is the config file.  Default configfile in primateVision/config/rec.cfg used if none specified.
 *\n
 
 * Ports accessed:\n
 * <ul>
 * <li> /icub/cam/left
 * <li> /icub/cam/right
 * <li> /icub/head/state:o
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /recserver/input/image/left
 * <li> /recserver/input/image/right
 * <li> /recserver/input/encoders
 * <li> /recserver/input/motion_request
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /recserver/output/serv_params
 * <li> /recserver/output/motion
 * <li> /recserver/output/rec_params
 * <li> /recserver/output/left_yb
 * <li> /recserver/output/left_ub
 * <li> /recserver/output/left_vb
 * <li> /recserver/output/left_ye
 * <li> /recserver/output/left_ue
 * <li> /recserver/output/left_ve
 * <li> /recserver/output/right_yb
 * <li> /recserver/output/right_ub
 * <li> /recserver/output/right_vb
 * <li> /recserver/output/right_ye
 * <li> /recserver/output/right_ue
 * <li> /recserver/output/right_ve
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/recserver/main.cc 
 *
 */


/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <string>
#include <iostream>
#include <qapplication.h>


#include "rec.h"
#include <yarp/os/Module.h>
#include <yarp/os/Property.h>

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
  RecServer*r;
  string fname;

  
public:
  CtrlModule() { }
  
    virtual bool open(Searchable &s)
  {
      
      Property options(s.toString());
      
      fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/rec.cfg");
      

      printf("Using config file: %s\n",fname.c_str());

      if (options.check("config"))
	fname=options.find("config").asString();
      
      r = new RecServer(&fname);
      
      
    
        return true;
    }

 

    virtual bool close()
    {
      r->exit();

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
 

  QApplication *a = new QApplication(argc, argv);


  Network yarp;
  
  if (!yarp.checkNetwork())
    return false;
  
  CtrlModule mod;
  
  return mod.runModule(argc,argv);


}

