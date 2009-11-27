// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef _YARP2_SERVERMOBILEEYE_
#define _YARP2_SERVERMOBILEEYE__

#include <stdio.h>

//#include <yarp/XSensMTx.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/DeviceDriver.h>
//#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
//#include <yarp/os/Vocab.h>
#include <yarp/os/Bottle.h>
//#include <yarp/dev/PreciselyTimed.h>
#include "MobileEye.h"


namespace yarp
{
	namespace dev
	{
		class MobileEyeServer;
	}
}

using namespace yarp::os;

/**
 * @ingroup dev_impl_wrapper
 *
 * Export the mobile eye gaze tracker's readings.
 * The network interface is a single Port.
 * We will stream bottles with a string and 3 integers:
 * 0 - timestamp, string
 * 1 - pupil diameter, int
 * 2 - horizontal gaze location, int
 * 3 - vertical gaze location, int
 *
 * @Frank Broz
 */
class yarp::dev::MobileEyeServer : public DeviceDriver, private Thread
{
private:
  bool spoke;
  //IGenericSensor *IMU; //The inertial device
  MobileEye *MOB;
  //IPreciselyTimed *iTimed;
  yarp::os::Port p;
  yarp::os::PortWriterBuffer<yarp::os::Bottle> writer;
 public:
  /**
   * Constructor.
   */
  MobileEyeServer()
    {
      MOB = NULL;
      spoke = false;
      //iTimed=0;
    }
  
  virtual ~MobileEyeServer() {
    if (MOB != NULL) close();
  }
  
  /**
   * Configure with a set of options. These are:
   * <TABLE>
   * <TR><TD> subdevice </TD><TD> Common name of device to wrap (e.g. "test_grabber"). </TD></TR>
   * <TR><TD> name </TD><TD> Port name to assign to this server (default /grabber). </TD></TR>
   * </TABLE>
   *
   * @param config The options to use
   * @return true iff the object could be configured.
   */
  virtual bool open(yarp::os::Searchable& config)
  {
    //p.setReader(*this);
    
    //Look for the device name (serial Port). Usually /dev/ttyUSB0
    yarp::os::Value *name;
    //if (config.check("subdevice",name))
    //  {
    //	printf("Subdevice %s\n", name->toString().c_str());
    //	if (name->isString())
    //	  {
    //    // maybe user isn't doing nested configuration
      //    yarp::os::Property p;
    //    p.setMonitor(config.getMonitor(),
    //		 "subdevice"); // pass on any monitoring
  //    p.fromString(config.toString());
    //    p.put("device",name->toString());
    //    //create the device driver
	    
    //poly.open(p);
    //  }
    //else
    //  poly.open(*name);
    //if (!poly.isValid())
    //  printf("cannot make <%s>\n", name->toString().c_str());
    //}
    //else
    //{
    //printf("\"--subdevice <name>\" not set for mobile eye\n");
    //
    //return false;
    //}
    
    MOB = new MobileEye();

    if (MOB!=NULL)
      writer.attach(p);
    
    //Look for the portname to register (--name option)
    if (config.check("name",name))
      p.open(name->asString());
    else
      p.open("/mobeyeserver");
    
    //Look for the portname to register (--name option)
    //			p.open(config.check("name", Value("/inertial")).asString());
    
    if (MOB!=NULL)
      {
	MOB->start();
	//also start the thread
	start();
	return true;
      }
    else
      return false;

  }
  

  virtual bool close()
  {
    if (MOB != NULL) {
      MOB->stop();
      MOB = NULL;
      //also stop the thread
      stop();
      return true;
    }
    return false;
  }
  
  
  virtual bool getGaze(yarp::os::Bottle &bot)
  {
    if (MOB==NULL)
      {
	return false;
      }
      else
	{
	  bot.clear();
	  return MOB->receive(bot);
	}
  }
  
  virtual void run() {

    printf("Server MobileEye starting \n");
    while (!isStopping()) {
      if (MOB!=NULL) {
	yarp::os::Bottle& bot = writer.get();
	getGaze(bot);
	//yarp::os::Stamp ts;
	//if (iTimed)
	// ts=iTimed->getLastInputStamp();
	//else
	//ts.update();
	
	if (!spoke)
	  {
	    printf("Writing a gaze reading.\n");
	    spoke = true;
	  }
	
	//p.setEnvelope(ts);
	writer.write();
      } else {
	printf("MobileEye device not created \n");
      }
      
    }
    printf("Server MobileEye stopping\n");
  }

  
};

#endif
