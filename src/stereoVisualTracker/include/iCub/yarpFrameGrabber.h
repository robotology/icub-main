// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
#ifndef __YARP_FRAME_GRABBER_H__
#define __YARP_FRAME_GRABBER_H__

#include <cv.h>
#include <cvaux.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include "yarp/os/Stamp.h"
//#include "iCub/frameGrabber.h"

using namespace yarp::os;
using namespace yarp::sig;

/**
 * a wrapper for getting an image from the network though a port
 * 
 */
class YarpFrameGrabber : public FrameGrabber
{

 protected:
  ImageOf<PixelRgb> *image;
  BufferedPort<ImageOf<PixelRgb> > port;//port receiving images
  
 public:
    Stamp stamp;
    virtual ~YarpFrameGrabber(){};
  /**
   * opens the recieving port
   * @param portname name of receiving port
   * @return true upon success, false upon failure
   */
  bool Init(char *portname){return port.open(portname);}

  /**
   * opens the recieving port and connects to image server port
   * @param portname name of recieving port
   * @param servername name of image server port
   * @return true upon success, false upon failure
   */
 bool Init(char *portname,char *servername)
    {return Init(portname)&& Network::connect(servername,portname);}

 /**
  * grabs a frame from the recieving port, and gets the IplImage
  * @param frame where to put the pointer to the IplImage
  * @param index not used
  */
  void GrabFrame(IplImage **frame, u32 index=0)
    {image = port.read(true);port.getEnvelope(stamp);*frame = (IplImage *)image->getIplImage();}
 
  /**
   * closes the port
   */
  void Kill(){port.close();};

	virtual CvSize GetSize(){return cvSize(image->width(),image->height());};

    double GetTime(){return stamp.getTime();}

};
#endif
