// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
 * email:   assif.mirza@robotcub.org
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

#ifndef __IHA_AC_IMAGEREADLOOP__
#define __IHA_AC_IMAGEREADLOOP__

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/debug.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace contrib {
        class ImageReadLoop;
    }
}

using namespace iCub::contrib;
using namespace iCub::iha;

class iCub::contrib::ImageReadLoop : public yarp::os::Thread
{
public:
	ImageReadLoop(yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelRgb> > &_imageport) :
		imageport(&_imageport) 
    { 
        p_currentImage=NULL;
    } 
	~ImageReadLoop(){
        ACE_OS::fprintf(stderr,"ImageReadLoop: destroyed\n");
    }
	void run();
	void onStop() {}
	void beforeStart() {}
	void afterStart() {}
	bool threadInit() { return true; }
	void threadRelease() {}

    yarp::sig::ImageOf< yarp::sig::PixelRgb >*  aquireCurrentImage();
    void releaseCurrentImage();
private:
	yarp::os::BufferedPort< yarp::sig::ImageOf<yarp::sig::PixelRgb> > *imageport;

    yarp::sig::ImageOf< yarp::sig::PixelRgb > * p_currentImage;
    Semaphore imageMutex;
};

#endif
