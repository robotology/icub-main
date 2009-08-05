// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <FileFrameGrabber.h>
#include <ImageStack.h>
#include <Flow.h>
#include <Defn.h>

#define FPS 15
#define FPS_DELAY (1.0/FPS)

//#include <cv.h>
//#include <cvaux.h>
//#include <highgui.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

int main(int argc, char *argv[]) {
	 int frame_count = 0;
	 int max_frames = 49;

    Network::init();

    // give YARP a factory for creating instances of FileFrameGrabber
    DriverCreator *file_grabber_factory = 
        new DriverCreatorOf<FileFrameGrabber>("file_grabber",
                                              "grabber",
                                              "FileFrameGrabber");
    Drivers::factory().add(file_grabber_factory); // hand factory over to YARP

    // use YARP to create and configure an instance of FileFrameGrabber
    Property config;
    if (argc==1) {
        // no arguments, use a default
        config.fromString("(device file_grabber) (pattern \"/home/cdmcc/spherical_testdata/landing_tests2/landing67_h24/landing67_00%d.ppm\")");
    } else {
        // expect something like '--device file_grabber --pattern "image/%03d.ppm"'
        //                    or '--device dragonfly'
        //                    or '--device test_grabber --period 0.5 --mode [ball]'
        config.fromCommand(argc,argv);
    }
    PolyDriver dd(config);
    if (!dd.isValid()) {
        printf("Failed to create and configure a device\n");
        exit(1);
    }
    IFrameGrabberImage *grabberInterface;
    if (!dd.view(grabberInterface)) {
        printf("Failed to view device through IFrameGrabberImage interface\n");
        exit(1);
    }

	 Port p;
	 p.open("/cam");
	 printf("file image source server listening on /cam\n");

	/*
	while(frame_count == 0){
		while(p.getOutputCount() < 1){
//			  printf("input count is %d\n", p.getInputCount());
			  Time::delay(0.1);
		}
	*/
	 	while(true){
			ImageOf<PixelRgb> img;
			//img2.resize(320,240);
    		grabberInterface->getImage(img);
    //		printf("Got a %dx%d image\n", img.width(), img.height());
			frame_count++;
			if(frame_count >= max_frames-1){
				frame_count = 0;
			}
			p.write(img);
			Time::delay(FPS_DELAY);
		}
	//}

    dd.close();

    Network::fini();
    return 0;
}
