
#include <stdio.h>
#include <stdlib.h>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::file;
using namespace yarp::sig::draw;

/*

What is a nose?

Real way to detect a nose:
  Move the head without altering the pose of the eyes, and 
  see what in the field of view stays constant.

Heuristic, frame-by-frame estimate:

  Nose = a blurred, out of focus gradient appearing from a 
  particular corner of the image, with the gradient going
  from darker in the corner to lighter towards the center

 */

void apply(ImageOf<PixelRgb>& src, ImageOf<PixelRgb>& dest) {
  dest = src;

  ImageOf<PixelFloat> mono;
  mono.copy(src);

  int scale = src.height();

  IMGFOR(dest,x,y) {
    bool ok = false;

    if (mono.safePixel(x,y)<50) {
      for (int d=scale/20; d<=scale/10; d+=(1+scale/80)) {
	ok = false;
	if (mono.safePixel(x,y-d)>mono.safePixel(x,y)) {
	  if (mono.safePixel(x-d,y)>mono.safePixel(x,y)) {
	    if (mono.safePixel(x-d,y-d)>mono.safePixel(x,y)) {
	      ok = true;
	    }
	  }
	}
	if (!ok) {
	  break;
	}
      }
    }
    if (ok) {
      for (int s=2; s<=scale/40; s++) {
	float diff = mono.safePixel(x-s,y-s)-mono.safePixel(x,y);
	float ratio = 500/((float)scale);
	if (!(diff>0 && diff<s*ratio)) {
	  ok = false;
	}
      }
      if (ok) {
	addCircle(dest,PixelRgb(255,0,0),x,y,3);
      }
    }
  }
}


void net(Property& options) {
  BufferedPort<ImageOf<PixelRgb> > port;
  Value *val;

  if (options.check("local",val)) {
    port.open(val->toString());
  } else {
    port.open("...");
  }

  if (options.check("remote",val)) {
    Network::connect(val->toString(),port.where().getName());
  }

  while (true) {
    printf("Waiting for input\n");
    if (port.read()) {
      ImageOf<PixelRgb>& prep = port.prepare();
      apply(*port.lastRead(),prep);
      printf("Writing output\n");
      port.write();
    }
  }
}


int static_nose_main(int argc, char *argv[]) {
  Property options;
  options.fromCommand(argc,argv);

  Value *val;
  if (options.check("remote",val)) {
    net(options);
  }

  if (!options.check("input",val)) {
    printf("Please call with --input filename.ppm or --remote /grabber\n");
    exit(1);
  }

  ImageOf<PixelRgb> image,output;
  printf("Reading image from %s\n", val->asString().c_str());
  read(image,val->asString());

  apply(image,output);
  printf("writing to output.ppm\n");
  write(output,"output.ppm");

  return 0;
}

