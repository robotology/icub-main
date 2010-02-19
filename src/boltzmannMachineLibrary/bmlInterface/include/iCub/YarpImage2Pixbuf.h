#ifndef __YARPIMAGE2PIXBUF__

#include <yarp/sig/Image.h>
#include <gtk/gtk.h>

bool yarpImage2Pixbuf(yarp::sig::ImageOf<yarp::sig::PixelRgb> *sourceImg, 
                      GdkPixbuf* destPixbuf);
bool yarpImage2Pixbuf(yarp::sig::ImageOf<yarp::sig::PixelMono> *sourceImg, 
                      GdkPixbuf* destPixbuf);
#endif


//----- end-of-file --- ( next line intentionally left blank ) ------------------

