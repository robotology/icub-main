#include <iCub/visualFilterThread.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

visualFilterThread::visualFilterThread(BufferedPort<ImageOf<PixelRgb> > *imageIn, BufferedPort<ImageOf<PixelRgb> > *imageOut, int *threshold)
{
   imagePortIn    = imageIn;
   imagePortOut   = imageOut;
   thresholdValue = threshold;
   interrupted=false;
}


bool visualFilterThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

   return true;
}

void visualFilterThread::run(){

   /* 
    * do some work ....
    * for example, convert the input image to a binary image using the threshold provided 
    */ 
   
   unsigned char value;

   while (isStopping() != true) 
   { // the thread continues to run until isStopping() returns true
 
      cout << "visualFilterThread: threshold value is " << *thresholdValue << endl;
      
      
      if(!interrupted)
          image = imagePortIn->read(true);
      
      
      if(image!=NULL){
          ImageOf<PixelRgb> &binary_image = imagePortOut->prepare();
          binary_image.resize(image->width(),image->height());

          for (x=0; x<image->width(); x++) {
             for (y=0; y<image->height(); y++) {

                 rgbPixel = image->safePixel(x,y);

                 if (((rgbPixel.r + rgbPixel.g + rgbPixel.b)/3) > *thresholdValue) {
                    value = (unsigned char) 255;
                 }
                 else {
                    value = (unsigned char) 0;
                 }

                 rgbPixel.r = value;
                 rgbPixel.g = value;
                 rgbPixel.b = value;

                 binary_image(x,y) = rgbPixel;
              }
           }

           imagePortOut->write();
      } //if
   }//while

   
}

void visualFilterThread::threadRelease() 
{
   /* for example, delete dynamically created data-structures */
}
