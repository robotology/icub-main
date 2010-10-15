/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
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

#ifndef __ICUB_YUV_PROC_H__
#define __ICUB_YUV_PROC_H__

#include <iostream>
#include <string>

#include <yarp/sig/Image.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <ipp.h>
#include "iCub/centsur.h"
 
class YUVThread : public yarp::os::RateThread
{
private:

    std::string moduleName;              //string containing module name 
    std::string inputPortName;           //string containing input port name 
    std::string outputPortName1;         //string containing output port name, intensity or hue
    std::string outputPortName2;         //string containing output port name, UV or saturation 
    std::string outputPortName3;         //string containing output port name, value 
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  imageInputPort;  //input port log polar RGB
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutPort1;   //output port intensity or hue process
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutPort2;   //output port colour(UV) or saturation process  
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutPort3;   //output port value process

    yarp::sig::ImageOf<yarp::sig::PixelRgb>   *inputExtImage;  // extended input image

    yarp::sig::ImageOf<yarp::sig::PixelMono>  *img_Y;          // extended output image, also reused for hsv
	yarp::sig::ImageOf<yarp::sig::PixelMono>  *img_UV;         // extended output image, also reused for hsv
    yarp::sig::ImageOf<yarp::sig::PixelMono>  *img_V;         // extended output image, also reused for hsv
	yarp::sig::ImageOf<yarp::sig::PixelMono>  *img_out_Y;      // output image, also reused for hsv
	yarp::sig::ImageOf<yarp::sig::PixelMono>  *img_out_UV;     // output image, also reused for hsv
    yarp::sig::ImageOf<yarp::sig::PixelMono>  *img_out_V;     // output image, only used for hsv

    IppiSize srcsize, origsize;     //ippsises used for calculations
    int ncsscale;                   // center surround scale
    bool allocated;                 // flag to check if the variables have been already allocated
    bool isYUV;                     // flag to check if which process to run (YUV or HSV)

    CentSur * centerSurr; 
   
    Ipp8u *orig;        //extended input image
    Ipp8u *colour;      //extended rgb+a image
    Ipp8u *yuva_orig;   //extended yuv+a image    
    Ipp8u** pyuva;      //extended yuv+a image used to extract y, u and v plane
    
    Ipp8u *first_plane;      //extended plane either y or h
    Ipp8u *second_plane;      //extended plane either u or v
    Ipp8u *third_plane;      //extended plane either v or s

    Ipp8u *tmp;         //extended tmp containing alpha
    Ipp32f *cs_tot_32f; //extended 
    Ipp8u *ycs_out;     //final extended intensity center surround image
    Ipp8u *scs_out;     //final extended intensity center surround image
    Ipp8u *vcs_out;     //final extended intensity center surround image
    Ipp8u *colcs_out;   //final extended coulour center surround image
    int img_psb, psb4, psb, ycs_psb, col_psb, psb_32f, f_psb, s_psb, t_psb; //images rowsizes
    
    /**
    * function that extendes the original image of the desired value for future convolutions (in-place operation)
    * @param origImage originalImage
    * @param extDimension dimension of the extention on each of the sides of the image
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* extender(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputOrigImage, int maxSize);

public:
    /**
    * constructor
    * @param p is the thread period in milliseconds (default = 100).
    * @param moduleName is passed to the thread in order to initialise all the ports correctly (default yuvProc)
    * @param imgType is passed to the thread in order to work on YUV or on HSV images (default yuv)
    */
    YUVThread(int p, std::string moduleName, std::string imgType);
    
    /**
     * destructor
     */
    ~YUVThread();

    bool threadInit();     
    void threadRelease();
    void run(); 
    void allocate(yarp::sig::ImageOf<yarp::sig::PixelRgb> *img);
    void deallocate();
    void onStop();

    void afterStart(bool s)
    {
        if (s)
            std::cout<<"Thread started successfully"<< std::endl;
        else
            std::cout<<"Thread did not start"<< std::endl;
    }
};

class yuvProc : public yarp::os::RFModule
{
   /* module parameters */
    std::string moduleName;      //string containing the module name passed to the thread
    std::string imageType;       //string containing the image type passed to the thread
    std::string handlerPortName; //string containing the name of the handler port

    yarp::os::Port handlerPort;      //a port to handle messages 
    
    /* pointer to the working thread */
    YUVThread *yuvThread;
    int period;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};

#endif

//empty line to make gcc happy
