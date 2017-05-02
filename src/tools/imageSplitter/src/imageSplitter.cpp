// Copyright (C) 2016 Istituto Italiano di Tecnologia - iCub Facility
// Author: Alberto Cardellino <alberto.cardellino@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>

#include <imageSplitter.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

ImageSplitter::ImageSplitter()
{
    horizontal = true;
    method = 2;
}

ImageSplitter::~ImageSplitter()
{
    horizontal = true;
    method = 2;
}

bool ImageSplitter::configure(yarp::os::ResourceFinder &rf)
{
    bool shouldConnect =false;
    if(rf.check("help"))
    {
        cout<<"Usage:"<<endl<<"<imageSplitter> --param1 arg1 --param2 arg2 ..."<<endl;
        cout<<"Here the available parameters:"<<endl;
        cout<<"align      specify if the alignement of the images is 'vertical' or 'horizontal'"<<endl;
        cout<<"local      prefix for the ports that will be opened by this module, if not specified the default is '/imageSplitter'"<<endl;
        cout<<"nameInput  name of the module's' input port, if not specified by default is <local> + '/input:i'"<<endl;
        cout<<"nameLeft   name of the output port for the 'left image', if not specified by default is <local> + '/left:o'"<<endl;
        cout<<"nameRight  name of the output port for the 'right image', if not specified by default is <local> + '/right:o'"<<endl;
        cout<<"remote     name of the source port, if specified it connects automatically to the module's input port"<<endl;
        std::exit(1);
    }
    // Check input parameters
    if(rf.check("align"))
    {
        yarp::os::ConstString align = rf.find("align").asString();
        if(align == "vertical")
        {
            horizontal = false;
        }
        else if(align == "horizontal")
        {
            horizontal = true;
        }
        else
        {
            yError() << " Incorrect parameter. Supported values for alignment are 'horizontal' or 'vertical'";
            return false;
        }
    }
    yarp::os::ConstString inputPortName;
    yarp::os::ConstString outLeftPortName;
    yarp::os::ConstString outRightPortName;


    if(rf.check("local"))
    {
        string rootName = rf.find("local").asString();
        inputPortName    = rootName + "/input:i";
        outLeftPortName  = rootName + "/left:o";
        outRightPortName = rootName + "/right:o";
    }
    else{
        inputPortName    = "/imageSplitter/input:i";
        outLeftPortName  = "/imageSplitter/left:o";
        outRightPortName = "/imageSplitter/right:o";
    }
    if (rf.check("nameInput"))
    {
        inputPortName = rf.find("nameInput").asString();
    }

    if(rf.check("nameLeft"))
    {
        outLeftPortName = rf.find("nameLeft").asString();
    }

    if(rf.check("nameRight"))
    {
        outRightPortName = rf.find("nameRight").asString();
    }

    // opening ports
    bool ret=true;
    ret &= inputPort.open(inputPortName);
    ret &= outLeftPort.open(outLeftPortName);
    ret &= outRightPort.open(outRightPortName);

    yarp::os::ConstString remotePortName;

    if(rf.check("remote"))
    {
        remotePortName = rf.find("remote").asString();
        shouldConnect = true;

    }
    else
        yInfo() << "ImageSplitter: waiting for connection to port" << inputPortName;

    if(!ret)
    {
        yError() << " Cannot open ports";
        return false;
    }

    // Connections
    if (shouldConnect){
        yInfo("connecting to the source...\n");
        if(! yarp::os::Network::connect(remotePortName, inputPortName))
        {
            yError() << "Cannot connect to remote port " << remotePortName;
            return false;
        }
    }

    // choose filling method
    if(rf.check("m"))
    {
        yarp::os::ConstString align = rf.find("m").asString();
        if(align == "pixel")
        {
            method = 0;
        }
        else if(align == "pixel2")
        {
            method = 1;
        }
        else if(align == "line")
        {
            method = 2;
        }
        else if(align == "whole")
        {
            if(horizontal)
                yError() << "Cannot use 'whole' method for input image horizontally aligned";
            method = 3;
        }
        else
        {
            yError() << "Methods are pixel, line, whole; got " << align;
            return false;
        }
    }

    yInfo() << "using method " << method;
    return true;
}

bool ImageSplitter::interruptModule()
{

    return true;
}

bool ImageSplitter::close()
{
    return true;
}

bool ImageSplitter::updateModule()
{
    ImageOf<PixelRgb> *inputImage    = inputPort.read();
    yarp::os::Stamp stamp;
    inputPort.getEnvelope(stamp);

    ImageOf<PixelRgb> &outLeftImage  = outLeftPort.prepare();
    ImageOf<PixelRgb> &outRightImage = outRightPort.prepare();

    inWidth  = inputImage->width();
    inHeight = inputImage->height();

    if(horizontal)  // input image is horizontally aligned
    {
        outWidth  = inWidth/2;
        outHeight = inHeight;
    }
    else
    {
        outWidth  = inWidth;
        outHeight = inHeight/2;
    }

    outLeftImage.setQuantum(inputImage->getQuantum());
    outRightImage.setQuantum(inputImage->getQuantum());
    outLeftImage.resize(outWidth, outHeight);
    outRightImage.resize(outWidth, outHeight);

    // alloc and compute some vars for efficency
    int h2, w2;
    unsigned char *pixelLeft, *pixelRight;
    unsigned char *pixelInputL, *pixelInputR;
    unsigned char *pixelInput = inputImage->getRawImage();
    int dualImage_rowSizeByte = inputImage->getRowSize();
    int singleImage_rowSizeByte = outLeftImage.getRowSize();
    int singleImage_wholeSizeByte = outWidth * outHeight * outLeftImage.getPixelSize();

    static int counter = 0;
    static double start = 0;
    start = yarp::os::Time::now();

    switch(method)
    {
        case 0: // pixel by pixel
        {
            if(horizontal)
            {
                for(int h=0; h<outHeight; h++)
                {
                    for(int w1=0; w1<outWidth; w1++)
                    {
                        w2 = w1+outWidth;
                        pixelLeft = outLeftImage.getPixelAddress(w1, h);
                        pixelLeft[0] = *(inputImage->getPixelAddress(w1, h)+0);
                        pixelLeft[1] = *(inputImage->getPixelAddress(w1, h)+1);
                        pixelLeft[2] = *(inputImage->getPixelAddress(w1, h)+2);

                        pixelRight = outRightImage.getPixelAddress(w1, h);
                        pixelRight[0] = *(inputImage->getPixelAddress(w2, h)+0);
                        pixelRight[1] = *(inputImage->getPixelAddress(w2, h)+1);
                        pixelRight[2] = *(inputImage->getPixelAddress(w2, h)+2);
                    }
                }
            }
            else
            {
                for(int h1=0; h1<outHeight; h1++)
                {
                    for(int w=0; w<outWidth; w++)
                    {
                        h2 = h1+outHeight;
                        pixelLeft = outLeftImage.getPixelAddress(w, h1);
                        pixelLeft[0] = *(inputImage->getPixelAddress(w, h1)+0);
                        pixelLeft[1] = *(inputImage->getPixelAddress(w, h1)+1);
                        pixelLeft[2] = *(inputImage->getPixelAddress(w, h1)+2);

                        pixelRight = outRightImage.getPixelAddress(w, h1);
                        pixelRight[0] = *(inputImage->getPixelAddress(w, h2)+0);
                        pixelRight[1] = *(inputImage->getPixelAddress(w, h2)+1);
                        pixelRight[2] = *(inputImage->getPixelAddress(w, h2)+2);
                    }
                }
            }
        } break;

        case 1: // pixel by pixel, a bit better
        {
            if(horizontal)
            {
                pixelLeft  = outLeftImage.getRawImage();
                pixelRight = outRightImage.getRawImage();

                pixelInputL = pixelInput;
                pixelInputR = pixelInput+singleImage_rowSizeByte;
                for(int h=0, idx=0, idx2=0; h<outHeight; h++)
                {
                    for(int w=0; w<outWidth; w++)
                    {
                        pixelLeft[idx++] = *(pixelInputL++);
                        pixelLeft[idx++] = *(pixelInputL++);
                        pixelLeft[idx++] = *(pixelInputL++);

                        pixelRight[idx2++] = *(pixelInputR++);
                        pixelRight[idx2++] = *(pixelInputR++);
                        pixelRight[idx2++] = *(pixelInputR++);
                    }
                    pixelInputL += singleImage_rowSizeByte;
                    pixelInputR += singleImage_rowSizeByte;
                }
            }
            else
            {

            }
        } break;

        case 2: // line by line
        {
            if(horizontal)
            {
                pixelLeft  = outLeftImage.getRawImage();
                pixelRight = outRightImage.getRawImage();

                for(int h=0; h<inHeight; h++)
                {
                    memcpy(pixelLeft  + h*singleImage_rowSizeByte, pixelInput,                          singleImage_rowSizeByte);
                    memcpy(pixelRight + h*singleImage_rowSizeByte, pixelInput+=singleImage_rowSizeByte, singleImage_rowSizeByte);
                    pixelInput+= dualImage_rowSizeByte/2;
                }
            }
            else
            {
                pixelLeft  = outLeftImage.getRawImage();
                pixelRight = outRightImage.getRawImage();
                pixelInputL = pixelInput;
                pixelInputR = pixelInput+singleImage_wholeSizeByte;

                for(int h=0; h<outHeight; h++)
                {
                    memcpy(pixelLeft  + h*singleImage_rowSizeByte, pixelInputL, singleImage_rowSizeByte);
                    memcpy(pixelRight + h*singleImage_rowSizeByte, pixelInputR, singleImage_rowSizeByte);
                    pixelInputL+= singleImage_rowSizeByte;
                    pixelInputR+= singleImage_rowSizeByte;
                }
            }
        } break;

        case 3: // whole image, only if input image is vertically aligned
        {
            if(horizontal)
            {
                yError() << "Cannot use this copy method with horizontally aligned source image.";
            }
            else
            {
                pixelLeft  = outLeftImage.getRawImage();
                pixelRight = outRightImage.getRawImage();

                memcpy(pixelLeft,  pixelInput,                            singleImage_wholeSizeByte);
                memcpy(pixelRight, pixelInput+ singleImage_wholeSizeByte, singleImage_wholeSizeByte);
            }
        } break;

        default:
        {
            yError() << " @line " << __LINE__ << "unhandled switch case, we should not be here!";
        }
    }

    static double end = 0;
    static double elapsed = 0;
    end = yarp::os::Time::now();
    elapsed += (end-start);

    counter++;
    if((counter % 100) == 0)
    {
        yInfo() << "Elapsed time: " << elapsed;
        elapsed = 0;
    }

    outLeftPort.setEnvelope(stamp);
    outRightPort.setEnvelope(stamp);

    outLeftPort.write();
    outRightPort.write();
    return true;
}


double ImageSplitter::getPeriod()
{    
   return 0.01;
}

