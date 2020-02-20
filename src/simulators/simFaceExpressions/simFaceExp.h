/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
#ifndef __ICUB_SIMFACEEXP_H__
#define __ICUB_SIMFACEEXP_H__

#include <iostream>
#include <string>

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#define NUM_ARRAY 24

class simFaceExp : public yarp::os::RFModule
{
private:
    /* module parameters */
    std::string moduleName;                         //string containing the module name
    std::string maskPath;                           //string containing the mask path name
    std::string handlerPortName;                    //string containing the name of the handler port

    yarp::os::Port handlerPort;                     //port to handle messages
    yarp::os::Port expressionVals;                  //incoming port for the hexadeimal values
    yarp::os::Port eyeLidsPos;                      //incoming port for the hexadeimal values

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgOut; //outgoing port sending the texture of the expression

    int width;                                      //integer containing the width of the mask
    int height;                                     //integer containing the height of the mask
    unsigned char *maskData;                        //data containing the mask itself
    unsigned char *data;                            //data containing the new texture for the emotions

    yarp::sig::ImageOf<yarp::sig::PixelRgb> mask;   //image containing the mask
    yarp::sig::ImageOf<yarp::sig::PixelRgb> image;  //image containing the new texture for the emotions

    int ledsActivation[24];                         //array containing led activations
    int eyeLids[8];                                 //array containing eyeLids activations
    float eyeLidPos;                                //float containing eyelid positions

    yarp::os::Bottle bot;                                     //bottle containing the commands

public:

    bool configure(yarp::os::ResourceFinder &rf);                           // configure all the module parameters and return true if successful
    bool interruptModule();                                                 // interrupt, e.g., the ports
    bool close();                                                           // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool readMask(const char * filename);
    double getPeriod();
    bool updateModule();
    float linearMap( float x, float min, float max, float outMin, float outMax );
    void prepareData(unsigned char *data, int width, int height);
    void setSubSystem(const char *command);
    void generateTexture();
};

#endif

//empty line to make gcc happy



