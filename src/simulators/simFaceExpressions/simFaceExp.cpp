/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff, Martin Peniak
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txtd
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <yarp/os/Log.h>

#include "simFaceExp.h"

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;

bool simFaceExp::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */
    Property options;
    string general = rf.findFile("general");
    options.fromConfigFile(general.c_str());

    moduleName          = options.check("name", 
                        Value("simFaceExp"), 
                        "module name (string)").asString();

    width               = options.check("width",
                        Value(512), 
                        "mask width").asInt();

    height              = options.check("height",
                        Value(512), 
                        "mask height").asInt();

    maskPath = rf.findFile("mask");

    setName(moduleName.c_str());

   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */

    handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal 
 
    if (!handlerPort.open(handlerPortName.c_str())) 
    {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    expressionVals.open("/icubSim/face/raw/in");
    imgOut.open("/face/image/out");
    eyeLidsPos.open("/face/eyelids");

    attach(handlerPort);    // attach to port

    //Load the mask

    if (readMask(maskPath.c_str()))
        yInfo("\n\nMask has been loaded correctly\n\n");
    else
    {
        yError("Mask has some issues please control files\n");
        return false;
    }
    return true ;      // let the RFModule know everything went well
}

bool simFaceExp::readMask(const char * filename)
{
    if (read(mask,filename))
    {
        maskData = new unsigned char [width*height*3];
        data = new unsigned char [width*height*3];

        maskData = mask.getRawImage();
        image.setQuantum(1);
        image.setExternal(data,width,height);
        image.resize(width,height);
        image.zero();
        return true;
    }
    else
        return false;
}

/* Called periodically every getPeriod() seconds */
bool simFaceExp::updateModule()
{
    bot.clear();
    expressionVals.read(bot);
    string message = bot.toString();
    yInfo("Message received: %s\n",message.c_str());
    setSubSystem(message.c_str());
    generateTexture();

    imgOut.prepare() = image;
    imgOut.write();
    //send eyelids position
    if ((bot.toString().c_str())[0]=='S')
    {
        bot.clear();
        bot.addInt(eyeLidPos);
        eyeLidsPos.write(bot);
        yInfo("Eye lids position sent: %s\n",bot.toString().c_str());
    }
   return true;
}

void simFaceExp::setSubSystem(const char *command)
{
    //variable holding position in the array
    int pos = 0;
    //get sub system id and set writing position in the array
    char systemID = command[0];
    if (systemID == 'L') pos = 0;
    else if (systemID == 'R') pos = 8;
    //else if (systemID == 'M') pos = 16;
    else if (systemID == 'M') pos = 21;
    
    //truncate string
    char hex[2];
    hex[0]=command[1];
    hex[1]=command[2];

    //convert hexdec to binary 
    int num = strtol(hex, (char **)NULL, 16);

    if (systemID == 'S')
    {
        for (int i=0;i<8;i++) 
            eyeLids[i] = (num >> i)&1;

        //print the array
        for (int i = 0; i<8;i++) 
            yDebug("%d", eyeLids[i]);
        yDebug("\n");

        eyeLidPos = linearMap((float)num,36,72,0,30);
        yDebug("eyeLid pos is %lf\n",eyeLidPos);
    }
    else 
    {
        if (pos == 21)
        {
            for (int i=0;i<8;i++) 
                ledsActivation[(pos)-i] = (num >> i)&1;
        }
        else
            for (int i=0;i<8;i++) 
                ledsActivation[pos+i] = (num >> i)&1;

        if (ledsActivation[19]==1 && ledsActivation[18]!=1)
        {
            ledsActivation[18]=1;
            ledsActivation[19]=0;
        }
        else if (ledsActivation[18]==1  && ledsActivation[19]!=1)
        {
             ledsActivation[18]=0;
             ledsActivation[19]=1;
        }
    }
}
//generate texture by checking leds that are switched on and drawing them 
void simFaceExp::generateTexture()
{
    for (int i=0;i<512;i++)
        for (int j =0;j<512;j++)
        {
            PixelRgb& maskPixel = mask.pixel(i,j);
            PixelRgb& imagePixel = image.pixel(i,j);
            if (maskPixel.b <NUM_ARRAY && ledsActivation[maskPixel.b]==1) 
            {
                imagePixel.r = 255;
                imagePixel.g = 120;
                imagePixel.b = 120;
            }
            else
            {
                imagePixel.r = 255;
                imagePixel.g = 255;
                imagePixel.b = 245;
            }
        }
    //write(image,"Texture.ppm");
}

void simFaceExp::prepareData(unsigned char *data, int width, int height) 
{
    image.setQuantum(1);
    image.setExternal(data,width,height);
}

float simFaceExp::linearMap( float x, float min, float max, float outMin, float outMax )
{
    float m = - ( outMax-outMin )/( max-min );
    float q = outMax - m*min;
    float ret = m*x+q;
    if (ret < outMin) return outMin;
    if (ret > outMax) return outMax;
    return ret;
}

double simFaceExp::getPeriod() 
{
    return 0.1;
}

bool simFaceExp::interruptModule()
{
    handlerPort.interrupt();
    expressionVals.interrupt();
    eyeLidsPos.interrupt();
    imgOut.interrupt();
    return true;
}

bool simFaceExp::close()
{
    yInfo("cleaning up\n");
    handlerPort.close();
    expressionVals.close();
    eyeLidsPos.close();
    imgOut.close();
    delete[] data;
    return true;
}

bool simFaceExp::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear(); 

    if (command.get(0).asString()=="quit") 
    {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") 
    {
        yInfo("Options:\n\n");
        yInfo("\t--name       name: module name (default: simFaceExpressions)\n");
        reply.addString("ok");
    }
    else
    {
        yWarning("command not known - type help for more info\n");
    }
    return true;
}



