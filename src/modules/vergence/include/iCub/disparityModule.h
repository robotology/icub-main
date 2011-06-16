// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Vadim Tikhanoff
 * email:   francesco.rea@iit.it
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

#include <iCub/disparityProcessor.h>
#include <iCub/disparityTool.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
// yarp
#include <yarp/math/Math.h>
#include <yarp/sig/Image.h>

class disparityModule : public RFModule {

private:
    /**
    * flag that indicates when the initialisation has been carried out
    */
    bool init_flag;
    Port cmdPort;
    
    std::string moduleName, robotName, robotPortName, ctrlType;
    std::string outputPortName; 

    //property to get encoders 
    yarp::os::Property optionsHead, optionsTorso;
    yarp::dev::IEncoders *encHead, *encTorso;

    yarp::dev::PolyDriver *robotHead, *robotTorso;

    yarp::sig::Vector fb, zl, pl, dl, ml;

    yarp::sig::Vector _q;
    yarp::sig::Vector _it;
    yarp::sig::Vector _o;
    yarp::sig:: Vector _epx;
    yarp::sig::Vector _tmp;
    yarp::sig::Vector _tmpEl;

    int _nFrame;

    double leftMax, leftMin, rightMax, rightMin;

    yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb > >    imageInLeft;  //left camera port
    yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb > >    imageInRight; //right camera port
    yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelMono > >   histoOutPort; //output histogram

    bool needLeft, needRight;
    int imgNumb;
    float ratio;
    FILE *fout;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgInL;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgInR;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> Limg;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> Rimg;

    /**
    * processor in charge of measuring the disparity and creating the histogram
    */
    disparityProcessor* currentProcessor;

    shift_Struct maxes[4];

public:
	
    typedef enum { KIN_LEFT = 1, KIN_RIGHT = 2, KIN_LEFT_PERI = 3, KIN_RIGHT_PERI = 4 } __kinType;

    disparityModule();
    ~disparityModule();
    /**
    * function for initialization and configuration of the RFModule
    * @param rf resourceFinder reference
    */
    virtual bool configure( yarp::os::ResourceFinder &rf );
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

    bool respond( const Bottle& command, Bottle& reply ) {

        string helpMessage =  string( getName().c_str() ) + 
            "commands are: \n" +  
            "help \n" + 
            "quit \n" +
            "ctrl use either ctrlGaze or arbitrer";
        reply.clear(); 

        if ( command.get(0).asString() == "quit" )
            return false;     
        else if ( command.get(0).asString() == "help" ) {
            cout << helpMessage;
            reply.addString( "ok" ); 
        }
        else if (command.get(0).asString() == "sus"){
            cout << " sending suspend signal" << endl;
            currentProcessor->suspend();
        }
        else if (command.get(0).asString() == "res"){
            cout << "sending resume signal" << endl;
            currentProcessor->resume();
        }
        else {
            cout << "command not known - type help for more info" << endl;
        }
        return true;
    }
    
    ImageOf<PixelMono> histo;//output image of the histogram sent on the port
};
//make gcc happy
