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
#include <iCub/DisparityTool.h>

// yarp
#include <yarp/math/Math.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

// std
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;

class disparityModule : public RFModule {

private:
    /**
    * flag that indicates when the initialisation has been carried out
    */
    bool init_flag;
    Port cmdPort;
    
    string moduleName, robotName, robotPortName;
    string outputPortName; 

    //property to get encoders 
    Property optionsHead, optionsTorso;
    IEncoders *encHead, *encTorso;

    PolyDriver *robotHead, *robotTorso;

    Vector fb, zl, pl, dl, ml;

    Vector _q;
    Vector _it;
    Vector _o;
    Vector _epx;
    Vector _tmp;
    Vector _tmpEl;

    int _nFrame;

    double leftMax, leftMin, rightMax, rightMin;

    BufferedPort < ImageOf<PixelRgb > >    imageInLeft;  //left camera port
    BufferedPort < ImageOf<PixelRgb > >    imageInRight; //right camera port
    BufferedPort < ImageOf<PixelMono > >   histoOutPort; //output histogram

    bool needLeft, needRight;
    int imgNumb;
    float ratio;
    FILE *fout;

    ImageOf<PixelRgb> *imgInL;
    ImageOf<PixelRgb> *imgInR;

    ImageOf<PixelRgb> Limg;
    ImageOf<PixelRgb> Rimg;

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

    virtual double getPeriod() {return 0.05;}

    bool respond( const Bottle& command, Bottle& reply ) {

    string helpMessage =  string( getName().c_str() ) + 
        "commands are: \n" +  
        "help \n" + 
        "quit \n";
    reply.clear(); 

    if ( command.get(0).asString() == "quit" )
        return false;     
    else if ( command.get(0).asString() == "help" ){
        cout << helpMessage;
        reply.addString( "ok" ); 
    }
    else
    {
        cout << "command not known - type help for more info" << endl;
    }
    return true;
    }
    
    ImageOf<PixelMono> histo;//output image of the histogram sent on the port
};
//make gcc happy

