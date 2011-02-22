// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick, Vadim Tikhanoff
* email:   paulfitz@alum.mit.edu, vadim.tikhanoff@iit.it
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

#ifndef __VIDEO_TEXTURE__
#define __VIDEO_TEXTURE__


/**
 * \file VideoTexture.h
 * \brief Header file for the video stream texture
 * \author Paul Fitzpatrick
 * \date 2008
 * \note Released under GNU GPL v2.0
 **/


#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>

#include <vector>
#include <string>
// std
#include <stdio.h>
#include <iostream>

using namespace std;

class TextureInput {
private:
    int textureIndex;
    double lastData;
    string moduleName;
    int t;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > port;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> cache, input;
	
public:
    TextureInput() {
        textureIndex = -1;
        lastData = -1000;
        t = 0;
    }
	~TextureInput(){
		port.close();
	} 
    void setName( string module );
    bool open(yarp::os::Searchable& config);
    void apply(unsigned int *textures);
};


class VideoTexture {
public:
    std::vector<TextureInput *> inputs;

    ~VideoTexture();

    string videoPort;
    bool add(const char *port, int textureIndex) {
        yarp::os::Property config;
        config.put("port",port);
        config.put("textureIndex",textureIndex);
        return add(config);
    }

    void setName( string module );

    bool add(yarp::os::Searchable& config);

    void apply(unsigned int *textures);
};


#endif
