// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick
* email:   paulfitz@alum.mit.edu
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

/**
 * \file VideoTexture.cpp
 * \brief This file deals with applying a video stream as a texture on an object in the world
 * \author Paul Fitzpatrick
 * \date 2008
 * \note Released under GNU GPL v2.0
 **/
#include "VideoTexture.h"

#include "SDL.h"
#include "SDL_opengl.h"

using namespace yarp::os;
using namespace yarp::sig;

#define SQUARE_DIM 512


VideoTexture::~VideoTexture() {
    for (unsigned int i=0; i< inputs.size(); i++) {
        delete inputs[i];
        inputs[i] = NULL;
    }
    inputs.clear();
}


void VideoTexture::apply(unsigned int *textures) {
    for (unsigned int i=0; i<inputs.size(); i++) {
        inputs[i]->apply(textures);
    }
}
bool VideoTexture::add(Searchable& config) {
    TextureInput *entry = new TextureInput;
    entry->setName(videoPort);

    if (entry!=NULL) {
        bool ok = entry->open(config);
        if (!ok) {
            delete entry;
            entry = NULL;
        } else {
            inputs.push_back(entry);
            
            return true;
        }
	}
    return false;
}

void VideoTexture::setName(string name) {
    this->videoPort = name;
}

void TextureInput::setName(string name) {
    this->moduleName = name;
}

bool TextureInput::open(Searchable& config) {
    textureIndex = config.check("textureIndex",Value(-1),
                                "texture index").asInt();

    ConstString texturePort = config.check("port",Value("/texture"),"local port name").asString();

    string portStr = this->moduleName + texturePort.c_str();
    port.open( portStr.c_str() );

    return true;
}



void TextureInput::apply(unsigned int *textures) {
    if (cache.width()!=SQUARE_DIM || cache.height()!=SQUARE_DIM) {
        cache.setQuantum(1);
        cache.setTopIsLowIndex(false);
        cache.resize(SQUARE_DIM,SQUARE_DIM);
        cache.zero();
    }
    ImageOf<PixelRgb> *img = port.read(false);
    unsigned char *data = (unsigned char*)cache.getRawImage();
    int width = SQUARE_DIM;
    int height = SQUARE_DIM;
    double now = Time::now();
    bool haveUpdate = false;
    if (img!=NULL) {
        cache.copy(*img,cache.width(),cache.height());
        lastData = now;
        haveUpdate = true;
    }
    if (now-lastData>2000) {
        for (int x=0; x<width; x++) {
            for (int y=0; y<height; y++) {
                unsigned char *pix1 = data + (y*height+x)*3;
                unsigned char *pix2 = pix1+1;
                unsigned char *pix3 = pix1+2;
                *pix1 = (x%25+t)*10;;
                *pix2 = 255;
                *pix3 = (y*25+t/2)*10;
            }
        }
        t++;
        haveUpdate = true;
    }
	if (haveUpdate && port.getInputCount()>0) {
        glBindTexture(GL_TEXTURE_2D, textures[textureIndex]);
        gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height,
                          GL_RGB, GL_UNSIGNED_BYTE, data);
    }

}


