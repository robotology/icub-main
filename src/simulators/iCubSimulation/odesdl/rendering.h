// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff
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

/**
 * \file rendering.h
 * \brief Header file for the rendering
 * \author Vadim Tikhanoff
 * \date 2008
 * \note Released under GNU GPL v2.0
 **/


#include "SDL.h"
#include "SDL_opengl.h"
#include <ode/ode.h>
#include "VideoTexture.h"
#include <yarp/os/ResourceFinder.h>
#include "xloader.h"
#include <iostream>
# include <stdio.h>
using std::cout;
using std::endl;
#ifndef __RENDERING_H__
#define __RENDERING_H__
#define FLOOR  0

typedef struct                       /**** BMP file info structure ****/
    {
    unsigned int   biSize;           /* Size of info header */
    int            biWidth;          /* Width of image */
    int            biHeight;         /* Height of image */
    unsigned short biPlanes;         /* Number of color planes */
    unsigned short biBitCount;       /* Number of bits per pixel */
    unsigned int   biCompression;    /* Type of compression to use */
    unsigned int   biSizeImage;      /* Size of image data */
    int            biXPelsPerMeter;  /* X pixels per meter */
    int            biYPelsPerMeter;  /* Y pixels per meter */
    unsigned int   biClrUsed;        /* Number of colors used */
    unsigned int   biClrImportant;   /* Number of important colors */
    char *data;
    } SIMBITMAPINFOHEADER;

typedef struct vector_s
{
    float   x,
            y,
            z;
} vector_t;

typedef struct vertex_s
{
    float   x,
            y,
            z;
} vertex_t;

extern unsigned int Texture[200];

void setupTexture(char *filename, int whichtexture);
bool setup_opengl(yarp::os::ResourceFinder& finder);
void DrawVideo(VideoTexture *video);
void DrawGround(bool wireframe);
void DrawBox(float length, float height, float width, bool wireframe, bool texture, int whichtexture);
void DrawSphere(float radius, bool wireframe, bool texture,int whichtexture);
void DrawCylinder(float radius, float length, bool wireframe, bool texture,int whichtexture);
void LDEsetM(const dReal *pos,const dReal *R);
GLuint LoadTextureRAW( const char * filename, int wrap );
void drawSkyDome(float x, float y, float z, float width, float height, float length);
void DrawX (dTriMeshX trimesh, int whichtexture);
int LoadBitmapTERMINAL(char *filename, int whichtexture);


#endif


