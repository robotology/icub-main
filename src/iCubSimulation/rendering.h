/**
 * \file rendering.h
 * \brief Header file for the rendering
 * \author Vadim Tikhanoff
 * \date 2008
 * \note Release under GNU GPL v2.0
 **/


#include <ode/ode.h>
#include "VideoTexture.h"
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
extern unsigned int Texture[200];

void setupTexture(char *filename, int whichtexture);
void setup_opengl();
void DrawVideo(VideoTexture *video);
void DrawGround(bool wireframe);
void DrawBox(float length, float height, float width, bool wireframe, bool texture, int whichtexture);
void DrawSphere(float radius, bool wireframe, bool texture,int whichtexture);
void DrawCylinder(float radius, float length, bool wireframe, bool texture,int whichtexture);
void LDEsetM(const dReal *pos,const dReal *R);
GLuint LoadTextureRAW( const char * filename, int wrap );
void drawSkyDome(float x, float y, float z, float width, float height, float length);
//void CreateCylinder(dReal length, dReal radius);
void DrawX (dTriMeshX trimesh, int whichtexture);
int LoadBitmapTERMINAL(char *filename, int whichtexture);


#endif


