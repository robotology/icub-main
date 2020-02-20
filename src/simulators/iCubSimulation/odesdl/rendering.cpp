// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff
* email:  vadim.tikhanoff@iit.it
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
 * \file rendering.cpp
 * \brief This file deals with setting up the OpenGL and the rendering of objects in the environment
 * \author Vadim Tikhanoff
 * \date 2008
 * \note Released under GNU GPL v2.0
 **/

#ifdef WIN32
#include <windows.h>
#endif
#include "SDL.h"
#include "SDL_opengl.h"
#include "rendering.h"
#pragma warning(disable:4244 4305)  //for VC++, no precision loss complaints
#include "VideoTexture.h"
#include <cstring>
#include <yarp/os/LogStream.h>

using namespace yarp::os;

using namespace std;

const GLfloat light_ambient[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
const GLfloat light_diffuse[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_position[] = { 0.0f, 50.0f, 5.0f, 1.0f };

GLenum mode;
GLuint nicetexture;
int num_texture1=15;
GLuint num_texture = 19;

unsigned int Texture[200];

GLUquadricObj *sphere;
GLUquadricObj *cylinder;
GLUquadricObj *cap;

static GLuint FindTextureRAW(const char *str, bool flag) {
    return LoadTextureRAW(str,flag);
}

//Function to setup OpenGl
bool setup_opengl(ResourceFinder& finder){
    glShadeModel( GL_SMOOTH );

    // Culling
    glCullFace( GL_BACK );
    glFrontFace( GL_CCW );
    glEnable( GL_CULL_FACE );
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);               // OpenGL light
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    //glLightfv(GL_LIGHT1, GL_POSITION, light_position );
    /// Some OpenGL settings
    GLfloat light_color[] = {0.0,0.0,0.0,1.0};  // colour
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, light_color);
    //glMaterialfv(GL_FRONT,GL_DIFFUSE,light_color); // colour
    glClearColor(0.0,0.0,0.0,0); // background color

    string floor = finder.findFile("floor");
    Texture[0] = LoadTextureRAW( floor.c_str(), false );

    string body1 = finder.findFile("body1");
    Texture[1] = LoadTextureRAW( body1.c_str(), false );

    string body2 = finder.findFile("body2");
    Texture[2] = LoadTextureRAW( body2.c_str(), false );

    string skybox_ft = finder.findFile("skybox_ft");
    Texture[3] = LoadTextureRAW( skybox_ft.c_str(), false );

    string skybox_bk = finder.findFile("skybox_bk");
    Texture[4] = LoadTextureRAW( skybox_bk.c_str(), false );

    string skybox_lt = finder.findFile("skybox_lt");
    Texture[5] = LoadTextureRAW( skybox_lt.c_str(), false );

    string skybox_rt = finder.findFile("skybox_rt");
    Texture[6] = LoadTextureRAW( skybox_rt.c_str(), false );

    string skybox_up = finder.findFile("skybox_up");
    Texture[7] = LoadTextureRAW( skybox_up.c_str(), false );

    string face = finder.findFile("face");
    Texture[8] = LoadTextureRAW( face.c_str(), false );

    if (!Texture[1]){
        yError("No texture loaded\n");
        return false;
    }
    return true;
}

void DrawGround(bool wireframe){
    glEnable(GL_TEXTURE_2D);
    if (wireframe)
    {

    for(float i = -500; i <= 500; i += 5)
    {
        glBegin(GL_LINES);
        glVertex3f(-500, 0, i);
        glVertex3f(500, 0, i);
        glVertex3f(i, 0,-500);
        glVertex3f(i, 0, 500);
        glEnd();
    }
}
    else
    {
        glBindTexture(GL_TEXTURE_2D, Texture[FLOOR]);
        glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexEnvi (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, 0 ? GL_MODULATE : GL_DECAL);

        glBegin (GL_QUADS);
        glNormal3f (0,0,1);
        glTexCoord2f (-50.0f * 0.5f + 0.5f,-50.0f * 0.5f + 0.5f);//replaced from 50
        glVertex3f (-50.0f, -50.0f, 0.0f);
        glTexCoord2f (50.0f * 0.5f + 0.5f, -50.0f * 0.5f + 0.5f);
        glVertex3f (50.0f, -50.0f, 0.0f);
        glTexCoord2f (50.0f * 0.5f + 0.5f, 50.0f * 0.5f + 0.5f);
        glVertex3f (50.0f, 50.0f, 0.0f);
        glTexCoord2f (-50.0f * 0.5f + 0.5f, 50.0f * 0.5f + 0.5f);
        glVertex3f (-50.0f, 50.0f, 0.0f);
        glEnd();
    }
}
void drawSkyDome(float x, float y, float z, float width, float height, float length)
{
    // Center the Skybox around the given x,y,z position
    x = x - width  / 2;
    y = y - height / 2;
    z = z - length / 2;
    float skyboxMaterial[]   = {1.0f,1.0f,1.0f,1.0f};
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    glRotatef(180,0,0,1);
    // Draw Front side
    glBindTexture(GL_TEXTURE_2D, Texture[6]);
    glBegin(GL_QUADS);
    glMaterialfv(GL_FRONT, GL_AMBIENT, skyboxMaterial);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(x,    y,  z+length);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(x,    y+height, z+length);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(x+width, y+height, z+length);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(x+width, y,  z+length);
    glEnd();

    // Draw Back side
    glBindTexture(GL_TEXTURE_2D,  Texture[5]);
    glBegin(GL_QUADS);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(x+width, y,  z);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(x+width, y+height, z);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(x,    y+height, z);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(x,    y,  z);
    glEnd();

    // Draw Left side
    glBindTexture(GL_TEXTURE_2D,  Texture[4]);
    glBegin(GL_QUADS);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(x,    y+height, z);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(x,    y+height, z+length);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(x,    y,  z+length);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(x,    y,  z);
    glEnd();

    // Draw Right side
    glBindTexture(GL_TEXTURE_2D,  Texture[3]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(x+width, y,  z);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(x+width, y,  z+length);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(x+width, y+height, z+length);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(x+width, y+height, z);
    glEnd();

    // Draw Up side
    glBindTexture(GL_TEXTURE_2D,  Texture[7]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(x+width, y+height, z);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(x+width, y+height, z+length);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(x,    y+height, z+length);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(x,    y+height, z);
    glEnd();

    // Draw Down side
    glBindTexture(GL_TEXTURE_2D,  Texture[7]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(x,    y,  z);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(x,    y,  z+length);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(x+width, y,  z+length);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(x+width, y,  z);
    glEnd();

    glDisable(GL_TEXTURE_2D);
}

void DrawBox(float width, float height, float length, bool wireframe, bool texture, int whichtexture){

    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);//blends the texture with the color of the object

    if (wireframe)
        mode = GL_LINE_LOOP;
    else mode = GL_QUADS;

    glPushMatrix();
    glScalef(width/2,height/2,length/2);
    if (texture){
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D,Texture[whichtexture]);
    }
    else{
        glDisable(GL_TEXTURE_2D);
    }
    glBegin(mode);
    //face 1
    glNormal3i(-1, 1,-1);
    glTexCoord2i(1,1);
    glVertex3i(-1, 1,-1);
    glNormal3i( 1, 1,-1);
    glTexCoord2i(0,1);
    glVertex3i( 1, 1,-1);
    glNormal3i( 1,-1,-1);
    glTexCoord2i(0,0);
    glVertex3i( 1,-1,-1);
    glNormal3i(-1,-1,-1);
    glTexCoord2i(1,0);
    glVertex3i(-1,-1,-1);
    //face 2
    glNormal3i(-1,-1,-1);
    glTexCoord2i(0,1);
    glVertex3i(-1,-1,-1);
    glNormal3i( 1,-1,-1);
    glTexCoord2i(1,1);
    glVertex3i( 1,-1,-1);
    glNormal3i( 1,-1, 1);
    glTexCoord2i(1,0);
    glVertex3i( 1,-1, 1);
    glNormal3i(-1,-1, 1);
    glTexCoord2i(0,0);
    glVertex3i(-1,-1, 1);
    // face 3
    glNormal3i( 1,-1, 1);
    glTexCoord2i(0,1);
    glVertex3i( 1,-1, 1);
    glNormal3i( 1,-1,-1);
    glTexCoord2i(1,1);
    glVertex3i( 1,-1,-1);
    glNormal3i( 1, 1,-1);
    glTexCoord2i(1,0);
    glVertex3i( 1, 1,-1);
    glNormal3i( 1, 1, 1);
    glTexCoord2i(0,0);
    glVertex3i( 1, 1, 1);
    //face 4
    glNormal3i( 1, 1,-1);
    glTexCoord2i(0,1);
    glVertex3i( 1, 1,-1);
    glNormal3i(-1, 1,-1);
    glTexCoord2i(1,1);
    glVertex3i(-1, 1,-1);
    glNormal3i(-1, 1, 1);
    glTexCoord2i(1,0);
    glVertex3i(-1, 1, 1);
    glNormal3i( 1, 1, 1);
    glTexCoord2i(0,0);
    glVertex3i( 1, 1, 1);
    //face 5
    glNormal3i(-1, 1, 1);
    glTexCoord2i(0,1);
    glVertex3i(-1, 1, 1);
    glNormal3i(-1, 1,-1);
    glTexCoord2i(1,1);
    glVertex3i(-1, 1,-1);
    glNormal3i(-1,-1,-1);
    glTexCoord2i(1,0);
    glVertex3i(-1,-1,-1);
    glNormal3i(-1,-1, 1);
    glTexCoord2i(0,0);
    glVertex3i(-1,-1, 1);
    //face 6
    glNormal3i( 1,-1, 1);
    glTexCoord2i(0,1);
    glVertex3i( 1,-1, 1);
    glNormal3i( 1, 1, 1);
    glTexCoord2i(1,1);
    glVertex3i( 1, 1, 1);
    glNormal3i(-1, 1, 1);
    glTexCoord2i(1,0);
    glVertex3i(-1, 1, 1);
    glNormal3i(-1,-1, 1);
    glTexCoord2i(0,0);
    glVertex3i(-1,-1, 1);
    glEnd();
    glPopMatrix();

    glDisable(GL_TEXTURE_2D);
    //glDisable( GL_BLEND );
}

void DrawSphere(float radius, bool wireframe, bool texture, int whichtexture){
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);//makes the texture
    sphere = gluNewQuadric();
    if (texture)
    {
        glEnable(GL_TEXTURE_2D);
        gluQuadricTexture(sphere, true);
        glBindTexture(GL_TEXTURE_2D, Texture[whichtexture]);
    }
    else
        glDisable(GL_TEXTURE_2D);

    if (wireframe)
    {
        gluQuadricDrawStyle(sphere, GLU_LINE);
    }
    else
        gluQuadricDrawStyle(sphere, GLU_FILL);

    gluSphere(sphere, radius, 20,20);

    glDisable(GL_TEXTURE_2D);
    gluDeleteQuadric(sphere);
}
void DrawCylinder(float radius, float length, bool wireframe, bool texture, int whichtexture){
    //glEnable( GL_BLEND );
    //glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);//blends the texture with the color of the object

    cylinder = gluNewQuadric();
    cap = gluNewQuadric();

    if (texture)
    {
        glEnable(GL_TEXTURE_2D);
        gluQuadricTexture(cylinder, true);
        gluQuadricTexture(cap, true);
        glBindTexture(GL_TEXTURE_2D, Texture[whichtexture]);
    }
    else
        glDisable(GL_TEXTURE_2D);

    if (wireframe)
    {
        glTranslatef(0,0,-length/2);
        gluQuadricDrawStyle(cylinder, GLU_LINE);
    }
    else
        glTranslatef(0,0,-length/2);

    gluQuadricDrawStyle(cylinder, GLU_FILL);
    gluCylinder(cylinder,radius,radius,length,20,3);

    glPushMatrix();
    glRotatef(180.0,1,0,0);
    gluDisk(cap,0,radius,20,10);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0,0,length);
    gluDisk(cap,0,radius,20,10);
    glPopMatrix();

    //glPushMatrix();
    //glLoadIdentity();
    glTranslatef(0,0,length/2);
    //glPopMatrix();
    glDisable(GL_TEXTURE_2D);
    gluDeleteQuadric(cylinder);
    gluDeleteQuadric(cap);
}
void LDEsetM(const dReal *pos,const dReal *R){
    float local_matrix[16];
    local_matrix[0]     =     R[0];
    local_matrix[1]     =     R[4];
    local_matrix[2]     =     R[8];
    local_matrix[3]     =     0;
    local_matrix[4]     =     R[1];
    local_matrix[5]     =     R[5];
    local_matrix[6]     =     R[9];
    local_matrix[7]     =     0;
    local_matrix[8]     =     R[2];
    local_matrix[9]     =     R[6];
    local_matrix[10]    =     R[10];
    local_matrix[11]    =     0;
    local_matrix[12]    =     pos[0];
    local_matrix[13]    =     pos[1];
    local_matrix[14]    =     pos[2];
    local_matrix[15]    =     1;
    glMultMatrixf(local_matrix);
}

GLuint LoadTextureRAW( const char * filename, int wrap )
{
    glEnable( GL_TEXTURE_2D );
    GLuint texture;
    int width, height;
    unsigned char * data;
    FILE * file;

    // allocate buffer
    width = 512;
    height = 512;
    data = (unsigned char*)malloc( width * height * 3 );

    // open texture data
    file = fopen( filename, "rb" );
    if ( file == NULL ) return 0;

    // read texture data
    int ret;
    ret = fread( data, width * height * 3, 1, file );
    fclose( file );

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);

    // free buffer
    free( data );
    glDisable( GL_TEXTURE_2D );
    return texture;
    //return 0;
}

void DrawVideo(VideoTexture *video) {
    // give opportunity to replace textures with video
    video->apply( Texture );
}

void DrawX (dTriMeshX trim, int whichtexture){

    glEnable(GL_TEXTURE_2D);
    glDisable( GL_CULL_FACE );

    float color[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);

    glBindTexture(GL_TEXTURE_2D, Texture[whichtexture]);

    glBegin(GL_TRIANGLES);
    for (int i=0; i<(int) (trim->IndexCount); )
    {
          // vertices: 3d vertices (no_vertices * 3)
          // indices: no_polygons * 3
          // meshcoord: no_polygons * 3 * 2

          // Coordinates of the first u,v texture
        glNormal3f( trim->NormCoord[trim->Indices[i] * 3 + 0], trim->NormCoord[trim->Indices[i] * 3 + 1], trim->NormCoord[trim->Indices[i] * 3 + 2 ]);//Normals
        glTexCoord2f( trim->MeshCoord[trim->Indices[i] * 2 + 0 ] , trim->MeshCoord[ trim->Indices[i] * 2 + 1] );
        // Coordinates of the first vertex
        glVertex3f( trim->Vertices[trim->Indices[i] * 3 + 0],   trim->Vertices[trim->Indices[i] * 3 + 1],  trim->Vertices[trim->Indices[i] * 3 + 2] );//Vertex definition
        i++;

        // Coordinates of the first u,v texture
        glNormal3f( trim->NormCoord[trim->Indices[i] * 3 + 0], trim->NormCoord[trim->Indices[i] * 3 + 1], trim->NormCoord[trim->Indices[i] * 3 + 2 ]);
        glTexCoord2f( trim->MeshCoord[trim->Indices[i] * 2 + 0 ] , trim->MeshCoord[ trim->Indices[i] * 2 + 1] );
        // Coordinates of the second vertex
        glVertex3f( trim->Vertices[trim->Indices[i] * 3 + 0],   trim->Vertices[trim->Indices[i] * 3 + 1],  trim->Vertices[trim->Indices[i] * 3 + 2] );//Vertex definition
        i++;

        // Coordinates of the first u,v texture
        glNormal3f( trim->NormCoord[trim->Indices[i] * 3 + 0], trim->NormCoord[trim->Indices[i] * 3 + 1], trim->NormCoord[trim->Indices[i] * 3 + 2 ]);
        glTexCoord2f( trim->MeshCoord[trim->Indices[i] * 2 + 0 ] , trim->MeshCoord[ trim->Indices[i] * 2 + 1] );
        // Coordinates of the Third vertex
        glVertex3f( trim->Vertices[trim->Indices[i] * 3 + 0],   trim->Vertices[trim->Indices[i] * 3 + 1],  trim->Vertices[trim->Indices[i] * 3 + 2] );//Vertex definition
        i++;
    }
    glEnd();
    glEnable( GL_CULL_FACE );
    glDisable(GL_TEXTURE_2D);
}

void setupTexture(char *filename, int whichtexture){

    Texture[whichtexture] = LoadBitmapTERMINAL(filename,  whichtexture);
}

int LoadBitmapTERMINAL(char *filename, int whichtexture)
{
    //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
    //glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
    // The next commands sets the texture parameters
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); // If the u,v coordinates overflow the range 0,1 the image is repeated
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // The magnification function ("linear" produces better results)
    //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST); //The minifying function
    glEnable(GL_TEXTURE_2D);

    //GLuint texture;
    FILE * file1;
    char temp1;
    long i;
    SIMBITMAPINFOHEADER infoheader1;
    //yDebug() << " RENDERING NUMBER TEXTURE " << whichtexture;
    yDebug()  << "Loading texture " <<  " '" << filename << "' ";
    num_texture++; // The counter of the current texture is increased
    if( (file1 = fopen(filename, "rb"))==NULL){ yError() << "Cannot load/find texture file "; return (0); }// Open the file for reading
    yDebug() << "......... OK! ";
    fseek(file1, 18, SEEK_CUR);  // start reading width & height

    size_t ret=0;
    ret=fread(&infoheader1.biWidth, sizeof(int), 1, file1);

    if (ret!=1)
        return 0;

    ret=fread(&infoheader1.biHeight, sizeof(int), 1, file1);
    if (ret!=1)
        return 0;

    ret=fread(&infoheader1.biPlanes, sizeof(short int), 1, file1);

    if (ret!=1)
        return 0;

    yDebug() << "Texture Size " <<  infoheader1.biHeight << " " << infoheader1.biWidth;
    if (infoheader1.biPlanes != 1) {
        yDebug("Planes from %s is not 1: %u\n", filename, infoheader1.biPlanes);
        return 0;
    }
    // read the bpp
    ret=fread(&infoheader1.biBitCount, sizeof(unsigned short int), 1, file1);

    if (ret!=1)
        return 0;

    if (infoheader1.biBitCount != 24) {
      yDebug("Bpp from %s is not 24: %d\n", filename, infoheader1.biBitCount);
      return 0;
    }
    fseek(file1, 24, SEEK_CUR);

    // read the data.
    infoheader1.data = (char *) malloc(infoheader1.biWidth * infoheader1.biHeight * 6);
    if (infoheader1.data == NULL) {
        yError("Error allocating memory for color-corrected image data\n");
        return 0;
    }

    if ((i = fread(infoheader1.data, infoheader1.biWidth * infoheader1.biHeight * 3, 1, file1)) != 1) {
        yError("Error reading image data from %s.\n", filename);
        return 0;
    }

    for (i=0; i<(infoheader1.biWidth * infoheader1.biHeight * 3); i+=3) { // reverse all of the colors. (bgr -> rgb)
        temp1 = infoheader1.data[i];
        infoheader1.data[i] = infoheader1.data[i+2];
        infoheader1.data[i+2] = temp1;
    }
    fclose(file1); // Closes the file stream

    //glGenTextures(1, &texture);
   glBindTexture(GL_TEXTURE_2D, whichtexture);// Bind the ID texture specified by the 2nd parameter

    yInfo() << "Finished Binding texture ";
    yInfo() << "Finished Setting parameters ";
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); // We don't combine the color with the original surface color, use only the texture map.
    yInfo() << "Finished Setting glTexEnvf ";
    // Finally we define the 2d texture
    glTexImage2D(GL_TEXTURE_2D, 0, 3, infoheader1.biWidth, infoheader1.biHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, infoheader1.data);
    yInfo() << "Finished Setting glTexImage2D ";

    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, infoheader1.biWidth, infoheader1.biHeight, GL_RGB, GL_UNSIGNED_BYTE, infoheader1.data);

    yInfo() << "Finished Setting gluBuild2DMipmaps ";
    free(infoheader1.data); // Free the memory we used to load the texture

    yInfo() << "Finished creating 3D Model................\n";
    //glDisable(GL_TEXTURE_2D);
    return ( whichtexture ); // Returns the current texture OpenGL ID
}
