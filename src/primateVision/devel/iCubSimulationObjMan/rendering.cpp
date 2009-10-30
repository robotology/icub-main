// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \file rendering.cpp
 * \brief This file deals with setting up the OpenGL and the rendering of objects in the environment 
 * \author Vadim Tikhanoff
 * \date 2008
 * \note Release under GNU GPL v2.0
 **/

#ifdef WIN32
#include <windows.h>
#endif
#include "SDL.h"
#include "SDL_opengl.h"
#include "rendering.h"
#pragma warning(disable:4244 4305)  //for VC++, no precision loss complaints
#include "SimConfig.h"
#include "VideoTexture.h"

yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > objManPort;
yarp::sig::ImageOf<yarp::sig::PixelRgb> objManCache, objManInput;

using namespace yarp::os;
using namespace yarp::sig;

#include <string>
using namespace std;

const GLfloat light_ambient[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
const GLfloat light_diffuse[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_position[] = { 0.0f, 5.0f, 5.0f, 0.0f };

GLenum mode;
GLuint nicetexture;

unsigned int Texture[200];

GLUquadricObj *sphere;
GLUquadricObj *cylinder;
GLUquadricObj *cap;

double objManlastData;
int inc;

static GLuint FindTextureRAW(const char *str, bool flag) {
    //SimConfig finder;

    return LoadTextureRAW(str,flag);
}

//Function to setup OpenGl
void setup_opengl(){
    glShadeModel( GL_SMOOTH );
    /* Culling. */
    glCullFace( GL_BACK );
    glFrontFace( GL_CCW );
    glEnable( GL_CULL_FACE );
	glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_LESS);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);               // OpenGL light
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
	
    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	/// Some OpenGL settings
    GLfloat light_color[] = {1,1,1,1};  // colour
    glMaterialfv(GL_FRONT,GL_SHININESS,light_color); // colour
    glClearColor(0.0,0.0,0.0,0); // background color

	/*	floor data/texture/crate.raw
body1 data/texture/metal2.raw
body2 data/texture/brushed-metal.raw
skybox_up data/texture/face.raw
skybox_front data/texture/skybox/ft.raw
skybox_back data/texture/skybox/bk.raw
skybox_left data/texture/skybox/lt.raw
skybox_right data/texture/skybox/rt.raw
face data/texture/face.raw
*/
	SimConfig finder;
	ConstString floor = finder.find("floor");
	Texture[1] = LoadTextureRAW( floor.c_str(), false );

	for (int i = 50; i < 200; i++){
    	Texture[i] = LoadTextureRAW( floor.c_str(), false );
	}
	ConstString body1 = finder.find("body1");
	Texture[6] = LoadTextureRAW( body1.c_str(), false );
	
	ConstString body2 = finder.find("body2");
	Texture[8] = LoadTextureRAW( body2.c_str(), false );
	
	ConstString skybox_ft = finder.find("skybox_ft");
	Texture[10] = LoadTextureRAW( skybox_ft.c_str(), false );
	
	ConstString skybox_bk = finder.find("skybox_bk");
	Texture[11] = LoadTextureRAW( skybox_bk.c_str(), false );

	ConstString skybox_lt = finder.find("skybox_lt");
	Texture[12] = LoadTextureRAW( skybox_lt.c_str(), false );

	ConstString skybox_rt = finder.find("skybox_rt");
	Texture[13] = LoadTextureRAW( skybox_rt.c_str(), false );

	ConstString skybox_up = finder.find("skybox_up");
	Texture[14] = LoadTextureRAW( skybox_up.c_str(), false );
	
	ConstString face = finder.find("face");
	Texture[15] = LoadTextureRAW( face.c_str(), false );

	if (!Texture[1]){
		printf("No texture loaded\n");
		exit(1);
	}
     objManlastData = -1000;
     inc = 0;
}

void swapTextures(int oldText, int newText){

     Texture[newText] = Texture[oldText];
}

void DrawGround(bool wireframe){  
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
    glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
glRotatef(180,0,0,1);
 // Draw Front side
 glBindTexture(GL_TEXTURE_2D, Texture[13]);
 glBegin(GL_QUADS); 
  glTexCoord2f(1.0f, 0.0f); glVertex3f(x,    y,  z+length);
  glTexCoord2f(1.0f, 1.0f); glVertex3f(x,    y+height, z+length);
  glTexCoord2f(0.0f, 1.0f); glVertex3f(x+width, y+height, z+length); 
  glTexCoord2f(0.0f, 0.0f); glVertex3f(x+width, y,  z+length);
 glEnd();

 // Draw Back side
 glBindTexture(GL_TEXTURE_2D,  Texture[12]);
 glBegin(GL_QUADS);  
  glTexCoord2f(1.0f, 0.0f); glVertex3f(x+width, y,  z);
  glTexCoord2f(1.0f, 1.0f); glVertex3f(x+width, y+height, z); 
  glTexCoord2f(0.0f, 1.0f); glVertex3f(x,    y+height, z);
  glTexCoord2f(0.0f, 0.0f); glVertex3f(x,    y,  z);
 glEnd();

 // Draw Left side
 glBindTexture(GL_TEXTURE_2D,  Texture[11]);
 glBegin(GL_QUADS);  
  glTexCoord2f(1.0f, 1.0f); glVertex3f(x,    y+height, z); 
  glTexCoord2f(0.0f, 1.0f); glVertex3f(x,    y+height, z+length); 
  glTexCoord2f(0.0f, 0.0f); glVertex3f(x,    y,  z+length);
  glTexCoord2f(1.0f, 0.0f); glVertex3f(x,    y,  z);  
 glEnd();

 // Draw Right side
 glBindTexture(GL_TEXTURE_2D,  Texture[10]);
 glBegin(GL_QUADS);  
  glTexCoord2f(0.0f, 0.0f); glVertex3f(x+width, y,  z);
  glTexCoord2f(1.0f, 0.0f); glVertex3f(x+width, y,  z+length);
  glTexCoord2f(1.0f, 1.0f); glVertex3f(x+width, y+height, z+length); 
  glTexCoord2f(0.0f, 1.0f); glVertex3f(x+width, y+height, z);
 glEnd();

 // Draw Up side
 glBindTexture(GL_TEXTURE_2D,  Texture[14]);
 glBegin(GL_QUADS);  
  glTexCoord2f(0.0f, 0.0f); glVertex3f(x+width, y+height, z);
  glTexCoord2f(1.0f, 0.0f); glVertex3f(x+width, y+height, z+length); 
  glTexCoord2f(1.0f, 1.0f); glVertex3f(x,    y+height, z+length);
  glTexCoord2f(0.0f, 1.0f); glVertex3f(x,    y+height, z);
 glEnd();

 // Draw Down side
 glBindTexture(GL_TEXTURE_2D,  Texture[14]);
 glBegin(GL_QUADS);  
  glTexCoord2f(0.0f, 0.0f); glVertex3f(x,    y,  z);
  glTexCoord2f(1.0f, 0.0f); glVertex3f(x,    y,  z+length);
  glTexCoord2f(1.0f, 1.0f); glVertex3f(x+width, y,  z+length); 
  glTexCoord2f(0.0f, 1.0f); glVertex3f(x+width, y,  z);
 glEnd();
}

void DrawBox(float width, float height, float length, bool wireframe, bool texture, int whichtexture){
	if (whichtexture == 2){
		glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	}else{
		glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);//blends the texture with the color of the object
	}
	if (wireframe) mode = GL_LINE_LOOP;
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
    /*glBegin(mode);
    //face 1
    glNormal3i(-1, 1,-1);
    glTexCoord2i(0,1);
    glVertex3i(-1, 1,-1);
    glNormal3i( 1, 1,-1);
    glTexCoord2i(1,1);
    glVertex3i( 1, 1,-1);
    glNormal3i( 1,-1,-1);
    glTexCoord2i(1,0);
    glVertex3i( 1,-1,-1);
    glNormal3i(-1,-1,-1);
    glTexCoord2i(0,0);
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
    glEnd();*/

	glBegin(GL_QUADS);
		// Front Face
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
		// Back Face
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
		// Top Face
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
		// Bottom Face
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		// Right face
		glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		// Left Face
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
	glEnd();

	glPopMatrix();

	glDisable(GL_TEXTURE_2D);
}

void DrawQuad(float width, float height, float length, int texture){

    glPushMatrix();
    glScalef( width/2, height/2, length/2);
    glEnable( GL_BLEND );
    glEnable(GL_TEXTURE_2D);
	glBlendFunc(GL_DST_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindTexture(GL_TEXTURE_2D,Texture[texture]); 

    glBegin(GL_QUADS);
		// Front Face
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
		// Back Face
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
		// Top Face
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
		// Bottom Face
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		// Right face
		glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
		glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		// Left Face
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
	glEnd();
    
    glDisable( GL_BLEND ); 
    glDisable(GL_TEXTURE_2D);
    glPopMatrix();
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

    return texture;
	//return 0;
}

void DrawVideo(VideoTexture *video) {
    // give opportunity to replace textures with video
#if 0
//    static bool first = true;
 //   static VideoTexture video;
 //   if (first) {
 //       Property options;
		
		SimConfig finder;
		ConstString videoconf = finder.find("video");
        
		options.fromConfigFile(videoconf.c_str());

        Bottle textures = options.findGroup("textures").tail();
        for (int i=0; i<textures.size(); i++) {
            ConstString name = textures.get(i).asString();
            printf("Adding video texture %s\n", name.c_str());
            video.add(options.findGroup(name.c_str()));
		}
        first = false;
	}
#endif
	video->apply(Texture);
}

void DrawObjManTextures(int texture){

	printf( " TEXTURE asked to draw  %d \n ",  texture);

	if (objManCache.width()!=SQUARE_DIMs || objManCache.height()!=SQUARE_DIMs) {
        objManCache.setQuantum(1);
        objManCache.setTopIsLowIndex(false);
        objManCache.resize(SQUARE_DIMs,SQUARE_DIMs);
        objManCache.zero();
    }
    ImageOf<PixelRgb> *img = objManPort.read(false);
    unsigned char *imgData = (unsigned char*)objManCache.getRawImage();
    int imgWidth = SQUARE_DIMs;
    int imgHeight = SQUARE_DIMs;
    double time = Time::now();
    bool haveUpdates = false;
    if (img!=NULL) {
        objManCache.copy(*img,objManCache.width(),objManCache.height());
        objManlastData = time;
        haveUpdates = true;
    }
    if (time-objManlastData>2000) {
        for (int x=0; x<imgWidth; x++) {
            for (int y=0; y<imgHeight; y++) {
                unsigned char *pix1 = imgData + (y*imgHeight+x)*3;
                unsigned char *pix2 = pix1+1;
                unsigned char *pix3 = pix1+2;
                *pix1 = (x%25+inc)*10;;
                *pix2 = 255;
                *pix3 = (y*25+inc/2)*10;
            }
        }
        inc++;
        haveUpdates = true;
    }
	if (haveUpdates && objManPort.getInputCount()>0) {
		printf("assigning texture Texture[%d]\n",texture );
        glBindTexture(GL_TEXTURE_2D, Texture[texture]);
        gluBuild2DMipmaps(GL_TEXTURE_2D, 3, imgWidth, imgHeight,
                          GL_RGB, GL_UNSIGNED_BYTE, imgData);
    }
}


void DrawObjManTexturesPort(int texture, int imgWidth, int imgHeight, unsigned char *imgData_, int pin){
	
		//temp image of same size as input:
		ImageOf<PixelRgba> img;
        img.resize(imgWidth,imgHeight);
		img.zero();
		//get a pointer to the temp data:
		unsigned char *pimg = (unsigned char*)img.getRawImage();
		//populate temp://we have y only //convert to RGB:
		for (int x=0; x<imgWidth; x++) {
       		for (int y=0; y<imgHeight; y++) {
				pimg[y*imgWidth*4 + x*4  ] = imgData_[y*pin+x]; //R
           		pimg[y*imgWidth*4 + x*4+1] = imgData_[y*pin+x]; //G
           		pimg[y*imgWidth*4 + x*4+2] = imgData_[y*pin+x]; //B
                if (imgData_[y*pin+x]>0){
                    pimg[y*imgWidth*4 + x*4+3] = 255; // ALPHA
                }
                else{
                    pimg[y*imgWidth*4 + x*4+3] = 0; // ALPHA
                }
			}
		}	
        

        glBindTexture(GL_TEXTURE_2D, Texture[texture]);
        gluBuild2DMipmaps(GL_TEXTURE_2D, 4, imgWidth, imgHeight,
                          GL_RGBA, GL_UNSIGNED_BYTE, pimg);
        
}

