/*
 * visionobj.h
 */

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Based on:
 *
 *   Qavimator
 *   Copyright (C) 2006 by Zi Ree   *
 *   Zi Ree @ SecondLife   *
 *   Released under the terms of the GNU GPL v2.0.
 */

#ifndef VISIONOBJ_H
#define VISIONOBJ_H

#include <qstring.h>

#ifdef __APPLE__
#include <OpenGL/glu.h>
#include <OpenGL/glut.h>  //  Include GLUT, OpenGL, and GLU libraries
#else
#include <GL/glu.h>
#include <GL/glut.h>  //  Include GLUT, OpenGL, and GLU libraries
#endif

#include <stdio.h>    //  Standard Input\Output C Library
#include <stdarg.h>   //  To use functions with variables arguments
#include <stdlib.h>   //  for malloc

//GLvoid *font_style = GLUT_BITMAP_TIMES_ROMAN_24;

class VisionObj
{
public:
    VisionObj(std::string name,
              double dimx,double dimy,double dimz,
              double posx,double posy,double posz,
              double rotx,double roty,double rotz,
              int r,int g,int b,double alpha)
    {   
        nTexID=0;
        bTextured=false;

        mName=name;
       
        set(dimx,dimy,dimz,posx,posy,posz,rotx,roty,rotz,r,g,b,alpha);

        //mEllipsoid=gluNewQuadric();

        //glClearColor(0.0,0.0,0.0,0.0);
    }

    bool operator==(std::string &name)
    {
        return mName==name;
    }

    ~VisionObj()
    {
        //if (mEllipsoid!=NULL) gluDeleteQuadric(mEllipsoid);                   
    }
    
    void set(double dimx,double dimy,double dimz,
             double posx,double posy,double posz,
             double rotx,double roty,double rotz,
             int r,int g,int b,double alpha)
    {
        mDimx=0.5*dimx; mDimy=0.5*dimy; mDimz=0.5*dimz;
        mPosx=posx; mPosy=posy; mPosz=posz;
        mRotx=rotx; mRoty=roty; mRotz=rotz;
        mR=double(r)/255.0; mG=double(g)/255.0; mB=double(b)/255.0;
        mAlpha=alpha;
    }

    void Draw()
    {
        glPushMatrix();

        glTranslated(mPosy,mPosz,mPosx);
        printw(0.0,0.0,1.2*mDimz,mName.c_str());

        glRotated(mRotz,0.0,1.0,0.0);
        glRotated(mRoty,1.0,0.0,0.0);
        glRotated(mRotx,0.0,0.0,1.0);

        //glColor4d(mR,mG,mB,1.0);
        glColor4d(mR,mG,mB,mAlpha);
        glScaled(mDimy,mDimz,mDimx);
        //gluSphere(mEllipsoid,1.0,16,16);
        
        static GLfloat v0[3]={ 1.0, 1.0, 1.0};
        static GLfloat v1[3]={ 1.0,-1.0, 1.0};
        static GLfloat v2[3]={ 1.0,-1.0,-1.0};
        static GLfloat v3[3]={ 1.0, 1.0,-1.0};

        static GLfloat v4[3]={-1.0, 1.0, 1.0};
        static GLfloat v5[3]={-1.0,-1.0, 1.0};
        static GLfloat v6[3]={-1.0,-1.0,-1.0};
        static GLfloat v7[3]={-1.0, 1.0,-1.0};

        if (bTextured)
        {
            //glClearColor(0.0f,0.0f,0.0f,0.0f);
            //glClear(GL_COLOR_BUFFER_BIT);
            
            // setup texture mapping
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D,nTexID);

            glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameterf (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, 0 ? GL_MODULATE : GL_DECAL);

            glBegin(GL_QUADS);
            glTexCoord2f(0.0,0.0);
            glVertex3f(1.0,-1.0,-1.0);
            //glVertex3fv(v2);
            glTexCoord2f(1.0,0.0);
            //glVertex3fv(v3);
            glVertex3f(1.0, 1.0,-1.0);
            glTexCoord2f(1.0,1.0);
            //glVertex3fv(v0);
            glVertex3f(1.0, 1.0, 1.0);
            glTexCoord2f(0.0,1.0);
            //glVertex3fv(v1);
            glVertex3f(1.0,-1.0, 1.0);

            /*
            glTexCoord2f(0.0,1.0);
            glVertex3fv(v0);
            glTexCoord2f(0.0,0.0);
            glVertex3fv(v3);
            glTexCoord2f(1.0,0.0);
            glVertex3fv(v7);
            glTexCoord2f(1.0,1.0);
            glVertex3fv(v4);

            glTexCoord2f(1.0,0.0);
            glVertex3fv(v0);
            glTexCoord2f(1.0,1.0);
            glVertex3fv(v4);
            glTexCoord2f(0.0,1.0);
            glVertex3fv(v5);
            glTexCoord2f(0.0,0.0);
            glVertex3fv(v1);

            glTexCoord2f(1.0,1.0);
            glVertex3fv(v1);
            glTexCoord2f(0.0,1.0);
            glVertex3fv(v5);
            glTexCoord2f(0.0,0.0);
            glVertex3fv(v6);
            glTexCoord2f(1.0,0.0);
            glVertex3fv(v2);        
        
            glTexCoord2f(0.0,1.0);
            glVertex3fv(v2);
            glTexCoord2f(0.0,0.0);
            glVertex3fv(v6);
            glTexCoord2f(1.0,0.0);
            glVertex3fv(v7);
            glTexCoord2f(1.0,1.0);
            glVertex3fv(v3);

            glTexCoord2f(0.0,0.0);
            glVertex3fv(v7);
            glTexCoord2f(1.0,0.0);
            glVertex3fv(v6);
            glTexCoord2f(1.0,1.0);
            glVertex3fv(v5);
            glTexCoord2f(0.0,1.0);
            glVertex3fv(v4); 
            */
            glEnd();
            
            //glDisable(GL_TEXTURE_2D);
        }
        else
        {
            glBegin(GL_QUADS);
            glVertex3fv(v0);
            glVertex3fv(v1);
            glVertex3fv(v2);
            glVertex3fv(v3);
            
            glVertex3fv(v0);
            glVertex3fv(v3);
            glVertex3fv(v7);
            glVertex3fv(v4);

            glVertex3fv(v0);
            glVertex3fv(v4);
            glVertex3fv(v5);
            glVertex3fv(v1);

            glVertex3fv(v1);
            glVertex3fv(v5);
            glVertex3fv(v6);
            glVertex3fv(v2);        
        
            glVertex3fv(v2);
            glVertex3fv(v6);
            glVertex3fv(v7);
            glVertex3fv(v3);

            glVertex3fv(v7);
            glVertex3fv(v6);
            glVertex3fv(v5);
            glVertex3fv(v4); 
            
            glEnd();
        }

        glPopMatrix();
    }

    //-------------------------------------------------------------------------
    //  Draws a string at the specified coordinates.
    //-------------------------------------------------------------------------
    void printw(float x, float y, float z,const char* text)
    {
        //glClear (GL_COLOR_BUFFER_BIT);

	    //  Specify the raster position for pixel operations.
	    glRasterPos3f (x, y, z);
	 
	    //  Draw the characters one by one
	    for (int i=0; text[i]!='\0'; ++i)
        {
	        glutBitmapCharacter(GLUT_BITMAP_9_BY_15,text[i]);
        }

        //glFlush();
	}

    GLuint nTexID;
    bool bTextured;
    
protected:
    std::string mName;
    double mDimx,mDimy,mDimz;
    double mPosx,mPosy,mPosz;
    double mRotx,mRoty,mRotz;
    double mR,mG,mB,mAlpha;

    //GLUquadricObj *mEllipsoid;
};

#endif
