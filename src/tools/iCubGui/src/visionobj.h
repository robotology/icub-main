/*
 * visionobj.h
 */

#ifndef VISIONOBJ_H
#define VISIONOBJ_H

#include <qstring.h>

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include <stdio.h>    //  Standard Input\Output C Library
#include <stdarg.h>   //  To use functions with variables arguments
#include <stdlib.h>   //  for malloc
#include <gl/glut.h>  //  Include GLUT, OpenGL, and GLU libraries

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
        mName=name;
       
        set(dimx,dimy,dimz,posx,posy,posz,rotx,roty,rotz,r,g,b,alpha);

        mEllipsoid=gluNewQuadric();

        //glClearColor(0.0,0.0,0.0,0.0);
    }

    bool operator==(std::string &name)
    {
        return mName==name;
    }

    ~VisionObj()
    {
        if (mEllipsoid!=NULL) gluDeleteQuadric(mEllipsoid);                   
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
        printw(0.0,1.2*mDimz,0.0,mName.c_str());

        glRotated(mRotz,0.0,1.0,0.0);
        glRotated(mRoty,1.0,0.0,0.0);
        glRotated(mRotx,0.0,0.0,1.0);

        glColor4d(mR,mG,mB,1.0);

        glColor4d(mR,mG,mB,mAlpha);

        glScaled(mDimy,mDimz,mDimx);
        gluSphere(mEllipsoid,1.0,16,16);

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
    
protected:
    std::string mName;
    double mDimx,mDimy,mDimz;
    double mPosx,mPosy,mPosz;
    double mRotx,mRoty,mRotz;
    double mR,mG,mB,mAlpha;

    GLUquadricObj *mEllipsoid;
};

#endif
