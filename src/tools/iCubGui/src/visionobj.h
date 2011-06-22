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
#include <yarp/os/Time.h>

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

class GuiObj
{
public:
    GuiObj(std::string& name,int r,int g,int b,double alpha)
    {
        mName=name;
        mR=double(r)/255.0; mG=double(g)/255.0; mB=double(b)/255.0;
        mAlpha=alpha;
        bDeleted=false;
    }
    
    virtual ~GuiObj(){}
    virtual void draw()=0;
    
    bool operator==(std::string &name)
    {
        return mName==name;
    }

    bool bDeleted;

protected:
    void printw(float x,float y,float z,const char* text)
    {
        //glClear(GL_COLOR_BUFFER_BIT);
        glRasterPos3f(x,y,z);
        for (int i=0; text[i]!='\0'; ++i)
        {
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15,text[i]);
        }
        //glFlush();
    }

    std::string mName;
    double mR,mG,mB,mAlpha;
};

class TrajectoryObj : public GuiObj
{
public:
    TrajectoryObj(std::string name,std::string label,int bufflen,double persistence,int r,int g,int b,double alpha,GLfloat width)
        : GuiObj(name,r,g,b,alpha)
    {
        mLabel=label;
        mWidth=width;
        mPersistence=persistence;

        mBufflen=bufflen;

        mIndex=0;
        mFull=false;

        mX=new double[mBufflen];
        mY=new double[mBufflen];
        mZ=new double[mBufflen];
        mT=new double[mBufflen];
    }

    void set(std::string& label,int bufflen,double persistence,int r,int g,int b,double alpha,double width)
    {
        mLabel=label;
        mWidth=width;
        mPersistence=persistence;
        mR=double(r)/255.0; mG=double(g)/255.0; mB=double(b)/255.0;
        mAlpha=alpha;

        if (mBufflen==bufflen) return;

        double *tX=new double[bufflen];
        double *tY=new double[bufflen];
        double *tZ=new double[bufflen];
        double *tT=new double[bufflen];

        int n=bufflen<mBufflen?bufflen:mBufflen;

        for (int i=0; i<n; ++n)
        {
            tX[i]=mX[(mIndex+i)%mBufflen];
            tY[i]=mY[(mIndex+i)%mBufflen];
            tZ[i]=mZ[(mIndex+i)%mBufflen];
            tT[i]=mT[(mIndex+i)%mBufflen];
        }

        delete [] mX; mX=tX;
        delete [] mY; mY=tY;
        delete [] mZ; mZ=tZ;
        delete [] mT; mT=tT;

        if (mFull)
        {
            if (bufflen>mBufflen) mFull=false;
        }
        else
        {
            if (bufflen<=mIndex) mFull=true; 
        }

        mIndex=0;

        mBufflen=bufflen;
    }
    
    void update(double x,double y,double z)
    {
        mX[mIndex]=x;
        mY[mIndex]=y;
        mZ[mIndex]=z;
        mT[mIndex]=yarp::os::Time::now();

        if (++mIndex==mBufflen)
        {
            mIndex=0;
            mFull=true;
        }
    }

    void draw()
    {        
        glColor4f(mR,mG,mB,1.0);
        glLineWidth(3.0);
        
        double now=yarp::os::Time::now();

        glBegin(GL_LINE_STRIP);
        if (mFull)
        {
            for (int i=mIndex; i<mBufflen; ++i)
            {
                if (now-mT[i]<mPersistence) glVertex3d(mX[i],mY[i],mZ[i]);
            }
        }
        for (int i=0; i<mIndex; ++i)
        {
            if (now-mT[i]<mPersistence) glVertex3d(mX[i],mY[i],mZ[i]);
        }
        glEnd();

        printw(mX[mIndex-1],mY[mIndex-1],mZ[mIndex-1]+50.0,mLabel.c_str());
    }

    ~TrajectoryObj()
    {
        delete [] mX;
        delete [] mY;
        delete [] mZ;
        delete [] mT;
    }

protected:
    int mIndex;
    bool mFull;
    int mBufflen;
    double mPersistence;
    double *mX;
    double *mY;
    double *mZ;
    double *mT;
    std::string mLabel;
    GLfloat mWidth;
};

class VisionObj : public GuiObj
{
public:
    VisionObj(std::string name,
              double dimx,double dimy,double dimz,
              double posx,double posy,double posz,
              double rotx,double roty,double rotz,
              int r,int g,int b,double alpha)
        : GuiObj(name,r,g,b,alpha)
    {   
        mW=mH=0;
        nTexID=0;
        bTextured=false;
        mTextureBuffer=NULL;
       
        set(dimx,dimy,dimz,posx,posy,posz,rotx,roty,rotz,r,g,b,alpha);
    }

    ~VisionObj()
    {
        if (bTextured) glDeleteTextures(1,&nTexID);
        bTextured=false;               
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

    void draw()
    {
        if (mTextureBuffer!=NULL)
        {
            glEnable(GL_TEXTURE_2D);
            
            if (bTextured) glDeleteTextures(1,&nTexID);
            
            GLuint texture;
            glGenTextures(1,&texture);
            nTexID=texture;
            bTextured=true;

	        glBindTexture(GL_TEXTURE_2D,texture);
	        gluBuild2DMipmaps(GL_TEXTURE_2D,3,mW,mH,GL_RGB,GL_UNSIGNED_BYTE,mTextureBuffer);
	        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
	        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
            glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_DECAL);

            delete [] mTextureBuffer;
            mTextureBuffer=NULL;
            glDisable(GL_TEXTURE_2D);
        }
        
        glPushMatrix();

        glTranslated(mPosx,mPosy,mPosz);
        printw(0.0,0.0,1.2*mDimz,mName.c_str());

        glRotated(mRotz,0.0,0.0,1.0);
        glRotated(mRoty,0.0,1.0,0.0);
        glRotated(mRotx,1.0,0.0,0.0);

        glColor4d(mR,mG,mB,mAlpha);

        if (bTextured)
        {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D,nTexID);

            glBegin(GL_QUADS);

            glTexCoord2f(1.0,0.0); glVertex3f( mDimx, mDimy, mDimz);
            glTexCoord2f(0.0,0.0); glVertex3f( mDimx,-mDimy, mDimz); 
            glTexCoord2f(0.0,1.0); glVertex3f( mDimx,-mDimy,-mDimz);
            glTexCoord2f(1.0,1.0); glVertex3f( mDimx, mDimy,-mDimz);

            glTexCoord2f(0.0,0.0); glVertex3f( mDimx, mDimy, mDimz);
            glTexCoord2f(0.0,1.0); glVertex3f( mDimx, mDimy,-mDimz);
            glTexCoord2f(1.0,1.0); glVertex3f(-mDimx, mDimy,-mDimz);
            glTexCoord2f(1.0,0.0); glVertex3f(-mDimx, mDimy, mDimz);

            glTexCoord2f(1.0,0.0); glVertex3f( mDimx,-mDimy, mDimz);
            glTexCoord2f(0.0,0.0); glVertex3f(-mDimx,-mDimy, mDimz);
            glTexCoord2f(0.0,1.0); glVertex3f(-mDimx,-mDimy,-mDimz);
            glTexCoord2f(1.0,1.0); glVertex3f( mDimx,-mDimy,-mDimz);        
        
            glTexCoord2f(0.0,1.0); glVertex3f(-mDimx, mDimy,-mDimz);
            glTexCoord2f(1.0,1.0); glVertex3f(-mDimx,-mDimy,-mDimz);
            glTexCoord2f(1.0,0.0); glVertex3f(-mDimx,-mDimy, mDimz);
            glTexCoord2f(0.0,0.0); glVertex3f(-mDimx, mDimy, mDimz); 

            glVertex3f( mDimx, mDimy, mDimz);
            glVertex3f(-mDimx, mDimy, mDimz);
            glVertex3f(-mDimx,-mDimy, mDimz);
            glVertex3f( mDimx,-mDimy, mDimz);

            glVertex3f( mDimx,-mDimy,-mDimz);
            glVertex3f(-mDimx,-mDimy,-mDimz);
            glVertex3f(-mDimx, mDimy,-mDimz);
            glVertex3f( mDimx, mDimy,-mDimz);

            glEnd();

            glDisable(GL_TEXTURE_2D);
        }
        else
        {
            glBegin(GL_QUADS);
            glVertex3f( mDimx, mDimy, mDimz);
            glVertex3f( mDimx,-mDimy, mDimz);
            glVertex3f( mDimx,-mDimy,-mDimz);
            glVertex3f( mDimx, mDimy,-mDimz);
            
            glVertex3f( mDimx, mDimy, mDimz);
            glVertex3f( mDimx, mDimy,-mDimz);
            glVertex3f(-mDimx, mDimy,-mDimz);
            glVertex3f(-mDimx, mDimy, mDimz);

            glVertex3f( mDimx, mDimy, mDimz);
            glVertex3f(-mDimx, mDimy, mDimz);
            glVertex3f(-mDimx,-mDimy, mDimz);
            glVertex3f( mDimx,-mDimy, mDimz);

            glVertex3f( mDimx,-mDimy, mDimz);
            glVertex3f(-mDimx,-mDimy, mDimz);
            glVertex3f(-mDimx,-mDimy,-mDimz);
            glVertex3f( mDimx,-mDimy,-mDimz);        
        
            glVertex3f( mDimx,-mDimy,-mDimz);
            glVertex3f(-mDimx,-mDimy,-mDimz);
            glVertex3f(-mDimx, mDimy,-mDimz);
            glVertex3f( mDimx, mDimy,-mDimz);

            glVertex3f(-mDimx, mDimy,-mDimz);
            glVertex3f(-mDimx,-mDimy,-mDimz);
            glVertex3f(-mDimx,-mDimy, mDimz);
            glVertex3f(-mDimx, mDimy, mDimz); 
            
            glEnd();
        }

        glPopMatrix();
    }

    int mW,mH;
    bool bTextured;
    unsigned char* mTextureBuffer;
    
protected:
    GLuint nTexID;
    double mDimx,mDimy,mDimz;
    double mPosx,mPosy,mPosz;
    double mRotx,mRoty,mRotz;
};

#endif
