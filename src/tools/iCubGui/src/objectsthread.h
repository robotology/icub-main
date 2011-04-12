/*
 * objectsthread.h
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

#ifndef OBJECTSTHREAD_H
#define OBJECTSTHREAD_H

#include <qstring.h>
#include <qthread.h>
#include <vector>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

class ParamsThread;
class TextureThread;
class ObjectsManager;

class ParamsThread : public QThread
{
public:
    ParamsThread(ObjectsManager *objManager,const char *portName)
    {
        mRunning=true;
        mObjManager=objManager;
        mPort.open(portName);
        
        start();
    }

    inline void run();

    ~ParamsThread()
    {
        mRunning=false;
        mPort.interrupt();
        mPort.close();
    }

protected:
    bool mRunning;
    yarp::os::Port mPort;
    ObjectsManager *mObjManager;
};

class TextureThread : public QThread
{
public:
    TextureThread(ObjectsManager *objManager,const char *portName)
    {
        mRunning=true;
        mObjManager=objManager;
        mPort.open(portName);
        
        start();
    }

    inline void run();

    ~TextureThread()
    {
        mRunning=false;
        mPort.interrupt();
        mPort.close();
    }

protected:
    bool mRunning;
    yarp::os::BufferedPort< yarp::sig::VectorOf<unsigned char> > mPort;
    ObjectsManager *mObjManager;
};

class ObjectsManager
{
public:
    ObjectsManager(const char *paramsPortName,const char *texPortName) : mMutex(1)
    {
        paramsThread=new ParamsThread(this,paramsPortName);
        textureThread=new TextureThread(this,texPortName);
        mTextures=0;
    }

    ~ObjectsManager()
    {
        delete paramsThread;
        delete textureThread;

        for (int i=0; i<(int)mObjects.size(); ++i)
        {
            if (mObjects[i]!=NULL) delete mObjects[i];
        }
    }

    inline void manage(yarp::os::Bottle &msg);

    inline void manage(yarp::sig::VectorOf<unsigned char> *img)
    {
        mMutex.wait();

        yarp::sig::VectorOf<unsigned char> &texture=*img;

        int xdim=texture[0];
        int ydim=texture[1];
        std::string objName;

        int c;
        for (c=2; texture[c]; ++c)
        {
            objName+=texture[c];
        }

        int xdim2,ydim2;
        for (xdim2=1; xdim2<xdim; xdim2*=2);
        for (ydim2=1; ydim2<ydim; ydim2*=2);

        //xdim2=256;
        //ydim2=256;

        unsigned char *buffer=new unsigned char[3*xdim2*ydim2];
        int index=0;

        for (int y=0; y<ydim2; ++y)
        {
            double v=double(y)/double(ydim2);
            double oy=v*double(ydim);

            int y0=int(oy);
            double b1=oy-double(y0);
            double b0=1.0-b1;

            for (int x=0; x<xdim2; ++x)
            {
                double u=double(x)/double(xdim2);
                double ox=u*double(xdim);

                int x0=int(ox);
                double a1=ox-double(x0);
                double a0=1.0-a1;

                int p00=c+x0+y0*xdim;
                int p01=p00+xdim;
                int p10=p00+1;
                int p11=p01+1;

                double grey=a0*b0*double(texture[p00])+a0*b1*double(texture[p01])+a1*b0*double(texture[p10])+a1*b1*double(texture[p11]);
                if (grey<0.0) grey=0.0; else if (grey>255.0) grey=255.0;
                unsigned char G=(unsigned char)grey;

                //printf("%d ",G);

                buffer[index++]=G;
                buffer[index++]=G;
                buffer[index++]=G;
                //buffer[index++]=255;
            }
        }

        for (int i=0; i<(int)mObjects.size(); ++i)
        {
            if (*mObjects[i]==objName)
            {
                glEnable(GL_TEXTURE_2D);
                GLuint texture;
                glGenTextures(1,&texture);
                mObjects[i]->nTexID=texture;
                mObjects[i]->bTextured=true;

	            glBindTexture(GL_TEXTURE_2D,texture);
	            gluBuild2DMipmaps(GL_TEXTURE_2D,3,xdim2,ydim2,GL_RGB,GL_UNSIGNED_BYTE,buffer);
	            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
	            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);

                delete [] buffer;

                printf("Added texture %u to %s (%d x %d)\n",mObjects[i]->nTexID,objName.c_str(),xdim2,ydim2);

                mMutex.post();
                return;
            }
        }

        mMutex.post();
    }

    void draw()
    {
        mMutex.wait();

        //glPushMatrix();
        //glScaled(0.075,0.075,0.075);
        //glTranslated(0.0,3.5,0.0);

        for (int i=0; i<(int)mObjects.size(); ++i)
        {
            mObjects[i]->Draw();
        }

        //glPopMatrix();
        mMutex.post();
    }
    
protected:
    int mTextures;
    ParamsThread  *paramsThread;
    TextureThread *textureThread;

    yarp::os::Semaphore mMutex;
    std::vector<VisionObj*> mObjects;
};

void ObjectsManager::manage(yarp::os::Bottle &msg)
    {
        mMutex.wait();

        yarp::os::ConstString cmd=msg.get(0).asString();

        if (cmd=="reset")
        {
            mObjects.clear();
        }
        else if (cmd=="delete")
        {
            std::string name(msg.get(1).asString().c_str());

            for (int i=0; i<(int)mObjects.size(); ++i)
            {
                if ((*mObjects[i])==name)
                {
                    if (mObjects[i]!=NULL) delete mObjects[i];

                    for (int j=i+1; j<(int)mObjects.size(); ++j)
                    {
                        mObjects[j-1]=mObjects[j];
                    }

                    mObjects.resize(mObjects.size()-1);

                    break;
                }
            }
        }
        else if (cmd=="object")
        {
            std::string name(msg.get(1).asString().c_str());

            double dx=msg.get(2).asDouble();
            double dy=msg.get(3).asDouble();
            double dz=msg.get(4).asDouble();

            double px=msg.get(5).asDouble();
            double py=msg.get(6).asDouble();
            double pz=msg.get(7).asDouble();

            double rx=msg.get(8).asDouble();
            double ry=msg.get(9).asDouble();
            double rz=msg.get(10).asDouble();

            int R=msg.get(11).asInt();
            int G=msg.get(12).asInt();
            int B=msg.get(13).asInt();
            double alpha=msg.get(14).asDouble();

            bool found=false;

            for (int i=0; i<(int)mObjects.size(); ++i)
            {
                if (*mObjects[i]==name)
                {
                    found=true;
                    mObjects[i]->set(dx,dy,dz,px,py,pz,rx,ry,rz,R,G,B,alpha);
                }
            }

            if (!found)
            {
                printf("Added object %s\n",name.c_str());
                mObjects.push_back(new VisionObj(name,dx,dy,dz,px,py,pz,rx,ry,rz,R,G,B,alpha));
            }
        }

        mMutex.post();
    }

    void TextureThread::run()
    {
        yarp::sig::VectorOf<unsigned char> *img;
            
        while (mRunning)
        {
            if ((img=mPort.read())!=NULL)
            {
                mObjManager->manage(img);
            }
        }
    }

    void ParamsThread::run()
    {
        yarp::os::Bottle msg;
            
        while (mRunning)
        {
            if (mPort.read(msg))
            {
                mObjManager->manage(msg);
            }
        }
    }

#endif
