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
#include <yarp/sig/Vector.h>
#include <iCub/skinDynLib/dynContactList.h>

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

class ObjectsManager
{
public:
    ObjectsManager(const char *objPortName,const char *texPortName,const char *forcePortName)
    {
        bReset=false;

        mObjPort.open(objPortName);
        mTexPort.open(texPortName);
        mForcePort.open(forcePortName);

        mObjPort.setStrict();
        mTexPort.setStrict();
        mForcePort.setStrict();
    }

    ~ObjectsManager()
    {
        mObjPort.interrupt();
        mObjPort.close();
        
        mTexPort.interrupt();
        mTexPort.close();

        mForcePort.interrupt();
        mForcePort.close();

        for (int i=0; i<(int)mObjects.size(); ++i)
        {
            if (mObjects[i]) delete mObjects[i];
        }

        for (int i=0; i<(int)mTrajectories.size(); ++i)
        {
            if (mTrajectories[i]) delete mTrajectories[i];
        }
    }

    void setAddressBook(BVHNode ***ab)
    {
        mAB=ab;
    }

    inline void manage(yarp::os::Bottle *msg);
    inline void manage(yarp::sig::VectorOf<unsigned char> *img);
    inline void manage(iCub::skinDynLib::dynContactList &forces);

    void draw()
    {
        // objects
        for (yarp::os::Bottle *botObj; botObj=mObjPort.read(false);)
        {
            manage(botObj);
        }

        // textures
        for (yarp::sig::VectorOf<unsigned char> *imgTex; imgTex=mTexPort.read(false);)
        {
            manage(imgTex);
        }

        for (iCub::skinDynLib::dynContactList *forces; forces=mForcePort.read(false);) 
        {        
            manage(*forces);
        }

        if (bReset)
        {
            bReset=false;
            
            for (int i=0; i<(int)mObjects.size(); ++i) delete mObjects[i];
            mObjects.clear();

            for (int i=0; i<(int)mTrajectories.size(); ++i) delete mTrajectories[i];
            mTrajectories.clear();

            return;
        }

        bool bPack=false;
        for (int i=0; i<(int)mObjects.size(); ++i)
        {
            if (mObjects[i]->bDeleted)
            {
                delete mObjects[i];
                mObjects[i]=NULL;
                bPack=true;
            }
            else
            {
                mObjects[i]->draw();
            }
        }

        if (bPack)
        {
            int newsize=0;
            int size=(int)mObjects.size();
            for (int i=0; i<size; ++i)
            {
                if (mObjects[i]!=NULL) mObjects[newsize++]=mObjects[i];
            }
            mObjects.resize(newsize);
        }

        bPack=false;
        for (int i=0; i<(int)mTrajectories.size(); ++i)
        {
            if (mTrajectories[i]->bDeleted)
            {
                delete mTrajectories[i];
                mTrajectories[i]=NULL;
                bPack=true;
            }
            else
            {
                mTrajectories[i]->draw();
            }
        }

        if (bPack)
        {
            int newsize=0;
            int size=(int)mTrajectories.size();
            for (int i=0; i<size; ++i)
            {
                if (mTrajectories[i]!=NULL) mTrajectories[newsize++]=mTrajectories[i];
            }
            mTrajectories.resize(newsize);
        }
    }
    
protected:
    bool bReset;
    std::vector<VisionObj*> mObjects;
    std::vector<TrajectoryObj*> mTrajectories;

    BVHNode ***mAB;

    yarp::os::BufferedPort<yarp::os::Bottle> mObjPort;
    yarp::os::BufferedPort<yarp::sig::VectorOf<unsigned char> > mTexPort;
    yarp::os::BufferedPort<iCub::skinDynLib::dynContactList> mForcePort;
};

void ObjectsManager::manage(yarp::os::Bottle *msg)
{
    yarp::os::ConstString cmd=msg->get(0).asString();

    if (cmd=="reset")
    {
        bReset=true;
        return;
    }

    if (cmd=="delete")
    {
        std::string name(msg->get(1).asString().c_str());
        
        for (int i=0; i<(int)mObjects.size(); ++i)
        {
            if (*mObjects[i]==name) mObjects[i]->bDeleted=true;
        }

        for (int i=0; i<(int)mTrajectories.size(); ++i)
        {
            if (*mTrajectories[i]==name) mTrajectories[i]->bDeleted=true;
        }

        return;
    }
    
    if (cmd=="object")
    {
        std::string name(msg->get(1).asString().c_str());

        double dx=msg->get(2).asDouble();
        double dy=msg->get(3).asDouble();
        double dz=msg->get(4).asDouble();

        double px=msg->get(5).asDouble();
        double py=msg->get(6).asDouble();
        double pz=msg->get(7).asDouble();

        double rx=msg->get(8).asDouble();
        double ry=msg->get(9).asDouble();
        double rz=msg->get(10).asDouble();

        int R=msg->get(11).asInt();
        int G=msg->get(12).asInt();
        int B=msg->get(13).asInt();
        double alpha=msg->get(14).asDouble();

        for (int i=0; i<(int)mObjects.size(); ++i)
        {
            if (*mObjects[i]==name)
            {
                mObjects[i]->set(dx,dy,dz,px,py,pz,rx,ry,rz,R,G,B,alpha);
                return;
            }
        }

        mObjects.push_back(new VisionObj(name,dx,dy,dz,px,py,pz,rx,ry,rz,R,G,B,alpha));
        
        return;
    }
    
    if (cmd=="trajectory")
    {
        std::string name(msg->get(1).asString().c_str());
        std::string label(msg->get(2).asString().c_str());

        int bufflen=msg->get(3).asInt();
        double persistence=msg->get(4).asDouble();

        int R=msg->get(5).asInt();
        int G=msg->get(6).asInt();
        int B=msg->get(7).asInt();

        double alpha=msg->get(8).asDouble();
        GLfloat width=(GLfloat)msg->get(9).asDouble();

        for (int i=0; i<(int)mTrajectories.size(); ++i)
        {
            if (*mTrajectories[i]==name)
            {
                mTrajectories[i]->set(label,bufflen,persistence,R,G,B,alpha,width);
                return;
            }
        }

        mTrajectories.push_back(new TrajectoryObj(name,label,bufflen,persistence,R,G,B,alpha,width));

        return;
    }
    
    if (cmd=="addpoint")
    {
        std::string name(msg->get(1).asString().c_str());

        double x=msg->get(2).asDouble();
        double y=msg->get(3).asDouble();
        double z=msg->get(4).asDouble();

        for (int i=0; i<(int)mTrajectories.size(); ++i)
        {
            if (*mTrajectories[i]==name)
            {
                mTrajectories[i]->update(x,y,z);
                return;
            }
        }

        return;
    }
}

void ObjectsManager::manage(yarp::sig::VectorOf<unsigned char> *img)
{
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
            int p10=p00+1;

            if (x0==xdim-1) p10=p00;

            int p01=p00+xdim;
            int p11=p10+xdim;

            if (y0==ydim-1)
            {
                p01=p00;
                p11=p10;
            }

            double grey=a0*b0*double(texture[p00])+a0*b1*double(texture[p01])+a1*b0*double(texture[p10])+a1*b1*double(texture[p11]);
            if (grey<0.0) grey=0.0; else if (grey>255.0) grey=255.0;

            unsigned char G=(unsigned char)grey;

            buffer[index++]=G;
            buffer[index++]=G;
            buffer[index++]=G;
        }
    }

    for (int i=0; i<(int)mObjects.size(); ++i)
    {
        if (*mObjects[i]==objName)
        {
            mObjects[i]->mW=xdim2;
            mObjects[i]->mH=ydim2;
            mObjects[i]->mTextureBuffer=buffer;
            return;
        }
    }
}

void ObjectsManager::manage(iCub::skinDynLib::dynContactList &forces)
{
    for (int p=0; p<8; ++p)
    {
        for (int l=0; l<8; ++l)
        {
            if (mAB[p][l]) mAB[p][l]->clearArrows();
        }
    }

    for (int i=0; i<(int)forces.size(); ++i)
    {
        yarp::sig::Vector P=forces[i].getCoP();

        double f=forces[i].getForceModule();
        yarp::sig::Vector F=forces[i].getForceDirection();
        yarp::sig::Vector M=forces[i].getMoment();
        
        int p=forces[i].getBodyPart();
        int l=forces[i].getLinkNumber();

        if (mAB[p][l])
        {
            mAB[p][l]->addArrow(new ForceArrow(P[0],P[1],P[2],f,F[0],F[1],F[2],M[0],M[1],M[2]));
        }
    }
}

#endif
