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

#include <string>
#include <qstring.h>
#include <qthread.h>
#include <vector>
#include <yarp/os/Log.h>
#include <yarp/sig/Vector.h>
#include <iCub/skinDynLib/skinContactList.h>

#include "bvhnoderoot.h"

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include <string.h>
extern std::string GUI_NAME;

class ObjectsManager
{
public:
    ObjectsManager(const char *objPortName,const char *texPortName,const char *forcePortName)
    {
        mObjPort.open((GUI_NAME+objPortName).c_str());
        mTexPort.open((GUI_NAME+texPortName).c_str());
        mForcePort.open((GUI_NAME+forcePortName).c_str());

        mObjPort.setStrict();
        mTexPort.setStrict();
        mForcePort.setStrict();

        /*
        mPx=mPy=mPz=0.0;
        mRx=mRy=mRz=0.0;
        for (int i=0; i<3; ++i)
            for (int j=0; j<3; ++j)
                R[i][j]=(double)(i==j);
        */
    }

    ~ObjectsManager()
    {
        mObjPort.interrupt();
        mObjPort.close();

        mTexPort.interrupt();
        mTexPort.close();

        mForcePort.interrupt();
        mForcePort.close();

        for (int i=0; i<(int)mObjectsRoot.size(); ++i)
        {
            if (mObjectsRoot[i]) delete mObjectsRoot[i];
        }

        for (int i=0; i<(int)mObjectsWorld.size(); ++i)
        {
            if (mObjectsWorld[i]) delete mObjectsWorld[i];
        }

        for (int i=0; i<(int)mTrajectoriesRoot.size(); ++i)
        {
            if (mTrajectoriesRoot[i]) delete mTrajectoriesRoot[i];
        }

        for (int i=0; i<(int)mTrajectoriesWorld.size(); ++i)
        {
            if (mTrajectoriesWorld[i]) delete mTrajectoriesWorld[i];
        }
    }

    void setAddressBook(BVHNode ***ab)
    {
        mAB=ab;
    }

    inline void manage(yarp::os::Bottle *msg);
    inline void manage(yarp::sig::VectorOf<unsigned char> *img);
    inline void manage(iCub::skinDynLib::skinContactList &forces);

    /*
    void readEncoders(double *enc)
    {
        static const double DEG2RAD=M_PI/180.0;

        mPx=enc[3];
        mPy=enc[4];
        mPz=enc[5];

        mRz=enc[0];
        mRy=enc[1];
        mRx=enc[2];

        double Rz=DEG2RAD*mRz;
        double Ry=DEG2RAD*mRy;
        double Rx=DEG2RAD*mRx;

        double cA=cos(Rz),sA=sin(Rz);
        double cB=cos(Ry),sB=sin(Ry);
        double cY=cos(Rx),sY=sin(Rx);

        R[0][0]=cA*cB; R[0][1]=cA*sB*sY-sA*cY; R[0][2]=cA*sB*cY+sA*sY;
        R[1][0]=sA*cB; R[1][1]=sA*sB*sY+cA*cY; R[1][2]=sA*sB*cY-cA*sY;
        R[2][0]=  -sB; R[2][1]=   cB*sY;       R[2][2]=   cB*cY;
    }
    */

    void update()
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

        for (iCub::skinDynLib::skinContactList *forces; forces=mForcePort.read(false);)
        {
            manage(*forces);
        }
    }

    void drawRootObjects()
    {
        for (int i=0; i<(int)mObjectsRoot.size(); ++i)
        {
            if (mObjectsRoot[i]) mObjectsRoot[i]->draw();
        }

        for (int i=0; i<(int)mTrajectoriesRoot.size(); ++i)
        {
            if(mTrajectoriesRoot[i]) mTrajectoriesRoot[i]->draw();
        }
    }

    void drawWorldObjects()
    {
        for (int i=0; i<(int)mObjectsWorld.size(); ++i)
        {
            if (mObjectsWorld[i]) mObjectsWorld[i]->draw();
        }

        for (int i=0; i<(int)mTrajectoriesWorld.size(); ++i)
        {
            if(mTrajectoriesWorld[i]) mTrajectoriesWorld[i]->draw();
        }
    }

protected:
    std::vector<VisionObj*> mObjectsRoot;
    std::vector<VisionObj*> mObjectsWorld;
    std::vector<TrajectoryObj*> mTrajectoriesRoot;
    std::vector<TrajectoryObj*> mTrajectoriesWorld;

    //double mPx,mPy,mPz;
    //double mRx,mRy,mRz;
    //double R[3][3];

    BVHNode ***mAB;

    yarp::os::BufferedPort<yarp::os::Bottle> mObjPort;
    yarp::os::BufferedPort<yarp::sig::VectorOf<unsigned char> > mTexPort;
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> mForcePort;
};

void ObjectsManager::manage(yarp::os::Bottle *msg)
{
    std::string cmd=msg->get(0).asString();

    if (cmd=="reset")
    {
        for (int i=0; i<(int)mObjectsRoot.size(); ++i) delete mObjectsRoot[i];
        mObjectsRoot.clear();

        for (int i=0; i<(int)mObjectsWorld.size(); ++i) delete mObjectsWorld[i];
        mObjectsWorld.clear();

        for (int i=0; i<(int)mTrajectoriesRoot.size(); ++i) delete mTrajectoriesRoot[i];
        mTrajectoriesRoot.clear();

        for (int i=0; i<(int)mTrajectoriesWorld.size(); ++i) delete mTrajectoriesWorld[i];
        mTrajectoriesWorld.clear();

        return;
    }

    if (cmd=="delete")
    {
        std::string name(msg->get(1).asString().c_str());

        int size=(int)mObjectsRoot.size();
        for (int i=0; i<size; ++i)
        {
            if (mObjectsRoot[i] && *mObjectsRoot[i]==name)
            {
                --size;
                delete mObjectsRoot[i];
                for (int j=i; j<size; ++j) mObjectsRoot[j]=mObjectsRoot[j+1];
                mObjectsRoot.resize(size);
                return;
            }
        }

        size=(int)mObjectsWorld.size();
        for (int i=0; i<(int)mObjectsWorld.size(); ++i)
        {
            if (mObjectsWorld[i] && *mObjectsWorld[i]==name)
            {
                --size;
                delete mObjectsWorld[i];
                for (int j=i; j<size; ++j) mObjectsWorld[j]=mObjectsWorld[j+1];
                mObjectsWorld.resize(size);
                return;
            }
        }

        size=(int)mTrajectoriesRoot.size();
        for (int i=0; i<(int)mTrajectoriesRoot.size(); ++i)
        {
            if (mTrajectoriesRoot[i] && *mTrajectoriesRoot[i]==name)
            {
                --size;
                delete mTrajectoriesRoot[i];
                for (int j=i; j<size; ++j) mTrajectoriesRoot[j]=mTrajectoriesRoot[j+1];
                mTrajectoriesRoot.resize(size);
                return;
            }
        }

        size=(int)mTrajectoriesWorld.size();
        for (int i=0; i<(int)mTrajectoriesWorld.size(); ++i)
        {
            if (mTrajectoriesWorld[i] && *mTrajectoriesWorld[i]==name)
            {
                --size;
                delete mTrajectoriesWorld[i];
                for (int j=i; j<size; ++j) mTrajectoriesWorld[j]=mTrajectoriesWorld[j+1];
                mTrajectoriesWorld.resize(size);
                return;
            }
        }

        return;
    }

    if (cmd=="object" || cmd=="object_with_label")
    {
        int idd=1;

        std::string name(msg->get(idd++).asString().c_str());
        std::string label(cmd=="object_with_label"?msg->get(idd++).asString().c_str():"");

        double dx=msg->get(idd++).asDouble();
        double dy=msg->get(idd++).asDouble();
        double dz=msg->get(idd++).asDouble();

        double px=msg->get(idd++).asDouble();
        double py=msg->get(idd++).asDouble();
        double pz=msg->get(idd++).asDouble();

        double rx=msg->get(idd++).asDouble();
        double ry=msg->get(idd++).asDouble();
        double rz=msg->get(idd++).asDouble();

        int r=msg->get(idd++).asInt();
        int g=msg->get(idd++).asInt();
        int b=msg->get(idd++).asInt();

        double alpha=msg->get(idd++).asDouble();

        for (int i=0; i<(int)mObjectsRoot.size(); ++i)
        {
            if (mObjectsRoot[i] && *mObjectsRoot[i]==name)
            {
                mObjectsRoot[i]->set(dx,dy,dz,px,py,pz,rx,ry,rz,r,g,b,alpha,label);
                return;
            }
        }

        for (int i=0; i<(int)mObjectsWorld.size(); ++i)
        {
            if (mObjectsWorld[i] && *mObjectsWorld[i]==name)
            {
                mObjectsWorld[i]->set(dx,dy,dz,px,py,pz,rx,ry,rz,r,g,b,alpha,label);
                return;
            }
        }

        bool bWorld=(msg->size()>idd && msg->get(idd).asString()=="WORLD");

        if (bWorld)
        {
            mObjectsWorld.push_back(new VisionObj(name,dx,dy,dz,px,py,pz,rx,ry,rz,r,g,b,alpha,label));
        }
        else
        {
            mObjectsRoot.push_back(new VisionObj(name,dx,dy,dz,px,py,pz,rx,ry,rz,r,g,b,alpha,label));
        }

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

        for (int i=0; i<(int)mTrajectoriesRoot.size(); ++i)
        {
            if (mTrajectoriesRoot[i] && *mTrajectoriesRoot[i]==name)
            {
                mTrajectoriesRoot[i]->set(label,bufflen,persistence,R,G,B,alpha,width);
                return;
            }
        }

        for (int i=0; i<(int)mTrajectoriesWorld.size(); ++i)
        {
            if (mTrajectoriesWorld[i] && *mTrajectoriesWorld[i]==name)
            {
                mTrajectoriesWorld[i]->set(label,bufflen,persistence,R,G,B,alpha,width);
                return;
            }
        }

        bool bWorld=(msg->size()>10 && msg->get(10).asString()=="WORLD");

        if (bWorld)
        {
            mTrajectoriesWorld.push_back(new TrajectoryObj(name,label,bufflen,persistence,R,G,B,alpha,width,bWorld));
        }
        else
        {
            mTrajectoriesRoot.push_back(new TrajectoryObj(name,label,bufflen,persistence,R,G,B,alpha,width,bWorld));
        }

        return;
    }

    if (cmd=="addpoint")
    {
        std::string name(msg->get(1).asString().c_str());

        double x=msg->get(2).asDouble();
        double y=msg->get(3).asDouble();
        double z=msg->get(4).asDouble();

        for (int i=0; i<(int)mTrajectoriesRoot.size(); ++i)
        {
            if (mTrajectoriesRoot[i] && *mTrajectoriesRoot[i]==name)
            {
                mTrajectoriesRoot[i]->update(x,y,z);
                return;
            }
        }

        for (int i=0; i<(int)mTrajectoriesWorld.size(); ++i)
        {
            if (mTrajectoriesWorld[i] && *mTrajectoriesWorld[i]==name)
            {
                mTrajectoriesWorld[i]->update(x,y,z);
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

    for (int i=0; i<(int)mObjectsRoot.size(); ++i)
    {
        if (mObjectsRoot[i] && *mObjectsRoot[i]==objName)
        {
            mObjectsRoot[i]->mW=xdim2;
            mObjectsRoot[i]->mH=ydim2;
            mObjectsRoot[i]->mTextureBuffer=buffer;
            return;
        }
    }

    for (int i=0; i<(int)mObjectsWorld.size(); ++i)
    {
        if (mObjectsWorld[i] && *mObjectsWorld[i]==objName)
        {
            mObjectsWorld[i]->mW=xdim2;
            mObjectsWorld[i]->mH=ydim2;
            mObjectsWorld[i]->mTextureBuffer=buffer;
            return;
        }
    }
}

void ObjectsManager::manage(iCub::skinDynLib::skinContactList &forces)
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

        if (p >= 0 && p < 8 && l >= 0 && l < 8)
        {
            if (mAB[p][l])
            {
                mAB[p][l]->addArrow(new ForceArrow(P[0],P[1],P[2],f,F[0],F[1],F[2],M[0],M[1],M[2]));
            }
        }
        else
        {
            yError("iCubGui: unexpected body part %d or link index %d contained in received skinContactList.", p, l);
        }

    }
}

#endif
