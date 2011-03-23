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

#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

class ObjectsThread : public QThread
{
public:
    ObjectsThread(const char *portName) : mMutex(1)
    {
        mPort.open(portName);
        mRunning=true;
        start();
    }

    ~ObjectsThread()
    {
        mRunning=false;
        mPort.interrupt();
        mPort.close();

        for (int i=0; i<(int)mObjects.size(); ++i)
        {
            if (mObjects[i]!=NULL) delete mObjects[i];
        }
    }

    void run()
    {
        //yarp::os::Bottle *msg;
        yarp::os::Bottle msg;
            
        while (mRunning)
        {
            //if ((msg=mPort.read())!=NULL)
            if (mPort.read(msg))
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
                        mObjects.push_back(new VisionObj(name,dx,dy,dz,px,py,pz,rx,ry,rz,R,G,B,alpha));
                    }
                }

                mMutex.post();
            }
        }
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
    bool mRunning;
    yarp::os::Semaphore mMutex;
    std::vector<VisionObj*> mObjects;
    //yarp::os::BufferedPort<yarp::os::Bottle> mPort;
    yarp::os::Port mPort;
};

#endif
