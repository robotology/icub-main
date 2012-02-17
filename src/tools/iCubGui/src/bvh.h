/*
 * bvh.h
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

#ifndef BVH_H
#define BVH_H

/////////////////////////////////////////////

// YARP

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <yarp/sig/Vector.h>

//#include <yarp/dev/GenericSensorInterfaces.h>
//#include <yarp/dev/ControlBoardInterfaces.h>
//#include <yarp/dev/PolyDriver.h>

//using namespace yarp::dev;
using namespace yarp::os;

/////////////////////////////////////////////

//#include <QtCore>

#include <yarp/os/ResourceFinder.h>

#include "bvhnode.h"
#include "bvhnoderpy_xyz.h"
#include "bvhnodedh.h"
#include "bvhnodeend.h"
#include "bvhnodeinertial.h"
#include "bvhnodeforcetorque.h"
#include "bvhnodelefthand.h"
#include "bvhnoderighthand.h"
#include "bvhnodeeye.h"
#include "bvhnoderoot.h"

#include "objectsthread.h"

class BVH
{
public:
    BVH(ObjectsManager* objManager=NULL);
    bool Create(yarp::os::ResourceFinder& config);
    ~BVH();
   
    QStringList partNames,bvhChannelName;
    
    void draw()
    {
        yarp::sig::Vector *enc =NULL;
        yarp::sig::Vector *encV=NULL;

        if (portEncTorso.getInputCount()>0)
        {
            encV=NULL;

            while (enc=portEncTorso.read(false)) encV=enc;
            
            if (encV) for (int i=0; i<nJTorso; ++i) dEncTorso[i]=(*encV)[i];
        }
        
        if (portEncHead.getInputCount()>0) 
        {
            encV=NULL;

            while (enc=portEncHead.read(false)) encV=enc;
            
            if (encV)
            {
                for (int i=0; i<nJHead; ++i) dEncHead[i]=(*encV)[i];

                double dLeftEye =dEncHead[4]-0.5*dEncHead[5];
                double dRightEye=dEncHead[4]+0.5*dEncHead[5];
            
                dEncHead[4]=dLeftEye;
                dEncHead[5]=dRightEye;
            }
        }

        if (portEncLeftArm.getInputCount()>0)
        {
            encV=NULL;

            while (enc=portEncLeftArm.read(false)) encV=enc;
            
            if (encV) for (int i=0; i<nJLeftArm; ++i) dEncLeftArm[i]=(*encV)[i];
        }

        if (portEncRightArm.getInputCount()>0)
        {
            encV=NULL;

            while (enc=portEncRightArm.read(false)) encV=enc;
            
            if (encV) for (int i=0; i<nJRightArm; ++i) dEncRightArm[i]=(*encV)[i];
        }

        if (portEncLeftLeg.getInputCount()>0)
        {
            encV=NULL;

            while (enc=portEncLeftLeg.read(false)) encV=enc;
            
            if (encV) for (int i=0; i<nJLeftLeg; ++i) dEncLeftLeg[i]=(*encV)[i];
        }

        if (portEncRightLeg.getInputCount()>0)
        {
            encV=NULL;

            while (enc=portEncRightLeg.read(false)) encV=enc;
            
            if (encV) for (int i=0; i<nJRightLeg; ++i) dEncRightLeg[i]=(*encV)[i];
        }

        glShadeModel(GL_SMOOTH);
        GLfloat ambientA[]={0.9,0.667,0.561,1};
        GLfloat diffuseA[]={0.9,0.667,0.561,0};
        GLfloat specularA[]={0.6,0.6,0.6,0.0};
        GLfloat shininessA=100.0;
        glMaterialfv(GL_FRONT,GL_AMBIENT,ambientA);
        glMaterialfv(GL_FRONT,GL_DIFFUSE,diffuseA);
        glMaterialfv(GL_FRONT,GL_SPECULAR,specularA);
        glMaterialf(GL_FRONT,GL_SHININESS,shininessA);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHT1);
        //glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        glEnable(GL_COLOR_MATERIAL);
        glEnable(GL_LINE_SMOOTH);
     
         // save current drawing matrix
        glPushMatrix();
        
        //static const double mm2inches=1.0/25.4;
        double dScale=0.075*dAvatarScale;
        glScaled(dScale,dScale,dScale);
        
        // visual compensation
        glTranslated(0.0,3.5,0.0);
        
        pRoot->draw(dEncBuffer,NULL);
        
        glPopMatrix();
    }

  protected:
    QString inputFile;
    QStringList tokens;
    int tokenPos;
    QString token();
    bool expect_token(const QString& expect);
    
    double dAvatarScale;
    BVHNode* bvhRead(yarp::os::ResourceFinder& config);
    BVHNode* bvhReadNode(yarp::os::ResourceFinder& config);
    BVHNode* pRoot;

    ObjectsManager *mObjectsManager;

    BVHNode ***mAB;
    
    // YARP
    
    QString robot;
    
    //PolyDriver* OpenDriver(QString part);
    //void CloseDriver(PolyDriver* &pDriver);
    
    //PolyDriver *pTorsoDriver,*pHeadDriver,*pLeftArmDriver,*pRightArmDriver,*pLeftLegDriver,*pRightLegDriver;
    //IEncoders *pEncTorso,*pEncHead,*pEncLeftArm,*pEncRightArm,*pEncLeftLeg,*pEncRightLeg;
    int nJTorso,nJHead,nJLeftArm,nJRightArm,nJLeftLeg,nJRightLeg;
    
    yarp::os::BufferedPort<yarp::sig::Vector> portEncTorso;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncHead;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncLeftArm;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncRightArm;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncLeftLeg;
    yarp::os::BufferedPort<yarp::sig::Vector> portEncRightLeg;

    double dEncBuffer[59];
    double *dEncTorso,*dEncHead,*dEncLeftArm,*dEncRightArm,*dEncLeftLeg,*dEncRightLeg,*dEncRoot;
};

#endif


