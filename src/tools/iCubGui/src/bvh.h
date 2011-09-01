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

#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp::dev;
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

#define __YARP

class BVH
{
public:
    BVH(ObjectsManager* objManager=NULL);
    bool Create(yarp::os::ResourceFinder& config);
    ~BVH();
   
    QStringList partNames,bvhChannelName;
    
    void draw()
    {
        #ifdef __YARP
        #define PAN  4
        #define VERG 5
        if (pEncTorso) pEncTorso->getEncoders(dEncTorso);
        if (pEncHead) 
        {
            pEncHead->getEncoders(dEncHead);
            double dLeftEye =dEncHead[PAN]-0.5*dEncHead[VERG];
            double dRightEye=dEncHead[PAN]+0.5*dEncHead[VERG];
            dEncHead[4]=dLeftEye;
            dEncHead[5]=dRightEye;
        }
        if (pEncLeftArm) pEncLeftArm->getEncoders(dEncLeftArm);
        if (pEncRightArm) pEncRightArm->getEncoders(dEncRightArm);
        if (pEncLeftLeg) pEncLeftLeg->getEncoders(dEncLeftLeg);
        if (pEncRightLeg) pEncRightLeg->getEncoders(dEncRightLeg);
        #endif

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
    
    PolyDriver* OpenDriver(QString part);
    void CloseDriver(PolyDriver* &pDriver);
    
    PolyDriver *pTorsoDriver,*pHeadDriver,*pLeftArmDriver,*pRightArmDriver,*pLeftLegDriver,*pRightLegDriver;
    IEncoders *pEncTorso,*pEncHead,*pEncLeftArm,*pEncRightArm,*pEncLeftLeg,*pEncRightLeg;
    int nJTorso,nJHead,nJLeftArm,nJRightArm,nJLeftLeg,nJRightLeg;
    
    double dEncBuffer[59];
    double *dEncTorso,*dEncHead,*dEncLeftArm,*dEncRightArm,*dEncLeftLeg,*dEncRightLeg,*dEncRoot;
};

#endif
