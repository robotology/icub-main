/*
 * bvh.h
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

#include <yarp/String.h> 

#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp;

/////////////////////////////////////////////

//#include <QtCore>

#include "bvhnode.h"

#define ICUB_INI "iCubGui.ini"
#define __YARP

class BVH
{
public:
    BVH(const QString& file);
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
        /*
        dEncBuffer[9]=-40.0;
        dEncBuffer[10]=60.0;
        dEncBuffer[12]=80.0;

        dEncBuffer[25]=-40.0;
        dEncBuffer[26]=60.0;
        dEncBuffer[28]=80.0;
        */
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
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_COLOR_MATERIAL);
        glEnable(GL_LINE_SMOOTH);
     
         // save current drawing matrix
        glPushMatrix();
        
        //static const double mm2inches=1.0/25.4;
        double dScale=0.075*dAvatarScale;
        glScaled(dScale,dScale,dScale);
        
        // visual compensation
        glTranslated(0.0,3.5,0.0);
        
        pRoot->draw(dEncBuffer,0);
        
        glPopMatrix();
    }

  protected:
    QString inputFile;
    QStringList tokens;
    int tokenPos;
    QString token();
    bool expect_token(const QString& expect);
    
    double dAvatarScale;
    BVHNode* bvhRead(const QString& file);
    BVHNode* bvhReadNode();
    BVHNode* pRoot;
    
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
