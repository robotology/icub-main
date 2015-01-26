/*
 * bvhnoderoot.h
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

#ifndef BVHNODEROOT_H
#define BVHNODEROOT_H

#include "bvhnoderpy_xyz.h"
#include "objectsthread.h"

class BVHNodeROOT : public BVHNodeXYZ_RPY
{
public:
    BVHNodeROOT(const QString& name,int enc,double x,double y,double z,iCubMesh* mesh,ObjectsManager* objManager)
        : BVHNodeXYZ_RPY(name,x,y,z)
    {
        nEnc=enc;
        pMesh=mesh;
        mObjectsManager=objManager;
    }

    virtual void drawJoint(){}

    virtual void draw(double *encoders,BVHNode *pSelected)
    {
        // world coordinates
        glPushMatrix();

        glTranslated(dX+encoders[nEnc+3],dY+encoders[nEnc+4],dZ+encoders[nEnc+5]);

        glRotated(encoders[nEnc+2],0.0,0.0,1.0); // yaw
        glRotated(encoders[nEnc+1],0.0,1.0,0.0); // pitch
        glRotated(encoders[nEnc  ],1.0,0.0,0.0); // roll

        // root coordinates
        glPushMatrix();

        if (pMesh)
        {
            glColor4f(0.9,0.8,0.7,1.0);
            pMesh->Draw();
        }

        drawArrows();

        for (int i=0; i<children.count(); ++i)
        {
            children[i]->draw(encoders,pSelected);
        }

        glPopMatrix();
        // root coordinates

        if (mObjectsManager)
        {
            mObjectsManager->update();
            mObjectsManager->drawRootObjects();
        }

        glPopMatrix();
        // world coordinates

        if (mObjectsManager) mObjectsManager->drawWorldObjects();
    }

protected:
    ObjectsManager *mObjectsManager;
};

#endif
