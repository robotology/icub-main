/*
 * bvhnoderpy_xyz.h
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

#ifndef BVHNODERPY_XYZ_H
#define BVHNODERPY_XYZ_H

#include "bvhnode.h"

class BVHNodeXYZ_RPY : public BVHNode
{
public:
    BVHNodeXYZ_RPY(const QString& name,double x,double y,double z,double yaw=0.0,double pitch=0.0,double roll=0.0)
        : BVHNode(name,-1,NULL)
    {
        dX=x; dY=y; dZ=z;

        dYaw=yaw; dPitch=pitch; dRoll=roll;
    }

    virtual void draw(double *encoders,BVHNode *pSelected)
    {
        glPushMatrix();

        glTranslated(dX,dY,dZ);

        glColor4f(0.5,0.5,0.5,1.0);
        glLineWidth(3.0);
        glBegin(GL_LINES);
        glVertex3d(0.0,0.0,0.0);
        glVertex3d(-dX,-dY,-dZ);
        glEnd();

        glRotated(dYaw,  0.0,0.0,1.0);
        glRotated(dPitch,0.0,1.0,0.0);
        glRotated(dRoll, 1.0,0.0,0.0);

        glColor4f(0.5,0.5,0.5,1.0);

        drawJoint();

        for(int i=0; i<children.count(); ++i)
        {
            children[i]->draw(encoders,pSelected);
        }

        glPopMatrix();
    }

protected:
    double dX,dY,dZ;
    double dYaw,dPitch,dRoll;
};

#endif
