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

class BVHNodeRPY_XYZ : public BVHNode
{
public:
    BVHNodeRPY_XYZ(const QString& name,double yaw,double pitch,double roll,double x,double y,double z) 
        : BVHNode(name,-1,NULL)
    {
        dYaw=yaw; dPitch=pitch; dRoll=roll;
        dX=x; dY=y; dZ=z;
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
        
        for(unsigned int i=0; i<children.count(); ++i)
        {
            children[i]->draw(encoders,pSelected);
        }
        
        glPopMatrix();
    }

protected:
    double dYaw,dPitch,dRoll,dX,dY,dZ;
};

#endif


