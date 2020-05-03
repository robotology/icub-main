/*
 * bvhnodeeye.h
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

#ifndef BVHNODEEYE_H
#define BVHNODEEYE_H

#include "bvhnodeend.h"

class BVHNodeEYE : public BVHNodeEND
{
public:

    BVHNodeEYE(const QString& name,int n,double a,double d,double alpha,double theta0,iCubMesh* mesh=0)
        : BVHNodeEND(name,n,a,d,alpha,theta0,mesh){}

    virtual void drawJoint()
    {
        glColor4f(1.0,1.0,1.0,1.0);
        gluSphere(cyl,20.32,16,16);
        glTranslated(0.0,0.0,20.32);
        glColor4f(0.0,0.0,0.0,1.0);
        gluSphere(cyl,5.08,16,16);
    }
};

#endif
