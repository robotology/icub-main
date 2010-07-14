/*
 * bvhnodeeye.h
 */

#ifndef BVHNODEEYE_H
#define BVHNODEEYE_H

#include "bvhnodeend.h"

class BVHNodeEYE : public BVHNodeEND 
{
public:
    
    BVHNodeEYE(const QString& name,double a,double d,double alpha,double theta0,iCubMesh* mesh=0)
        : BVHNodeEND(name,a,d,alpha,theta0,mesh){}
        
    virtual void drawJoint()
    {
        glColor4f(1.0,1.0,1.0,1.0);
        glutSolidSphere(20.32,16,16);
        glTranslated(0.0,0.0,20.32);
        glColor4f(0.0,0.0,0.0,1.0);
        glutSolidSphere(5.08,16,16);
    }
};

#endif
