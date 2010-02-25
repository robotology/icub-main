/*
 * bvhnodeend.h
 */

#ifndef BVHNODEEND_H
#define BVHNODEEND_H

#include "bvhnodedh.h"

class BVHNodeEND : public BVHNodeDH 
{
public:
    
    BVHNodeEND(const QString& name,double a,double d,double alpha,double theta0,iCubMesh* mesh=0)
        : BVHNodeDH(name,-1,a,d,alpha,theta0,0.0,0.0,mesh){}
        
    virtual void drawJoint(){}
        
    virtual void draw(double* encoders,BVHNode* pSelected)
    {  
        glPushMatrix();
        
        glRotated(dTheta0,0.0,0.0,1.0);
        glTranslated(dA,0.0,dD);
        
        glColor4f(0.5,0.5,0.5,1.0);
        glLineWidth(3.0);
        glBegin(GL_LINES);
        glVertex3d(0.0,0.0,0.0);
        glVertex3d(-dA,0.0,-dD);
        glEnd();
        
        glRotated(dAlpha,1.0,0.0,0.0);
        
        if (pMesh)
        { 
            glColor4f(0.9,0.8,0.7,1.0);
            pMesh->Draw();
        }

        drawJoint();
        
        glPopMatrix();   
    }
};  
  
#endif
