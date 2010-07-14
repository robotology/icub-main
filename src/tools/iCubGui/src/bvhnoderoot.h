/*
 * bvhnoderoot.h
 */

#ifndef BVHNODEROOT_H
#define BVHNODEROOT_H

#include "bvhnoderpy_xyz.h"

class BVHNodeROOT : public BVHNodeRPY_XYZ
{
public:
    BVHNodeROOT(const QString& name,int enc,double yaw,double pitch,double roll,double x,double y,double z,iCubMesh* mesh=0) 
        : BVHNodeRPY_XYZ(name,enc,yaw,pitch,roll,x,y,z,-180.0,180.0,mesh){}
        
    virtual void drawJoint(){}
        
    virtual void draw(double *encoders,BVHNode *pSelected)
    {
        glPushMatrix();
        
        glTranslated(encoders[nEnc+3],encoders[nEnc+4],encoders[nEnc+5]);
        glRotated(encoders[nEnc],  0.0,0.0,1.0);
        glRotated(encoders[nEnc+1],0.0,1.0,0.0);
        glRotated(encoders[nEnc+2],1.0,0.0,0.0);    
    
        glTranslated(dX,dY,dZ);
       
        glRotated(dYaw,  0.0,0.0,1.0);
        glRotated(dPitch,0.0,1.0,0.0);
        glRotated(dRoll, 1.0,0.0,0.0);
        
        if (pMesh)
        { 
            glColor4f(0.9,0.8,0.7,1.0);
            pMesh->Draw();
        }
    
        for (unsigned int i=0; i<children.count(); ++i)
        {
            children[i]->draw(encoders,pSelected);
        }
        
        glPopMatrix();
    } 
}; 

#endif
