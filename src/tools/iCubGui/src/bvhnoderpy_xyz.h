/*
 * bvhnoderpy_xyz.h
 */

#ifndef BVHNODERPY_XYZ_H
#define BVHNODERPY_XYZ_H

#include "bvhnode.h"

class BVHNodeRPY_XYZ : public BVHNode
{
public:
    BVHNodeRPY_XYZ(const QString& name,int enc,double yaw,double pitch,double roll,double x,double y,double z,
                   double rotmin=-180.0,double rotmax=180.0,iCubMesh* mesh=0) : BVHNode(name,enc,mesh)
    {
        dYaw=yaw; dPitch=pitch; dRoll=roll;
        dX=x; dY=y; dZ=z;
        
        dRotMin=rotmin; dRotMax=rotmax;
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
               
        if (pSelected==this) 
            glColor4f(1.0,0.0,0.0,1.0);
        else
            glColor4f(0.5,0.5,0.5,1.0);
            
        drawJoint();
       
        glRotated(encoders[nEnc],0.0,0.0,1.0);
        
        if (pMesh)
        { 
            glColor4f(0.9,0.8,0.7,1.0);
            pMesh->Draw();
        }
        
        for(unsigned int i=0; i<children.count(); ++i)
        {
            children[i]->draw(encoders,pSelected);
        }
        
        glPopMatrix();
    }

protected:
    double dYaw,dPitch,dRoll,dX,dY,dZ;
    double dRotMin,dRotMax;
};

#endif
