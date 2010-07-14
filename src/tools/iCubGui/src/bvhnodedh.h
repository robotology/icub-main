/*
 * bvhnodedh.h
 */

#ifndef BVHNODEDH_H
#define BVHNODEDH_H

#include "bvhnode.h"

class BVHNodeDH : public BVHNode
{
public:
    BVHNodeDH(const QString& name,int enc,double a,double d,double alpha,double theta0,double thetamin=-180.0,double thetamax=180.0,iCubMesh* mesh=0) 
    : BVHNode(name,enc,mesh)
    {
        dA=a; dD=d; dAlpha=alpha; dTheta0=theta0; dThetaMin=thetamin; dThetaMax=thetamax;
    }

    virtual void draw(double *encoders,BVHNode* pSelected)
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
        
        if (pSelected==this) 
            glColor4f(1.0,0.0,0.0,1.0);
        else
            glColor4f(0.5,0.5,0.5,1.0);
        
        glRotated(encoders[nEnc],0.0,0.0,1.0);
        
        if (pMesh)
        { 
            glColor4f(0.9,0.8,0.7,m_Alpha);
            pMesh->Draw();
        }

        glColor4f(0.5,0.5,0.5,1.0);
        drawJoint();
    
        for (unsigned int i=0; i<children.count(); ++i)
        {
            children[i]->draw(encoders,pSelected);
        }
        
        glPopMatrix();   
    }

protected:
    void drawArc(double dOmega)
    {            
        static const double dRadius=80.0;
        static const double dDeg2Rad=M_PI/180.0,dThr=4.0*dDeg2Rad,dRad2Deg=180.0/M_PI;

        if (dOmega<-dThr || dOmega>dThr)
        {
            double dNeg=dOmega>=0.0?1.0:-1.0;
            double dAngle=dNeg*dOmega;

            glBegin(GL_LINE_STRIP);
            for (double angle=-dAngle; angle<=dAngle; angle+=dDeg2Rad)
                glVertex3d(-dRadius*cos(angle),dRadius*sin(angle),0.0);
            glEnd();
 
            glRotated(dOmega*dRad2Deg,0.0,0.0,1.0);
            glTranslated(-dRadius,0.0,0.0);
            glRotated(dNeg*90.0,1.0,0.0,0.0);
            glutSolidCone(7.5,30.0,16,16);
        }
    }

    void drawArrow(double dMag)
    {            
        glBegin(GL_LINES);
        glVertex3d(0.0,0.0,0.0);
        glVertex3d(0.0,0.0,dMag);
        glEnd(); 
 
        glTranslated(0.0,0.0,dMag);
        glRotated(dMag<0.0?180.0:0.0,1.0,0.0,0.0);
        glutSolidCone(7.5,30.0,16,16);
    }

    double dA,dD,dAlpha,dTheta0,dThetaMin,dThetaMax;
};

#endif
