/*
 * bvhnoderighthand.h
 */

#ifndef BVHNODERIGHTHAND_H
#define BVHNODERIGHTHAND_H

#include "bvhnodeend.h"

class BVHNodeRIGHTHAND : public BVHNodeEND 
{
public:
    
    BVHNodeRIGHTHAND(const QString& name,int enc,double a,double d,double alpha,double theta0,iCubMesh* mesh=0)
        : BVHNodeEND(name,a,d,alpha,theta0,mesh){ nEnc=enc; }
        
    void FingerSegment(double length)
    {
		length-=1.0;
        gluDisk(cyl,0.0,5.0,16,16);
        gluCylinder(cyl,5.0,5.0,length,16,16);
        glTranslated(0.0,0.0,length);
        gluDisk(cyl,0.0,5.0,16,16);
		glTranslated(0.0,0.0,1.0);
    }
        
    virtual void draw(double* encoders,BVHNode* pSelected)
    { 
        /*
        encoders[nEnc+7]=10.0;
        encoders[nEnc+8]=30.0;
        encoders[nEnc+9]=20.0;
        encoders[nEnc+10]=20.0;
        encoders[nEnc+11]=20.0;
        encoders[nEnc+12]=20.0;
        encoders[nEnc+13]=20.0;
        encoders[nEnc+14]=20.0;
        encoders[nEnc+15]=20.0;
        */ 
        glPushMatrix();
        
        glRotated(dTheta0,0.0,0.0,1.0);
        glTranslated(dA,0.0,dD);
        
        glRotated(dAlpha,1.0,0.0,0.0);
        
        //drawJoint();
        
        glColor4f(0.2,0.2,0.2,1.0);
        glBegin(GL_QUADS);
        glNormal3d(0.0,0.0,-1.0);
        glVertex3d(-63.0,-12.0,-16.0);
        glNormal3d(0.0,0.0,-1.0);
        glVertex3d(15.0,-30.0,-16.0);
        glNormal3d(0.0,0.0,-1.0);
        glVertex3d(15.0,30.0,-16.0);
        glNormal3d(0.0,0.0,-1.0);
        glVertex3d(-63.0,12.0,-16.0);
        glNormal3d(0.0,0.0,1.0);
        glVertex3d(-63.0,-12.0,-6.0);
        glNormal3d(0.0,0.0,1.0);
        glVertex3d(15.0,-30.0,-6.0);
        glNormal3d(0.0,0.0,1.0);
        glVertex3d(15.0,30.0,-6.0);
        glNormal3d(0.0,0.0,1.0);
        glVertex3d(-63.0,12.0,-6.0);
        
        glNormal3d(0.0,-1.0,0.0);
        glVertex3d(-63.0,-12.0,-16.0);
        glNormal3d(0.0,-1.0,0.0);
        glVertex3d(-63.0,-12.0,-6.0);
        glNormal3d(0.0,-1.0,0.0);
        glVertex3d(15.0,-30.0,-6.0);
        glNormal3d(0.0,-1.0,0.0);
        glVertex3d(15.0,-30.0,-16.0);        

        glNormal3d(0.0,1.0,0.0);
        glVertex3d(15.0,30.0,-16.0);        
        glNormal3d(0.0,1.0,0.0);
        glVertex3d(15.0,30.0,-6.0);
        glNormal3d(0.0,1.0,0.0);
        glVertex3d(-63.0,12.0,-6.0);
        glNormal3d(0.0,1.0,0.0);
        glVertex3d(-63.0,12.0,-16.0);
        glEnd();

        glColor4f(0.5,0.5,0.5,1.0);
                
        //thumb
        glPushMatrix();
        glTranslated(-30.0,-7.5,-5.0);
        //glRotated(-75.0,0.0,0.0,1.0);
        glRotated(-90.0,0.0,0.0,1.0);
        
        glRotated(70.0-encoders[nEnc+8],0.0,1.0,0.0);
        FingerSegment(10.0);    
        
        glRotated(-encoders[nEnc+9],1.0,0.0,0.0);
        FingerSegment(20.0);        
         
        glRotated(-encoders[nEnc+10],1.0,0.0,0.0);
        FingerSegment(20.0);
                
        glRotated(-encoders[nEnc+10],1.0,0.0,0.0);
        FingerSegment(20.0);                 
        glPopMatrix();
        
        // middle
        glPushMatrix();
        glTranslated(15.0,-7.5,-11.0);
        glRotated(-7.5+0.25*encoders[nEnc+7],0.0,0.0,1.0);
        glRotated(90.0-encoders[nEnc+13],0.0,1.0,0.0);
        FingerSegment(35.0);        
        
        glRotated(-encoders[nEnc+14],0.0,1.0,0.0);
        FingerSegment(22.0);
                
        glRotated(-encoders[nEnc+14],0.0,1.0,0.0);
        FingerSegment(13.0);
        glPopMatrix();
                
        // index
        glPushMatrix();
        glTranslated(15.0,-22.5,-11.0);
        glRotated(-22.5+0.75*encoders[nEnc+7],0.0,0.0,1.0);
        glRotated(90.0-encoders[nEnc+11],0.0,1.0,0.0);
        FingerSegment(31.0);
                
        glRotated(-encoders[nEnc+12],0.0,1.0,0.0);
        FingerSegment(19.0);
                
        glRotated(-encoders[nEnc+12],0.0,1.0,0.0);
        FingerSegment(12.0);
        glPopMatrix();

        // ring
        glPushMatrix();
        glTranslated(15.0,7.5,-11.0);
        glRotated(7.5-0.25*encoders[nEnc+7],0.0,0.0,1.0);
        glRotated(90.0-encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(31.0);
                
        glRotated(-encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(19.0);
                
        glRotated(-encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(12.0);
        glPopMatrix();

        // pinky
        glPushMatrix();
        glTranslated(15.0,22.5,-11.0);
        glRotated(22.5-0.75*encoders[nEnc+7],0.0,0.0,1.0);
        glRotated(90.0-encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(27.0);
                
        glRotated(-encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(17.0);
                
        glRotated(-encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(10.0);
        glPopMatrix();        
                
        if (pMesh)
        { 
            glColor4f(0.9,0.8,0.7,1.0);
            pMesh->Draw();
        }
        
        glPopMatrix();   
    }
};  

#endif
