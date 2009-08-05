/*
 * bvhnode.h
 */

#ifndef BVHNODE_H
#define BVHNODE_H

//#include <QtCore>

#ifdef __APPLE__
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <QApplication.h>
#else
#if defined(WIN32) || defined(WIN64)
#include <windows.h>
#endif
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#include <qstring.h>
#include <qvaluelist.h>
#include <qslider.h>
#include "mesh.h"

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

//typedef enum { BVH_NULL=0, BVH_ROOT, BVH_JOINT, BVH_END } BVHNodeType;

class BVHNode
{
public:
    BVHNode(const QString& name,int enc=-1,iCubMesh *mesh=0)
    {
        children.clear();
        
        m_name=name;
        nEnc=enc;
        pMesh=mesh;
        cyl=gluNewQuadric();
        gluQuadricDrawStyle(cyl,GLU_FILL);
    }
    virtual ~BVHNode()
    {
        for(unsigned int i=0; i<children.count(); ++i)
        {
            delete children[i];
        }
    
        if (pMesh) delete pMesh;
        gluDeleteQuadric(cyl);
    }

    const QString& name() const;
    int numChildren() const;
    //BVHNode* child(int num);
    void addChild(BVHNode* newChild);
    //void insertChild(BVHNode* newChild,int index);
    //void removeChild(BVHNode* child);
    
    virtual void draw(double *encoders,BVHNode* pSelected)=0;
    virtual void setSliders(QSlider *rx,QSlider *ry,QSlider *rz,QSlider *px,QSlider *py,QSlider *pz){}
        
protected:
    virtual void drawRotJoint()
    {
        //GLUquadricObj *cyl=gluNewQuadric();
        //gluQuadricDrawStyle(cyl,GLU_FILL);
        glTranslated(0.0,0.0,-12.7);
        gluDisk(cyl,0.0,10.16,16,16);
        gluCylinder(cyl,10.16,10.16,25.4,16,16);
        glTranslated(0.0,0.0,25.4);
        gluDisk(cyl,0.0,10.16,16,16);
        glTranslated(0.0,0.0,-12.7);
        //gluDeleteQuadric(cyl);
    }

    GLUquadricObj *cyl;

    void setName(const QString& newName);

    QString m_name;
    QValueList<BVHNode*> children;
    
    int nEnc;
    iCubMesh *pMesh;
};

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
        /*
        GLint renderMode;
        glGetIntegerv(GL_RENDER_MODE,&renderMode);
        bool bSelecting=renderMode==GL_SELECT;
        */
        
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
            
        drawRotJoint();
       
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
            
        drawRotJoint();
        
        glRotated(encoders[nEnc],0.0,0.0,1.0);
        
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

protected:
    double dA,dD,dAlpha,dTheta0,dThetaMin,dThetaMax;
};

class BVHNodeEND : public BVHNodeDH 
{
public:
    
    BVHNodeEND(const QString& name,double a,double d,double alpha,double theta0,iCubMesh* mesh=0)
        : BVHNodeDH(name,-1,a,d,alpha,theta0,0.0,0.0,mesh){}
        
    virtual void drawJoint()
    {
        //glColor4f(0.0,1.0,0.0,1.0);
        //glutSolidSphere(10.16,16,16);
        /*
        glLineWidth(3.0);
        glColor4f(1.0,0.0,0.0,1.0);
        glBegin(GL_LINES);
        glVertex3d(0.0,0.0,0.0);
        glVertex3d(100.0,0.0,0.0);
        glEnd(); 

        glColor4f(0.0,1.0,0.0,1.0);
        glBegin(GL_LINES);
        glVertex3d(0.0,0.0,0.0);
        glVertex3d(0.0,100.0,0.0);
        glEnd(); 

        glColor4f(0.0,0.0,1.0,1.0);
        glBegin(GL_LINES);
        glVertex3d(0.0,0.0,0.0);
        glVertex3d(0.0,0.0,100.0);
        glEnd();
        */ 
    }
        
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
        
        drawJoint();
        
        if (pMesh)
        { 
            glColor4f(0.9,0.8,0.7,1.0);
            pMesh->Draw();
        }
        
        glPopMatrix();   
    }
};  
  
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

class BVHNodeINERTIAL : public BVHNodeEND 
{
public:
    
    BVHNodeINERTIAL(const QString& name,double a,double d,double alpha,double theta0,QString portIMUName,iCubMesh* mesh=0)
        : BVHNodeEND(name,a,d,alpha,theta0,mesh)
        { 
            memset(dInertial,0,sizeof(dInertial));
            
            bHasInertial=portIMU.open("/iCubGui/inertial");
            
			const int RETRY=2;
            if (bHasInertial)
            {
                for (int i=0; i<=RETRY; ++i)
			    {
			        if (i==RETRY) 
			        {
			            bHasInertial=false;
			            break;
			        }
			        
			        if (yarp::os::Network::connect(portIMUName.latin1(),"/iCubGui/inertial")) break;
			        yarp::os::Time::delay(1.0);
			    }
            } 
        }
        
	virtual ~BVHNodeINERTIAL(){ portIMU.interrupt(); portIMU.close(); qDebug("CLOSING INERTIAL"); }
        
    void drawArc(double dOmega)
    {            
        static const double dRadius=110.0;
        static const double dDeg2Rad=M_PI/180.0,dThr=4.0*dDeg2Rad,dRad2Deg=180.0/M_PI;

        if (dOmega<-dThr || dOmega>dThr)
        {
            //dOmega*=2.0;
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
        
    virtual void drawJoint()
    {
        if (bHasInertial)
        {
            pIMUData=portIMU.read(false);
            
            if (pIMUData)
            {
                for (int i=3; i<9; ++i)
                    dInertial[i]=pIMUData->get(i).asDouble();
            }
        /*
        }
        else
        {
            dInertial[3]=dInertial[4]=0.0;
            dInertial[5]=10.0;
        }
        */
            glTranslated(70.0,0.0,230.0);
            glColor4f(0.4,0.4,1.0,1.0);
            glutSolidCube(22.0);
            
            // Accelerometer
            glLineWidth(3.0);
            glColor4f(1.0,0.0,0.0,1.0);
            glBegin(GL_LINES);
            glVertex3d(0.0,0.0,0.0);
            glVertex3d(-10.0*dInertial[3],0.0,0.0);
            glEnd(); 

            glColor4f(0.0,1.0,0.0,1.0);
            glBegin(GL_LINES);
            glVertex3d(0.0,0.0,0.0);
            glVertex3d(0.0,-10.0*dInertial[4],0.0);
            glEnd(); 

            glColor4f(0.0,0.0,1.0,1.0);
            glBegin(GL_LINES);
            glVertex3d(0.0,0.0,0.0);
            glVertex3d(0.0,0.0,-10.0*dInertial[5]);
            glEnd(); 
        
            // Gyro
            
            glLineWidth(2.0);
            glDisable(GL_LINE_SMOOTH);
        
            //dInertial[8]=60.0;
            //dInertial[7]=50.0;
            //dInertial[6]=40.0;
        
            glColor4f(0.0,0.0,1.0,1.0);
            glPushMatrix();
            glRotated(-90.0,0.0,0.0,1.0);
            drawArc(dInertial[8]);
            glPopMatrix();
        
            glColor4f(0.0,1.0,0.0,1.0);
            glPushMatrix();
            glRotated(-90.0,1.0,0.0,0.0);
            drawArc(dInertial[7]);
            glPopMatrix();
        
            glColor4f(1.0,0.0,0.0,1.0);
            glPushMatrix();
            glRotated(90.0,0.0,1.0,0.0);
            drawArc(dInertial[6]);
            glPopMatrix();        
        
            glEnable(GL_LINE_SMOOTH);
        }
    }
protected:
    double dInertial[12];
    bool bHasInertial;
    yarp::os::BufferedPort<yarp::os::Bottle> portIMU;
    yarp::os::Bottle *pIMUData;
};

class BVHNodeLEFTHAND : public BVHNodeEND 
{
public:
    
    BVHNodeLEFTHAND(const QString& name,int enc,double a,double d,double alpha,double theta0,iCubMesh* mesh=0)
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
        
        /*
        glColor4f(0.5,0.5,0.5,1.0);
        glLineWidth(3.0);
        glBegin(GL_LINES);
        glVertex3d(0.0,0.0,0.0);
        glVertex3d(-dA,0.0,-dD);
        glEnd();
        */
        
        glRotated(dAlpha,1.0,0.0,0.0);
        
        //drawJoint();
        
        glColor4f(0.2,0.2,0.2,1.0);
        glBegin(GL_QUADS);
        glNormal3d(0.0,0.0,1.0);
        glVertex3d(-63.0,-12.0,16.0);
        glNormal3d(0.0,0.0,1.0);
        glVertex3d(15.0,-30.0,16.0);
        glNormal3d(0.0,0.0,1.0);
        glVertex3d(15.0,30.0,16.0);
        glNormal3d(0.0,0.0,1.0);
        glVertex3d(-63.0,12.0,16.0);
        glNormal3d(0.0,0.0,-1.0);
        glVertex3d(-63.0,-12.0,6.0);
        glNormal3d(0.0,0.0,-1.0);
        glVertex3d(15.0,-30.0,6.0);
        glNormal3d(0.0,0.0,-1.0);
        glVertex3d(15.0,30.0,6.0);
        glNormal3d(0.0,0.0,-1.0);
        glVertex3d(-63.0,12.0,6.0);
        
        glNormal3d(0.0,-1.0,0.0);
        glVertex3d(-63.0,-12.0,16.0);
        glNormal3d(0.0,-1.0,0.0);
        glVertex3d(-63.0,-12.0,6.0);
        glNormal3d(0.0,-1.0,0.0);
        glVertex3d(15.0,-30.0,6.0);
        glNormal3d(0.0,-1.0,0.0);
        glVertex3d(15.0,-30.0,16.0);        

        glNormal3d(0.0,1.0,0.0);
        glVertex3d(15.0,30.0,16.0);        
        glNormal3d(0.0,1.0,0.0);
        glVertex3d(15.0,30.0,6.0);
        glNormal3d(0.0,1.0,0.0);
        glVertex3d(-63.0,12.0,6.0);
        glNormal3d(0.0,1.0,0.0);
        glVertex3d(-63.0,12.0,16.0);
        glEnd();

        glColor4f(0.5,0.5,0.5,1.0);
                
        //thumb
        glPushMatrix();
        glTranslated(-30.0,-7.5,5.0);
        //glRotated(-75.0,0.0,0.0,1.0);
        glRotated(-90.0,0.0,0.0,1.0);
        
        glRotated(110.0+encoders[nEnc+8],0.0,1.0,0.0);
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
        glTranslated(15.0,-7.5,11.0);
        glRotated(-7.5+0.25*encoders[nEnc+7],0.0,0.0,1.0);
        glRotated(90.0+encoders[nEnc+13],0.0,1.0,0.0);
        FingerSegment(35.0);        
        
        glRotated(encoders[nEnc+14],0.0,1.0,0.0);
        FingerSegment(22.0);
                
        glRotated(encoders[nEnc+14],0.0,1.0,0.0);
        FingerSegment(13.0);
        glPopMatrix();
                
        // index
        glPushMatrix();
        glTranslated(15.0,-22.5,11.0);
        glRotated(-22.5+0.75*encoders[nEnc+7],0.0,0.0,1.0);
        glRotated(90.0+encoders[nEnc+11],0.0,1.0,0.0);
        FingerSegment(31.0);
                
        glRotated(encoders[nEnc+12],0.0,1.0,0.0);
        FingerSegment(19.0);
                
        glRotated(encoders[nEnc+12],0.0,1.0,0.0);
        FingerSegment(12.0);
        glPopMatrix();

        // ring
        glPushMatrix();
        glTranslated(15.0,7.5,11.0);
        glRotated(7.5-0.25*encoders[nEnc+7],0.0,0.0,1.0);
        glRotated(90.0+encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(31.0);
                
        glRotated(encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(19.0);
                
        glRotated(encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(12.0);
        glPopMatrix();

        // pinky
        glPushMatrix();
        glTranslated(15.0,22.5,11.0);
        glRotated(22.5-0.75*encoders[nEnc+7],0.0,0.0,1.0);
        glRotated(90.0+encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(27.0);
                
        glRotated(encoders[nEnc+15],0.0,1.0,0.0);
        FingerSegment(17.0);
                
        glRotated(encoders[nEnc+15],0.0,1.0,0.0);
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
