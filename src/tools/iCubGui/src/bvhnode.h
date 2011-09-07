/*
 * bvhnode.h
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

#include <vector>
#include <qstring.h>
#include <qvaluelist.h>
#include <qslider.h>
#include "mesh.h"

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

class ForceArrow
{
public:
    ForceArrow(double x,double y,double z,double f,double fx,double fy,double fz,double mx,double my,double mz)
    {
        static const double dRad2Deg=180.0/M_PI;

        px=1000.0*x; py=1000.0*y; pz=1000.0*z;

        fth=0.0;
        fax=0.0;
        fay=0.0;
        faz=1.0;

        bForce=(f>mForceThr);
        
        if (bForce)
        {
            fm=mForceGain*f-20.0;
            if (fm<0.0) fm=0.0;
            
            double a=fx*fx+fy*fy;
            
            if (a>0.0)
            {
                a=sqrt(a);
                fth=dRad2Deg*atan2(a,fz);
                fax=-fy/a;
                fay= fx/a;
                faz= 0.0;
            } 
        }
        
        mth=0.0;
        max=0.0;
        may=0.0;
        maz=1.0;

        double m=mx*mx+my*my+mz*mz;

        bTorque=(m>mTorqueThr);

        if (bTorque)
        {
            m=sqrt(m);

            mx/=m; my/=m; mz/=m;
            mm=mTorqueGain*m-20.0;
            if (mm<0.0) mm=0.0;

            double a=mx*mx+my*my;

            if (a>0.0)
            {
                a=sqrt(a);
                mth=dRad2Deg*atan2(a,mz);
                max=-my/a;
                may= mx/a;
                maz= 0.0;
            }
        }

        gluQuadricDrawStyle(cyl=gluNewQuadric(),GLU_FILL);
    }
    ~ForceArrow()
    {
        if (cyl) gluDeleteQuadric(cyl);
    }

    void draw()
    {
        if (!cyl) return;

        if (bForce)
        {
            glPushMatrix();
            glColor4f(1.0f,0.0f,0.0f,1.0f);
            glTranslated(px,py,pz);
            glRotated(fth,fax,fay,faz);
            glTranslated(0.0,0.0,-20.0); // cone base
            glutSolidCone(5.0,20.0,16,4);
            glTranslated(0.0,0.0,-fm);
            gluCylinder(cyl,2.5,2.5,fm,16,4);
            glPopMatrix();
        }

        if (bTorque)
        {
            glPushMatrix();
            glColor4f(0.0f,0.0f,1.0f,1.0f);
            glTranslated(px,py,pz);
            glRotated(mth,max,may,maz);
            glTranslated(0.0,0.0,-20.0); // cone base
            glutSolidCone(5.0,20.0,16,4);
            glTranslated(0.0,0.0,-mm);
            gluCylinder(cyl,2.5,2.5,mm,16,4);
            glPopMatrix();
        }
    }

    static void setParams(double fg,double ft,double tg,double tt)
    {
        mForceGain=fg;
        mForceThr=ft;
        mTorqueGain=tg;
        mTorqueThr=tt;
    }

protected:
    GLUquadricObj *cyl;
    double px,py,pz;

    double fm,fth,fax,fay,faz;
    double mm,mth,max,may,maz;

    bool bForce,bTorque;

    static double mForceThr;
    static double mForceGain;
    static double mTorqueThr;
    static double mTorqueGain;
};

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

        m_Alpha=1.0;
    }
    virtual ~BVHNode()
    {
        for(unsigned int i=0; i<children.count(); ++i)
        {
            delete children[i];
        }
    
        if (pMesh) delete pMesh;
        gluDeleteQuadric(cyl);

        clearArrows();
    }

    bool isValid(){ return cyl!=NULL; }

    const QString& name() const
    {
        return m_name;
    }
   
    int numChildren() const
    {
        return children.count();
    }
    
    void addChild(BVHNode* pChild)
    {
        children.append(pChild);
    }
    
    virtual void draw(double *encoders,BVHNode* pSelected)=0;
    virtual void setSliders(QSlider *rx,QSlider *ry,QSlider *rz,QSlider *px,QSlider *py,QSlider *pz){}

    void addArrow(ForceArrow *pArrow)
    {
        mArrows.push_back(pArrow);
    }

    void clearArrows()
    {
        for (int a=0; a<(int)mArrows.size(); ++a)
        {
            if (mArrows[a]) delete mArrows[a];
        }

        mArrows.clear();
    }
        
protected:
    virtual void drawJoint()
    {
        glTranslated(0.0,0.0,-12.7);
        gluDisk(cyl,0.0,10.16,16,16);
        gluCylinder(cyl,10.16,10.16,25.4,16,16);
        glTranslated(0.0,0.0,25.4);
        gluDisk(cyl,0.0,10.16,16,16);
        glTranslated(0.0,0.0,-12.7);
    }

    void drawArrows()
    {
        for (int a=0; a<(int)mArrows.size(); ++a)
        {
            if (mArrows[a])
            {
                mArrows[a]->draw();
            }
        }
    }

    void setName(const QString& name)
    {
        m_name=name;
    }

    GLUquadricObj *cyl;
    QString m_name;
    QValueList<BVHNode*> children;
    
    int nEnc;
    iCubMesh *pMesh;

    float m_Alpha;

    std::vector<ForceArrow*> mArrows;
};

#endif
