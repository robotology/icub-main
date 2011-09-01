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
    ForceArrow(double x,double y,double z,double f,double fx,double fy,double fz)
    {
        static const double dRad2Deg=180.0/M_PI;

        px=x; py=y; pz=z;

        m=f-20.0;
        if (m<0.0) m=0.0;

        double a=sqrt(fx*fx+fy*fy);
            
        if (a!=0.0)
        {
            th=dRad2Deg*atan2(a,fz);
            ax=-fy/a;
            ay= fx/a;
            az= 0.0;
        }
        else
        {
            th=0.0;
            ax=0.0;
            ay=0.0;
            az=1.0;
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

        glPushMatrix();
        glColor4f(1.0f,0.0f,0.0f,1.0f);
        glTranslated(px,py,pz);
        glRotated(th,ax,ay,az);
        glTranslated(0.0,0.0,-20.0); // cone base
        glutSolidCone(5.0,20.0,16,4);
        glTranslated(0.0,0.0,-m);
        gluCylinder(cyl,2.5,2.5,m,16,4);
        glPopMatrix();
    }

protected:
    GLUquadricObj *cyl;

    double px,py,pz,m;
    double th,ax,ay,az;
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
