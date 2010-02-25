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
    }

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
};

#endif
