/*
 * bvhnodeforcetorque.h
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

#ifndef BVHNODEFORCETORQUE_H
#define BVHNODEFORCETORQUE_H

#include "bvhnodedh.h"

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>

//#include <yarp/dev/ControlBoardInterfaces.h>
//#include <yarp/dev/PolyDriver.h>
//#include <yarp/dev/CanBusInterface.h>

class BVHNodeForceTorque : public BVHNodeDH
{
    public:
    
    BVHNodeForceTorque(const QString& name,const QString& portName,int enc,double a,double d,double alpha,double theta0,iCubMesh* mesh=0)
        : BVHNodeDH(name,enc,a,d,alpha,theta0,mesh)
    {
        mPort.open(portName.ascii());

        if (pMesh)
        {
            //delete pMesh;
            //pMesh=0;
        }

        //m_Alpha=0.5;

        for (int i=0; i<6; ++i) dForceTorque[i]=0.0;
    }
        
	virtual ~BVHNodeForceTorque()
    {	
        mPort.close();
    }
    
    virtual void drawJoint()
    {
        BVHNodeDH::drawJoint();

        glColor4f(0.4,0.4,1.0,1.0);
        glPushMatrix();
        glTranslated(0.0,0.0,15.0);
        gluDisk(cyl,0.0,27.5,16,16);
        gluCylinder(cyl,27.5,27.5,18.0,16,16);
        glTranslated(0.0,0.0,18.0);
        gluDisk(cyl,0.0,27.5,16,16);
        glTranslated(0.0,0.0,-9.0);

        glDisable(GL_DEPTH_TEST);

        if (mPort.getInputCount()>0)
        {
            yarp::sig::Vector *ft=mPort.read(false);

            if (ft) for (int i=0; i<6; ++i) dForceTorque[i]=(*ft)[i];
        }

        // Force
        glLineWidth(3.0);

        //dForceTorque[0]=dForceTorque[1]=dForceTorque[2]=300.0;

        // X 
        glPushMatrix();
        glColor4f(1.0,0.0,0.0,1.0);
        glRotated(90.0,0.0,1.0,0.0);
        drawArrow(0.1*dForceTorque[1]);
        glPopMatrix();

        // Y
        glColor4f(0.0,1.0,0.0,1.0);
        glPushMatrix();
        glRotated(-90.0,1.0,0.0,0.0);
        drawArrow(0.1*dForceTorque[2]);
        glPopMatrix();

        // Z
        glColor4f(0.0,0.0,1.0,1.0);
        glPushMatrix();
        drawArrow(0.1*dForceTorque[0]);
        glPopMatrix();

        // Torque

        glLineWidth(2.0);
        glDisable(GL_LINE_SMOOTH);

        // X
        glColor4f(1.0,0.0,0.0,1.0);
        glPushMatrix();
        glRotated(90.0,0.0,1.0,0.0);
        drawArc(0.0005*dForceTorque[4]);
        glPopMatrix();

        // Y
        glColor4f(0.0,1.0,0.0,1.0);
        glPushMatrix();
        glRotated(-90.0,1.0,0.0,0.0);
        glRotated(180.0,0.0,0.0,10.0);
        drawArc(-0.0005*dForceTorque[3]);
        glPopMatrix();

        // Z
        glColor4f(0.0,0.0,1.0,1.0);
        glPushMatrix();
        glRotated(180.0,0.0,0.0,10.0);
        drawArc(0.0005*dForceTorque[5]);
        glPopMatrix();

        glPopMatrix();

        glEnable(GL_LINE_SMOOTH);

        glEnable(GL_DEPTH_TEST);
    }

protected:
    double dForceTorque[6];
    
    yarp::os::BufferedPort<yarp::sig::Vector> mPort;
};

#endif


