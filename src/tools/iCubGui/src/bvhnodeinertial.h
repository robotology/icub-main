/*
 * bvhnodeinertial.h
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

#ifndef BVHNODEINERTIAL_H
#define BVHNODEINERTIAL_H

#include "bvhnodeend.h"

class BVHNodeINERTIAL : public BVHNodeEND 
{
public:
    
    BVHNodeINERTIAL(const QString& name,double a,double d,double alpha,double theta0,QString portIMUName,iCubMesh* mesh=0)
        : BVHNodeEND(name,-1,a,d,alpha,theta0,mesh)
        { 
            memset(dInertial,0,sizeof(dInertial));
            
            bHasInertial=portIMU.open("/iCubGui/inertial");
            
			const int RETRY=4;
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
                    
			        yarp::os::Time::delay(0.5);
			    }
            } 
        }
        
	virtual ~BVHNodeINERTIAL()
    { 
        portIMU.interrupt(); 
        portIMU.close(); 
        qDebug("CLOSING INERTIAL"); 
    }
        
    virtual void drawJoint()
    {
        if (bHasInertial)
        {
            pIMUData=portIMU.read(false);
            
            if (pIMUData)
            {
                for (int i=3; i<9; ++i)
                {
                    dInertial[i]=pIMUData->get(i).asDouble();
                }
            }

            glTranslated(40.0,0.0,230.0);
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

#endif
